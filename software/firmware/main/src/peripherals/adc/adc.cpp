#include "adc.hpp"

#include <FreeRTOS.h>
#include <stm32g4xx_hal.h>
#include <task.h>

#include "config.hpp"
#include "debug.hpp"
#include "error.hpp"
#include "gpio.hpp"

void adc_init(void);
void gpio_init(void);
void dma_init(void);
void ocp_init(void);

uint8_t adc_instance_to_idx(ADC_TypeDef* instance);
ADC_HandleTypeDef* adc_instance_init(ADC_TypeDef* instance);

uint16_t rank_idx_to_val(uint8_t rank_idx);

static DMA_HandleTypeDef DMA_ADC1;
static DMA_HandleTypeDef DMA_ADC2;
static volatile uint16_t DMA_BUFFERS[2][6] = {
    {}};  // DMA_ADC1 and DMA_ADC2 buffers
volatile uint16_t* ptr_VMOT_raw = nullptr;
volatile uint16_t* ptr_U_current_raw = nullptr;
volatile uint16_t* ptr_V_current_raw = nullptr;
volatile uint16_t* ptr_W_current_raw = nullptr;
static volatile bool ADC1_conversion_complete = false,
                     ADC2_conversion_complete = false;

struct ADCChannelData {
    ADC_HandleTypeDef* handle = NULL;
    uint8_t dma_buf_idx1 = 0;
    uint8_t dma_buf_idx2 = 0;
};

static ADCChannelData VSENSE_VMOT_ADC_data;
static ADCChannelData ISENSE_U_PHASE_ADC_data;
static ADCChannelData ISENSE_V_PHASE_ADC_data;
static ADCChannelData ISENSE_W_PHASE_ADC_data;

static float U_current_offset = 0;
static float V_current_offset = 0;
static float W_current_offset = 0;

struct ADCData {
    ADC_HandleTypeDef handle;
    bool inited = false;
};

static ADCData adc_data[2];

static uint8_t ADC1_num_conversions = 0, ADC2_num_conversions = 0;

static DAC_HandleTypeDef DAC_3;
static COMP_HandleTypeDef U_PHASE_COMP;
static COMP_HandleTypeDef V_PHASE_COMP;
static COMP_HandleTypeDef W_PHASE_COMP;

void adc::init(void) {
    gpio_init();
    dma_init();
    adc_init();
    ocp_init();
    ptr_VMOT_raw = &DMA_BUFFERS[VSENSE_VMOT_ADC_data.dma_buf_idx1]
                               [VSENSE_VMOT_ADC_data.dma_buf_idx2];
    ptr_U_current_raw = &DMA_BUFFERS[ISENSE_U_PHASE_ADC_data.dma_buf_idx1]
                                    [ISENSE_U_PHASE_ADC_data.dma_buf_idx2];
    ptr_V_current_raw = &DMA_BUFFERS[ISENSE_V_PHASE_ADC_data.dma_buf_idx1]
                                    [ISENSE_V_PHASE_ADC_data.dma_buf_idx2];
    ptr_W_current_raw = &DMA_BUFFERS[ISENSE_W_PHASE_ADC_data.dma_buf_idx1]
                                    [ISENSE_W_PHASE_ADC_data.dma_buf_idx2];
    return;
}

void adc::calibrate_offset(void) {
    float U_current_sum = 0, V_current_sum = 0, W_current_sum = 0;
    for (int i = 0; i < CURRENT_OFFSET_CALIBRATION_CYCLES; i++) {
        vTaskDelay(pdMS_TO_TICKS(1));
        Values values = read();
        U_current_sum += values.current_U;
        V_current_sum += values.current_V;
        W_current_sum += values.current_W;
    }
    U_current_offset = U_current_sum / (float)CURRENT_OFFSET_CALIBRATION_CYCLES;
    V_current_offset = V_current_sum / (float)CURRENT_OFFSET_CALIBRATION_CYCLES;
    W_current_offset = W_current_sum / (float)CURRENT_OFFSET_CALIBRATION_CYCLES;
    return;
}

void adc::start_conversions(void) {
    debug::debug("starting ADC conversions");
    if (ADC1_num_conversions > 0) {
        if (HAL_ADC_Start_DMA(&(adc_data[0].handle),
                              (uint32_t*)(&(DMA_BUFFERS[0][0])),
                              ADC1_num_conversions) != HAL_OK) {
            debug::error("Failed to start ADC1 read");
            error::handler();
        }
    }
    if (ADC2_num_conversions > 0) {
        if (HAL_ADC_Start_DMA(&(adc_data[1].handle),
                              (uint32_t*)(&(DMA_BUFFERS[1][0])),
                              ADC2_num_conversions) != HAL_OK) {
            debug::error("Failed to start ADC2 read");
            error::handler();
        }
    }
}

constexpr float ADC_VOLTAGE_MULTIPLIER = ADC_VOLTAGE / ADC_RANGE;
constexpr float VSENSE_VMOT_K = ADC_VOLTAGE_MULTIPLIER * VSENSE_VMOT_MULTIPLIER;
constexpr float ISENSE_K = ADC_VOLTAGE_MULTIPLIER * ISENSE_CURRENT_PER_VOLT;
adc::Values adc::read(void) {
    ADC1_conversion_complete = false;
    ADC2_conversion_complete = false;

    // get raw readings
    float VSENSE_VMOT_raw_reading = (float)(*ptr_VMOT_raw);
    float ISENSE_U_PHASE_raw_reading = (float)(*ptr_U_current_raw);
    float ISENSE_V_PHASE_raw_reading = (float)(*ptr_V_current_raw);
    float ISENSE_W_PHASE_raw_reading = (float)(*ptr_W_current_raw);

    Values values;

    values.voltage_VMOT = VSENSE_VMOT_raw_reading * VSENSE_VMOT_K;
    values.current_U = ISENSE_U_PHASE_raw_reading * ISENSE_K - U_current_offset;
#if DIRECTION_REVERSED == true
    values.current_V = ISENSE_W_PHASE_raw_reading * ISENSE_K - W_current_offset;
    values.current_W = ISENSE_V_PHASE_raw_reading * ISENSE_K - V_current_offset;
#elif DIRECTION_REVERSED == false
    values.current_V = ISENSE_V_PHASE_raw_reading * ISENSE_K - V_current_offset;
    values.current_W = ISENSE_W_PHASE_raw_reading * ISENSE_K - W_current_offset;
#endif

    return values;
}

void adc::set_OCP_current(float current) {
    float offset_average =
        (U_current_offset + V_current_offset + W_current_offset) / 3;
    uint32_t threshold =
        ((current + offset_average) / ISENSE_CURRENT_PER_VOLT) /
        ADC_VOLTAGE_MULTIPLIER;
    if (threshold > 4095) {
        threshold = 4095;
    }
    HAL_DAC_SetValue(&DAC_3, DAC_CHANNEL_1, DAC_ALIGN_12B_R, threshold);
    HAL_DAC_SetValue(&DAC_3, DAC_CHANNEL_2, DAC_ALIGN_12B_R, threshold);
    return;
}

void adc::DMA_ADC1_handler(void) {
    HAL_DMA_IRQHandler(&DMA_ADC1);
    return;
}
void adc::DMA_ADC2_handler(void) {
    HAL_DMA_IRQHandler(&DMA_ADC2);
    return;
}
// void adc::ADC1_conversion_complete_callback(void) {
//     ADC1_conversion_complete = true;
//     return;
// }
// void adc::ADC2_conversion_complete_callback(void) {
//     ADC2_conversion_complete = true;
//     return;
// }

void adc_init(void) {
    // ADC12 peripheral clock source
    RCC_PeriphCLKInitTypeDef ADC12_clk_init = {};
    ADC12_clk_init.PeriphClockSelection = RCC_PERIPHCLK_ADC12;
    ADC12_clk_init.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
    if (HAL_RCCEx_PeriphCLKConfig(&ADC12_clk_init) != HAL_OK) {
        debug::error("Failed to configure ADC12 peripheral clock source");
    }

    // calculate the ADC configuration
#define CALCULATE_ADC_CHANNEL_CONFIG(peripheral_name)                       \
    if (peripheral_name##_ADC == ADC1) {                                    \
        ADC1_num_conversions++;                                             \
        peripheral_name##_ADC_data.dma_buf_idx1 = 0;                        \
        peripheral_name##_ADC_data.dma_buf_idx2 = ADC1_num_conversions - 1; \
    } else if (peripheral_name##_ADC == ADC2) {                             \
        ADC2_num_conversions++;                                             \
        peripheral_name##_ADC_data.dma_buf_idx1 = 1;                        \
        peripheral_name##_ADC_data.dma_buf_idx2 = ADC2_num_conversions - 1; \
    }
    CALCULATE_ADC_CHANNEL_CONFIG(ISENSE_U_PHASE);
    CALCULATE_ADC_CHANNEL_CONFIG(ISENSE_V_PHASE);
    CALCULATE_ADC_CHANNEL_CONFIG(ISENSE_W_PHASE);
    CALCULATE_ADC_CHANNEL_CONFIG(VSENSE_VMOT);
#undef CALCULATE_ADC_CHANNEL_CONFIG

    // initialise ADCs
    VSENSE_VMOT_ADC_data.handle = adc_instance_init(VSENSE_VMOT_ADC);
    ISENSE_U_PHASE_ADC_data.handle = adc_instance_init(ISENSE_U_PHASE_ADC);
    ISENSE_V_PHASE_ADC_data.handle = adc_instance_init(ISENSE_V_PHASE_ADC);
    ISENSE_W_PHASE_ADC_data.handle = adc_instance_init(ISENSE_W_PHASE_ADC);

    // configure channels
    ADC_ChannelConfTypeDef channel_config = {};
#define CONFIGURE_ADC_CHANNEL(peripheral_name)                                \
    channel_config.Channel = peripheral_name##_CHANNEL;                       \
    channel_config.Rank =                                                     \
        rank_idx_to_val(peripheral_name##_ADC_data.dma_buf_idx2);             \
    channel_config.SamplingTime = peripheral_name##_SAMPLETIME;               \
    channel_config.SingleDiff = ADC_SINGLE_ENDED;                             \
    channel_config.OffsetNumber = ADC_OFFSET_NONE;                            \
    channel_config.Offset = 0;                                                \
    channel_config.OffsetSign = ADC_OFFSET_SIGN_POSITIVE;                     \
    channel_config.OffsetSaturation = DISABLE;                                \
    if (HAL_ADC_ConfigChannel(peripheral_name##_ADC_data.handle,              \
                              &channel_config) != HAL_OK) {                   \
        debug::error("Failed to configure " #peripheral_name " ADC channel"); \
        error::handler();                                                     \
    }
    CONFIGURE_ADC_CHANNEL(VSENSE_VMOT);
    CONFIGURE_ADC_CHANNEL(ISENSE_U_PHASE);
    CONFIGURE_ADC_CHANNEL(ISENSE_V_PHASE);
    CONFIGURE_ADC_CHANNEL(ISENSE_W_PHASE);
#undef CONFIGURE_ADC_CHANNEL
    return;
}

void gpio_init(void) {
    gpio::init(VSENSE_VMOT_PIN, gpio::Mode::ANALOG, gpio::Pull::NOPULL,
               gpio::Speed::MEDIUM);
    gpio::init(ISENSE_U_PHASE_PIN, gpio::Mode::ANALOG, gpio::Pull::NOPULL,
               gpio::Speed::MEDIUM);
    gpio::init(ISENSE_V_PHASE_PIN, gpio::Mode::ANALOG, gpio::Pull::NOPULL,
               gpio::Speed::MEDIUM);
    gpio::init(ISENSE_W_PHASE_PIN, gpio::Mode::ANALOG, gpio::Pull::NOPULL,
               gpio::Speed::MEDIUM);
    return;
}

ADC_HandleTypeDef* adc_instance_init(ADC_TypeDef* instance) {
    uint8_t instance_idx = adc_instance_to_idx(instance);
    ADC_HandleTypeDef* handle = &(adc_data[instance_idx].handle);
    if (adc_data[instance_idx].inited) {
        return handle;
    }
    __HAL_RCC_ADC12_CLK_ENABLE();

    handle->Instance = instance;
    handle->Init.ClockPrescaler = ADC_CLK_PRESCALER;
    handle->Init.Resolution = ADC_RESOLUTION_12B;
    handle->Init.DataAlign = ADC_DATAALIGN_RIGHT;  // little endian
    handle->Init.GainCompensation = 0;  // no way to calibrate for now
    handle->Init.ScanConvMode = ADC_SCAN_ENABLE;
    handle->Init.EOCSelection = ADC_EOC_SEQ_CONV;
    handle->Init.LowPowerAutoWait = DISABLE;
    handle->Init.ContinuousConvMode = DISABLE;
    if (instance_idx == 0) {
        handle->Init.NbrOfConversion = ADC1_num_conversions;
    } else if (instance_idx == 1) {
        handle->Init.NbrOfConversion = ADC2_num_conversions;
    }
    handle->Init.DiscontinuousConvMode = DISABLE;
    // handle->Init.ExternalTrigConv = ADC_SOFTWARE_START;
    handle->Init.ExternalTrigConv = ADC_EXTERNALTRIG_T1_TRGO2;
    handle->Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
    handle->Init.DMAContinuousRequests = ENABLE;
    handle->Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
    handle->Init.OversamplingMode = DISABLE;

    if (HAL_ADC_Init(handle) != HAL_OK) {
        debug::error("Failed to initialise ADC %u", instance_idx + 1);
        error::handler();
    }

    if (HAL_ADCEx_Calibration_Start(handle, ADC_SINGLE_ENDED) != HAL_OK) {
        debug::error("Failed to calibrate ADC %u", instance_idx + 1);
        error::handler();
    }

    if (instance_idx == 0) {
        __HAL_LINKDMA(handle, DMA_Handle, DMA_ADC1);
    } else if (instance_idx == 1) {
        __HAL_LINKDMA(handle, DMA_Handle, DMA_ADC2);
    }

    adc_data[instance_idx].inited = true;
    return handle;
}

uint8_t adc_instance_to_idx(ADC_TypeDef* instance) {
    if (instance == ADC1) {
        return 0;
    } else if (instance == ADC2) {
        return 1;
    }
    return 255;
}

void dma_init(void) {
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_DMAMUX1_CLK_ENABLE();

    DMA_ADC1.Instance = ADC1_DMA_INSTANCE;
    DMA_ADC1.Init.Request = DMA_REQUEST_ADC1;
    DMA_ADC1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    DMA_ADC1.Init.PeriphInc = DMA_PINC_DISABLE;
    DMA_ADC1.Init.MemInc = DMA_MINC_ENABLE;
    DMA_ADC1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    DMA_ADC1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    DMA_ADC1.Init.Mode = DMA_CIRCULAR;
    DMA_ADC1.Init.Priority = DMA_PRIORITY_HIGH;

    DMA_ADC2.Instance = ADC2_DMA_INSTANCE;
    DMA_ADC2.Init.Request = DMA_REQUEST_ADC2;
    DMA_ADC2.Init.Direction = DMA_PERIPH_TO_MEMORY;
    DMA_ADC2.Init.PeriphInc = DMA_PINC_DISABLE;
    DMA_ADC2.Init.MemInc = DMA_MINC_ENABLE;
    DMA_ADC2.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    DMA_ADC2.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    DMA_ADC2.Init.Mode = DMA_CIRCULAR;
    DMA_ADC2.Init.Priority = DMA_PRIORITY_HIGH;

    if (HAL_DMA_Init(&DMA_ADC1) != HAL_OK) {
        debug::error("Failed to init DMA for ADC1");
        error::handler();
    }
    if (HAL_DMA_Init(&DMA_ADC2) != HAL_OK) {
        debug::error("Failed to init DMA for ADC2");
        error::handler();
    }

    // allow all to preempt (FOC handler runs here)
    if (ADC1_DMA_INSTANCE == DMA1_Channel1) {
        HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 4, 0);
        HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
    }
    if (ADC2_DMA_INSTANCE == DMA1_Channel2) {
        HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 4, 0);
        HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
    }

    return;
}

uint16_t rank_idx_to_val(uint8_t rank_idx) {
    // only 6 ranks are needed right now
    switch (rank_idx) {
        case 0:
            return ADC_REGULAR_RANK_1;
            break;
        case 1:
            return ADC_REGULAR_RANK_2;
            break;
        case 2:
            return ADC_REGULAR_RANK_3;
            break;
        case 3:
            return ADC_REGULAR_RANK_4;
            break;
        case 4:
            return ADC_REGULAR_RANK_5;
            break;
        case 5:
            return ADC_REGULAR_RANK_6;
            break;
    }
    return 65535;
}

void ocp_init(void) {
    __HAL_RCC_DAC3_CLK_ENABLE();
    __HAL_RCC_SYSCFG_CLK_ENABLE();

    __HAL_RCC_DAC3_FORCE_RESET();
    __HAL_RCC_DAC3_RELEASE_RESET();
    DAC_3.Instance = DAC3;
    if (HAL_DAC_Init(&DAC_3) != HAL_OK) {
        debug::error("Failed to init DAC3 for OCP");
        error::handler();
    }
    DAC_ChannelConfTypeDef dac_channel_config = {};
    dac_channel_config.DAC_HighFrequency =
        DAC_HIGH_FREQUENCY_INTERFACE_MODE_DISABLE;
    dac_channel_config.DAC_DMADoubleDataMode = DISABLE;
    dac_channel_config.DAC_SignedFormat = DISABLE;
    dac_channel_config.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
    dac_channel_config.DAC_Trigger = DAC_TRIGGER_NONE;
    dac_channel_config.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
    dac_channel_config.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_ENABLE;
    dac_channel_config.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
    if (HAL_DAC_ConfigChannel(&DAC_3, &dac_channel_config, DAC_CHANNEL_1) !=
        HAL_OK) {
        debug::error("Failed to config DAC3 CH1 for OCP");
        error::handler();
    }
    if (HAL_DAC_ConfigChannel(&DAC_3, &dac_channel_config, DAC_CHANNEL_2) !=
        HAL_OK) {
        debug::error("Failed to config DAC3 CH2 for OCP");
        error::handler();
    }

    // DAC3->CR &= ~DAC_CR_EN1;
    // uint32_t mode_mask = 0x7UL;
    // uint32_t mode_3 = 0x3UL;
    // DAC3->MCR &= (~DAC_MCR_MODE1_Msk);
    // DAC3->MCR |= (4UL << DAC_MCR_MODE1_Pos);
    // DAC->CR |= DAC_CR_EN1;
    // for (volatile int i = 0; i < 1000; i++);
    // DAC3->DHR12R1 = 4095;

    HAL_DAC_Start(&DAC_3, DAC_CHANNEL_1);
    HAL_DAC_Start(&DAC_3, DAC_CHANNEL_2);
    HAL_DAC_SetValue(&DAC_3, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 4095);
    HAL_DAC_SetValue(&DAC_3, DAC_CHANNEL_2, DAC_ALIGN_12B_R, 4095);
    vTaskDelay(pdMS_TO_TICKS(5));

    COMP_InitTypeDef comp_config = {};
    comp_config.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
    comp_config.Hysteresis = COMP_HYSTERESIS_HIGH;
    comp_config.BlankingSrce = COMP_BLANKINGSRC_NONE;
    comp_config.TriggerMode = COMP_TRIGGERMODE_NONE;
    U_PHASE_COMP.Init = comp_config;
    V_PHASE_COMP.Init = comp_config;
    W_PHASE_COMP.Init = comp_config;
    U_PHASE_COMP.Instance = OCP_U_PHASE_COMP;
    U_PHASE_COMP.Init.InputPlus = OCP_U_PHASE_COMP_INPUT;
    U_PHASE_COMP.Init.InputMinus = OCP_U_PHASE_COMP_INVERTING_INPUT;
    if (HAL_COMP_Init(&U_PHASE_COMP) != HAL_OK) {
        debug::error("Failed to initialise U phase OCP comparator");
        error::handler();
    }
    V_PHASE_COMP.Instance = OCP_V_PHASE_COMP;
    V_PHASE_COMP.Init.InputPlus = OCP_V_PHASE_COMP_INPUT;
    V_PHASE_COMP.Init.InputMinus = OCP_V_PHASE_COMP_INVERTING_INPUT;
    if (HAL_COMP_Init(&V_PHASE_COMP) != HAL_OK) {
        debug::error("Failed to initialise V phase OCP comparator");
        error::handler();
    }
    W_PHASE_COMP.Instance = OCP_W_PHASE_COMP;
    W_PHASE_COMP.Init.InputPlus = OCP_W_PHASE_COMP_INPUT;
    W_PHASE_COMP.Init.InputMinus = OCP_W_PHASE_COMP_INVERTING_INPUT;
    if (HAL_COMP_Init(&W_PHASE_COMP) != HAL_OK) {
        debug::error("Failed to initialise W phase OCP comparator");
        error::handler();
    }

    gpio::init(OCP_U_PHASE_PIN, gpio::Mode::ANALOG, gpio::Pull::NOPULL,
               gpio::Speed::HIGH);
    gpio::init(OCP_V_PHASE_PIN, gpio::Mode::ANALOG, gpio::Pull::NOPULL,
               gpio::Speed::HIGH);
    gpio::init(OCP_W_PHASE_PIN, gpio::Mode::ANALOG, gpio::Pull::NOPULL,
               gpio::Speed::HIGH);

    HAL_COMP_Start(&U_PHASE_COMP);
    HAL_COMP_Start(&V_PHASE_COMP);
    HAL_COMP_Start(&W_PHASE_COMP);

    return;
}
