#include "thermistor.hpp"

#include <stm32h7xx_hal.h>

#include "config.hpp"
#include "debug.hpp"

namespace constants {
const float R_0 = 1000.0f;
const float A = 3.9083e-3f;
const float B = -5.775e-7f;
const float A_2 = A * A;
}  // namespace constants

struct HardwareConfig {
    gpio::PinConfig pin = {};
    ADC_TypeDef* ADC_instance = NULL;
    uint32_t ADC_channel = 0;
};
struct State {
    ADC_HandleTypeDef* handle = NULL;
    ADC_ChannelConfTypeDef conf = {};
    uint32_t resolution = 0;
};

enum class InitInstanceStatus : uint8_t {
    OK,
    ADC_INIT_FAILED,
    ADC_CALIBRATION_FAILED,
};

static const HardwareConfig hwconfigs[8] = {
    {THERM1_PIN, THERM1_ADC, THERM1_CHANNEL},
    {THERM2_PIN, THERM2_ADC, THERM2_CHANNEL},
    {THERM3_PIN, THERM3_ADC, THERM3_CHANNEL},
    {THERM4_PIN, THERM4_ADC, THERM4_CHANNEL},
    {THERM5_PIN, THERM5_ADC, THERM5_CHANNEL},
    {THERM6_PIN, THERM6_ADC, THERM6_CHANNEL},
    {THERM7_PIN, THERM7_ADC, THERM7_CHANNEL},
    {THERM8_PIN, THERM8_ADC, THERM8_CHANNEL},
};
static State state[8] = {};
static ADC_HandleTypeDef handles[3] = {};
static const uint32_t resolutions[3] = {1 << 16, 1 << 16, 1 << 16};

float calc_temp(float resistance);
InitInstanceStatus init_instance(ADC_TypeDef* instance);
void init_channel(uint8_t idx);
uint8_t adc_instance_to_idx(ADC_TypeDef* instance);

thermistor::InitStatus thermistor::init(void) {
    __HAL_RCC_ADC12_CLK_ENABLE();
    __HAL_RCC_ADC3_CLK_ENABLE();

    InitInstanceStatus instance_status = init_instance(ADC1);
    switch (instance_status) {
        case InitInstanceStatus::OK:
            break;
        case InitInstanceStatus::ADC_INIT_FAILED:
            return InitStatus::ADC1_INIT_FAILED;
            break;
        case InitInstanceStatus::ADC_CALIBRATION_FAILED:
            return InitStatus::ADC1_CALIBRATION_FAILED;
            break;
    }
    instance_status = init_instance(ADC2);
    switch (instance_status) {
        case InitInstanceStatus::OK:
            break;
        case InitInstanceStatus::ADC_INIT_FAILED:
            return InitStatus::ADC2_INIT_FAILED;
            break;
        case InitInstanceStatus::ADC_CALIBRATION_FAILED:
            return InitStatus::ADC2_CALIBRATION_FAILED;
            break;
    }
    instance_status = init_instance(ADC3);
    switch (instance_status) {
        case InitInstanceStatus::OK:
            break;
        case InitInstanceStatus::ADC_INIT_FAILED:
            return InitStatus::ADC3_INIT_FAILED;
            break;
        case InitInstanceStatus::ADC_CALIBRATION_FAILED:
            return InitStatus::ADC3_CALIBRATION_FAILED;
            break;
    }

    for (int i = 0; i < 8; i++) {
        init_channel(i);
    }

    return InitStatus::OK;
}

float thermistor::read(Port port) {
    uint8_t idx = (uint8_t)port;
    if (HAL_ADC_ConfigChannel(state[idx].handle, &state[idx].conf) != HAL_OK) {
        return NAN;
    }
    HAL_ADC_Start(state[idx].handle);
    if (HAL_ADC_PollForConversion(state[idx].handle, 10) != HAL_OK) {
        return NAN;
    }

    uint16_t val = HAL_ADC_GetValue(state[idx].handle);
    debug::log("val: %u", val);
    float ratio = ((float)val / (float)state[idx].resolution);
    // V = ADC_VREF * R / (R + Rfixed)
    // V * R + V * Rfixed = ADC_VREF * R
    // R * (ADC_VREF - V) = V * Rfixed
    // R = (V * Rfixed) / (ADC_VREF - V)
    float resistance = (ratio * THERMISTOR_R) / (1.0f - ratio);
    float temp = calc_temp(resistance);
    return temp;
}

void init_channel(uint8_t idx) {
    uint8_t instance_idx = adc_instance_to_idx(hwconfigs[idx].ADC_instance);
    state[idx].handle = &handles[instance_idx];
    state[idx].resolution = resolutions[instance_idx];

    ADC_ChannelConfTypeDef* conf = &state[idx].conf;
    conf->Channel = hwconfigs[idx].ADC_channel;
    conf->Rank = ADC_REGULAR_RANK_1;
    conf->SamplingTime = THERM_SAMPLETIME;
    conf->SingleDiff = ADC_SINGLE_ENDED;
    conf->OffsetNumber = ADC_OFFSET_NONE;
    conf->Offset = 0;

    gpio::init(hwconfigs[idx].pin, gpio::Mode::ANALOG, gpio::Pull::NOPULL,
               gpio::Speed::LOW);
    return;
}

InitInstanceStatus init_instance(ADC_TypeDef* instance) {
    uint8_t instance_idx = adc_instance_to_idx(instance);
    ADC_HandleTypeDef* handle = &handles[instance_idx];
    handle->Instance = instance;
    handle->Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
    if (instance == ADC3) {
        handle->Init.Resolution = ADC_RESOLUTION_12B;
        handle->Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
        handle->Init.Oversampling.Ratio = 0b10100;  // 64x oversampling, HAL bug
        handle->Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_2;
    } else {
        handle->Init.Resolution = ADC_RESOLUTION_16B;
        handle->Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
        handle->Init.Oversampling.Ratio = 64;
        handle->Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_6;
    }
    handle->Init.ScanConvMode = ADC_SCAN_DISABLE;
    handle->Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    handle->Init.LowPowerAutoWait = DISABLE;
    handle->Init.ContinuousConvMode = DISABLE;
    handle->Init.NbrOfConversion = 1;
    handle->Init.DiscontinuousConvMode = DISABLE;
    handle->Init.ExternalTrigConv = ADC_SOFTWARE_START;
    handle->Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    handle->Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
    handle->Init.Overrun = ADC_OVR_DATA_PRESERVED;
    handle->Init.OversamplingMode = ENABLE;
    handle->Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
    handle->Init.Oversampling.OversamplingStopReset =
        ADC_REGOVERSAMPLING_CONTINUED_MODE;

    if (HAL_ADC_Init(handle) != HAL_OK) {
        return InitInstanceStatus::ADC_INIT_FAILED;
    }
    if (HAL_ADCEx_Calibration_Start(handle, ADC_CALIB_OFFSET,
                                    ADC_SINGLE_ENDED) != HAL_OK) {
        return InitInstanceStatus::ADC_CALIBRATION_FAILED;
    }
    return InitInstanceStatus::OK;
}

float calc_temp(float resistance) {
    // R(t) = R_0 * (1 + A * T + B * T^2)
    // B * R_0 * T^2 + A * R_0 * T + (R_0 - R(t)) = 0
    // T = (-(A * R_0) + sqrt((A * R_0) ^ 2 - 4 * (B * R_0) * (R_0 - R(t)))) /
    // (2 * B * R_0)
    // T = (-A + sqrt(A^2 - 4 * B * (1 - R(t) / R_0))) / (2 * B)
    float temp =
        (-constants::A +
         sqrtf(constants::A_2 -
               4 * constants::B * (1.0f - (resistance / constants::R_0)))) /
        (2.0f * constants::B);
    return temp;
}

uint8_t adc_instance_to_idx(ADC_TypeDef* instance) {
    if (instance == ADC1) {
        return 0;
    } else if (instance == ADC2) {
        return 1;
    } else if (instance == ADC3) {
        return 2;
    }
    return 255;
}
