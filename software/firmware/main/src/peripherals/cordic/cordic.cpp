#include "cordic.hpp"

#include <limits.h>
#include <stm32g4xx_hal.h>

#include "config.hpp"
#include "debug.hpp"
#include "error.hpp"

static CORDIC_HandleTypeDef handle;
volatile static bool calculation_complete = false;

void cordic::init(void) {
    __HAL_RCC_CORDIC_CLK_ENABLE();
    CORDIC_ConfigTypeDef config = {};

    handle.Instance = CORDIC;
    if (HAL_CORDIC_Init(&handle) != HAL_OK) {
        debug::fatal("Cordic initialisation failed");
        error::handler();
    }
    config.Function = CORDIC_FUNCTION_SINE;
    config.Precision = CORDIC_PRECISION_6CYCLES;
    config.Scale = CORDIC_SCALE_0;
    config.NbWrite = CORDIC_NBWRITE_1;
    config.NbRead = CORDIC_NBREAD_1;
    config.InSize = CORDIC_INSIZE_16BITS;
    config.OutSize = CORDIC_OUTSIZE_16BITS;

    if (HAL_CORDIC_Configure(&handle, &config) != HAL_OK) {
        debug::fatal("Cordic configuration failed");
        error::handler();
    }
}

constexpr float PI_TO_Q31 = 32768.0f / M_PI;
constexpr float Q31_TO_FLOAT = 1.0f / 32768.0f;
cordic::SinCosVal cordic::sincos(float theta) {
    int16_t q15_theta = (int16_t)(theta * PI_TO_Q31);
    int32_t input = (0x7FFF << 16) | (q15_theta & 0xFFFF);
    CORDIC->WDATA = input;
    while (HAL_IS_BIT_CLR(CORDIC->CSR, CORDIC_CSR_RRDY));
    volatile int32_t output = (int32_t)CORDIC->RDATA;
    int16_t q15_sin = (int16_t)(output & 0xFFFF);
    int16_t q15_cos = (int16_t)(output >> 16);
    float sin = (float)q15_sin * Q31_TO_FLOAT;
    float cos = (float)q15_cos * Q31_TO_FLOAT;
    return SinCosVal{sin, cos};
}
