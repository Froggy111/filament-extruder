#include "mpu.hpp"

#include <stm32h7xx_hal.h>

void mpu::init(void) {
    MPU_Region_InitTypeDef mpu_config = {};

    HAL_MPU_Disable();

    // default configurations (memory protection)
    mpu_config.Enable = MPU_REGION_ENABLE;
    mpu_config.Number = MPU_REGION_NUMBER0;
    mpu_config.BaseAddress = 0x0;
    mpu_config.Size = MPU_REGION_SIZE_4GB;
    mpu_config.SubRegionDisable = 0x87;
    mpu_config.TypeExtField = MPU_TEX_LEVEL0;
    mpu_config.AccessPermission = MPU_REGION_NO_ACCESS;
    mpu_config.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
    mpu_config.IsShareable = MPU_ACCESS_SHAREABLE;
    mpu_config.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
    mpu_config.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
    HAL_MPU_ConfigRegion(&mpu_config);

    // SRAM4 (32KB) non cacheable for DMA
    mpu_config.Enable = MPU_REGION_ENABLE;
    mpu_config.Number =
        MPU_REGION_NUMBER1;               // Region 1 has priority over Region 0
    mpu_config.BaseAddress = 0x38000000;  // Start of SRAM4
    mpu_config.Size = MPU_REGION_SIZE_32KB;  // H723 has 32KB SRAM4
    mpu_config.SubRegionDisable = 0x00;      // Enable all sub-regions
    mpu_config.TypeExtField = MPU_TEX_LEVEL0;
    mpu_config.AccessPermission = MPU_REGION_FULL_ACCESS;
    mpu_config.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
    mpu_config.IsShareable = MPU_ACCESS_SHAREABLE;
    mpu_config.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
    mpu_config.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
    HAL_MPU_ConfigRegion(&mpu_config);

    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}
