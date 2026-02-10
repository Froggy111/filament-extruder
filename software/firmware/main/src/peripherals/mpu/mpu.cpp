#include "mpu.hpp"

#include <stm32h7xx_hal.h>

void mpu::init(void) {
    MPU_Region_InitTypeDef mpu_config = {};

    HAL_MPU_Disable();

    /* Region 0: Background (No Access) */
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

    /* Region 1: FLASH (512KB) */
    mpu_config.Number = MPU_REGION_NUMBER1;
    mpu_config.BaseAddress = 0x08000000;
    mpu_config.Size = MPU_REGION_SIZE_512KB;
    mpu_config.SubRegionDisable = 0x00;
    mpu_config.TypeExtField = MPU_TEX_LEVEL1;
    mpu_config.AccessPermission = MPU_REGION_FULL_ACCESS;
    mpu_config.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
    mpu_config.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
    mpu_config.IsCacheable = MPU_ACCESS_CACHEABLE;
    mpu_config.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
    HAL_MPU_ConfigRegion(&mpu_config);

    /* Region 2: DTCM (128KB) - Normal, Cached */
    mpu_config.Number = MPU_REGION_NUMBER2;
    mpu_config.BaseAddress = 0x20000000;
    mpu_config.Size = MPU_REGION_SIZE_128KB;
    mpu_config.AccessPermission = MPU_REGION_FULL_ACCESS;
    mpu_config.IsCacheable = MPU_ACCESS_CACHEABLE;
    mpu_config.IsBufferable = MPU_ACCESS_BUFFERABLE;
    HAL_MPU_ConfigRegion(&mpu_config);

    /*
     * Region 3: RAM_D1 (320KB -> MPU uses 512KB size)
     * SETTING: CACHEABLE (Write-Back)
     * Why: Now contains .bss (Heaps, Variables).
     */
    mpu_config.Number = MPU_REGION_NUMBER3;
    mpu_config.BaseAddress = 0x24000000;
    mpu_config.Size = MPU_REGION_SIZE_512KB;
    mpu_config.AccessPermission = MPU_REGION_FULL_ACCESS;
    mpu_config.IsShareable = MPU_ACCESS_SHAREABLE;
    mpu_config.IsCacheable = MPU_ACCESS_CACHEABLE;
    mpu_config.IsBufferable = MPU_ACCESS_BUFFERABLE;
    HAL_MPU_ConfigRegion(&mpu_config);

    /*
     * Region 4: RAM_D2 (32KB)
     * SETTING: NOT CACHEABLE
     * Why: Contains Ethernet Rx/Tx Descriptors (.RxDescripSection).
     * DMA updates these, and CPU must see changes immediately.
     */
    mpu_config.Number = MPU_REGION_NUMBER4;
    mpu_config.BaseAddress = 0x30000000;
    mpu_config.Size = MPU_REGION_SIZE_32KB;
    mpu_config.AccessPermission = MPU_REGION_FULL_ACCESS;
    mpu_config.IsShareable = MPU_ACCESS_SHAREABLE;
    mpu_config.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
    mpu_config.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
    HAL_MPU_ConfigRegion(&mpu_config);

    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
    SCB_EnableICache();
    SCB_EnableDCache();
}
