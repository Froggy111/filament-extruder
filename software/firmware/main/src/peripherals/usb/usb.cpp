#include "usb.hpp"

#include <FreeRTOS.h>
#include <semphr.h>
#include <stdio.h>
#include <stm32h7xx_hal.h>
#include <task.h>

#include <climits>

#include "config.hpp"
#include "error.hpp"
#include "usb_device.h"
#include "usbd_cdc_if.h"

void usb_flush_task(void *args);

void usb::init(void) {
    if (MX_USB_Device_Init() != USBD_OK) {
        error::handler();
    }
}

void usb::write(uint8_t *buf, uint16_t length) {
    uint16_t written_length = MIN(length, USB_TX_BUFFER_SIZE - 1);
    CDC_Transmit_HS(buf, length);
}
