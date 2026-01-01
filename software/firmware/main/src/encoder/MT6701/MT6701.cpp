#include "config.hpp"
#if ENCODER_TYPE == MT6701

#include <stm32g4xx_hal.h>

#include "MT6701.hpp"
#include "debug.hpp"
#include "error.hpp"
#include "spi.hpp"

SPI_TypeDef* SPI_INSTANCE = MT6701_SPI_INSTANCE;

// disable warnings
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#pragma GCC diagnostic ignored "-Wc++20-extensions"
spi::Config MT6701_SPI = {
    .instance = SPI_INSTANCE,
    .mode = spi::Mode::master,
    .direction = spi::Direction::rx_only,
    .polarity = spi::Polarity::low,
    .phase = spi::Phase::second,
    .baud_rate = spi::BaudRate::_10Mb,
    .first_bit = spi::FirstBit::MSB,
    .SCLK = MT6701_SCLK,
    .MISO = MT6701_MISO,
    .NCS = MT6701_NCS,
    .use_dma = true,
    .rx_dma_channel = MT6701_SPI_RX_DMA_INSTANCE,
    .rx_dma_request = MT6701_SPI_RX_DMA_REQUEST,
};
#pragma GCC diagnostic pop

int8_t direction = 1;

alignas(4) volatile uint8_t buf[3] = {};

int64_t rollover_count = 0;
uint16_t last_reading = 0;

enum class MagneticFieldStatus : uint8_t {
    normal = 0,
    too_strong = 1,
    too_weak = 2,
};

struct EncoderData {
    uint16_t angle = 0;
    MagneticFieldStatus magnetic_field_status = MagneticFieldStatus::normal;
    bool push_button_detected = false;
    bool track_normal = false;
    bool crc_passed = false;
};

EncoderData read(void);
EncoderData read_DMA(void);

void encoder::init(void) {
    spi::init(MT6701_SPI);
    last_reading = read().angle;
    rollover_count = 0;
}

int64_t encoder::get_count(void) {
    EncoderData reading;
    while (!reading.crc_passed || !reading.track_normal ||
           (reading.magnetic_field_status != MagneticFieldStatus::normal)) {
        reading = read();
    }
    int delta = (int)reading.angle - (int)last_reading;
    if (delta > (int)ENCODER_RESOLUTION / 2) {
        // backwards rollover
        rollover_count -= 1;
    } else if (delta < -(int)ENCODER_RESOLUTION / 2) {
        // forwards rollover
        rollover_count += 1;
    }
    last_reading = reading.angle;
    int64_t total_count = rollover_count * ENCODER_RESOLUTION + reading.angle;
    return ENCODER_DIRECTION * total_count;
}

void encoder::DMA_RX_handler(void) {
    HAL_DMA_IRQHandler(&MT6701_SPI.rx_dma);
    return;
}

void encoder::SPI_handler(void) {
    HAL_SPI_IRQHandler(&MT6701_SPI.handle);
    return;
}

void encoder::rx_complete_callback(void) {
    spi::end_dma_receive(MT6701_SPI);
    return;
}

void encoder::start_DMA_read(void) {
    spi::start_dma_receive(MT6701_SPI, (uint8_t*)buf, 3);
    return;
}

int64_t encoder::get_DMA_reading(void) {
    EncoderData reading;
    reading = read_DMA();
    // if failed, do blocking read until succeed
    while (!reading.crc_passed || !reading.track_normal ||
           (reading.magnetic_field_status != MagneticFieldStatus::normal)) {
        reading = read();
    }
    int delta = (int)reading.angle - (int)last_reading;
    if (delta > (int)ENCODER_RESOLUTION / 2) {
        // backwards rollover
        rollover_count -= 1;
    } else if (delta < -(int)ENCODER_RESOLUTION / 2) {
        // forwards rollover
        rollover_count += 1;
    }
    last_reading = reading.angle;
    int64_t total_count = rollover_count * ENCODER_RESOLUTION + reading.angle;
    return ENCODER_DIRECTION * total_count;
}

const uint8_t CRC6_table[64] = {
    0x00, 0x03, 0x06, 0x05, 0x0C, 0x0F, 0x0A, 0x09, 0x18, 0x1B, 0x1E,
    0x1D, 0x14, 0x17, 0x12, 0x11, 0x30, 0x33, 0x36, 0x35, 0x3C, 0x3F,
    0x3A, 0x39, 0x28, 0x2B, 0x2E, 0x2D, 0x24, 0x27, 0x22, 0x21, 0x23,
    0x20, 0x25, 0x26, 0x2F, 0x2C, 0x29, 0x2A, 0x3B, 0x38, 0x3D, 0x3E,
    0x37, 0x34, 0x31, 0x32, 0x13, 0x10, 0x15, 0x16, 0x1F, 0x1C, 0x19,
    0x1A, 0x0B, 0x08, 0x0D, 0x0E, 0x07, 0x04, 0x01, 0x02};

EncoderData read(void) {
    spi::receive(MT6701_SPI, (uint8_t*)buf, 3);
    uint8_t crc = buf[2] & 0b00111111;

    // check CRC
    uint32_t crc_data =
        ((uint32_t)buf[0] << 10) | ((uint32_t)buf[1] << 2) | (buf[2] >> 6);
    // CRC6 verification
    uint8_t crc_calculated = 0;
    uint8_t crc_index = 0;
    crc_index = (uint8_t)((crc_data >> 12) & 0b00111111);
    crc_calculated = (uint8_t)((crc_data >> 6) & 0b00111111);
    crc_index = crc_calculated ^ CRC6_table[crc_index];
    crc_calculated = (uint8_t)(crc_data & 0b00111111);
    crc_index = crc_calculated ^ CRC6_table[crc_index];
    crc_calculated = CRC6_table[crc_index];
    crc_calculated &= 0b00111111;
    if (crc_calculated != crc) {
        debug::error("Encoder CRC failed. CRC provided: %u, CRC calculated: %u",
                     crc, crc_calculated);
        EncoderData data;
        data.crc_passed = false;
        return data;
    }

    EncoderData data;
    data.angle = ((uint16_t)(buf[0] << 6) | (buf[1] >> 2));
    data.magnetic_field_status = (MagneticFieldStatus)(buf[2] & 0b11000000);
    data.push_button_detected = buf[1] & 0b00000001;
    data.track_normal = !(bool)(buf[1] & 0b00000010);
    data.crc_passed = true;

    return data;
}

EncoderData read_DMA(void) {
    // block until DMA receive done
    while (!spi::is_dma_receive_done(MT6701_SPI)) {
    }
    uint8_t crc = buf[2] & 0b00111111;

    // check CRC
    uint32_t crc_data =
        ((uint32_t)buf[0] << 10) | ((uint32_t)buf[1] << 2) | (buf[2] >> 6);
    // CRC6 verification
    uint8_t crc_calculated = 0;
    uint8_t crc_index = 0;
    crc_index = (uint8_t)((crc_data >> 12) & 0b00111111);
    crc_calculated = (uint8_t)((crc_data >> 6) & 0b00111111);
    crc_index = crc_calculated ^ CRC6_table[crc_index];
    crc_calculated = (uint8_t)(crc_data & 0b00111111);
    crc_index = crc_calculated ^ CRC6_table[crc_index];
    crc_calculated = CRC6_table[crc_index];
    crc_calculated &= 0b00111111;
    if (crc_calculated != crc) {
        debug::error("Encoder CRC failed. CRC provided: %u, CRC calculated: %u",
                     crc, crc_calculated);
        EncoderData data;
        data.crc_passed = false;
        return data;
    }

    EncoderData data;
    data.angle = ((uint16_t)(buf[0] << 6) | (buf[1] >> 2));
    data.magnetic_field_status = (MagneticFieldStatus)(buf[2] & 0b11000000);
    data.push_button_detected = buf[1] & 0b00000001;
    data.track_normal = !(bool)(buf[1] & 0b00000010);
    data.crc_passed = true;

    return data;
}

#endif
