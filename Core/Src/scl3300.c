#include "scl3300.h"
#include <math.h>

static uint8_t SCL3300_CRC8(uint8_t bit_val, uint8_t crc)
{
    uint8_t temp = (uint8_t)(crc & 0x80);
    if (bit_val == 0x01) {
        temp ^= 0x80;
    }
    crc <<= 1;
    if (temp > 0) {
        crc ^= 0x1D;
    }
    return crc;
}

uint8_t SCL3300_CalculateCRC(uint32_t data)
{
    uint8_t crc = 0xFF;
    for (int i = 31; i > 7; i--) {
        uint8_t bit_val = (uint8_t)((data >> i) & 0x01);
        crc = SCL3300_CRC8(bit_val, crc);
    }
    return (uint8_t)~crc;
}

uint32_t SCL3300_Transfer32(SCL3300_HandleTypeDef *dev, uint32_t cmd)
{
    uint8_t tx[4];
    uint8_t rx[4] = {0};

    tx[0] = (uint8_t)((cmd >> 24) & 0xFF);
    tx[1] = (uint8_t)((cmd >> 16) & 0xFF);
    tx[2] = (uint8_t)((cmd >> 8)  & 0xFF);
    tx[3] = (uint8_t)(cmd & 0xFF);

    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(dev->hspi, tx, rx, 4, 100);
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);

    // Microsecond delay between SPI transfers (minimum 10us required by SCL3300)
    for (volatile int i = 0; i < 50; i++) { __NOP(); }

    return ((uint32_t)rx[0] << 24) | ((uint32_t)rx[1] << 16) |
           ((uint32_t)rx[2] << 8)  | (uint32_t)rx[3];
}

static bool SCL3300_ProcessFrame(SCL3300_HandleTypeDef *dev, uint32_t frame, int16_t *out_data)
{
    // Murata SCL3300 RS (Return Status) bits are located at bits [25:24]
    uint8_t rs = (uint8_t)((frame >> 24) & 0x03);
    uint8_t crc_rx = (uint8_t)(frame & 0xFF);
    uint8_t crc_calc = SCL3300_CalculateCRC(frame);

    dev->last_rs = rs;

    if (crc_rx != crc_calc) {
        dev->crc_error = true;
        return false;
    }

    if (rs != 0x01) { // RS = 01 (Normal Operation OK)
        dev->status_error = true;
        return false;
    }

    if (out_data) {
        *out_data = (int16_t)((frame >> 8) & 0xFFFF);
    }
    return true;
}

bool SCL3300_Init(SCL3300_HandleTypeDef *dev, SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin, uint8_t mode)
{
    dev->hspi = hspi;
    dev->cs_port = cs_port;
    dev->cs_pin = cs_pin;
    dev->mode = (mode >= 1 && mode <= 4) ? mode : 4;
    dev->crc_error = false;
    dev->status_error = false;

    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
    HAL_Delay(10);

    // Step 1: Reset & select bank 0
    SCL3300_Transfer32(dev, SCL3300_CMD_SWITCH_BANK0);
    SCL3300_Transfer32(dev, SCL3300_CMD_SW_RESET);
    HAL_Delay(10);

    // Step 2: Select mode
    uint32_t mode_cmd = SCL3300_CMD_MODE_4;
    if (dev->mode == 1) mode_cmd = SCL3300_CMD_MODE_1;
    else if (dev->mode == 2) mode_cmd = SCL3300_CMD_MODE_2;
    else if (dev->mode == 3) mode_cmd = SCL3300_CMD_MODE_3;

    SCL3300_Transfer32(dev, mode_cmd);
    SCL3300_Transfer32(dev, SCL3300_CMD_ENABLE_ANG);
    HAL_Delay(100); // Allow internal filter stabilization

    // Step 3: Clear status summary flags
    SCL3300_Transfer32(dev, SCL3300_CMD_READ_STATUS);
    SCL3300_Transfer32(dev, SCL3300_CMD_READ_STATUS);
    SCL3300_Transfer32(dev, SCL3300_CMD_READ_STATUS);

    // Step 4: Verify WHOAMI (Expect 0xC1 in data byte)
    SCL3300_Transfer32(dev, SCL3300_CMD_READ_WHOAMI);
    uint32_t resp = SCL3300_Transfer32(dev, SCL3300_CMD_READ_WHOAMI);
    uint8_t who = (uint8_t)((resp >> 8) & 0xFF);

    dev->whoami = who;
    return (who == 0xC1);
}

bool SCL3300_ReadData(SCL3300_HandleTypeDef *dev)
{
    dev->crc_error = false;
    dev->status_error = false;
    bool ok = true;
    int16_t val;

    SCL3300_Transfer32(dev, SCL3300_CMD_SWITCH_BANK0);
    SCL3300_Transfer32(dev, SCL3300_CMD_READ_ACC_X);

    // Frame 1: response to RdAccX -> AccX
    uint32_t f1 = SCL3300_Transfer32(dev, SCL3300_CMD_READ_ACC_Y);
    if (SCL3300_ProcessFrame(dev, f1, &val)) dev->raw_acc_x = val; else ok = false;

    // Frame 2: response to RdAccY -> AccY
    uint32_t f2 = SCL3300_Transfer32(dev, SCL3300_CMD_READ_ACC_Z);
    if (SCL3300_ProcessFrame(dev, f2, &val)) dev->raw_acc_y = val; else ok = false;

    // Frame 3: response to RdAccZ -> AccZ
    uint32_t f3 = SCL3300_Transfer32(dev, SCL3300_CMD_READ_TEMP);
    if (SCL3300_ProcessFrame(dev, f3, &val)) dev->raw_acc_z = val; else ok = false;

    // Frame 4: response to RdTemp -> Temp
    uint32_t f4 = SCL3300_Transfer32(dev, SCL3300_CMD_READ_ANG_X);
    if (SCL3300_ProcessFrame(dev, f4, &val)) dev->raw_temp = val; else ok = false;

    // Frame 5: response to RdAngX -> AngX
    uint32_t f5 = SCL3300_Transfer32(dev, SCL3300_CMD_READ_ANG_Y);
    if (SCL3300_ProcessFrame(dev, f5, &val)) dev->raw_ang_x = val; else ok = false;

    // Frame 6: response to RdAngY -> AngY
    uint32_t f6 = SCL3300_Transfer32(dev, SCL3300_CMD_READ_ANG_Z);
    if (SCL3300_ProcessFrame(dev, f6, &val)) dev->raw_ang_y = val; else ok = false;

    // Frame 7: response to RdAngZ -> AngZ
    uint32_t f7 = SCL3300_Transfer32(dev, SCL3300_CMD_READ_WHOAMI);
    if (SCL3300_ProcessFrame(dev, f7, &val)) dev->raw_ang_z = val; else ok = false;

    // Convert raw values to physical units if frame sequence was valid
    if (ok) {
        float acc_div = 12000.0f;
        if (dev->mode == 1) acc_div = 6000.0f;
        else if (dev->mode == 2) acc_div = 3000.0f;

        dev->acc_x_g = (float)dev->raw_acc_x / acc_div;
        dev->acc_y_g = (float)dev->raw_acc_y / acc_div;
        dev->acc_z_g = (float)dev->raw_acc_z / acc_div;

        dev->angle_x_deg = ((float)dev->raw_ang_x / 16384.0f) * 90.0f;
        dev->angle_y_deg = ((float)dev->raw_ang_y / 16384.0f) * 90.0f;
        dev->angle_z_deg = ((float)dev->raw_ang_z / 16384.0f) * 90.0f;

        dev->temp_c = -273.0f + ((float)dev->raw_temp / 18.9f);
    } else if (dev->status_error || dev->last_rs == 3) {
        // --- Auto-Recovery for SCL3300 Latched Status Error (RS = 3) ---
        // Reading Status Summary twice clears the latched status error flags (e.g. from rapid movement/bump)
        SCL3300_Transfer32(dev, SCL3300_CMD_SWITCH_BANK0);
        SCL3300_Transfer32(dev, SCL3300_CMD_READ_STATUS);
        SCL3300_Transfer32(dev, SCL3300_CMD_READ_STATUS);

        // Verify if status cleared
        uint32_t test_frame = SCL3300_Transfer32(dev, SCL3300_CMD_READ_WHOAMI);
        uint8_t test_rs = (uint8_t)((test_frame >> 24) & 0x03);
        if (test_rs != 0x01) {
            // Re-initialize sensor if error condition persists
            SCL3300_Init(dev, dev->hspi, dev->cs_port, dev->cs_pin, dev->mode);
        }
    }

    return ok;
}
