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

bool SCL3300_Init(SCL3300_HandleTypeDef *dev, SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin, uint8_t mode)
{
    dev->hspi = hspi;
    dev->cs_port = cs_port;
    dev->cs_pin = cs_pin;
    dev->mode = (mode >= 1 && mode <= 4) ? mode : 4;

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
    SCL3300_Transfer32(dev, SCL3300_CMD_SWITCH_BANK0);
    SCL3300_Transfer32(dev, SCL3300_CMD_READ_ACC_X);
    
    uint32_t r_accy = SCL3300_Transfer32(dev, SCL3300_CMD_READ_ACC_Y);
    dev->raw_acc_x  = (int16_t)((r_accy >> 8) & 0xFFFF);

    uint32_t r_accz = SCL3300_Transfer32(dev, SCL3300_CMD_READ_ACC_Z);
    dev->raw_acc_y  = (int16_t)((r_accz >> 8) & 0xFFFF);

    uint32_t r_sto  = SCL3300_Transfer32(dev, SCL3300_CMD_READ_STO);
    dev->raw_acc_z  = (int16_t)((r_sto >> 8) & 0xFFFF);

    uint32_t r_temp = SCL3300_Transfer32(dev, SCL3300_CMD_READ_TEMP);

    uint32_t r_angx = SCL3300_Transfer32(dev, SCL3300_CMD_READ_ANG_X);
    dev->raw_temp   = (int16_t)((r_angx >> 8) & 0xFFFF);

    uint32_t r_angy = SCL3300_Transfer32(dev, SCL3300_CMD_READ_ANG_Y);
    dev->raw_ang_x  = (int16_t)((r_angy >> 8) & 0xFFFF);

    uint32_t r_angz = SCL3300_Transfer32(dev, SCL3300_CMD_READ_ANG_Z);
    dev->raw_ang_y  = (int16_t)((r_angz >> 8) & 0xFFFF);

    uint32_t r_stat = SCL3300_Transfer32(dev, SCL3300_CMD_READ_STATUS);
    dev->raw_ang_z  = (int16_t)((r_stat >> 8) & 0xFFFF);

    uint32_t r_who  = SCL3300_Transfer32(dev, SCL3300_CMD_READ_WHOAMI);
    dev->status_sum = (uint16_t)((r_who >> 8) & 0xFFFF);

    // Convert raw values to physical units
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

    return true;
}
