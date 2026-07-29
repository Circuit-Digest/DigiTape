#ifndef SCL3300_H
#define SCL3300_H

#include "stm32g4xx_hal.h"
#include <stdbool.h>

#define SCL3300_CMD_READ_ACC_X    0x040000F7
#define SCL3300_CMD_READ_ACC_Y    0x080000FD
#define SCL3300_CMD_READ_ACC_Z    0x0C0000FB
#define SCL3300_CMD_READ_STO      0x100000E9
#define SCL3300_CMD_READ_TEMP     0x140000EF
#define SCL3300_CMD_READ_ANG_X    0x240000C7
#define SCL3300_CMD_READ_ANG_Y    0x280000CD
#define SCL3300_CMD_READ_ANG_Z    0x2C0000CB
#define SCL3300_CMD_READ_STATUS   0x180000E5
#define SCL3300_CMD_READ_WHOAMI   0x40000091
#define SCL3300_CMD_ENABLE_ANG    0xB0001F6F
#define SCL3300_CMD_MODE_1        0xB400001F
#define SCL3300_CMD_MODE_2        0xB4000102
#define SCL3300_CMD_MODE_3        0xB4000225
#define SCL3300_CMD_MODE_4        0xB4000338
#define SCL3300_CMD_SW_RESET      0xB4002098
#define SCL3300_CMD_SWITCH_BANK0  0xFC000073

typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef      *cs_port;
    uint16_t           cs_pin;
    uint8_t            mode;
    
    // Raw register values
    int16_t            raw_acc_x;
    int16_t            raw_acc_y;
    int16_t            raw_acc_z;
    int16_t            raw_ang_x;
    int16_t            raw_ang_y;
    int16_t            raw_ang_z;
    int16_t            raw_temp;
    uint16_t           status_sum;
    uint8_t            whoami;

    // Return Status & Health Flags
    uint8_t            last_rs;     // Bits [29:28] of response: 1 = Normal OK, 0 = Startup, 3 = Fault
    bool               crc_error;   // True if any CRC check failed
    bool               status_error;// True if RS != 1 (Normal)

    // Converted physical values
    float              acc_x_g;
    float              acc_y_g;
    float              acc_z_g;
    float              angle_x_deg;
    float              angle_y_deg;
    float              angle_z_deg;
    float              temp_c;
} SCL3300_HandleTypeDef;

uint32_t SCL3300_Transfer32(SCL3300_HandleTypeDef *dev, uint32_t cmd);
uint8_t  SCL3300_CalculateCRC(uint32_t data);
bool     SCL3300_Init(SCL3300_HandleTypeDef *dev, SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin, uint8_t mode);
bool     SCL3300_ReadData(SCL3300_HandleTypeDef *dev);

#endif /* SCL3300_H */
