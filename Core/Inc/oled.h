#ifndef OLED_H
#define OLED_H

#include "stm32g4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

#define OLED_I2C_ADDR       (0x3C << 1) // 8-bit I2C Write Address (0x78)
#define OLED_WIDTH          128
#define OLED_HEIGHT         64

#define OLED_COLOR_BLACK    0
#define OLED_COLOR_WHITE    1

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t            i2c_addr;
    uint8_t            buffer[1024]; // 128x64 pixels / 8 bits per byte = 1024 bytes
    bool               is_sh1106;    // Set true for 1.3" SH1106 OLED displays
} OLED_HandleTypeDef;

bool OLED_Init(OLED_HandleTypeDef *dev, I2C_HandleTypeDef *hi2c);
void OLED_Clear(OLED_HandleTypeDef *dev);
void OLED_UpdateScreen(OLED_HandleTypeDef *dev);
void OLED_DrawPixel(OLED_HandleTypeDef *dev, uint8_t x, uint8_t y, uint8_t color);
void OLED_DrawLine(OLED_HandleTypeDef *dev, uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t color);
void OLED_DrawRect(OLED_HandleTypeDef *dev, uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t color);
void OLED_FillRect(OLED_HandleTypeDef *dev, uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t color);
void OLED_DrawStringSmall(OLED_HandleTypeDef *dev, uint8_t x, uint8_t y, const char *str, uint8_t color);
void OLED_DrawStringLarge(OLED_HandleTypeDef *dev, uint8_t x, uint8_t y, const char *str, uint8_t color);
void OLED_Printf(OLED_HandleTypeDef *dev, uint8_t x, uint8_t y, uint8_t size, const char *fmt, ...);

#endif /* OLED_H */
