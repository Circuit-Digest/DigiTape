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
void OLED_DisplayOn(OLED_HandleTypeDef *dev);
void OLED_DisplayOff(OLED_HandleTypeDef *dev);
void OLED_DrawPixel(OLED_HandleTypeDef *dev, int16_t x, int16_t y, uint8_t color);
void OLED_DrawLine(OLED_HandleTypeDef *dev, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint8_t color);
void OLED_DrawRect(OLED_HandleTypeDef *dev, int16_t x, int16_t y, int16_t w, int16_t h, uint8_t color);
void OLED_FillRect(OLED_HandleTypeDef *dev, int16_t x, int16_t y, int16_t w, int16_t h, uint8_t color);
void OLED_DrawCircle(OLED_HandleTypeDef *dev, int16_t x0, int16_t y0, int16_t radius, uint8_t color);
void OLED_FillCircle(OLED_HandleTypeDef *dev, int16_t x0, int16_t y0, int16_t radius, uint8_t color);

void OLED_DrawStringSmall(OLED_HandleTypeDef *dev, uint8_t x, uint8_t y, const char *str, uint8_t color);
void OLED_DrawStringLarge(OLED_HandleTypeDef *dev, uint8_t x, uint8_t y, const char *str, uint8_t color);
void OLED_Printf(OLED_HandleTypeDef *dev, uint8_t x, uint8_t y, uint8_t size, const char *fmt, ...);

// Graphical Icon Primitives
void OLED_DrawBatteryIcon(OLED_HandleTypeDef *dev, uint8_t x, uint8_t y, uint8_t pct, uint8_t color);
void OLED_DrawLaserIcon(OLED_HandleTypeDef *dev, uint8_t x, uint8_t y, bool active, uint8_t color);
void OLED_DrawDatumIcon(OLED_HandleTypeDef *dev, uint8_t x, uint8_t y, bool is_rear, uint8_t color);
void OLED_DrawBubbleLevel(OLED_HandleTypeDef *dev, uint8_t center_x, uint8_t center_y, uint8_t radius, float pitch_deg, float roll_deg);
void OLED_DrawString3x(OLED_HandleTypeDef *dev, uint8_t x, uint8_t y, const char *str, uint8_t color);
void OLED_DrawInvertedString(OLED_HandleTypeDef *dev, uint8_t x, uint8_t y, const char *str, uint8_t pad_w, uint8_t pad_h);
void OLED_DrawRectIcon(OLED_HandleTypeDef *dev, uint8_t x, uint8_t y, uint8_t active_edge, bool blink_on);
void OLED_DrawCubeIcon(OLED_HandleTypeDef *dev, uint8_t x, uint8_t y, uint8_t active_edge, bool blink_on);
void OLED_DrawTriangleIcon(OLED_HandleTypeDef *dev, uint8_t x, uint8_t y);
void OLED_DrawCylinderIcon(OLED_HandleTypeDef *dev, uint8_t x, uint8_t y, uint8_t active_edge, bool blink_on);
void OLED_DrawFullScreenBubble(OLED_HandleTypeDef *dev, float pitch_deg, float roll_deg, float angle_z, float temp_c);
void OLED_DrawMenuModeIcon(OLED_HandleTypeDef *dev, uint8_t x, uint8_t y, uint8_t mode_idx, uint8_t color);
void OLED_DrawLevelBars(OLED_HandleTypeDef *dev, float pitch_elev_deg, float roll_deg);
void OLED_Draw7SegmentDigit(OLED_HandleTypeDef *dev, uint8_t x, uint8_t y, char c, uint8_t w, uint8_t h, uint8_t t, uint8_t color);
void OLED_Draw7SegmentString(OLED_HandleTypeDef *dev, uint8_t x, uint8_t y, const char *str, uint8_t w, uint8_t h, uint8_t t, uint8_t color);
void OLED_DrawCarouselModeIcon(OLED_HandleTypeDef *dev, int16_t x, int16_t y, uint8_t mode_idx, uint8_t color);

#endif /* OLED_H */
