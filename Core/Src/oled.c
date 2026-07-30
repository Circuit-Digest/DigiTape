#include "oled.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

// Standard 5x7 Font (ASCII 32 to 126)
static const uint8_t Font5x7[][5] = {
    {0x00, 0x00, 0x00, 0x00, 0x00}, // Space
    {0x00, 0x00, 0x5F, 0x00, 0x00}, // !
    {0x00, 0x07, 0x00, 0x07, 0x00}, // "
    {0x14, 0x7F, 0x14, 0x7F, 0x14}, // #
    {0x24, 0x2A, 0x7F, 0x2A, 0x12}, // $
    {0x23, 0x13, 0x08, 0x64, 0x62}, // %
    {0x36, 0x49, 0x55, 0x22, 0x50}, // &
    {0x00, 0x05, 0x03, 0x00, 0x00}, // '
    {0x00, 0x1C, 0x22, 0x41, 0x00}, // (
    {0x00, 0x41, 0x22, 0x1C, 0x00}, // )
    {0x08, 0x2A, 0x1C, 0x2A, 0x08}, // *
    {0x08, 0x08, 0x3E, 0x08, 0x08}, // +
    {0x00, 0x50, 0x30, 0x00, 0x00}, // ,
    {0x08, 0x08, 0x08, 0x08, 0x08}, // -
    {0x00, 0x60, 0x60, 0x00, 0x00}, // .
    {0x20, 0x10, 0x08, 0x04, 0x02}, // /
    {0x3E, 0x51, 0x49, 0x45, 0x3E}, // 0
    {0x00, 0x42, 0x7F, 0x40, 0x00}, // 1
    {0x42, 0x61, 0x51, 0x49, 0x46}, // 2
    {0x21, 0x41, 0x45, 0x4B, 0x31}, // 3
    {0x18, 0x14, 0x12, 0x7F, 0x10}, // 4
    {0x27, 0x45, 0x45, 0x45, 0x39}, // 5
    {0x3C, 0x4A, 0x49, 0x49, 0x30}, // 6
    {0x01, 0x71, 0x09, 0x05, 0x03}, // 7
    {0x36, 0x49, 0x49, 0x49, 0x36}, // 8
    {0x06, 0x49, 0x49, 0x29, 0x1E}, // 9
    {0x00, 0x36, 0x36, 0x00, 0x00}, // :
    {0x00, 0x56, 0x36, 0x00, 0x00}, // ;
    {0x08, 0x14, 0x22, 0x41, 0x00}, // <
    {0x14, 0x14, 0x14, 0x14, 0x14}, // =
    {0x00, 0x41, 0x22, 0x14, 0x08}, // >
    {0x02, 0x01, 0x51, 0x09, 0x06}, // ?
    {0x32, 0x49, 0x79, 0x41, 0x3E}, // @
    {0x7E, 0x11, 0x11, 0x11, 0x7E}, // A
    {0x7F, 0x49, 0x49, 0x49, 0x36}, // B
    {0x3E, 0x41, 0x41, 0x41, 0x22}, // C
    {0x7F, 0x41, 0x41, 0x22, 0x1C}, // D
    {0x7F, 0x49, 0x49, 0x49, 0x41}, // E
    {0x7F, 0x09, 0x09, 0x09, 0x01}, // F
    {0x3E, 0x41, 0x49, 0x49, 0x7A}, // G
    {0x7F, 0x08, 0x08, 0x08, 0x7F}, // H
    {0x00, 0x41, 0x7F, 0x41, 0x00}, // I
    {0x20, 0x40, 0x41, 0x3F, 0x01}, // J
    {0x7F, 0x08, 0x14, 0x22, 0x41}, // K
    {0x7F, 0x40, 0x40, 0x40, 0x40}, // L
    {0x7F, 0x02, 0x0C, 0x02, 0x7F}, // M
    {0x7F, 0x04, 0x08, 0x10, 0x7F}, // N
    {0x3E, 0x41, 0x41, 0x41, 0x3E}, // O
    {0x7F, 0x09, 0x09, 0x09, 0x06}, // P
    {0x3E, 0x41, 0x51, 0x21, 0x5E}, // Q
    {0x7F, 0x09, 0x19, 0x29, 0x46}, // R
    {0x46, 0x49, 0x49, 0x49, 0x31}, // S
    {0x01, 0x01, 0x7F, 0x01, 0x01}, // T
    {0x3F, 0x40, 0x40, 0x40, 0x3F}, // U
    {0x1F, 0x20, 0x40, 0x20, 0x1F}, // V
    {0x3F, 0x40, 0x30, 0x40, 0x3C}, // W
    {0x63, 0x14, 0x08, 0x14, 0x63}, // X
    {0x07, 0x08, 0x70, 0x08, 0x07}, // Y
    {0x61, 0x51, 0x49, 0x45, 0x43}, // Z
    {0x00, 0x7F, 0x41, 0x41, 0x00}, // [
    {0x02, 0x04, 0x08, 0x10, 0x20}, // Backslash
    {0x00, 0x41, 0x41, 0x7F, 0x00}, // ]
    {0x04, 0x02, 0x01, 0x02, 0x04}, // ^
    {0x40, 0x40, 0x40, 0x40, 0x40}, // _
    {0x00, 0x01, 0x02, 0x04, 0x00}, // `
    {0x20, 0x54, 0x54, 0x54, 0x78}, // a
    {0x7F, 0x48, 0x44, 0x44, 0x38}, // b
    {0x38, 0x44, 0x44, 0x44, 0x20}, // c
    {0x38, 0x44, 0x44, 0x48, 0x7F}, // d
    {0x38, 0x54, 0x54, 0x54, 0x18}, // e
    {0x08, 0x7E, 0x09, 0x01, 0x02}, // f
    {0x0C, 0x52, 0x52, 0x52, 0x3E}, // g
    {0x7F, 0x08, 0x04, 0x04, 0x78}, // h
    {0x00, 0x44, 0x7D, 0x40, 0x00}, // i
    {0x20, 0x40, 0x44, 0x3D, 0x00}, // j
    {0x7F, 0x10, 0x28, 0x44, 0x00}, // k
    {0x00, 0x41, 0x7F, 0x40, 0x00}, // l
    {0x7C, 0x04, 0x18, 0x04, 0x78}, // m
    {0x7C, 0x08, 0x04, 0x04, 0x78}, // n
    {0x38, 0x44, 0x44, 0x44, 0x38}, // o
    {0x7C, 0x14, 0x14, 0x14, 0x08}, // p
    {0x08, 0x14, 0x14, 0x18, 0x7C}, // q
    {0x7C, 0x08, 0x04, 0x04, 0x08}, // r
    {0x48, 0x54, 0x54, 0x54, 0x20}, // s
    {0x04, 0x3E, 0x44, 0x24, 0x00}, // t
    {0x3C, 0x40, 0x40, 0x20, 0x7C}, // u
    {0x1C, 0x20, 0x40, 0x20, 0x1C}, // v
    {0x3C, 0x40, 0x30, 0x40, 0x3C}, // w
    {0x44, 0x28, 0x10, 0x28, 0x44}, // x
    {0x0C, 0x50, 0x50, 0x50, 0x3C}, // y
    {0x44, 0x64, 0x54, 0x4C, 0x44}, // z
    {0x00, 0x08, 0x36, 0x41, 0x00}, // {
    {0x00, 0x00, 0x7F, 0x00, 0x00}, // |
    {0x00, 0x41, 0x36, 0x08, 0x00}, // }
    {0x0C, 0x02, 0x0C, 0x10, 0x0C}  // ~
};

static void OLED_WriteCommand(OLED_HandleTypeDef *dev, uint8_t cmd)
{
    uint8_t tx[2] = {0x00, cmd};
    HAL_I2C_Master_Transmit(dev->hi2c, dev->i2c_addr, tx, 2, 100);
}

static void OLED_WriteData(OLED_HandleTypeDef *dev, uint8_t *data, uint16_t len)
{
    uint8_t tx[256];
    tx[0] = 0x40; // Data stream control byte
    for (uint16_t i = 0; i < len; i++) {
        tx[i + 1] = data[i];
    }
    HAL_I2C_Master_Transmit(dev->hi2c, dev->i2c_addr, tx, len + 1, 100);
}

bool OLED_Init(OLED_HandleTypeDef *dev, I2C_HandleTypeDef *hi2c)
{
    dev->hi2c = hi2c;
    dev->i2c_addr = OLED_I2C_ADDR;
    dev->is_sh1106 = true; // 1.3 inch OLEDs generally use SH1106 controller

    // Check if display responds on I2C2
    if (HAL_I2C_IsDeviceReady(dev->hi2c, dev->i2c_addr, 2, 100) != HAL_OK) {
        return false;
    }

    // OLED Initialization commands
    static const uint8_t init_cmds[] = {
        0xAE,       // Display OFF
        0xD5, 0x80, // Clock Divide Ratio
        0xA8, 0x3F, // Multiplex Ratio 1/64
        0xD3, 0x00, // Display Offset
        0x40,       // Start Line 0
        0x8D, 0x14, // Enable Charge Pump
        0x20, 0x02, // Page Addressing Mode
        0xA1,       // Segment Re-map (Horizontal Flip)
        0xC8,       // COM Output Scan Direction (Vertical Flip)
        0xDA, 0x12, // COM Pins Hardware Config
        0x81, 0xCF, // Contrast Control
        0xD9, 0xF1, // Pre-charge Period
        0xDB, 0x40, // VCOMH Deselect Level
        0xA4,       // Entire Display ON
        0xA6,       // Normal Display
        0xAF        // Display ON
    };

    for (uint8_t i = 0; i < sizeof(init_cmds); i++) {
        OLED_WriteCommand(dev, init_cmds[i]);
    }

    OLED_Clear(dev);
    OLED_UpdateScreen(dev);
    return true;
}

void OLED_Clear(OLED_HandleTypeDef *dev)
{
    memset(dev->buffer, 0x00, sizeof(dev->buffer));
}

void OLED_UpdateScreen(OLED_HandleTypeDef *dev)
{
    uint8_t column_offset = dev->is_sh1106 ? 2 : 0; // SH1106 1.3" starts at column 2

    for (uint8_t page = 0; page < 8; page++) {
        OLED_WriteCommand(dev, 0xB0 + page);
        OLED_WriteCommand(dev, 0x00 + (column_offset & 0x0F));
        OLED_WriteCommand(dev, 0x10 + ((column_offset >> 4) & 0x0F));

        OLED_WriteData(dev, &dev->buffer[page * OLED_WIDTH], OLED_WIDTH);
    }
}

void OLED_DrawPixel(OLED_HandleTypeDef *dev, uint8_t x, uint8_t y, uint8_t color)
{
    if (x >= OLED_WIDTH || y >= OLED_HEIGHT) return;

    if (color == OLED_COLOR_WHITE) {
        dev->buffer[x + (y / 8) * OLED_WIDTH] |= (1 << (y % 8));
    } else {
        dev->buffer[x + (y / 8) * OLED_WIDTH] &= ~(1 << (y % 8));
    }
}

void OLED_DrawLine(OLED_HandleTypeDef *dev, uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint8_t color)
{
    int dx = abs(x2 - x1), sx = x1 < x2 ? 1 : -1;
    int dy = -abs(y2 - y1), sy = y1 < y2 ? 1 : -1;
    int err = dx + dy, e2;

    while (1) {
        OLED_DrawPixel(dev, x1, y1, color);
        if (x1 == x2 && y1 == y2) break;
        e2 = 2 * err;
        if (e2 >= dy) { err += dy; x1 += sx; }
        if (e2 <= dx) { err += dx; y1 += sy; }
    }
}

void OLED_DrawRect(OLED_HandleTypeDef *dev, uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t color)
{
    OLED_DrawLine(dev, x, y, x + w - 1, y, color);
    OLED_DrawLine(dev, x, y + h - 1, x + w - 1, y + h - 1, color);
    OLED_DrawLine(dev, x, y, x, y + h - 1, color);
    OLED_DrawLine(dev, x + w - 1, y, x + w - 1, y + h - 1, color);
}

void OLED_FillRect(OLED_HandleTypeDef *dev, uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint8_t color)
{
    for (uint8_t i = x; i < x + w; i++) {
        for (uint8_t j = y; j < y + h; j++) {
            OLED_DrawPixel(dev, i, j, color);
        }
    }
}

void OLED_DrawCircle(OLED_HandleTypeDef *dev, uint8_t x0, uint8_t y0, uint8_t radius, uint8_t color)
{
    int f = 1 - radius;
    int ddF_x = 1;
    int ddF_y = -2 * radius;
    int x = 0;
    int y = radius;

    OLED_DrawPixel(dev, x0, y0 + radius, color);
    OLED_DrawPixel(dev, x0, y0 - radius, color);
    OLED_DrawPixel(dev, x0 + radius, y0, color);
    OLED_DrawPixel(dev, x0 - radius, y0, color);

    while (x < y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;

        OLED_DrawPixel(dev, x0 + x, y0 + y, color);
        OLED_DrawPixel(dev, x0 - x, y0 + y, color);
        OLED_DrawPixel(dev, x0 + x, y0 - y, color);
        OLED_DrawPixel(dev, x0 - x, y0 - y, color);
        OLED_DrawPixel(dev, x0 + y, y0 + x, color);
        OLED_DrawPixel(dev, x0 - y, y0 + x, color);
        OLED_DrawPixel(dev, x0 + y, y0 - x, color);
        OLED_DrawPixel(dev, x0 - y, y0 - x, color);
    }
}

void OLED_FillCircle(OLED_HandleTypeDef *dev, uint8_t x0, uint8_t y0, uint8_t radius, uint8_t color)
{
    for (int y = -radius; y <= radius; y++) {
        for (int x = -radius; x <= radius; x++) {
            if (x * x + y * y <= radius * radius) {
                OLED_DrawPixel(dev, x0 + x, y0 + y, color);
            }
        }
    }
}

void OLED_DrawStringSmall(OLED_HandleTypeDef *dev, uint8_t x, uint8_t y, const char *str, uint8_t color)
{
    while (*str) {
        char c = *str - 32;
        if (c < 0 || c > 94) c = 0;

        for (uint8_t i = 0; i < 5; i++) {
            uint8_t line = Font5x7[(uint8_t)c][i];
            for (uint8_t j = 0; j < 8; j++) {
                if (line & (1 << j)) {
                    OLED_DrawPixel(dev, x + i, y + j, color);
                }
            }
        }
        x += 6;
        str++;
    }
}

void OLED_DrawStringLarge(OLED_HandleTypeDef *dev, uint8_t x, uint8_t y, const char *str, uint8_t color)
{
    while (*str) {
        char c = *str - 32;
        if (c < 0 || c > 94) c = 0;

        for (uint8_t i = 0; i < 5; i++) {
            uint8_t line = Font5x7[(uint8_t)c][i];
            for (uint8_t j = 0; j < 8; j++) {
                if (line & (1 << j)) {
                    // Double width & height for large font
                    OLED_DrawPixel(dev, x + (i * 2),     y + (j * 2),     color);
                    OLED_DrawPixel(dev, x + (i * 2) + 1, y + (j * 2),     color);
                    OLED_DrawPixel(dev, x + (i * 2),     y + (j * 2) + 1, color);
                    OLED_DrawPixel(dev, x + (i * 2) + 1, y + (j * 2) + 1, color);
                }
            }
        }
        x += 12;
        str++;
    }
}

void OLED_Printf(OLED_HandleTypeDef *dev, uint8_t x, uint8_t y, uint8_t size, const char *fmt, ...)
{
    char buf[64];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    if (size == 1) {
        OLED_DrawStringSmall(dev, x, y, buf, OLED_COLOR_WHITE);
    } else {
        OLED_DrawStringLarge(dev, x, y, buf, OLED_COLOR_WHITE);
    }
}

// Graphical Icon Primitives
void OLED_DrawBatteryIcon(OLED_HandleTypeDef *dev, uint8_t x, uint8_t y, uint8_t pct, uint8_t color)
{
    // Draw Battery Body (14x7 px)
    OLED_DrawRect(dev, x, y, 14, 7, color);
    OLED_DrawLine(dev, x + 14, y + 2, x + 14, y + 4, color); // Tip

    // Fill inner bars according to percentage (0..100)
    uint8_t fill_w = (pct * 10) / 100;
    if (fill_w > 10) fill_w = 10;
    if (fill_w > 0) {
        OLED_FillRect(dev, x + 2, y + 2, fill_w, 3, color);
    }
}

void OLED_DrawLaserIcon(OLED_HandleTypeDef *dev, uint8_t x, uint8_t y, bool active, uint8_t color)
{
    if (active) {
        // Starburst / Laser Active Icon
        OLED_DrawLine(dev, x + 3, y,     x + 3, y + 6, color);
        OLED_DrawLine(dev, x,     y + 3, x + 6, y + 3, color);
        OLED_DrawPixel(dev, x + 1, y + 1, color);
        OLED_DrawPixel(dev, x + 5, y + 1, color);
        OLED_DrawPixel(dev, x + 1, y + 5, color);
        OLED_DrawPixel(dev, x + 5, y + 5, color);
    } else {
        // Idle Dot
        OLED_DrawRect(dev, x + 2, y + 2, 3, 3, color);
    }
}

void OLED_DrawDatumIcon(OLED_HandleTypeDef *dev, uint8_t x, uint8_t y, bool is_rear, uint8_t color)
{
    // Draw Device Body Outline
    OLED_DrawRect(dev, x, y, 6, 8, color);

    if (is_rear) {
        // Arrow pointing from rear (bottom)
        OLED_DrawPixel(dev, x + 2, y + 7, color);
        OLED_DrawPixel(dev, x + 3, y + 7, color);
        OLED_DrawLine(dev, x + 2, y + 5, x + 3, y + 5, color);
    } else {
        // Arrow pointing from front (top)
        OLED_DrawPixel(dev, x + 2, y, color);
        OLED_DrawPixel(dev, x + 3, y, color);
        OLED_DrawLine(dev, x + 2, y + 2, x + 3, y + 2, color);
    }
}

void OLED_DrawBubbleLevel(OLED_HandleTypeDef *dev, uint8_t center_x, uint8_t center_y, uint8_t radius, float pitch_deg, float roll_deg)
{
    // 1. Draw Target Circle & Crosshair
    OLED_DrawCircle(dev, center_x, center_y, radius, OLED_COLOR_WHITE);
    OLED_DrawCircle(dev, center_x, center_y, 2, OLED_COLOR_WHITE); // Inner center ring
    OLED_DrawLine(dev, center_x - radius - 3, center_y, center_x + radius + 3, center_y, OLED_COLOR_WHITE);
    OLED_DrawLine(dev, center_x, center_y - radius - 3, center_x, center_y + radius + 3, OLED_COLOR_WHITE);

    // 2. OLED Screen relative mapping:
    // - Left edge of OLED screen points to ToF/Laser (Angle X) -> dx controls Horizontal displacement
    // - Top edge of OLED screen points to Top of PCB (Angle Y)  -> dy controls Vertical displacement
    float dx = (pitch_deg / 15.0f) * (radius - 2);
    float dy = (roll_deg  / 15.0f) * (radius - 2);

    int bx = center_x + (int)dx;
    int by = center_y + (int)dy;

    // Clamp inside circle boundary
    if (bx < center_x - radius + 2) bx = center_x - radius + 2;
    if (bx > center_x + radius - 2) bx = center_x + radius - 2;
    if (by < center_y - radius + 2) by = center_y - radius + 2;
    if (by > center_y + radius - 2) by = center_y + radius - 2;

    // 3. Draw moving bubble
    bool is_level = (fabsf(pitch_deg) < 0.5f && fabsf(roll_deg) < 0.5f);
    if (is_level) {
        OLED_FillCircle(dev, bx, by, 3, OLED_COLOR_WHITE); // Solid bubble when perfectly level
    } else {
        OLED_DrawCircle(dev, bx, by, 2, OLED_COLOR_WHITE); // Hollow bubble when unlevel
    }
}

void OLED_DrawString3x(OLED_HandleTypeDef *dev, uint8_t x, uint8_t y, const char *str, uint8_t color)
{
    while (*str) {
        char c = *str - 32;
        if (c < 0 || c > 94) c = 0;

        for (uint8_t i = 0; i < 5; i++) {
            uint8_t line = Font5x7[(uint8_t)c][i];
            for (uint8_t j = 0; j < 8; j++) {
                if (line & (1 << j)) {
                    for (uint8_t dx = 0; dx < 3; dx++) {
                        for (uint8_t dy = 0; dy < 3; dy++) {
                            OLED_DrawPixel(dev, x + (i * 3) + dx, y + (j * 3) + dy, color);
                        }
                    }
                }
            }
        }
        x += 18; // 15px char width + 3px gap
        str++;
    }
}

void OLED_DrawInvertedString(OLED_HandleTypeDef *dev, uint8_t x, uint8_t y, const char *str, uint8_t pad_w, uint8_t pad_h)
{
    uint8_t len = strlen(str);
    uint8_t width = len * 6; // 6px per small char
    uint8_t height = 8;
    
    // Draw white background
    OLED_FillRect(dev, x, y, width + 2 * pad_w, height + 2 * pad_h, OLED_COLOR_WHITE);
    
    // Draw black text
    OLED_DrawStringSmall(dev, x + pad_w, y + pad_h, str, OLED_COLOR_BLACK);
}

void OLED_DrawRectIcon(OLED_HandleTypeDef *dev, uint8_t x, uint8_t y, uint8_t active_edge, bool blink_on)
{
    // Draw normal edges
    OLED_DrawLine(dev, x, y, x + 15, y, OLED_COLOR_WHITE);       // Top
    OLED_DrawLine(dev, x, y, x, y + 11, OLED_COLOR_WHITE);       // Left
    
    // Bottom (Length)
    if (active_edge == 0) {
        if (blink_on) {
            OLED_DrawLine(dev, x, y + 11, x + 15, y + 11, OLED_COLOR_WHITE);
            OLED_DrawLine(dev, x, y + 10, x + 15, y + 10, OLED_COLOR_WHITE);
        }
    } else {
        OLED_DrawLine(dev, x, y + 11, x + 15, y + 11, OLED_COLOR_WHITE);
    }
    
    // Right (Width)
    if (active_edge == 1) {
        if (blink_on) {
            OLED_DrawLine(dev, x + 15, y, x + 15, y + 11, OLED_COLOR_WHITE);
            OLED_DrawLine(dev, x + 14, y, x + 14, y + 11, OLED_COLOR_WHITE);
        }
    } else {
        OLED_DrawLine(dev, x + 15, y, x + 15, y + 11, OLED_COLOR_WHITE);
    }
}

void OLED_DrawCubeIcon(OLED_HandleTypeDef *dev, uint8_t x, uint8_t y, uint8_t active_edge, bool blink_on)
{
    // Normal edges
    OLED_DrawLine(dev, x+9, y+15, x+9, y+6, OLED_COLOR_WHITE);  // Front right vertical
    OLED_DrawLine(dev, x+9, y+6, x+0, y+6, OLED_COLOR_WHITE);   // Front top horizontal
    OLED_DrawLine(dev, x+9, y+6, x+15, y+0, OLED_COLOR_WHITE);  // Top right depth
    OLED_DrawLine(dev, x+15, y+9, x+15, y+0, OLED_COLOR_WHITE); // Back right vertical
    OLED_DrawLine(dev, x+0, y+6, x+6, y+0, OLED_COLOR_WHITE);   // Top left depth
    OLED_DrawLine(dev, x+6, y+0, x+15, y+0, OLED_COLOR_WHITE);  // Back top horizontal

    // Bottom-front (Length)
    if (active_edge == 0) {
        if (blink_on) {
            OLED_DrawLine(dev, x+0, y+15, x+9, y+15, OLED_COLOR_WHITE);
            OLED_DrawLine(dev, x+0, y+14, x+9, y+14, OLED_COLOR_WHITE);
        }
    } else {
        OLED_DrawLine(dev, x+0, y+15, x+9, y+15, OLED_COLOR_WHITE);
    }

    // Right-front depth (Width)
    if (active_edge == 1) {
        if (blink_on) {
            OLED_DrawLine(dev, x+9, y+15, x+15, y+9, OLED_COLOR_WHITE);
            OLED_DrawLine(dev, x+9, y+14, x+15, y+8, OLED_COLOR_WHITE);
        }
    } else {
        OLED_DrawLine(dev, x+9, y+15, x+15, y+9, OLED_COLOR_WHITE);
    }

    // Left-vertical (Height)
    if (active_edge == 2) {
        if (blink_on) {
            OLED_DrawLine(dev, x+0, y+15, x+0, y+6, OLED_COLOR_WHITE);
            OLED_DrawLine(dev, x+1, y+15, x+1, y+6, OLED_COLOR_WHITE);
        }
    } else {
        OLED_DrawLine(dev, x+0, y+15, x+0, y+6, OLED_COLOR_WHITE);
    }
}

void OLED_DrawTriangleIcon(OLED_HandleTypeDef *dev, uint8_t x, uint8_t y)
{
    // Right angle at bottom-left
    OLED_DrawLine(dev, x+5, y, x+5, y+15, OLED_COLOR_WHITE);
    OLED_DrawLine(dev, x+5, y+15, x+15, y+15, OLED_COLOR_WHITE);
    OLED_DrawLine(dev, x+5, y, x+15, y+15, OLED_COLOR_WHITE);

    // Label 'H' next to vertical side (small 3x5 size)
    OLED_DrawLine(dev, x, y+5, x, y+9, OLED_COLOR_WHITE);
    OLED_DrawLine(dev, x+2, y+5, x+2, y+9, OLED_COLOR_WHITE);
    OLED_DrawPixel(dev, x+1, y+7, OLED_COLOR_WHITE);
}

void OLED_DrawCylinderIcon(OLED_HandleTypeDef *dev, uint8_t x, uint8_t y, uint8_t active_edge, bool blink_on)
{
    // Top ellipse
    if (active_edge == 0) {
        if (blink_on) {
            OLED_DrawLine(dev, x+4, y, x+11, y, OLED_COLOR_WHITE);
            OLED_DrawLine(dev, x+4, y+4, x+11, y+4, OLED_COLOR_WHITE);
            OLED_DrawLine(dev, x+2, y+1, x+3, y+1, OLED_COLOR_WHITE);
            OLED_DrawLine(dev, x+12, y+1, x+13, y+1, OLED_COLOR_WHITE);
            OLED_DrawLine(dev, x+2, y+3, x+3, y+3, OLED_COLOR_WHITE);
            OLED_DrawLine(dev, x+12, y+3, x+13, y+3, OLED_COLOR_WHITE);
            OLED_DrawPixel(dev, x+1, y+2, OLED_COLOR_WHITE);
            OLED_DrawPixel(dev, x+14, y+2, OLED_COLOR_WHITE);
            
            // Double thickness for blink
            OLED_DrawLine(dev, x+4, y+1, x+11, y+1, OLED_COLOR_WHITE);
            OLED_DrawLine(dev, x+4, y+5, x+11, y+5, OLED_COLOR_WHITE);
        }
    } else {
        OLED_DrawLine(dev, x+4, y, x+11, y, OLED_COLOR_WHITE);
        OLED_DrawLine(dev, x+4, y+4, x+11, y+4, OLED_COLOR_WHITE);
        OLED_DrawLine(dev, x+2, y+1, x+3, y+1, OLED_COLOR_WHITE);
        OLED_DrawLine(dev, x+12, y+1, x+13, y+1, OLED_COLOR_WHITE);
        OLED_DrawLine(dev, x+2, y+3, x+3, y+3, OLED_COLOR_WHITE);
        OLED_DrawLine(dev, x+12, y+3, x+13, y+3, OLED_COLOR_WHITE);
        OLED_DrawPixel(dev, x+1, y+2, OLED_COLOR_WHITE);
        OLED_DrawPixel(dev, x+14, y+2, OLED_COLOR_WHITE);
    }
    
    // Bottom ellipse lower half
    OLED_DrawLine(dev, x+4, y+15, x+11, y+15, OLED_COLOR_WHITE);
    OLED_DrawLine(dev, x+2, y+14, x+3, y+14, OLED_COLOR_WHITE);
    OLED_DrawLine(dev, x+12, y+14, x+13, y+14, OLED_COLOR_WHITE);
    OLED_DrawPixel(dev, x+1, y+13, OLED_COLOR_WHITE);
    OLED_DrawPixel(dev, x+14, y+13, OLED_COLOR_WHITE);
    
    // Bottom ellipse upper half (dotted)
    OLED_DrawPixel(dev, x+4, y+11, OLED_COLOR_WHITE);
    OLED_DrawPixel(dev, x+6, y+11, OLED_COLOR_WHITE);
    OLED_DrawPixel(dev, x+8, y+11, OLED_COLOR_WHITE);
    OLED_DrawPixel(dev, x+10, y+11, OLED_COLOR_WHITE);
    OLED_DrawPixel(dev, x+2, y+12, OLED_COLOR_WHITE);
    OLED_DrawPixel(dev, x+13, y+12, OLED_COLOR_WHITE);

    // Left vertical (height)
    if (active_edge == 1) {
        if (blink_on) {
            OLED_DrawLine(dev, x+1, y+2, x+1, y+13, OLED_COLOR_WHITE);
            OLED_DrawLine(dev, x+2, y+2, x+2, y+13, OLED_COLOR_WHITE); // Double
        }
    } else {
        OLED_DrawLine(dev, x+1, y+2, x+1, y+13, OLED_COLOR_WHITE);
    }
    
    // Right vertical
    OLED_DrawLine(dev, x+14, y+2, x+14, y+13, OLED_COLOR_WHITE);
}

void OLED_DrawFullScreenBubble(OLED_HandleTypeDef *dev, float pitch_deg, float roll_deg, float angle_z, float temp_c)
{
    // 1. Render Top Horizontal & Right Vertical Leveling Bars
    // OLED_DrawLevelBars(dev, pitch_deg, roll_deg);

    // 2. Left Side Telemetry Readout Column (X: 2..46) with 7-Segment Digits
    char buf[16];

    // Row 0 (Y: 6): Pitch Elevation (-Angle X)
    snprintf(buf, sizeof(buf), "%.1f", pitch_deg);
    OLED_DrawStringSmall(dev, 2, 6, "E:", OLED_COLOR_WHITE);
    OLED_Draw7SegmentString(dev, 14, 6, buf, 6, 10, 1, OLED_COLOR_WHITE);

    // Row 1 (Y: 20): Side Roll (Angle Y)
    snprintf(buf, sizeof(buf), "%.1f", roll_deg);
    OLED_DrawStringSmall(dev, 2, 20, "R:", OLED_COLOR_WHITE);
    OLED_Draw7SegmentString(dev, 14, 20, buf, 6, 10, 1, OLED_COLOR_WHITE);

    // Row 2 (Y: 34): Z Angle
    snprintf(buf, sizeof(buf), "%.1f", angle_z);
    OLED_DrawStringSmall(dev, 2, 34, "Z:", OLED_COLOR_WHITE);
    OLED_Draw7SegmentString(dev, 14, 34, buf, 6, 10, 1, OLED_COLOR_WHITE);

    // Row 3 (Y: 48): Temperature
    snprintf(buf, sizeof(buf), "%.1f", temp_c);
    OLED_DrawStringSmall(dev, 2, 48, "T:", OLED_COLOR_WHITE);
    OLED_Draw7SegmentString(dev, 14, 48, buf, 6, 10, 1, OLED_COLOR_WHITE);

    // 3. Right Side Maximized 2D Bubble Target Graph (Center: 83, 32 | Radius: 25px)
    uint8_t cx = 83;
    uint8_t cy = 32;
    uint8_t r = 25;

    // Crosshairs
    // OLED_DrawLine(dev, 52, cy, 114, cy, OLED_COLOR_WHITE);
    // OLED_DrawLine(dev, cx, 6, cx, 58, OLED_COLOR_WHITE);

    // Target rings
    OLED_DrawCircle(dev, cx, cy, r, OLED_COLOR_WHITE);
    OLED_DrawCircle(dev, cx, cy, 4, OLED_COLOR_WHITE);

    // Moving bubble dot
    float dx = (pitch_deg / 15.0f) * (r - 3);
    float dy = (roll_deg / 15.0f) * (r - 3);

    int bx = cx + (int)dx;
    int by = cy + (int)dy;

    // Clamp inside circle boundary
    if (bx < cx - r + 3) bx = cx - r + 3;
    if (bx > cx + r - 3) bx = cx + r - 3;
    if (by < cy - r + 3) by = cy - r + 3;
    if (by > cy + r - 3) by = cy + r - 3;

    bool is_level = (fabsf(pitch_deg) < 0.5f && fabsf(roll_deg) < 0.5f);
    if (is_level) {
        OLED_FillCircle(dev, bx, by, 5, OLED_COLOR_WHITE); 
    } else {
        OLED_DrawCircle(dev, bx, by, 3, OLED_COLOR_WHITE); 
    }
}

void OLED_DrawMenuModeIcon(OLED_HandleTypeDef *dev, uint8_t x, uint8_t y, uint8_t mode_idx, uint8_t color)
{
    switch (mode_idx) {
        case 0: // DISTANCE: Laser Meter Body + Beam
            OLED_DrawRect(dev, x + 1, y + 2, 7, 8, color);
            OLED_DrawLine(dev, x + 8, y + 5, x + 13, y + 5, color);
            OLED_DrawPixel(dev, x + 14, y + 5, color);
            break;

        case 1: // LEVEL: Bubble Target
            OLED_DrawCircle(dev, x + 7, y + 6, 5, color);
            OLED_DrawPixel(dev, x + 7, y + 6, color);
            OLED_DrawLine(dev, x + 7, y + 1, x + 7, y + 2, color);
            OLED_DrawLine(dev, x + 7, y + 10, x + 7, y + 11, color);
            OLED_DrawLine(dev, x + 2, y + 6, x + 3, y + 6, color);
            OLED_DrawLine(dev, x + 11, y + 6, x + 12, y + 6, color);
            break;

        case 2: // HEIGHT: Right Triangle
            OLED_DrawLine(dev, x + 2, y + 1, x + 2, y + 11, color);
            OLED_DrawLine(dev, x + 2, y + 11, x + 12, y + 11, color);
            OLED_DrawLine(dev, x + 2, y + 1, x + 12, y + 11, color);
            break;

        case 3: // AREA: Rectangle
            OLED_DrawRect(dev, x + 1, y + 2, 13, 8, color);
            break;

        case 4: // VOLUME: Isometric Cube
            OLED_DrawCubeIcon(dev, x, y - 2, 3, false);
            break;

        case 5: // CYLINDER: 3D Cylinder
            OLED_DrawCylinderIcon(dev, x, y - 2, 2, false);
            break;

        case 6: // MAXMIN: Min/Max Up-Down Arrows
            // Up Arrow (MIN)
            OLED_DrawLine(dev, x + 3, y + 1, x + 3, y + 11, color);
            OLED_DrawLine(dev, x + 1, y + 3, x + 3, y + 1, color);
            OLED_DrawLine(dev, x + 5, y + 3, x + 3, y + 1, color);
            // Down Arrow (MAX)
            OLED_DrawLine(dev, x + 11, y + 1, x + 11, y + 11, color);
            OLED_DrawLine(dev, x + 9, y + 9, x + 11, y + 11, color);
            OLED_DrawLine(dev, x + 13, y + 9, x + 11, y + 11, color);
            break;

        case 7: // MEMORY: Document / Log Lines
            OLED_DrawRect(dev, x + 3, y + 1, 9, 11, color);
            OLED_DrawLine(dev, x + 5, y + 4, x + 9, y + 4, color);
            OLED_DrawLine(dev, x + 5, y + 7, x + 9, y + 7, color);
            break;

        default:
            break;
    }
}

void OLED_DrawLevelBars(OLED_HandleTypeDef *dev, float pitch_elev_deg, float roll_deg)
{
    // 1. Top Horizontal Bar for Laser Pitch Elevation (-Angle X)
    // Position: X: 8..104 (Width 97px), Y: 16..20 (Height 5px)
    uint8_t h_x = 8;
    uint8_t h_y = 16;
    uint8_t h_w = 97;
    uint8_t h_h = 5;
    uint8_t h_center = h_x + h_w / 2; // X: 56

    OLED_DrawRect(dev, h_x, h_y, h_w, h_h, OLED_COLOR_WHITE);
    OLED_DrawLine(dev, h_center, h_y, h_center, h_y + h_h - 1, OLED_COLOR_WHITE);

    // Laser Pitch Elevation (-15° to +15° range mapped to +/-42px)
    float pitch_offset = (pitch_elev_deg / 15.0f) * 42.0f;
    if (pitch_offset < -42.0f) pitch_offset = -42.0f;
    if (pitch_offset > 42.0f)  pitch_offset = 42.0f;
    int h_marker = h_center + (int)pitch_offset;

    bool pitch_level = (fabsf(pitch_elev_deg) < 0.5f);
    if (pitch_level) {
        OLED_FillRect(dev, h_marker - 2, h_y + 1, 5, 3, OLED_COLOR_WHITE);
    } else {
        OLED_DrawRect(dev, h_marker - 2, h_y + 1, 5, 3, OLED_COLOR_WHITE);
    }

    // 2. Right Vertical Bar for Side Roll Tilt (Angle Y)
    // Position: X: 120..124 (Width 5px), Y: 16..58 (Height 43px)
    uint8_t v_x = 120;
    uint8_t v_y = 16;
    uint8_t v_w = 5;
    uint8_t v_h = 43;
    uint8_t v_center = v_y + v_h / 2; // Y: 37

    OLED_DrawRect(dev, v_x, v_y, v_w, v_h, OLED_COLOR_WHITE);
    OLED_DrawLine(dev, v_x, v_center, v_x + v_w - 1, v_center, OLED_COLOR_WHITE);

    // Side Roll Tilt (top edge up = positive = marker moves UP towards Y:16)
    float roll_offset = (-roll_deg / 15.0f) * 18.0f;
    if (roll_offset < -18.0f) roll_offset = -18.0f;
    if (roll_offset > 18.0f)  roll_offset = 18.0f;
    int v_marker = v_center + (int)roll_offset;

    bool roll_level = (fabsf(roll_deg) < 0.5f);
    if (roll_level) {
        OLED_FillRect(dev, v_x + 1, v_marker - 2, 3, 5, OLED_COLOR_WHITE);
    } else {
        OLED_DrawRect(dev, v_x + 1, v_marker - 2, 3, 5, OLED_COLOR_WHITE);
    }
}

static void OLED_DrawSegHex(OLED_HandleTypeDef *dev, uint8_t seg, uint8_t x, uint8_t y, uint8_t segWd, uint8_t segHt, uint8_t segThick, bool is_on)
{
    if (!is_on) return; // Clean 100% noise-free unlit segments!

    uint8_t ofs = segThick / 2;

    switch (seg) {
        case 0: // top
        case 3: // bottom
        case 6: // middle
            for (uint8_t i = 0; i <= ofs; i++) {
                OLED_DrawLine(dev, x + i, y + ofs - i, x + segWd - 1 - i, y + ofs - i, OLED_COLOR_WHITE);
                OLED_DrawLine(dev, x + i, y + ofs + i, x + segWd - 1 - i, y + ofs + i, OLED_COLOR_WHITE);
            }
            break;

        case 1: // right-top
        case 2: // right-bottom
        case 4: // left-bottom
        case 5: // left-top
            for (uint8_t i = 0; i <= ofs; i++) {
                OLED_DrawLine(dev, x + ofs - i, y + i, x + ofs - i, y + segHt - 1 - i, OLED_COLOR_WHITE);
                OLED_DrawLine(dev, x + ofs + i, y + i, x + ofs + i, y + segHt - 1 - i, OLED_COLOR_WHITE);
            }
            break;
    }
}

void OLED_Draw7SegmentDigit(OLED_HandleTypeDef *dev, uint8_t x, uint8_t y, char c, uint8_t w, uint8_t h, uint8_t t, uint8_t color)
{
    if (c == '.') {
        OLED_FillRect(dev, x + 1, y + h - t - 1, t, t, color);
        return;
    }
    if (c == ' ') return;

    uint8_t mask = 0x00;
    if (c >= '0' && c <= '9') {
        static const uint8_t masks[10] = {
            0x3F, // '0'
            0x06, // '1'
            0x5B, // '2'
            0x4F, // '3'
            0x66, // '4'
            0x6D, // '5'
            0x7D, // '6'
            0x07, // '7'
            0x7F, // '8'
            0x6F  // '9'
        };
        mask = masks[c - '0'];
    } else if (c == '-') {
        mask = 0x40; // 'g' segment
    }

    if (mask == 0x00) return;

    uint8_t ofs = 1 + t / 2;
    uint8_t segWd = (w > ofs * 2 + 2) ? (w - ofs * 2) : 4;
    uint8_t segHt = (h > ofs * 2 + 3) ? ((h - ofs * 2 - 3) / 2) : 6;

    // Segment 0: Top (a)
    OLED_DrawSegHex(dev, 0, x + ofs, y, segWd, segHt, t, (mask & 0x01) != 0);

    // Segment 1: Right-Top (b)
    OLED_DrawSegHex(dev, 1, x + w - t, y + ofs, segWd, segHt, t, (mask & 0x02) != 0);

    // Segment 2: Right-Bottom (c)
    OLED_DrawSegHex(dev, 2, x + w - t, y + ofs + segHt + 1, segWd, segHt, t, (mask & 0x04) != 0);

    // Segment 3: Bottom (d)
    OLED_DrawSegHex(dev, 3, x + ofs, y + segHt + segHt + 2, segWd, segHt, t, (mask & 0x08) != 0);

    // Segment 4: Left-Bottom (e)
    OLED_DrawSegHex(dev, 4, x, y + ofs + segHt + 1, segWd, segHt, t, (mask & 0x10) != 0);

    // Segment 5: Left-Top (f)
    OLED_DrawSegHex(dev, 5, x, y + ofs, segWd, segHt, t, (mask & 0x20) != 0);

    // Segment 6: Middle (g)
    OLED_DrawSegHex(dev, 6, x + ofs, y + segHt + 1, segWd, segHt, t, (mask & 0x40) != 0);
}

void OLED_Draw7SegmentString(OLED_HandleTypeDef *dev, uint8_t x, uint8_t y, const char *str, uint8_t w, uint8_t h, uint8_t t, uint8_t color)
{
    while (*str) {
        char c = *str;
        if (c >= '0' && c <= '9') {
            OLED_Draw7SegmentDigit(dev, x, y, c, w, h, t, color);
            x += w + 3;
        } else if (c == '-') {
            OLED_Draw7SegmentDigit(dev, x, y, '-', w, h, t, color);
            x += w + 3;
        } else if (c == ' ') {
            x += 4; // Compact 4px spacing between digits and unit badge
        } else if (c == '.') {
            OLED_Draw7SegmentDigit(dev, x, y, '.', w, h, t, color);
            x += t + 3;
        } else {
            // Unit badge (e.g. CM, MM, M, IN, cm2): baseline-aligned small font
            uint8_t unit_y = (h > 10) ? (y + h - 8) : y;
            OLED_DrawStringSmall(dev, x, unit_y, str, color);
            break;
        }
        str++;
    }
}
