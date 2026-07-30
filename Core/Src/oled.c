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
