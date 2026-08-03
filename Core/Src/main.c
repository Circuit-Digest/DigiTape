/*
 * main.c
 * Main firmware implementation for Digi-Tape Tool
 *
 * Copyright (c) 2026 Dharagesh and Circuit Digest
 * https://github.com/Circuit-Digest/Digi-Tape-Tool
 * Licensed under GNU General Public License v3.0
 */

/*
  ██████╗  ██████╗  ██████╗ ██████╗     ████████╗██████╗ ██████╗ ███████╗
  ██╔══██╗   ██║   ██╔════╝   ██║       ╚══██╔══╝██╔══██╗██╔══██╗██╔════╝
  ██║  ██║   ██║   ██║  ███╗  ██║          ██║   ███████║██████╔╝█████╗  
  ██║  ██║   ██║   ██║   ██║  ██║          ██║   ██╔══██║██╔═══╝ ██╔══╝  
  ██████╔╝ ██████╗ ╚██████╔╝██████╗        ██║   ██║  ██║██║     ███████╗
  ╚═════╝  ╚═════╝  ╚═════╝ ╚═════╝        ╚═╝   ╚═╝  ╚═╝╚═╝     ╚══════╝
*/
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body - Digital Measurement Device
  *                   Modes: DIST, LEVEL, HEIGHT, AREA, VOLUME, CYLINDER, MAX/MIN, MEMORY
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <math.h>
#include "scl3300.h"
#include "vl53lx_api.h"
#include "oled.h"
#include <stdbool.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    int32_t  counter;
    uint8_t  prev_a;
    uint8_t  prev_b;
    uint8_t  prev_sw;
    uint32_t press_start_tick;
    uint32_t last_turn_tick;
    bool     short_press;
    bool     long_press;
    bool     long_press_handled;
    bool     press_twist; // Press + Twist chorded gesture flag (Switch held while turning knob)
} Encoder_t;

typedef enum {
    APP_STATE_MENU = 0,
    APP_STATE_MEASURE,
    APP_STATE_SETTINGS
} AppState_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEG_TO_RAD(deg) ((deg) * 0.017453292519943295f)
#define M_PI_F 3.14159265358979323846f
#define DOUBLE_PRESS_WINDOW_MS 400
#define TOF_VERTICAL_OFFSET_CM 2.4f // 1.8cm optical center + 0.6cm enclosure base = 2.4cm total vertical mounting offset
#define HEIGHT_GAIN_SCALE      1.0989f // 2-point empirical calibration slope gain
#define HEIGHT_BIAS_OFFSET     +2.242f // +4.0cm offset shift correction (36.0cm -> 40.0cm exact match)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc3;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
OLED_HandleTypeDef       oled_dev;
SCL3300_HandleTypeDef    scl_dev;
VL53LX_Dev_t             vl53_dev_inst;
VL53LX_DEV               vl53_dev = &vl53_dev_inst;
VL53LX_DEV               p_vl53 = &vl53_dev_inst;

Encoder_t             encoder = {0};

uint8_t  selected_mode = 0;       // Highlighted item in 4x2 Menu Grid (0..7)
AppState_t app_state = APP_STATE_MENU; // Global app state
bool     in_settings = false;     // Full-screen Settings overlay active
bool     in_settings_edit = false;// Settings Parameter Value Edit Mode active
uint8_t  settings_item = 0;       // 0: Unit, 1: Datum, 2: Rear Offset

uint8_t  unit_mode = 0;           // 0: CM, 1: MM, 2: M, 3: INCH
uint8_t  datum_mode = 0;          // 0: REAR (bottom), 1: FRONT (top)
float    rear_offset_cm = 6.6f;   // 6.6 cm device body length offset when REAR datum selected

bool     laser_active = false;
uint32_t last_short_press_tick = 0; // For global double-press detection

float    laser_pitch_elev = 0.0f;   // Global Laser Pitch Elevation (-Angle X)
float    side_roll = 0.0f;          // Global Screen Side Roll (Angle Y)
int16_t  carousel_anim_x = 0;       // Smooth sliding animation offset for fitness band carousel menu

// Auto Sleep & Motion Wakeup Control
#define AUTO_SLEEP_TIMEOUT_MS  180000UL // 3 Minutes (180 Seconds) Inactivity Timeout
bool     device_sleeping = false;
uint32_t last_activity_tick = 0;
float    prev_acc_x = 0.0f;
float    prev_acc_y = 0.0f;
float    prev_acc_z = 0.0f;

bool     hold_active = false;
int      hold_distance_mm = -1;
float    hold_pitch_elev = 0.0f;

float    min_dist_cm = 9999.0f;
float    max_dist_cm = 0.0f;

bool     boot_complete = false;
uint32_t blink_tick = 0;
bool     blink_on = true;

// Multi-shot measurement accumulators for area, volume, height, cylinder
uint8_t  multi_shot_step = 0;
float    shot1_cm = -1.0f;
float    shot2_cm = -1.0f;
float    shot3_cm = -1.0f;

// Saved History Storage (last 10 measurements)
#define MAX_HISTORY 10
float    history_val[MAX_HISTORY];
float    history_buffer[MAX_HISTORY];
uint8_t  history_mode[MAX_HISTORY];
uint8_t  history_unit[MAX_HISTORY];
uint8_t  history_count = 0;
uint8_t  history_view_idx = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_ADC3_Init(void);
/* USER CODE BEGIN PFP */
float Read_Battery_Voltage(void);
uint8_t Calculate_Battery_Percentage(float v_bat);
void Format_Distance_String(float dist_cm, uint8_t unit, char *out_str, size_t max_len);
void Add_To_History(float dist_cm);
void Save_Settings_To_Flash(void);
void Load_Settings_From_Flash(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Encoder_Init(Encoder_t *e)
{
    e->counter = 0;
    e->prev_a = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12);
    e->prev_b = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13);
    e->prev_sw = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14);
    e->press_start_tick = 0;
    e->last_turn_tick = 0;
    e->short_press = false;
    e->long_press = false;
    e->long_press_handled = false;
}

void Reset_Multi_Shot(void)
{
    multi_shot_step = 0;
    shot1_cm = -1.0f;
    shot2_cm = -1.0f;
    shot3_cm = -1.0f;
}

void Encoder_Update(Encoder_t *e)
{
    // 1. Read Quadrature / Spring-Return Encoder Pins (Hongyan RS11: A = PB12, B = PB13)
    uint8_t curr_a = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12);
    uint8_t curr_b = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13);

    uint32_t now = HAL_GetTick();
    int dir = 0;

    // Dual-edge detection with 150ms spring-return lockout for Hongyan RS11 (+15 deg CW / -15 deg CCW toggle switch)
    if (now - e->last_turn_tick >= 150) {
        if (e->prev_a == GPIO_PIN_SET && curr_a == GPIO_PIN_RESET) {
            dir = (curr_b == GPIO_PIN_SET) ? 1 : -1;
            e->last_turn_tick = now;
        } else if (e->prev_b == GPIO_PIN_SET && curr_b == GPIO_PIN_RESET) {
            dir = (curr_a == GPIO_PIN_SET) ? -1 : 1;
            e->last_turn_tick = now;
        }
    }

    uint8_t curr_sw = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14);

    if (dir != 0) {
        if (curr_sw == GPIO_PIN_RESET) {
            // Press + Twist chorded gesture detected! (Switch held while twisting encoder)
            e->press_twist = true;
            e->long_press_handled = true; // Prevent triggering short press on release
            printf("\r\n>>> [GESTURE DETECTED] Switch Held + Encoder Twist! <<<\r\n\r\n");
        } else {
            e->counter += dir;

            if (app_state == APP_STATE_SETTINGS || in_settings) {
                if (in_settings_edit) {
                    // ITEM EDIT MODE: Turning Encoder Changes Selected Parameter Value
                    if (settings_item == 0) { // UNIT MODE (0: CM, 1: MM, 2: M, 3: INCH)
                        int u = (int)unit_mode + dir;
                        if (u < 0) u = 3;
                        if (u > 3) u = 0;
                        unit_mode = (uint8_t)u;
                    } else if (settings_item == 1) { // DATUM MODE (0: REAR, 1: FRONT)
                        datum_mode = (datum_mode == 0) ? 1 : 0;
                    } else if (settings_item == 2) { // REAR OFFSET (0.0cm .. 20.0cm in 0.1cm steps)
                        rear_offset_cm += (dir > 0) ? 0.1f : -0.1f;
                        if (rear_offset_cm < -0.01f) rear_offset_cm = 20.0f;
                        if (rear_offset_cm > 20.01f) rear_offset_cm = 0.0f;
                    }
                } else {
                    // MENU NAVIGATION MODE: Scroll through 3 Settings Items (0: Unit, 1: Datum, 2: Rear Offset)
                    int item = (int)settings_item + dir;
                    if (item < 0) item = 2;
                    if (item > 2) item = 0;
                    settings_item = (uint8_t)item;
                }
            } else if (app_state == APP_STATE_MENU) {
                // Scroll through all 8 available modes in fitness band carousel
                int m = (int)selected_mode + dir;
                if (m < 0) m = 7;
                if (m > 7) m = 0;
                selected_mode = (uint8_t)m;
                carousel_anim_x = (dir > 0) ? 36 : -36;
            } else if (app_state == APP_STATE_MEASURE && selected_mode == 7) {
                // In History Memory Mode, scroll through saved records
                if (history_count > 0) {
                    int h_idx = (int)history_view_idx + dir;
                    if (h_idx < 0) h_idx = history_count - 1;
                    if (h_idx >= history_count) h_idx = 0;
                    history_view_idx = (uint8_t)h_idx;
                }
            }

            printf("\r\n>>> [ENCODER BUMP] Dir: %d | Counter: %ld | Mode: %u | Settings Item: %u <<<\r\n\r\n",
                   dir, (long)e->counter, selected_mode, settings_item);
        }
    }

    e->prev_a = curr_a;
    e->prev_b = curr_b;

    // 2. Short-Press & Long-Press Detection

    // Button Pressed Down
    if (e->prev_sw == GPIO_PIN_SET && curr_sw == GPIO_PIN_RESET) {
        e->press_start_tick = HAL_GetTick();
        e->long_press_handled = false;
    }

    // Button Held Down (Check for Long Press > 800ms)
    if (curr_sw == GPIO_PIN_RESET && !e->long_press_handled) {
        if (HAL_GetTick() - e->press_start_tick >= 800) {
            e->long_press = true;
            e->long_press_handled = true;
        }
    }

    // Button Released
    if (e->prev_sw == GPIO_PIN_RESET && curr_sw == GPIO_PIN_SET) {
        uint32_t press_duration = HAL_GetTick() - e->press_start_tick;
        if (press_duration >= 50 && press_duration < 800 && !e->long_press_handled) {
            e->short_press = true;
        }
    }

    e->prev_sw = curr_sw;

    if (dir != 0 || e->short_press || e->long_press || curr_sw == GPIO_PIN_RESET) {
        last_activity_tick = now;
        if (device_sleeping) {
            device_sleeping = false;
            OLED_DisplayOn(&oled_dev);
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET); // Hardware XSHUT Power ON (PC11)
            if (p_vl53) VL53LX_StartMeasurement(p_vl53);
            printf(" -> [WAKEUP] Encoder Interaction! Display & ToF Active.\r\n");
        }
    }
}

// --- Distance-Adaptive Long-Range Moving Median & EMA Filter ---
#define TOF_FILTER_WINDOW 7
static int     tof_window_buf[TOF_FILTER_WINDOW] = {0};
static uint8_t tof_window_idx = 0;
static uint8_t tof_window_count = 0;
static float   filtered_tof_mm = -1.0f;

int Filter_ToF_Distance_MM(int raw_mm)
{
    if (raw_mm <= 0) return (int)filtered_tof_mm;

    // 1. Add sample to sliding window buffer
    tof_window_buf[tof_window_idx] = raw_mm;
    tof_window_idx = (tof_window_idx + 1) % TOF_FILTER_WINDOW;
    if (tof_window_count < TOF_FILTER_WINDOW) tof_window_count++;

    // 2. Selection sort to find Median (outlier spike rejection)
    int sorted[TOF_FILTER_WINDOW];
    for (uint8_t i = 0; i < tof_window_count; i++) sorted[i] = tof_window_buf[i];

    for (uint8_t i = 0; i < tof_window_count - 1; i++) {
        for (uint8_t j = i + 1; j < tof_window_count; j++) {
            if (sorted[i] > sorted[j]) {
                int tmp = sorted[i];
                sorted[i] = sorted[j];
                sorted[j] = tmp;
            }
        }
    }

    int median_mm = sorted[tof_window_count / 2];

    // 3. Distance-Adaptive EMA Filter Weight
    // At short range (500mm): alpha = 0.60 (Fast response)
    // At long range (4000mm): alpha = 0.23 (Damped steady)
    float alpha = 1.0f / (1.0f + 0.0008f * (float)median_mm);
    if (alpha < 0.08f) alpha = 0.08f;
    if (alpha > 0.60f) alpha = 0.60f;

    if (filtered_tof_mm < 0.0f) {
        filtered_tof_mm = (float)median_mm; // Seed initial value
    } else {
        filtered_tof_mm = (alpha * (float)median_mm) + ((1.0f - alpha) * filtered_tof_mm);
    }

    return (int)(filtered_tof_mm + 0.5f);
}

// --- ST Multi-Target Histogram & Signal Quality Evaluator ---
float live_signal_mcps  = 0.0f;
float live_ambient_mcps = 0.0f;

int Process_ST_MultiTarget_Ranging(VL53LX_MultiRangingData_t *p_data)
{
    if (!p_data || p_data->NumberOfObjectsFound == 0) return -1;

    int best_mm = -1;
    float max_signal = -1.0f;

    for (uint8_t i = 0; i < p_data->NumberOfObjectsFound; i++) {
        VL53LX_TargetRangeData_t *target = &p_data->RangeData[i];
        
        // Convert ST 16.16 fixed-point Mcps to float
        float sig_mcps = (float)target->SignalRateRtnMegaCps / 65536.0f;
        float amb_mcps = (float)target->AmbientRateRtnMegaCps / 65536.0f;

        if (i == 0) {
            live_signal_mcps  = sig_mcps;
            live_ambient_mcps = amb_mcps;
        }

        // Accept target if RangeStatus is 0 (Valid target return)
        if (target->RangeStatus == 0 && target->RangeMilliMeter > 10) {
            if (sig_mcps > max_signal) {
                max_signal = sig_mcps;
                best_mm    = target->RangeMilliMeter;
            }
        }
    }

    // Fallback: If no RangeStatus==0 target was found, use RangeData[0]
    if (best_mm < 0 && p_data->RangeData[0].RangeMilliMeter > 10) {
        best_mm = p_data->RangeData[0].RangeMilliMeter;
    }

    return best_mm;
}

float Calculate_Net_Distance_CM(int raw_mm, float pitch_elev_deg)
{
    if (raw_mm < 0) return -1.0f;
    float dist_cm = (float)raw_mm / 10.0f;

    // Apply ToF Oblique Angle Reflectance & FoV Elongation Correction Factor C(theta)
    // C(theta) = 1.0 - 0.160 * sin^2(theta)
    // Corrects ToF distance stretching at steep tilt angles (e.g. 85.5cm raw -> 75.5cm true at 59 deg tilt)
    float rad = DEG_TO_RAD(fabsf(pitch_elev_deg));
    float sin_val = sinf(rad);
    float corr_factor = 1.0f - 0.160f * (sin_val * sin_val);
    if (corr_factor < 0.70f) corr_factor = 0.70f;

    dist_cm *= corr_factor;

    if (datum_mode == 0) { // REAR Datum (+rear_offset_cm)
        dist_cm += rear_offset_cm;
    }
    return dist_cm;
}

void Format_Distance_String(float dist_cm, uint8_t unit, char *out_str, size_t max_len)
{
    if (dist_cm < 0) {
        snprintf(out_str, max_len, "---");
        return;
    }

    switch (unit) {
        case 0: // CM
            snprintf(out_str, max_len, "%.1f CM", dist_cm);
            break;
        case 1: // MM
            snprintf(out_str, max_len, "%.0f MM", dist_cm * 10.0f);
            break;
        case 2: // M
            snprintf(out_str, max_len, "%.3f M", dist_cm / 100.0f);
            break;
        case 3: // INCH
            snprintf(out_str, max_len, "%.1f IN", dist_cm / 2.54f);
            break;
        default:
            snprintf(out_str, max_len, "%.1f CM", dist_cm);
            break;
    }
}

void Format_Area_String(float area_cm2, uint8_t unit, char *out_str, size_t max_len)
{
    if (area_cm2 < 0) {
        snprintf(out_str, max_len, " ---");
        return;
    }

    switch (unit) {
        case 0: // CM^2
            snprintf(out_str, max_len, "%.1f cm2", area_cm2);
            break;
        case 1: // MM^2
            snprintf(out_str, max_len, "%.0f mm2", area_cm2 * 100.0f);
            break;
        case 2: // M^2
            snprintf(out_str, max_len, "%.3f m2", area_cm2 / 10000.0f);
            break;
        case 3: // SQ FT
            snprintf(out_str, max_len, "%.2f sqft", area_cm2 / 929.0304f);
            break;
        default:
            snprintf(out_str, max_len, "%.1f cm2", area_cm2);
            break;
    }
}

void Format_Volume_String(float vol_cm3, uint8_t unit, char *out_str, size_t max_len)
{
    if (vol_cm3 < 0) {
        snprintf(out_str, max_len, " ---");
        return;
    }

    switch (unit) {
        case 0: // CM^3
            snprintf(out_str, max_len, "%.1f cm3", vol_cm3);
            break;
        case 1: // MM^3
            snprintf(out_str, max_len, "%.0f mm3", vol_cm3 * 1000.0f);
            break;
        case 2: // M^3
            snprintf(out_str, max_len, "%.4f m3", vol_cm3 / 1000000.0f);
            break;
        case 3: // CU FT
            snprintf(out_str, max_len, "%.3f cuft", vol_cm3 / 28316.846592f);
            break;
        default:
            snprintf(out_str, max_len, "%.1f cm3", vol_cm3);
            break;
    }
}

void Add_To_History(float dist_cm)
{
    if (dist_cm < 0) return;

    // Shift history entries right
    for (int i = 9; i > 0; i--) {
        history_buffer[i] = history_buffer[i - 1];
    }
    history_buffer[0] = dist_cm;
    if (history_count < 10) history_count++;
    history_view_idx = 0;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC3_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_USART3_UART_Init();
  MX_USB_Device_Init();

  /* USER CODE BEGIN 2 */
  // 1. Initial State: Keep CAT4002A EN (PA4) LOW (Laser OFF)
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  // 2. Calibrate ADC3 for accurate battery voltage readings on PB1
  HAL_ADCEx_Calibration_Start(&hadc3, ADC_SINGLE_ENDED);

  // 3. Initialize Rotary Encoder (SW: PB14, A: PB12, B: PB13)
  Encoder_Init(&encoder);

  // 4. Hardware Reset & Boot Pulse for VL53L4CX via PC11 (XSHUT)
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET); // Hold XSHUT low
  HAL_Delay(50);                                         // Hold reset low
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET);   // Drive XSHUT high
  HAL_Delay(100);                                        // Boot delay for ToF internal MCU initialization

  // 5. Allow USB CDC port to enumerate on PC terminal
  HAL_Delay(1500);

  printf("\r\n===============================================\r\n");
  printf("  STM32G491 MEASURE METER DEMO   \r\n");
  printf("===============================================\r\n");

  // 6. Initialize OLED Display (I2C2 on PA8/PA9)
  printf("[INIT] Initializing 1.3\" OLED Display on I2C2 (PA8/PA9)...\r\n");
  bool oled_ok = OLED_Init(&oled_dev, &hi2c2);
  if (oled_ok) {
      printf(" -> [OK] 1.3\" OLED Display Initialized (Addr: 0x3C)\r\n");
      // Phase 1: Sweep line animation
      for (int sweep = 0; sweep < 128; sweep += 16) {
          OLED_Clear(&oled_dev);
          OLED_FillRect(&oled_dev, 0, 0, sweep + 16, 64, OLED_COLOR_WHITE);
          OLED_UpdateScreen(&oled_dev);
          HAL_Delay(25);
      }
      // Phase 2: Animated Digital Measuring Tape & Laser Splash Screen
      OLED_Clear(&oled_dev);

      // 1. Title Banner
      OLED_DrawStringLarge(&oled_dev, 10, 2, "DIGI-TAPE", OLED_COLOR_WHITE);

      // 2. Digital Measuring Tape Ruler Ticks (Y: 18..28)
      OLED_DrawRect(&oled_dev, 4, 18, 120, 10, OLED_COLOR_WHITE);
      for (uint8_t tick_x = 8; tick_x <= 116; tick_x += 4) {
          uint8_t tick_h = ((tick_x - 8) % 16 == 0) ? 6 : 3;
          OLED_DrawLine(&oled_dev, tick_x, 18, tick_x, 18 + tick_h, OLED_COLOR_WHITE);
      }

      // 3. Laser Meter Body & Beam Animation (Y: 34..48)
      OLED_DrawRect(&oled_dev, 8, 34, 18, 14, OLED_COLOR_WHITE);
      OLED_DrawCircle(&oled_dev, 22, 41, 2, OLED_COLOR_WHITE);
      OLED_DrawRect(&oled_dev, 104, 33, 4, 16, OLED_COLOR_WHITE); // Target Wall

      for (uint8_t bx = 26; bx <= 100; bx += 6) {
          OLED_DrawLine(&oled_dev, 26, 41, bx, 41, OLED_COLOR_WHITE);
          OLED_DrawPixel(&oled_dev, bx + 2, 41, OLED_COLOR_WHITE);
          OLED_UpdateScreen(&oled_dev);
          HAL_Delay(35);
      }

      // 4. Impact Flash & Subtitle Banner
      OLED_DrawPixel(&oled_dev, 102, 39, OLED_COLOR_WHITE);
      OLED_DrawPixel(&oled_dev, 102, 43, OLED_COLOR_WHITE);
      OLED_DrawStringSmall(&oled_dev, 10, 53, "LOADING..." , OLED_COLOR_WHITE);
      OLED_UpdateScreen(&oled_dev);
      HAL_Delay(1200);
      boot_complete = true;
  } else {
      printf(" -> [WARN] OLED Display Not Detected on I2C2\r\n");
  }

  // 7. Initialize SCL3300 Inclinometer (SPI1)
  printf("[INIT] Initializing SCL3300 SPI1 (CS: PA3)...\r\n");
  bool scl_ok = SCL3300_Init(&scl_dev, &hspi1, GPIOA, GPIO_PIN_3, 4);
  if (scl_ok) {
      printf(" -> [OK] SCL3300 Initialized (WHOAMI: 0x%02X)\r\n", scl_dev.whoami);
  } else {
      printf(" -> [WARN] SCL3300 Init Warning (WHOAMI: 0x%02X)\r\n", scl_dev.whoami);
  }

  // 8. Initialize VL53L4CX Distance Sensor (I2C1)
  p_vl53->I2cHandle = &hi2c1;
  p_vl53->I2cDevAddr = 0x52; // 8-bit I2C Address

  printf("[INIT] Initializing VL53L4CX I2C1 (SCL: PA15, SDA: PB7)...\r\n");
  int vl53_status = VL53LX_WaitDeviceBooted(p_vl53);
  if (vl53_status == VL53LX_ERROR_NONE) {
      vl53_status = VL53LX_DataInit(p_vl53);
      if (vl53_status == VL53LX_ERROR_NONE) {
          // Configure ST internal APIs for 6.0m Maximum Range, High Accuracy & Crosstalk Cancellation
          VL53LX_SetDistanceMode(p_vl53, VL53LX_DISTANCEMODE_LONG);
          VL53LX_SetMeasurementTimingBudgetMicroSeconds(p_vl53, 200000); // 200ms integration time for max long-range sensitivity
          VL53LX_SetXTalkCompensationEnable(p_vl53, 1);                  // Enable Cover Glass Crosstalk Cancellation!

          vl53_status = VL53LX_StartMeasurement(p_vl53);
          if (vl53_status == VL53LX_ERROR_NONE) {
              printf(" -> [OK] VL53L4CX Active (LONG Mode, 200ms Budget, Crosstalk Comp ENABLED, 6m Max Range)\r\n");
          } else {
              printf(" -> [ERROR] VL53LX_StartMeasurement failed: %d\r\n", vl53_status);
          }
      } else {
          printf(" -> [ERROR] VL53LX_DataInit failed: %d\r\n", vl53_status);
      }
  } else {
      printf(" -> [ERROR] VL53LX_WaitDeviceBooted failed: %d\r\n", vl53_status);
  }

  // 9. Load Non-Volatile User Settings (Unit, Datum, Rear Offset) from STM32 Flash Page 255
  Load_Settings_From_Flash();
  last_activity_tick = HAL_GetTick();

  printf("===============================================\r\n");
  printf(" System Ready! Rotate Encoder / Press Button... \r\n");
  printf("===============================================\r\n\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  VL53LX_MultiRangingData_t ranging_data;
  uint8_t vl53_ready = 0;
  uint32_t sample_count = 0;
  uint32_t last_ui_tick = HAL_GetTick();

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t now = HAL_GetTick();

    // --- 1. Continuous Non-Blocking Polling of Rotary Encoder (SW: PB14, A: PB12, B: PB13) ---
    Encoder_Update(&encoder);

    // --- 2. Non-Blocking 33 Hz UI & Telemetry Timer (30 ms) ---
    if (now - last_ui_tick >= 30) {
        last_ui_tick = now;
        sample_count++;

        if (HAL_GetTick() - blink_tick >= 500) {
            blink_tick = HAL_GetTick();
            blink_on = !blink_on;
        }

        // --- Handle Long Press (> 800ms): Toggle Settings Menu Overlay ---
        if (encoder.long_press) {
            encoder.long_press = false;
            if (app_state == APP_STATE_SETTINGS || in_settings) {
                app_state = APP_STATE_MENU;
                in_settings = false;
            } else {
                app_state = APP_STATE_SETTINGS;
                in_settings = true;
            }
            printf("\r\n>>> [LONG PRESS] Settings Menu %s <<<\r\n\r\n", (app_state == APP_STATE_SETTINGS) ? "ENTERED" : "EXITED");
        }

        // --- Read Live Ranging Distance ---
        int live_raw_mm = -1;
        if (vl53_status == VL53LX_ERROR_NONE) {
            if (VL53LX_GetMeasurementDataReady(p_vl53, &vl53_ready) == VL53LX_ERROR_NONE && vl53_ready != 0) {
                if (VL53LX_GetMultiRangingData(p_vl53, &ranging_data) == VL53LX_ERROR_NONE) {
                    int target_mm = Process_ST_MultiTarget_Ranging(&ranging_data);
                    if (target_mm > 0) {
                        live_raw_mm = Filter_ToF_Distance_MM(target_mm);
                        if (app_state == APP_STATE_MEASURE && !hold_active) {
                            last_activity_tick = now; // Active ranging continuously resets sleep timer!
                        }
                    }
                    VL53LX_ClearInterruptAndStartMeasurement(p_vl53);
                }
            }
        }

        if (!hold_active && live_raw_mm >= 0) {
            hold_distance_mm = live_raw_mm;
            hold_pitch_elev  = laser_pitch_elev;
        }

        float active_net_cm = Calculate_Net_Distance_CM(hold_distance_mm, hold_pitch_elev);

        // Update MAX / MIN Continuous Ranging Tracker in MAXMIN Mode (selected_mode == 6)
        if (app_state == APP_STATE_MEASURE && selected_mode == 6 && active_net_cm > 0.0f) {
            if (active_net_cm < min_dist_cm) min_dist_cm = active_net_cm;
            if (active_net_cm > max_dist_cm) max_dist_cm = active_net_cm;
        }

        // --- Handle Press + Twist Chorded Gesture (Switch Held + Encoder Twist -> Global BACK to Main Menu) ---
        if (encoder.press_twist) {
            encoder.press_twist = false;
            app_state = APP_STATE_MENU;
            in_settings = false;
            hold_active = false;
            Reset_Multi_Shot();
            printf("\r\n>>> [PRESS + TWIST GESTURE: GO BACK TO MAIN MENU] <<<\r\n\r\n");
        }

        // --- Handle Switch Presses (Single Press: Select/Action | Double Press: Global BACK to Menu) ---
        if (encoder.short_press) {
            encoder.short_press = false;

            if (HAL_GetTick() - last_short_press_tick < DOUBLE_PRESS_WINDOW_MS) {
                // --- DOUBLE PRESS ---
                last_short_press_tick = 0;
                if (in_settings_edit) {
                    // Double press in Edit Mode: Confirm value, save to Flash, and exit Edit Mode back to Settings menu
                    in_settings_edit = false;
                    Save_Settings_To_Flash();
                    printf("\r\n>>> [DOUBLE PRESS] Confirmed & Exited Edit Mode to Settings Menu <<<\r\n\r\n");
                } else {
                    // Double press in Settings / Measure Mode: Return to Main Menu
                    app_state = APP_STATE_MENU;
                    in_settings = false;
                    in_settings_edit = false;
                    hold_active = false;
                    Reset_Multi_Shot();
                    printf("\r\n>>> [DOUBLE PRESS: GO BACK TO MAIN MENU] <<<\r\n\r\n");
                }
            } else {
                last_short_press_tick = HAL_GetTick();

                // --- SINGLE PRESS: CONFIRMATION / SELECTION / MODE ACTION ---
                if (app_state == APP_STATE_MENU) {
                    app_state = APP_STATE_MEASURE;
                    hold_active = false;
                    Reset_Multi_Shot();
                    printf("\r\n>>> [SINGLE PRESS] Confirmed & Entered Mode %u <<<\r\n\r\n", selected_mode);
                } else if (app_state == APP_STATE_SETTINGS || in_settings) {
                    if (!in_settings_edit) {
                        // Click on parameter item -> Enter EDIT MODE!
                        in_settings_edit = true;
                        printf("\r\n>>> [SETTINGS] Entered EDIT MODE for Item %u <<<\r\n\r\n", settings_item);
                    } else {
                        // Click while in EDIT MODE -> Confirm value, Save to Flash, & exit EDIT MODE!
                        in_settings_edit = false;
                        Save_Settings_To_Flash(); // Save settings immediately to NVM Flash Page 255
                        printf("\r\n>>> [SETTINGS] Confirmed & Saved Item %u (Unit:%u | Datum:%u | Offset:%.1fcm) <<<\r\n\r\n",
                               settings_item, unit_mode, datum_mode, rear_offset_cm);
                    }
                } else if (app_state == APP_STATE_MEASURE) {
                    if (selected_mode == 0 || selected_mode == 2) { // DIST or HEIGHT
                        hold_active = !hold_active;
                        if (hold_active) {
                            Add_To_History(active_net_cm);
                            printf("\r\n>>> [HOLD & SAVE] Measurement Frozen: %.1f CM <<<\r\n\r\n", active_net_cm);
                        } else {
                            printf("\r\n>>> [UNHOLD] Resumed Live Ranging <<<\r\n\r\n");
                        }
                    } else if (selected_mode == 6) { // MAXMIN
                        min_dist_cm = 9999.0f;
                        max_dist_cm = 0.0f;
                        printf("\r\n>>> [MAX/MIN RESET] Reset Min/Max Trackers <<<\r\n\r\n");
                    } else if (selected_mode == 3) { // AREA MODE MULTI-SHOT
                        if (multi_shot_step == 0) {
                            shot1_cm = active_net_cm;
                            multi_shot_step = 1;
                            printf("\r\n>>> [AREA SHOT 1] Length: %.1f CM <<<\r\n\r\n", shot1_cm);
                        } else if (multi_shot_step == 1) {
                            shot2_cm = active_net_cm;
                            multi_shot_step = 2;
                            Add_To_History(shot1_cm * shot2_cm / 100.0f);
                            printf("\r\n>>> [AREA SHOT 2] Width: %.1f CM | Area: %.2f cm2 <<<\r\n\r\n", shot2_cm, shot1_cm * shot2_cm);
                        } else {
                            Reset_Multi_Shot();
                        }
                    } else if (selected_mode == 4) { // VOLUME MODE MULTI-SHOT
                        if (multi_shot_step == 0) {
                            shot1_cm = active_net_cm;
                            multi_shot_step = 1;
                        } else if (multi_shot_step == 1) {
                            shot2_cm = active_net_cm;
                            multi_shot_step = 2;
                        } else if (multi_shot_step == 2) {
                            shot3_cm = active_net_cm;
                            multi_shot_step = 3;
                            printf("\r\n>>> [VOLUME RESULT] L:%.1f W:%.1f H:%.1f | Vol:%.1f cm3 <<<\r\n\r\n",
                                   shot1_cm, shot2_cm, shot3_cm, shot1_cm * shot2_cm * shot3_cm);
                        } else {
                            Reset_Multi_Shot();
                        }
                    } else if (selected_mode == 5) { // CYLINDER MODE MULTI-SHOT
                        if (multi_shot_step == 0) {
                            shot1_cm = active_net_cm;
                            multi_shot_step = 1;
                        } else if (multi_shot_step == 1) {
                            shot2_cm = active_net_cm;
                            multi_shot_step = 2;
                            float radius = shot1_cm / 2.0f;
                            float area = M_PI_F * radius * radius;
                            float vol = area * shot2_cm;
                            printf("\r\n>>> [CYLINDER RESULT] D:%.1f H:%.1f | Area:%.1f Vol:%.1f <<<\r\n\r\n",
                                   shot1_cm, shot2_cm, area, vol);
                        } else {
                            Reset_Multi_Shot();
                        }
                    }
                }
            }
        }

        // --- 4. Read SCL3300 Inclinometer & Motion Data ---
        bool scl_valid = SCL3300_ReadData(&scl_dev);
        if (scl_valid) {
            laser_pitch_elev = -scl_dev.angle_x_deg;
            side_roll        =  scl_dev.angle_y_deg;

            // Motion Acceleration Delta: Detect if user moved or picked up device
            float delta_acc = fabsf(scl_dev.acc_x_g - prev_acc_x) +
                              fabsf(scl_dev.acc_y_g - prev_acc_y) +
                              fabsf(scl_dev.acc_z_g - prev_acc_z);
            prev_acc_x = scl_dev.acc_x_g;
            prev_acc_y = scl_dev.acc_y_g;
            prev_acc_z = scl_dev.acc_z_g;

            if (delta_acc > 0.03f) { // Sensitive movement detection resets activity timer
                last_activity_tick = now;
                if (device_sleeping) {
                    device_sleeping = false;
                    OLED_DisplayOn(&oled_dev);
                    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_SET); // Hardware XSHUT Power ON (PC11)
                    if (p_vl53) VL53LX_StartMeasurement(p_vl53);
                    printf(" -> [WAKEUP] Motion Pick-up (Delta G: %.3fg)! Display & ToF Active.\r\n", delta_acc);
                }
            }
        }

        // --- 5. Inactivity Timeout Check (3 Minutes Auto-Sleep) ---
        if (!device_sleeping && (now - last_activity_tick >= AUTO_SLEEP_TIMEOUT_MS)) {
            device_sleeping = true;
            laser_active = false;
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);  // Laser OFF
            if (p_vl53) VL53LX_StopMeasurement(p_vl53);            // ToF Low-Power Sleep
            HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET); // Hardware XSHUT Zero-Current Shutdown (PC11)
            OLED_DisplayOff(&oled_dev);                           // OLED Sleep (Display OFF)
            printf(" -> [AUTO-SLEEP] 3-Min Inactivity. Low-Power Hardware Sleep Active.\r\n");
        }

        // Sleep Idle Loop when sleeping to conserve maximum battery power
        if (device_sleeping) {
            HAL_Delay(50);
            continue;
        }

        // Laser auto-ON in active measurement modes (DIST, HEIGHT, AREA, VOL, CYL, MAXMIN)
        bool should_laser = (app_state == APP_STATE_MEASURE &&
                            (selected_mode == 0 || selected_mode == 2 || selected_mode == 3 || selected_mode == 4 || selected_mode == 5 || selected_mode == 6) &&
                            !hold_active);
        if (should_laser != laser_active) {
            laser_active = should_laser;
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, laser_active ? GPIO_PIN_SET : GPIO_PIN_RESET);
        }

        // --- 6. Read 1S Li-Ion Battery Voltage & Percentage via ADC3 (PB1) ---
        float v_bat = Read_Battery_Voltage();
        uint8_t bat_pct = Calculate_Battery_Percentage(v_bat);

        // --- 5. Update 1.3\" OLED Display ---
        if (oled_ok) {
            OLED_Clear(&oled_dev);

            if (app_state == APP_STATE_MENU) {
                // --- ANIMATED FITNESS BAND CAROUSEL MENU SCREEN ---
                // Interpolate sliding animation offset toward 0
                if (carousel_anim_x > 0) {
                    carousel_anim_x -= 9;
                    if (carousel_anim_x < 0) carousel_anim_x = 0;
                } else if (carousel_anim_x < 0) {
                    carousel_anim_x += 9;
                    if (carousel_anim_x > 0) carousel_anim_x = 0;
                }

                // 1. Header Bar (Y: 0..11)
                OLED_FillRect(&oled_dev, 0, 0, 128, 12, OLED_COLOR_WHITE);
                OLED_DrawStringSmall(&oled_dev, 2, 2, "MODE CAROUSEL", OLED_COLOR_BLACK);
                OLED_DrawBatteryIcon(&oled_dev, 110, 2, bat_pct, OLED_COLOR_BLACK);
                char bat_txt[6];
                snprintf(bat_txt, sizeof(bat_txt), "%d%%", bat_pct);
                OLED_DrawStringSmall(&oled_dev, 86, 2, bat_txt, OLED_COLOR_BLACK);

                // 2. 3D Carousel Cards (Y: 15..41)
                uint8_t prev_m = (selected_mode + 7) % 8;
                uint8_t next_m = (selected_mode + 1) % 8;

                // Left Side Preview Icon (Prev)
                int16_t prev_x = 8 + carousel_anim_x;
                if (prev_x > -20 && prev_x < 120) {
                    OLED_DrawRect(&oled_dev, prev_x, 18, 24, 22, OLED_COLOR_WHITE);
                    OLED_DrawCarouselModeIcon(&oled_dev, prev_x, 17, prev_m, OLED_COLOR_WHITE);
                }

                // Right Side Preview Icon (Next)
                int16_t next_x = 96 + carousel_anim_x;
                if (next_x > -20 && next_x < 120) {
                    OLED_DrawRect(&oled_dev, next_x, 18, 24, 22, OLED_COLOR_WHITE);
                    OLED_DrawCarouselModeIcon(&oled_dev, next_x, 17, next_m, OLED_COLOR_WHITE);
                }

                // Center Focused Main Card (Selected Mode)
                int16_t center_x = 48 + carousel_anim_x;
                OLED_FillRect(&oled_dev, center_x, 15, 32, 26, OLED_COLOR_WHITE);
                OLED_DrawCarouselModeIcon(&oled_dev, center_x + 2, 16, selected_mode, OLED_COLOR_BLACK);

                // 3. Centered Mode Title (Y: 44..52)
                const char *mode_titles[8] = {
                    "1. DISTANCE METER",
                    "2. SPIRIT LEVEL",
                    "3. HEIGHT (PYTH)",
                    "4. AREA CALCULATOR",
                    "5. VOLUME CALCULATOR",
                    "6. CYLINDER METER",
                    "7. MAX/MIN TRACKING",
                    "8. MEMORY LOG"
                };
                uint8_t title_len = strlen(mode_titles[selected_mode]);
                uint8_t title_x = (title_len * 6 < 126) ? ((128 - title_len * 6) / 2) : 2;
                OLED_DrawStringSmall(&oled_dev, title_x, 44, mode_titles[selected_mode], OLED_COLOR_WHITE);

                // 4. Fitness Band Carousel Pager Dots (Y: 57..62)
                uint8_t start_dots_x = (128 - (8 * 8 - 4)) / 2; // Center 8 dots
                for (uint8_t i = 0; i < 8; i++) {
                    uint8_t dot_x = start_dots_x + i * 8;
                    if (i == selected_mode) {
                        OLED_FillRect(&oled_dev, dot_x, 57, 5, 5, OLED_COLOR_WHITE); // Active dot
                    } else {
                        OLED_DrawPixel(&oled_dev, dot_x + 2, 59, OLED_COLOR_WHITE); // Inactive dot
                    }
                }
            } else if (app_state == APP_STATE_SETTINGS || in_settings) {
                // --- FULL SCREEN SETTINGS OVERLAY ---
                OLED_FillRect(&oled_dev, 0, 0, 128, 12, OLED_COLOR_WHITE);
                if (in_settings_edit) {
                    OLED_DrawStringSmall(&oled_dev, 2, 2, "SETTINGS [EDIT]", OLED_COLOR_BLACK);
                } else {
                    OLED_DrawStringSmall(&oled_dev, 2, 2, "SETTINGS", OLED_COLOR_BLACK);
                }
                OLED_DrawLine(&oled_dev, 0, 12, 127, 12, OLED_COLOR_WHITE);

                const char *unit_names[] = {"CM", "MM", "M", "INCH"};
                const char *datum_names[] = {"REAR", "FRONT"};

                for (int i = 0; i < 3; i++) {
                    int row_y = 16 + i * 16;
                    if (i == settings_item) {
                        OLED_FillRect(&oled_dev, 0, row_y, 128, 14, OLED_COLOR_WHITE);
                    }
                    uint8_t color = (i == settings_item) ? OLED_COLOR_BLACK : OLED_COLOR_WHITE;
                    
                    char val_str[16];
                    if (i == 0) {
                        if (i == settings_item && in_settings_edit) {
                            snprintf(val_str, sizeof(val_str), "< %s >", unit_names[unit_mode]);
                        } else {
                            snprintf(val_str, sizeof(val_str), "%s", unit_names[unit_mode]);
                        }
                    } else if (i == 1) {
                        if (i == settings_item && in_settings_edit) {
                            snprintf(val_str, sizeof(val_str), "< %s >", datum_names[datum_mode]);
                        } else {
                            snprintf(val_str, sizeof(val_str), "%s", datum_names[datum_mode]);
                        }
                    } else if (i == 2) {
                        if (i == settings_item && in_settings_edit) {
                            snprintf(val_str, sizeof(val_str), "<%.1fcm>", rear_offset_cm);
                        } else {
                            snprintf(val_str, sizeof(val_str), "%.1fcm", rear_offset_cm);
                        }
                    }

                    switch (i) {
                        case 0:
                            OLED_DrawStringSmall(&oled_dev, 4, row_y + 3, "UNIT", color);
                            OLED_DrawStringSmall(&oled_dev, 66, row_y + 3, val_str, color);
                            break;
                        case 1:
                            OLED_DrawStringSmall(&oled_dev, 4, row_y + 3, "DATUM", color);
                            OLED_DrawStringSmall(&oled_dev, 66, row_y + 3, val_str, color);
                            break;
                        case 2:
                            OLED_DrawStringSmall(&oled_dev, 4, row_y + 3, "OFFSET", color);
                            OLED_DrawStringSmall(&oled_dev, 66, row_y + 3, val_str, color);
                            break;
                    }
                }
            } else {
                // --- APP_STATE_MEASURE: ACTIVE MEASUREMENT DISPLAY ---
                uint8_t active_mode = selected_mode;

                if (active_mode == 1) {
                    OLED_DrawFullScreenBubble(&oled_dev, scl_dev.angle_x_deg, scl_dev.angle_y_deg, 
                                              scl_dev.angle_z_deg, scl_dev.temp_c);
                } else {
                    OLED_FillRect(&oled_dev, 0, 0, 128, 12, OLED_COLOR_WHITE);

                    const char *mode_names[8] = {"DIST", "LEVEL", "HEIGHT", "AREA", "VOLUME", "CYL", "MAXMIN", "MEMORY"};
                    OLED_DrawStringSmall(&oled_dev, 2, 2, mode_names[active_mode], OLED_COLOR_BLACK);

                    OLED_DrawBatteryIcon(&oled_dev, 110, 2, bat_pct, OLED_COLOR_BLACK);
                    char bat_txt[6];
                    snprintf(bat_txt, sizeof(bat_txt), "%d%%", bat_pct);
                    OLED_DrawStringSmall(&oled_dev, 86, 2, bat_txt, OLED_COLOR_BLACK);
                    OLED_DrawDatumIcon(&oled_dev, 50, 1, (datum_mode == 0), OLED_COLOR_BLACK);
                    OLED_DrawLaserIcon(&oled_dev, 62, 2, laser_active, OLED_COLOR_BLACK);
                    OLED_DrawLine(&oled_dev, 0, 12, 127, 12, OLED_COLOR_WHITE);

                    char prim_str[16];
                    char sec1[48] = {0};
                    char sec2[48] = {0};

                    if (active_mode == 0) {
                        // Draw visual horizontal (side roll) & vertical (pitch elevation) leveling bars
                        OLED_DrawLevelBars(&oled_dev, laser_pitch_elev, side_roll);
                        Format_Distance_String(active_net_cm, unit_mode, prim_str, sizeof(prim_str));
                    } else if (active_mode == 2) {
                        // HEIGHT Mode (Pythagoras indirect height measurement with 2-point empirical linear calibration)
                        float active_pitch = hold_active ? hold_pitch_elev : laser_pitch_elev;
                        float rad = DEG_TO_RAD(active_pitch);
                        float indirect_height_cm = 0.0f;
                        if (active_net_cm >= 0.0f) {
                            float raw_h = active_net_cm * sinf(rad);
                            if (datum_mode == 0) { // REAR Datum (device base resting on reference surface)
                                float uncal_h = raw_h - TOF_VERTICAL_OFFSET_CM * cosf(rad);
                                indirect_height_cm = (uncal_h * HEIGHT_GAIN_SCALE) + HEIGHT_BIAS_OFFSET;
                                if (indirect_height_cm < 0.0f) indirect_height_cm = 0.0f;
                            } else { // FRONT Datum
                                indirect_height_cm = (raw_h * HEIGHT_GAIN_SCALE) + HEIGHT_BIAS_OFFSET;
                                if (indirect_height_cm < 0.0f) indirect_height_cm = 0.0f;
                            }
                        }
                        char hyp_str[16];
                        Format_Distance_String(active_net_cm, unit_mode, hyp_str, sizeof(hyp_str));
                        Format_Distance_String(indirect_height_cm, unit_mode, prim_str, sizeof(prim_str));
                        snprintf(sec1, sizeof(sec1), "HYP: %s", hyp_str);
                        snprintf(sec2, sizeof(sec2), "ANG: %5.1f deg", active_pitch);
                        OLED_DrawStringSmall(&oled_dev, 2, 14, sec1, OLED_COLOR_WHITE);
                        OLED_DrawStringSmall(&oled_dev, 2, 24, sec2, OLED_COLOR_WHITE);
                        OLED_DrawTriangleIcon(&oled_dev, 110, 14);
                    } else if (active_mode == 6) {
                        char min_str[16], max_str[16];
                        Format_Distance_String((min_dist_cm < 9990.0f) ? min_dist_cm : -1.0f, unit_mode, min_str, sizeof(min_str));
                        Format_Distance_String((max_dist_cm > 0.0f) ? max_dist_cm : -1.0f, unit_mode, max_str, sizeof(max_str));
                        snprintf(sec1, sizeof(sec1), "MIN: %s", min_str);
                        snprintf(sec2, sizeof(sec2), "MAX: %s", max_str);
                        Format_Distance_String(active_net_cm, unit_mode, prim_str, sizeof(prim_str));
                        OLED_DrawStringSmall(&oled_dev, 2, 14, sec1, OLED_COLOR_WHITE);
                        OLED_DrawStringSmall(&oled_dev, 2, 24, sec2, OLED_COLOR_WHITE);
                    } else if (active_mode == 3) {
                        char l_str[16], w_str[16];
                        Format_Distance_String(shot1_cm, unit_mode, l_str, sizeof(l_str));
                        Format_Distance_String(shot2_cm, unit_mode, w_str, sizeof(w_str));
                        char live_str[16];
                        Format_Distance_String(active_net_cm, unit_mode, live_str, sizeof(live_str));
                        
                        if (multi_shot_step == 0) {
                            snprintf(sec1, sizeof(sec1), "L: ---");
                            OLED_DrawRectIcon(&oled_dev, 100, 14, 0, blink_on);
                            snprintf(prim_str, sizeof(prim_str), "%s", live_str);
                        } else if (multi_shot_step == 1) {
                            snprintf(sec1, sizeof(sec1), "L: %s", l_str);
                            OLED_DrawRectIcon(&oled_dev, 100, 14, 1, blink_on);
                            snprintf(prim_str, sizeof(prim_str), "%s", live_str);
                        } else {
                            snprintf(sec1, sizeof(sec1), "L: %s", l_str);
                            snprintf(sec2, sizeof(sec2), "W: %s", w_str);
                            OLED_DrawRectIcon(&oled_dev, 100, 14, 2, false);
                            float area_cm2 = shot1_cm * shot2_cm;
                            Format_Area_String(area_cm2, unit_mode, prim_str, sizeof(prim_str));
                        }
                        OLED_DrawStringSmall(&oled_dev, 2, 14, sec1, OLED_COLOR_WHITE);
                        if (sec2[0]) OLED_DrawStringSmall(&oled_dev, 2, 24, sec2, OLED_COLOR_WHITE);
                    } else if (active_mode == 4) {
                        char l_str[16], w_str[16], h_str[16];
                        Format_Distance_String(shot1_cm, unit_mode, l_str, sizeof(l_str));
                        Format_Distance_String(shot2_cm, unit_mode, w_str, sizeof(w_str));
                        Format_Distance_String(shot3_cm, unit_mode, h_str, sizeof(h_str));
                        char live_str[16];
                        Format_Distance_String(active_net_cm, unit_mode, live_str, sizeof(live_str));

                        if (multi_shot_step == 0) {
                            snprintf(sec1, sizeof(sec1), "L: ---");
                            OLED_DrawCubeIcon(&oled_dev, 100, 14, 0, blink_on);
                            snprintf(prim_str, sizeof(prim_str), "%s", live_str);
                        } else if (multi_shot_step == 1) {
                            snprintf(sec1, sizeof(sec1), "L: %s", l_str);
                            OLED_DrawCubeIcon(&oled_dev, 100, 14, 1, blink_on);
                            snprintf(prim_str, sizeof(prim_str), "%s", live_str);
                        } else if (multi_shot_step == 2) {
                            snprintf(sec1, sizeof(sec1), "L:%s W:%s", l_str, w_str);
                            snprintf(sec2, sizeof(sec2), "H: ---");
                            OLED_DrawCubeIcon(&oled_dev, 100, 14, 2, blink_on);
                            snprintf(prim_str, sizeof(prim_str), "%s", live_str);
                        } else {
                            snprintf(sec1, sizeof(sec1), "L:%s W:%s", l_str, w_str);
                            snprintf(sec2, sizeof(sec2), "H: %s", h_str);
                            OLED_DrawCubeIcon(&oled_dev, 100, 14, 3, false);
                            float vol_cm3 = shot1_cm * shot2_cm * shot3_cm;
                            Format_Volume_String(vol_cm3, unit_mode, prim_str, sizeof(prim_str));
                        }
                        OLED_DrawStringSmall(&oled_dev, 2, 14, sec1, OLED_COLOR_WHITE);
                        if (sec2[0]) OLED_DrawStringSmall(&oled_dev, 2, 24, sec2, OLED_COLOR_WHITE);
                    } else if (active_mode == 5) {
                        char d_str[16], h_str[16];
                        Format_Distance_String(shot1_cm, unit_mode, d_str, sizeof(d_str));
                        Format_Distance_String(shot2_cm, unit_mode, h_str, sizeof(h_str));
                        char live_str[16];
                        Format_Distance_String(active_net_cm, unit_mode, live_str, sizeof(live_str));

                        if (multi_shot_step == 0) {
                            snprintf(sec1, sizeof(sec1), "D: ---");
                            OLED_DrawCylinderIcon(&oled_dev, 100, 14, 0, blink_on);
                            snprintf(prim_str, sizeof(prim_str), "%s", live_str);
                        } else if (multi_shot_step == 1) {
                            snprintf(sec1, sizeof(sec1), "D: %s", d_str);
                            OLED_DrawCylinderIcon(&oled_dev, 100, 14, 1, blink_on);
                            snprintf(prim_str, sizeof(prim_str), "%s", live_str);
                        } else {
                            snprintf(sec1, sizeof(sec1), "D: %s", d_str);
                            snprintf(sec2, sizeof(sec2), "H: %s", h_str);
                            OLED_DrawCylinderIcon(&oled_dev, 100, 14, 2, false);
                            float r = shot1_cm / 2.0f;
                            float vol_cm3 = M_PI_F * r * r * shot2_cm;
                            Format_Volume_String(vol_cm3, unit_mode, prim_str, sizeof(prim_str));
                        }
                        OLED_DrawStringSmall(&oled_dev, 2, 14, sec1, OLED_COLOR_WHITE);
                        if (sec2[0]) OLED_DrawStringSmall(&oled_dev, 2, 24, sec2, OLED_COLOR_WHITE);
                    } else if (active_mode == 7) {
                        if (history_count == 0) {
                            snprintf(sec1, sizeof(sec1), "NO SAVED RECORDS");
                            snprintf(prim_str, sizeof(prim_str), " ---");
                        } else {
                            snprintf(sec1, sizeof(sec1), "RECORD #%02d / %02d", history_view_idx + 1, history_count);
                            Format_Distance_String(history_buffer[history_view_idx], unit_mode, prim_str, sizeof(prim_str));
                        }
                        OLED_DrawStringSmall(&oled_dev, 2, 14, sec1, OLED_COLOR_WHITE);
                    }

                    if (active_mode != 0) {
                        OLED_DrawLine(&oled_dev, 0, 34, 127, 34, OLED_COLOR_WHITE);
                    }
                    // Primary reading zone: Maximized 7-segment digits (30px height, 4px thick) with compact unit badge
                    if (active_mode == 0) {
                        uint16_t str_w = 0;
                        const char *p = prim_str;
                        while (*p) {
                            if ((*p >= '0' && *p <= '9') || *p == '-') str_w += 18;
                            else if (*p == '.') str_w += 7;
                            else if (*p == ' ') str_w += 4;
                            else { str_w += strlen(p) * 6; break; }
                            p++;
                        }
                        uint8_t prim_x = (str_w < 118) ? ((118 - str_w) / 2 + 1) : 1;
                        OLED_Draw7SegmentString(&oled_dev, prim_x, 26, prim_str, 15, 30, 3, OLED_COLOR_WHITE);
                    } else {
                        uint16_t str_w = 0;
                        const char *p = prim_str;
                        while (*p) {
                            if ((*p >= '0' && *p <= '9') || *p == '-') str_w += 16;
                            else if (*p == '.') str_w += 6;
                            else if (*p == ' ') str_w += 4;
                            else { str_w += strlen(p) * 6; break; }
                            p++;
                        }
                        uint8_t prim_x = (str_w < 124) ? (126 - str_w) : 1;
                        OLED_Draw7SegmentString(&oled_dev, prim_x, 38, prim_str, 13, 22, 3, OLED_COLOR_WHITE);
                    }
                }
            }

            OLED_UpdateScreen(&oled_dev);
        }

        // --- 7. Output Telemetry via USB CDC ---
        printf("#%05lu | [STATE:%d MODE:%u] | [DATUM:%s] | [BAT:%.2fV(%3d%%)] | [LASER:%s] | [SCL3300:%s] E:%6.2f R:%6.2f | [VL53] D:%.1fcm (R:%dmm)\r\n",
               (unsigned long)sample_count,
               (int)app_state, selected_mode,
               (datum_mode == 0) ? "REAR" : "FRONT",
               v_bat, bat_pct,
               laser_active ? "ON " : "OFF",
               scl_valid ? "OK " : "ERR",
               laser_pitch_elev, side_roll,
               active_net_cm, hold_distance_mm);
    }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Common config
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.GainCompensation = 0;
  hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc3.Init.LowPowerAutoWait = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc3.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel - Set 640.5 cycles for 50k source impedance
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x20303E5D;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x20303E5D;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32; // Safe clock division for SCL3300
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3 | GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA3 (SCL3300 CS Pin - Output High) */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);

  /*Configure GPIO pin : PA4 (CAT4002A EN/DIM Laser Driver Control) */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 (Encoder A), PB13 (Encoder B), PB14 (Encoder SW) with Internal Pull-Ups */
  GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC11 (VL53L4CX XSHUT Pin) */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// Read 1S battery voltage via ADC3 Channel 1 (PB1) with 16-sample averaging
// Filtered Battery Voltage & Percentage with 32-sample trimmed mean, IIR Low-Pass Filter, and Hysteresis
static float static_filtered_vbat = -1.0f;
static uint8_t static_displayed_pct = 255;
static uint32_t last_bat_sample_tick = 0;

float Read_Battery_Voltage(void)
{
    uint32_t now = HAL_GetTick();

    // Sample battery every 500ms to avoid sampling overhead on every frame
    if (now - last_bat_sample_tick < 500 && static_filtered_vbat > 0.0f) {
        return static_filtered_vbat;
    }
    last_bat_sample_tick = now;

    // 1. Take 32 ADC samples
    uint16_t samples_buf[32];
    uint8_t count = 0;

    for (uint8_t i = 0; i < 32; i++) {
        HAL_ADC_Start(&hadc3);
        if (HAL_ADC_PollForConversion(&hadc3, 5) == HAL_OK) {
            samples_buf[count++] = (uint16_t)HAL_ADC_GetValue(&hadc3);
        }
        HAL_ADC_Stop(&hadc3);
    }

    if (count == 0) return (static_filtered_vbat > 0.0f) ? static_filtered_vbat : 3.70f;

    // 2. Selection Sort to trim top 25% and bottom 25% outlier samples (laser pulse noise)
    for (uint8_t i = 0; i < count - 1; i++) {
        for (uint8_t j = i + 1; j < count; j++) {
            if (samples_buf[i] > samples_buf[j]) {
                uint16_t tmp = samples_buf[i];
                samples_buf[i] = samples_buf[j];
                samples_buf[j] = tmp;
            }
        }
    }

    // Average middle 50% samples
    uint8_t start_idx = count / 4;
    uint8_t end_idx = count - start_idx;
    uint32_t sum = 0;
    uint8_t valid_cnt = 0;
    for (uint8_t i = start_idx; i < end_idx; i++) {
        sum += samples_buf[i];
        valid_cnt++;
    }

    float raw_avg = (valid_cnt > 0) ? ((float)sum / (float)valid_cnt) : 2048.0f;

    // 12-bit ADC (0..4095) with 3.30V VREF & 1:1 Resistor Divider (V_BAT = V_ADC * 2.0)
    float v_adc = (raw_avg / 4095.0f) * 3.30f;
    float v_bat_inst = v_adc * 2.0f;

    // 3. Heavy IIR Low-Pass Filter (alpha = 0.08)
    if (static_filtered_vbat < 0.0f) {
        static_filtered_vbat = v_bat_inst; // Initial seed on boot
    } else {
        static_filtered_vbat = (0.08f * v_bat_inst) + (0.92f * static_filtered_vbat);
    }

    return static_filtered_vbat;
}

// Convert 1S Li-Ion battery voltage to percentage (4.20V = 100%, 3.30V = 0%) with 1% Hysteresis
uint8_t Calculate_Battery_Percentage(float v_bat)
{
    uint8_t raw_pct = 0;
    if (v_bat >= 4.20f) raw_pct = 100;
    else if (v_bat <= 3.30f) raw_pct = 0;
    else raw_pct = (uint8_t)(((v_bat - 3.30f) / (4.20f - 3.30f)) * 100.0f);

    // Initial seed
    if (static_displayed_pct == 255) {
        static_displayed_pct = raw_pct;
        return static_displayed_pct;
    }

    // 4. Hysteresis: Step smoothly by 1% to prevent rapid toggling
    if (raw_pct > static_displayed_pct && (raw_pct - static_displayed_pct) >= 1) {
        static_displayed_pct++;
    } else if (raw_pct < static_displayed_pct && (static_displayed_pct - raw_pct) >= 1) {
        static_displayed_pct--;
    }

    return static_displayed_pct;
}

// Redirect standard printf to USB CDC Transmit
int _write(int file, char *ptr, int len)
{
    CDC_Transmit_FS((uint8_t*)ptr, len);
    return len;
}

// --- STM32G4 Flash Settings Persistence (Page 255 @ 0x0807F800) ---
#define SETTINGS_FLASH_ADDR   0x0807F800UL
#define SETTINGS_MAGIC        0x55AA1234UL

void Save_Settings_To_Flash(void)
{
    HAL_FLASH_Unlock();

    // Erase Flash Page 255
    FLASH_EraseInitTypeDef erase_init;
    uint32_t page_error = 0;
    erase_init.TypeErase = FLASH_TYPEERASE_PAGES;
    erase_init.Banks     = FLASH_BANK_1;
    erase_init.Page      = 255;
    erase_init.NbPages   = 1;

    if (HAL_FLASHEx_Erase(&erase_init, &page_error) == HAL_OK) {
        // DoubleWord 1: Magic (32 bits) | Unit (8 bits) | Datum (8 bits)
        uint64_t dw1 = ((uint64_t)SETTINGS_MAGIC) | 
                       ((uint64_t)unit_mode << 32) | 
                       ((uint64_t)datum_mode << 40);

        // DoubleWord 2: float rear_offset_cm bitwise cast to uint32_t
        uint32_t offset_bits = 0;
        memcpy(&offset_bits, &rear_offset_cm, sizeof(float));
        uint64_t dw2 = ((uint64_t)offset_bits) | ((uint64_t)(unit_mode ^ datum_mode ^ 0xA5) << 32);

        HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, SETTINGS_FLASH_ADDR, dw1);
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, SETTINGS_FLASH_ADDR + 8, dw2);
        printf("\r\n>>> [NVM FLASH] Settings Saved! Unit:%u, Datum:%u, Offset:%.1fcm <<<\r\n\r\n",
               unit_mode, datum_mode, rear_offset_cm);
    }

    HAL_FLASH_Lock();
}

void Load_Settings_From_Flash(void)
{
    uint64_t dw1 = *(__IO uint64_t*)SETTINGS_FLASH_ADDR;
    uint64_t dw2 = *(__IO uint64_t*)(SETTINGS_FLASH_ADDR + 8);

    uint32_t magic = (uint32_t)(dw1 & 0xFFFFFFFFUL);
    if (magic == SETTINGS_MAGIC) {
        unit_mode  = (uint8_t)((dw1 >> 32) & 0xFF);
        datum_mode = (uint8_t)((dw1 >> 40) & 0xFF);

        uint32_t offset_bits = (uint32_t)(dw2 & 0xFFFFFFFFUL);
        memcpy(&rear_offset_cm, &offset_bits, sizeof(float));

        // Sanity bounds check
        if (unit_mode > 3) unit_mode = 0;
        if (datum_mode > 1) datum_mode = 0;
        if (rear_offset_cm < 0.0f || rear_offset_cm > 50.0f) rear_offset_cm = 5.5f;

        printf("\r\n>>> [NVM FLASH] Settings Loaded from Flash! Unit:%u, Datum:%u, Offset:%.1fcm <<<\r\n\r\n",
               unit_mode, datum_mode, rear_offset_cm);
    } else {
        printf("\r\n>>> [NVM FLASH] First Boot / No Saved Settings. Using Defaults. <<<\r\n\r\n");
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
