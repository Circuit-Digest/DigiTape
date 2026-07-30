/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body - Commercial Grade Digital Measurement Device
  *                   Modes: DIST, LEVEL, HEIGHT, AREA, VOLUME, CYLINDER, MAX/MIN, MEMORY
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
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
uint8_t  settings_item = 0;       // 0: Unit, 1: Datum, 2: Rear Offset

uint8_t  unit_mode = 0;           // 0: CM, 1: MM, 2: M, 3: INCH
uint8_t  datum_mode = 0;          // 0: REAR (bottom), 1: FRONT (top)
float    rear_offset_cm = 5.5f;   // 5.5 cm device body length offset when REAR datum selected

bool     laser_active = false;
uint32_t last_short_press_tick = 0; // For global double-press detection

float    laser_pitch_elev = 0.0f;   // Global Laser Pitch Elevation (-Angle X)
float    side_roll = 0.0f;          // Global Screen Side Roll (Angle Y)

bool     hold_active = false;
int      hold_distance_mm = -1;

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
static uint8_t Read_Battery_Percentage(void);
static float Convert_Distance_Unit(float dist_cm, uint8_t unit);
static void Format_Distance_String(float dist_cm, uint8_t unit, char *out_buf, size_t buf_len);
static void Save_History(float val_cm, uint8_t mode, uint8_t unit);
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

    if (dir != 0) {
        e->counter += dir;

        if (app_state == APP_STATE_SETTINGS || in_settings) {
            // Scroll through 3 Settings Items (0: Unit, 1: Datum, 2: Rear Offset)
            int item = (int)settings_item + dir;
            if (item < 0) item = 2;
            if (item > 2) item = 0;
            settings_item = (uint8_t)item;
        } else if (app_state == APP_STATE_MENU) {
            // Scroll through all 8 available modes in feature phone menu grid
            int m = (int)selected_mode + dir;
            if (m < 0) m = 7;
            if (m > 7) m = 0;
            selected_mode = (uint8_t)m;
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

    e->prev_a = curr_a;
    e->prev_b = curr_b;

    // 2. Read Switch Pin (SW = PB14) with Short-Press & Long-Press Detection
    uint8_t curr_sw = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14);

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
}

float Calculate_Net_Distance_CM(int raw_mm)
{
    if (raw_mm < 0) return -1.0f;
    float dist_cm = (float)raw_mm / 10.0f;

    // Apply ToF Oblique Angle Reflectance & FoV Elongation Correction Factor C(theta)
    // C(theta) = 1.0 - 0.160 * sin^2(theta)
    // Corrects ToF distance stretching at steep tilt angles (e.g. 85.5cm raw -> 75.5cm true at 59 deg tilt)
    float rad = DEG_TO_RAD(fabsf(laser_pitch_elev));
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
  printf("  STM32G491 PROFESSIONAL MEASURE METER DEMO   \r\n");
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
      // Phase 2: Brand reveal
      OLED_Clear(&oled_dev);
      OLED_DrawString3x(&oled_dev, 14, 2, "DIGI", OLED_COLOR_WHITE);
      OLED_DrawString3x(&oled_dev, 20, 34, "TAPE", OLED_COLOR_WHITE);
    //   OLED_DrawStringSmall(&oled_dev, 28, 48, "PRO METER V2.0", OLED_COLOR_WHITE);
    //   OLED_DrawLine(&oled_dev, 10, 57, 118, 57, OLED_COLOR_WHITE);
      OLED_UpdateScreen(&oled_dev);
      HAL_Delay(2000);
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
          vl53_status = VL53LX_StartMeasurement(p_vl53);
          if (vl53_status == VL53LX_ERROR_NONE) {
              printf(" -> [OK] VL53L4CX Measurement Started\r\n");
          } else {
              printf(" -> [ERROR] VL53LX_StartMeasurement failed: %d\r\n", vl53_status);
          }
      } else {
          printf(" -> [ERROR] VL53LX_DataInit failed: %d\r\n", vl53_status);
      }
  } else {
      printf(" -> [ERROR] VL53LX_WaitDeviceBooted failed: %d\r\n", vl53_status);
  }

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
    // --- 1. Continuous Non-Blocking Polling of Rotary Encoder (SW: PB14, A: PB12, B: PB13) ---
    Encoder_Update(&encoder);

    // --- 2. Non-Blocking 33 Hz UI & Telemetry Timer (30 ms) ---
    if (HAL_GetTick() - last_ui_tick >= 30) {
        last_ui_tick = HAL_GetTick();
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
        uint8_t num_objects = 0;
        if (vl53_status == VL53LX_ERROR_NONE) {
            if (VL53LX_GetMeasurementDataReady(p_vl53, &vl53_ready) == VL53LX_ERROR_NONE && vl53_ready != 0) {
                if (VL53LX_GetMultiRangingData(p_vl53, &ranging_data) == VL53LX_ERROR_NONE) {
                    num_objects = ranging_data.NumberOfObjectsFound;
                    if (num_objects > 0) {
                        live_raw_mm = ranging_data.RangeData[0].RangeMilliMeter;
                    }
                    VL53LX_ClearInterruptAndStartMeasurement(p_vl53);
                }
            }
        }

        if (!hold_active && live_raw_mm >= 0) {
            hold_distance_mm = live_raw_mm;
        }

        float active_net_cm = Calculate_Net_Distance_CM(hold_distance_mm);

        // Update MAX / MIN Continuous Ranging Tracker in MAXMIN Mode (selected_mode == 6)
        if (app_state == APP_STATE_MEASURE && selected_mode == 6 && active_net_cm > 0.0f) {
            if (active_net_cm < min_dist_cm) min_dist_cm = active_net_cm;
            if (active_net_cm > max_dist_cm) max_dist_cm = active_net_cm;
        }

        // --- Handle Switch Presses (Single Press: Select/Action | Double Press: Global BACK to Menu) ---
        if (encoder.short_press) {
            encoder.short_press = false;

            if (HAL_GetTick() - last_short_press_tick < DOUBLE_PRESS_WINDOW_MS) {
                // --- DOUBLE PRESS: GLOBAL BACK TO MAIN MENU SCREEN ---
                last_short_press_tick = 0;
                app_state = APP_STATE_MENU;
                in_settings = false;
                hold_active = false;
                Reset_Multi_Shot();
                printf("\r\n>>> [DOUBLE PRESS: GO BACK TO MAIN MENU] <<<\r\n\r\n");
            } else {
                last_short_press_tick = HAL_GetTick();

                // --- SINGLE PRESS: CONFIRMATION / SELECTION / MODE ACTION ---
                if (app_state == APP_STATE_MENU) {
                    app_state = APP_STATE_MEASURE;
                    hold_active = false;
                    Reset_Multi_Shot();
                    printf("\r\n>>> [SINGLE PRESS] Confirmed & Entered Mode %u <<<\r\n\r\n", selected_mode);
                } else if (app_state == APP_STATE_SETTINGS || in_settings) {
                    // Toggle Settings Values
                    if (settings_item == 0) { // Unit
                        unit_mode = (unit_mode + 1) % 4;
                    } else if (settings_item == 1) { // Datum
                        datum_mode = (datum_mode + 1) % 2;
                    } else if (settings_item == 2) { // Rear Offset Adjust
                        rear_offset_cm += 1.0f;
                        if (rear_offset_cm > 20.0f) rear_offset_cm = 0.0f;
                    }
                    printf("\r\n>>> [SETTINGS TOGGLE] Unit: %u | Datum: %u | Offset: %.1fcm <<<\r\n\r\n",
                           unit_mode, datum_mode, rear_offset_cm);
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

        // Laser auto-ON in active measurement modes (DIST, HEIGHT, AREA, VOL, CYL, MAXMIN)
        bool should_laser = (app_state == APP_STATE_MEASURE &&
                            (selected_mode == 0 || selected_mode == 2 || selected_mode == 3 || selected_mode == 4 || selected_mode == 5 || selected_mode == 6) &&
                            !hold_active);
        if (should_laser != laser_active) {
            laser_active = should_laser;
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, laser_active ? GPIO_PIN_SET : GPIO_PIN_RESET);
        }

        // --- 3. Read 1S Li-Ion Battery Voltage & Percentage via ADC3 (PB1) ---
        float v_bat = Read_Battery_Voltage();
        uint8_t bat_pct = Calculate_Battery_Percentage(v_bat);

        // --- 4. Read SCL3300 Inclinometer Data ---
        bool scl_valid = SCL3300_ReadData(&scl_dev);

        // Laser Elevation Pitch Angle: ToF laser points along -X axis -> Elev = -Angle X
        laser_pitch_elev = -scl_dev.angle_x_deg;
        side_roll        =  scl_dev.angle_y_deg;

        // --- 5. Update 1.3\" OLED Display ---
        if (oled_ok) {
            OLED_Clear(&oled_dev);

            if (app_state == APP_STATE_MENU) {
                // --- FEATURE PHONE MAIN MENU SCREEN (ALL 8 MODES IN SINGLE MENU GRID) ---
                OLED_FillRect(&oled_dev, 0, 0, 128, 12, OLED_COLOR_WHITE);
                OLED_DrawStringSmall(&oled_dev, 2, 2, "SELECT MODE", OLED_COLOR_BLACK);
                OLED_DrawBatteryIcon(&oled_dev, 110, 2, bat_pct, OLED_COLOR_BLACK);
                char bat_txt[6];
                snprintf(bat_txt, sizeof(bat_txt), "%d%%", bat_pct);
                OLED_DrawStringSmall(&oled_dev, 86, 2, bat_txt, OLED_COLOR_BLACK);

                // 4x2 Mode Grid (Y: 15..48) with graphical icons
                for (uint8_t i = 0; i < 8; i++) {
                    uint8_t row = i / 4;
                    uint8_t col = i % 4;
                    uint8_t x = 2 + col * 32;
                    uint8_t y = 15 + row * 18;

                    if (i == selected_mode) {
                        OLED_FillRect(&oled_dev, x, y, 30, 16, OLED_COLOR_WHITE);
                        OLED_DrawMenuModeIcon(&oled_dev, x + 7, y + 2, i, OLED_COLOR_BLACK);
                    } else {
                        OLED_DrawRect(&oled_dev, x, y, 30, 16, OLED_COLOR_WHITE);
                        OLED_DrawMenuModeIcon(&oled_dev, x + 7, y + 2, i, OLED_COLOR_WHITE);
                    }
                }

                // Bottom Title Banner (Y: 51..63)
                OLED_DrawLine(&oled_dev, 0, 50, 127, 50, OLED_COLOR_WHITE);
                const char *mode_titles[8] = {
                    "> 1. DISTANCE METER",
                    "> 2. SPIRIT LEVEL",
                    "> 3. HEIGHT (PYTH)",
                    "> 4. AREA CALCULATOR",
                    "> 5. VOLUME CALCULATOR",
                    "> 6. CYLINDER CALCULATOR",
                    "> 7. MAX/MIN TRACKING",
                    "> 8. MEMORY LOG"
                };
                OLED_DrawStringSmall(&oled_dev, 2, 53, mode_titles[selected_mode], OLED_COLOR_WHITE);
            } else if (app_state == APP_STATE_SETTINGS || in_settings) {
                // --- FULL SCREEN SETTINGS OVERLAY ---
                OLED_FillRect(&oled_dev, 0, 0, 128, 12, OLED_COLOR_WHITE);
                OLED_DrawStringSmall(&oled_dev, 2, 2, "SETTINGS", OLED_COLOR_BLACK);
                OLED_DrawLine(&oled_dev, 0, 12, 127, 12, OLED_COLOR_WHITE);

                const char *unit_names[] = {"CM", "MM", "M", "INCH"};
                const char *datum_names[] = {"REAR", "FRONT"};
                char offset_str[12];
                snprintf(offset_str, sizeof(offset_str), "%.1f cm", rear_offset_cm);

                for (int i = 0; i < 3; i++) {
                    int row_y = 16 + i * 16;
                    if (i == settings_item) {
                        OLED_FillRect(&oled_dev, 0, row_y, 128, 14, OLED_COLOR_WHITE);
                    }
                    uint8_t color = (i == settings_item) ? OLED_COLOR_BLACK : OLED_COLOR_WHITE;
                    
                    switch (i) {
                        case 0:
                            OLED_DrawStringSmall(&oled_dev, 4, row_y + 3, "UNIT", color);
                            OLED_DrawStringSmall(&oled_dev, 80, row_y + 3, unit_names[unit_mode], color);
                            break;
                        case 1:
                            OLED_DrawStringSmall(&oled_dev, 4, row_y + 3, "DATUM", color);
                            OLED_DrawStringSmall(&oled_dev, 80, row_y + 3, datum_names[datum_mode], color);
                            break;
                        case 2:
                            OLED_DrawStringSmall(&oled_dev, 4, row_y + 3, "OFFSET", color);
                            OLED_DrawStringSmall(&oled_dev, 80, row_y + 3, offset_str, color);
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
                    char sec1[32] = {0};
                    char sec2[32] = {0};

                    if (active_mode == 0) {
                        // Draw visual horizontal (side roll) & vertical (pitch elevation) leveling bars
                        OLED_DrawLevelBars(&oled_dev, laser_pitch_elev, side_roll);
                        Format_Distance_String(active_net_cm, unit_mode, prim_str, sizeof(prim_str));
                    } else if (active_mode == 2) {
                        // HEIGHT Mode (Pythagoras indirect height measurement with 1.8cm ToF vertical mounting offset compensation)
                        float rad = DEG_TO_RAD(laser_pitch_elev);
                        float indirect_height_cm = 0.0f;
                        if (active_net_cm >= 0.0f) {
                            float raw_h = active_net_cm * sinf(rad);
                            if (datum_mode == 0) { // REAR Datum (device base resting on reference surface)
                                // Subtract ToF 1.5cm vertical mounting offset to align height with device base
                                indirect_height_cm = raw_h - 1.5f * cosf(rad);
                                if (indirect_height_cm < 0.0f) indirect_height_cm = 0.0f;
                            } else { // FRONT Datum
                                indirect_height_cm = (raw_h >= 0.0f) ? raw_h : 0.0f;
                            }
                        }
                        char hyp_str[16];
                        Format_Distance_String(active_net_cm, unit_mode, hyp_str, sizeof(hyp_str));
                        Format_Distance_String(indirect_height_cm, unit_mode, prim_str, sizeof(prim_str));
                        snprintf(sec1, sizeof(sec1), "HYP: %s", hyp_str);
                        snprintf(sec2, sizeof(sec2), "ANG: %5.1f deg", laser_pitch_elev);
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
float Read_Battery_Voltage(void)
{
    uint32_t adc_sum = 0;
    const uint8_t samples = 16;

    for (uint8_t i = 0; i < samples; i++) {
        HAL_ADC_Start(&hadc3);
        if (HAL_ADC_PollForConversion(&hadc3, 10) == HAL_OK) {
            adc_sum += HAL_ADC_GetValue(&hadc3);
        }
        HAL_ADC_Stop(&hadc3);
    }

    float raw_avg = (float)adc_sum / (float)samples;

    // 12-bit ADC (0..4095) with 3.30V VREF
    // Voltage at PB1 = (raw_avg / 4095.0) * 3.30V
    // 1:1 Resistor Divider (100k / 100k) -> V_BAT = V_ADC * 2.0
    float v_adc = (raw_avg / 4095.0f) * 3.30f;
    return v_adc * 2.0f;
}

// Convert 1S Li-Ion / LiPo battery voltage to percentage (4.20V = 100%, 3.30V = 0%)
uint8_t Calculate_Battery_Percentage(float v_bat)
{
    if (v_bat >= 4.20f) return 100;
    if (v_bat <= 3.30f) return 0;
    return (uint8_t)(((v_bat - 3.30f) / (4.20f - 3.30f)) * 100.0f);
}

// Redirect standard printf to USB CDC Transmit
int _write(int file, char *ptr, int len)
{
    CDC_Transmit_FS((uint8_t*)ptr, len);
    return len;
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
