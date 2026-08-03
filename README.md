# DigiTape - Digital Measuring Tool

```
  ██████╗  ██████╗  ██████╗ ██████╗     ████████╗██████╗ ██████╗ ███████╗
  ██╔══██╗   ██║   ██╔════╝   ██║       ╚══██╔══╝██╔══██╗██╔══██╗██╔════╝
  ██║  ██║   ██║   ██║  ███╗  ██║          ██║   ███████║██████╔╝█████╗  
  ██║  ██║   ██║   ██║   ██║  ██║          ██║   ██╔══██║██╔═══╝ ██╔══╝  
  ██████╔╝ ██████╗ ╚██████╔╝██████╗        ██║   ██║  ██║██║     ███████╗
  ╚═════╝  ╚═════╝  ╚═════╝ ╚═════╝        ╚═╝   ╚═╝  ╚═╝╚═╝     ╚══════╝
```

[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)
[![MCU: STM32G491CEU6](https://img.shields.io/badge/MCU-STM32G491CEU6-orange.svg)](https://www.st.com/en/microcontrollers-microprocessors/stm32g491ce.html)
[![Sensor: ST VL53L4CX](https://img.shields.io/badge/ToF-ST%20VL53L4CX-green.svg)](https://www.st.com/en/imaging-and-photonics-solutions/vl53l4cx.html)
[![Inclinometer: Murata SCL3300](https://img.shields.io/badge/Inclinometer-SCL3300-red.svg)](https://www.murata.com/en-global/products/sensor/inclinometer/scl3300)
[![Publisher: Circuit Digest](https://img.shields.io/badge/Publisher-Circuit%20Digest-brightgreen.svg)](https://circuitdigest.com/)

A pocket-size multi-mode digital laser measuring tape device built around the **STM32G491CEU6** 32-bit ARM Cortex-M4F microcontroller. Featuring **ST VL53L4CX 6.0-meter Time-of-Flight (ToF) ranging**, **Murata SCL3300 3-axis inclinometer level sensing**, an **animated 60FPS smartwatch-style carousel UI**, and **non-volatile internal Flash memory persistence**.

---

## Key Features

* **8 Comprehensive Operating Modes**:
  1. **Distance Meter (`DIST`)**: Real-time continuous ToF distance measurement (up to 6.0 meters / 600 cm) with live roll & pitch level alignment bars.
  2. **Spirit Level (`LEVEL`)**: 2D digital bubble level + 3D pitch/roll angle readout ($0.01^\circ$ resolution) and temperature display.
  3. **Height Meter (`HEIGHT`)**: Pythagoras indirect height measurement with automatic ToF vertical mounting offset & 2-point empirical tilt calibration.
  4. **Area Calculator (`AREA`)**: 2-shot interactive room area calculation ($L \times W \implies \text{cm}^2 / \text{m}^2$).
  5. **Volume Calculator (`VOLUME`)**: 3-shot 3D space volume calculation ($L \times W \times H \implies \text{cm}^3 / \text{m}^3$).
  6. **Cylinder Meter (`CYLINDER`)**: Pipe and tank volume calculation ($\pi \cdot R^2 \cdot H$).
  7. **Max / Min Tracker (`MAXMIN`)**: Dynamic real-time minimum and maximum distance boundary tracker.
  8. **Memory Log (`MEMORY`)**: Persistent 10-slot measurement record buffer.

* **ST VL53L4CX Ranging Optimization**:
  - **Multi-Target Histogram Ranging**: Penetrates transparent glass windows and filters out foreground obstacles to lock onto background target walls.
  - **Cover Glass Crosstalk Cancellation**: Cancels internal VCSEL reflections inside protective enclosure windows.

* **Power Management & Dual Auto-Wakeup**:
  - Hardware `XSHUT` zero-current shutdown.
  - 3-Minute Inactivity Auto-Sleep.
  - SCL3300 Motion Pick-up ($\Delta G > 0.03g$) & Encoder Turn/Press instant auto-wakeup.

---

## User Controls & Navigation Guide

### 1. Hardware Controls Overview
- **Rotary Encoder Knob (Hongyan RS11)**: Spring-return $\pm 15^\circ$ rotation knob (flicks CW / CCW and snaps back to center).
- **Center Push Button Switch (SW)**: Integrated tactile push-button (`PB14`).

---

### 2. Main Screen & Measurement Navigation

| Action / Gesture | Hardware Trigger | Mode / Context | Function / Operation |
| :--- | :--- | :--- | :--- |
| **Rotate Knob (CW / CCW)** | Flick Knob $\pm 15^\circ$ | **Carousel Menu** | Scroll through 8 measurement modes (smooth 60FPS animation). |
| **Rotate Knob (CW / CCW)** | Flick Knob $\pm 15^\circ$ | **Memory Log (`MEMORY`)** | Browse through 10 saved measurement history records. |
| **Single-Press** | Press Button | **Carousel Menu** | **Select & Enter** highlighted measurement mode. |
| **Single-Press** | Press Button | **`DIST` / `HEIGHT` Modes** | **Toggle `HOLD` Mode** (freezes measurement reading & angle; turns OFF laser). |
| **Single-Press** | Press Button | **`AREA` Mode** | Take Shot 1 (Length) $\rightarrow$ Take Shot 2 (Width) $\rightarrow$ Calculate Area ($\text{cm}^2 / \text{m}^2$). |
| **Single-Press** | Press Button | **`VOLUME` Mode** | Take Shot 1 (Length) $\rightarrow$ Shot 2 (Width) $\rightarrow$ Shot 3 (Height) $\rightarrow$ Calculate Volume ($\text{cm}^3 / \text{m}^3$). |
| **Single-Press** | Press Button | **`MAXMIN` Mode** | Reset Min / Max distance trackers. |
| **Double-Press** | Quick 2x Click ($< 400\text{ms}$) | **Any Mode / Settings** | **Global BACK** $\rightarrow$ Return immediately to Main Carousel Menu. |
| **Long-Press** | Hold Button ($> 800\text{ms}$) | **Any Screen** | **Toggle Settings Menu** overlay. |
| **Press + Twist** | Hold Switch + Rotate Knob | **Any Active Mode** | **Chorded Gesture BACK** $\rightarrow$ Return immediately to Main Carousel Menu. |

---

### 3. Settings Menu Navigation & Value Editing

```
[Main Carousel] ──(Long-Press >800ms)──> [Settings Overlay]
                                                │
                          Rotate Knob ───> Scroll items (UNIT, DATUM, OFFSET)
                                                │
                          Single-Press ──> Enter ITEM EDIT MODE (SETTINGS [EDIT])
                                                │
                          Rotate Knob ───> Fast-adjust value (0.1cm steps for offset)
                                                │
                    Single or Double-Press ─> Confirm & Auto-Save to Flash Page 255
```

| Action / Gesture | Context | Function |
| :--- | :--- | :--- |
| **Rotate Knob** | Settings List Mode | Scroll between items (`0: UNIT`, `1: DATUM`, `2: REAR OFFSET`). |
| **Single-Press** | Settings List Mode | Enter **ITEM EDIT MODE** (`SETTINGS [EDIT]`). Value displays in `< angle brackets >`. |
| **Rotate Knob** | **ITEM EDIT MODE** | **Change Value**: <br>• **`UNIT`**: `CM` $\leftrightarrow$ `MM` $\leftrightarrow$ `M` $\leftrightarrow$ `INCH` <br>• **`DATUM`**: `REAR` $\leftrightarrow$ `FRONT` <br>• **`OFFSET`**: Adjust body length in **0.1 cm (1 mm)** steps! |
| **Single-Press** | **ITEM EDIT MODE** | Confirm new value, **Save to NVM Flash Page 255**, and return to Settings List. |
| **Double-Press** | **ITEM EDIT MODE** | Confirm value, Save to Flash, and return to Settings List. |
| **Double-Press** | Settings List Mode | Exit Settings and return to Main Carousel Menu. |

---

## Hardware Pinout Mapping

| Component | Signal | STM32G4 Pin | Description |
| :--- | :--- | :--- | :--- |
| **MCU** | Main Core | STM32G491CEU6 | 170 MHz ARM Cortex-M4F, 512KB Flash, 128KB RAM |
| **OLED Display** | I2C2 SDA | `PA8` | 1.3" 128x64 SH1106 OLED (Address `0x3C`) |
| | I2C2 SCL | `PA9` | |
| **ToF Sensor** | I2C1 SCL | `PA15` | ST VL53L4CX 6.0m ToF Sensor (Address `0x52`) |
| | I2C1 SDA | `PB7` | |
| | ToF INT | `PC10` | Hardware Interrupt Line |
| | ToF XSHUT | `PC11` | Hardware Zero-Power Shutdown Control |
| **Inclinometer** | SPI1 CS | `PA3` | Murata / VTI SCL3300 3-Axis Inclinometer |
| | SPI1 SCK | `PA5` | SPI Clock |
| | SPI1 MISO | `PA6` | SPI Master In Slave Out |
| | SPI1 MOSI | `PA7` | SPI Master Out Slave In |
| **Laser Diode** | EN | `PA4` | CAT4002A Constant-Current Laser Driver |
| **Battery Monitor**| ADC3 CH1 | `PB1` | 1S Li-Ion Voltage Divider ($100\text{ k}\Omega / 100\text{ k}\Omega$) |
| **Rotary Encoder** | SW | `PB14` | Hongyan RS11 Spring-Return Switch |
| | Phase A | `PB12` | Encoder Phase A |
| | Phase B | `PB13` | Encoder Phase B |

---

## Software & Driver Architecture

- **Toolchain**: STM32CubeIDE (GCC ARM Embedded Toolchain).
- **HAL Drivers**: STM32G4 HAL Driver Package + ST VL53L4CX Ultra-Lite Driver (ULD).
- **Core Files**:
  - `Core/Src/main.c`: Main firmware logic, state machine, ToF filtering, calibration, and Flash storage.
  - `Core/Src/oled.c`: SH1106 OLED vector graphics, 7-segment font renderer, and 60FPS carousel engine.
  - `Core/Src/scl3300.c`: SPI driver and angle/acceleration conversion for Murata SCL3300.

---