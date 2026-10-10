# Product Requirements Document
## Vehicle Event Logging and Response System

| Field | Value |
|-------|-------|
| **Project** | Vehicle Event Logging and Response System |
| **Course** | 23CSE304 — Embedded Systems |
| **Team** | Team 11 |
| **Platform** | STM32F411CEU6 (ARM Cortex-M4) |
| **Toolchain** | Keil µVision — register-level C (no HAL, no STL) |
| **Version** | 1.0 |

---

## 1. Problem Statement

A vehicle may experience sudden impact, overheating, or obstacle proximity without any automated detection or alerting mechanism. This system continuously monitors three sensor inputs, classifies events in firmware, and transmits alerts over UART — all implemented at the register level on an STM32 microcontroller.

---

## 2. Scope

### In Scope
- MPU6050 motion/impact sensing via I²C
- LM35 temperature sensing via ADC
- HC-SR04 obstacle distance sensing via GPIO + Timer
- Threshold-based event classification in firmware
- UART alert transmission on event detection
- LED/buzzer local indication (optional, if hardware available)
- Register-level peripheral configuration (no HAL, no CMSIS drivers)
- Development and testing in Keil µVision

### Out of Scope
- SD card logging
- RTC timestamping
- Wireless/Bluetooth communication
- Android Auto / CAN bus integration
- Braking / motor response
- Collision verification (vs. motion anomaly)

---

## 3. Hardware Configuration Block

> **All pin assignments are defined in a single configuration header** (`config.h`).  
> Change only this file during final assembly — no changes needed elsewhere.

```c
/* ================================================================
 * config.h  —  Hardware Configuration Block
 * Change ONLY this file to match your physical wiring.
 * ================================================================ */

/* --- MCU Clock -------------------------------------------------- */
#define SYS_CLOCK_MHZ       84u          /* STM32F411 @ 84 MHz     */

/* --- I2C1 : MPU6050 -------------------------------------------- */
/* Typical: PB6=SCL, PB7=SDA (I2C1 AF4) */
#define MPU_I2C             I2C1
#define MPU_I2C_CLK_EN      (RCC->APB1ENR |= RCC_APB1ENR_I2C1EN)
#define MPU_GPIO_PORT       GPIOB
#define MPU_GPIO_CLK_EN     (RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN)
#define MPU_SCL_PIN         6u           /* PB6                    */
#define MPU_SDA_PIN         7u           /* PB7                    */
#define MPU_I2C_ADDR        0x68u        /* AD0 low → 0x68         */

/* --- ADC1 : LM35 ----------------------------------------------- */
/* Typical: PA0 = ADC1_IN0 */
#define LM35_GPIO_PORT      GPIOA
#define LM35_GPIO_CLK_EN    (RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN)
#define LM35_ADC_CHANNEL    0u           /* IN0 = PA0              */
#define LM35_ADC_PIN        0u           /* PA0                    */
#define VREF_MV             3300u        /* ADC reference (mV)     */

/* --- GPIO + TIM2 : HC-SR04 ------------------------------------- */
/* Typical: PA8=TRIG, PA9=ECHO (TIM1_CH2) – adjust as needed      */
#define HCSR04_GPIO_PORT    GPIOA
#define HCSR04_GPIO_CLK_EN  (RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN)
#define HCSR04_TRIG_PIN     8u           /* PA8 — GPIO output      */
#define HCSR04_ECHO_PIN     9u           /* PA9 — timer input cap  */
#define HCSR04_TIM          TIM1         /* Timer for echo capture */
#define HCSR04_TIM_CLK_EN   (RCC->APB2ENR |= RCC_APB2ENR_TIM1EN)
#define HCSR04_TIM_CH       2u           /* TIM1_CH2               */
#define HCSR04_TIMEOUT_US   25000u       /* ~4 m max range timeout */

/* --- UART2 : Alert output -------------------------------------- */
/* Typical: PA2=TX, PA3=RX (USART2 AF7) */
#define ALERT_UART          USART2
#define ALERT_UART_CLK_EN   (RCC->APB1ENR |= RCC_APB1ENR_USART2EN)
#define ALERT_UART_GPIO     GPIOA
#define ALERT_UART_GPIO_EN  (RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN)
#define ALERT_UART_TX_PIN   2u           /* PA2                    */
#define ALERT_UART_RX_PIN   3u           /* PA3                    */
#define ALERT_BAUD          115200u

/* --- Local Indicators (optional) ------------------------------- */
#define LED_GPIO_PORT       GPIOC
#define LED_GPIO_CLK_EN     (RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN)
#define LED_WARN_PIN        13u          /* PC13 — onboard LED     */
#define BUZZER_GPIO_PORT    GPIOB
#define BUZZER_GPIO_CLK_EN  (RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN)
#define BUZZER_PIN          0u           /* PB0                    */

/* --- Event Thresholds ------------------------------------------ */
#define TEMP_THRESHOLD_C    60u          /* Alert if T >= 60 °C    */
#define DIST_THRESHOLD_CM   20u          /* Alert if d <= 20 cm    */
#define ACCEL_THRESHOLD_G_X100  150u     /* Alert if |a| >= 1.5 g  */
                                         /* (stored as 1.5 × 100)  */
```

---

## 4. System Architecture

### 4.1 Module Breakdown

```
project/
├── config.h          ← Hardware configuration (ONLY file changed per board)
├── main.c            ← System init + main loop
├── systick.h/.c      ← SysTick delay and scheduling
├── uart.h/.c         ← UART init + transmit
├── i2c.h/.c          ← I2C master driver (register-level)
├── mpu6050.h/.c      ← MPU6050 sensor driver
├── adc.h/.c          ← ADC init + single conversion
├── lm35.h/.c         ← LM35 temperature read + classification
├── hcsr04.h/.c       ← HC-SR04 trigger/capture + distance calc
├── event.h/.c        ← Event types, flags, UART alert formatting
└── test/
    ├── test_uart.c   ← UART loopback test
    ├── test_mpu.c    ← MPU6050 register read test
    ├── test_lm35.c   ← ADC conversion test
    └── test_hcsr04.c ← Echo timing test
```

### 4.2 Event Classification

Each sensor produces one of three states:

```
STATE_NORMAL    (0) — reading within safe range
STATE_WARNING   (1) — reading approaching threshold
STATE_CRITICAL  (2) — threshold exceeded, UART alert sent
```

### 4.3 Main Loop Flow

```
INIT:
  SysTick → 1 ms tick
  UART     → 115200 baud
  I2C      → MPU6050 wakeup + config
  ADC      → LM35 channel config
  TIMx     → HC-SR04 input capture

LOOP (every ~100 ms):
  1. Read MPU6050 → classify motion event
  2. Read LM35    → classify temperature event
  3. Read HC-SR04 → classify proximity event
  4. For each CRITICAL event:
       → Format alert string
       → Transmit over UART
       → Toggle LED / buzzer
  5. Delay(100 ms)
```

---

## 5. Functional Requirements

### 5.1 MPU6050 — Motion / Impact Detection

| ID | Requirement |
|----|-------------|
| F-MPU-01 | Initialize MPU6050 over I²C at startup: wake from sleep, set sample rate, configure DLPF |
| F-MPU-02 | Read 6 bytes of accelerometer data (X, Y, Z) in a single I²C burst read |
| F-MPU-03 | Combine high/low bytes into signed 16-bit integers |
| F-MPU-04 | Scale raw ADC counts to g-force using configured full-scale range (default ±2 g → 16384 LSB/g) |
| F-MPU-05 | Establish a stationary baseline on startup (average of first N readings) |
| F-MPU-06 | Subtract baseline from live readings before threshold comparison |
| F-MPU-07 | Classify event as CRITICAL if `abs(any_axis) >= ACCEL_THRESHOLD_G_X100` |
| F-MPU-08 | Output: event state + axis values (in g × 100, integer arithmetic only) |

**Register sequence (I2C)**:
```
PWR_MGMT_1  (0x6B) ← 0x00        // wake from sleep
SMPLRT_DIV  (0x19) ← 0x07        // 1 kHz / (1+7) = 125 Hz sample rate
CONFIG      (0x1A) ← 0x06        // DLPF ~5 Hz
ACCEL_CONFIG(0x1C) ← 0x00        // ±2 g range
Read 6 bytes from ACCEL_XOUT_H (0x3B)
```

---

### 5.2 LM35 — Temperature Monitoring

| ID | Requirement |
|----|-------------|
| F-LM-01 | Configure ADC1 for 12-bit single-conversion mode on the configured channel |
| F-LM-02 | Set PA_x (configured pin) to analog mode via `GPIOx_MODER` |
| F-LM-03 | Trigger conversion with `SWSTART`; poll `EOC` flag; read `ADC_DR` |
| F-LM-04 | Convert ADC count to millivolts: `V_mV = (count * VREF_MV) / 4095` |
| F-LM-05 | Convert millivolts to temperature: `T_C = V_mV / 10` (LM35: 10 mV/°C) |
| F-LM-06 | Classify CRITICAL if `T_C >= TEMP_THRESHOLD_C` |
| F-LM-07 | All arithmetic in integer (no float); accuracy ±1°C is acceptable |

**Register sequence (ADC)**:
```
RCC_APB2ENR  ← ADC1EN = 1
GPIOx_MODER  ← pin bits = 11 (analog)
ADC_CR1      ← RES = 00 (12-bit)
ADC_SMPR2    ← channel sample time = 111 (480 cycles)
ADC_SQR3     ← SQ1 = channel number
ADC_CR2      ← ADON = 1
ADC_CR2      ← SWSTART = 1 → poll SR[EOC] → read DR
```

---

### 5.3 HC-SR04 — Obstacle Proximity

| ID | Requirement |
|----|-------------|
| F-HC-01 | Configure TRIG pin as GPIO output; set ECHO pin to timer input capture alternate function |
| F-HC-02 | Generate a ≥10 µs HIGH pulse on TRIG using SysTick delay |
| F-HC-03 | Capture rising and falling edges of ECHO using timer input capture |
| F-HC-04 | Compute echo duration: `t_us = (falling_count - rising_count) * timer_tick_us` |
| F-HC-05 | Compute distance: `d_cm = t_us / 58` |
| F-HC-06 | Implement timeout: if ECHO never goes HIGH within `HCSR04_TIMEOUT_US`, mark reading as invalid |
| F-HC-07 | Classify CRITICAL if `d_cm <= DIST_THRESHOLD_CM` (and reading is valid) |
| F-HC-08 | Wait ≥60 ms between measurements to prevent echo interference |

**Register sequence (Timer input capture)**:
```
RCC_APBxENR  ← TIMxEN = 1
GPIOx_MODER  ← ECHO pin = 10 (alternate function)
GPIOx_AFR    ← set AF for TIMx_CHy
TIMx_PSC     ← prescaler for 1 µs tick (e.g. 83 for 84 MHz)
TIMx_ARR     ← 0xFFFF (free-running)
TIMx_CCMR1   ← CC2S = 01 (IC on TI2), IC2F = 0000
TIMx_CCER    ← CC2E = 1 (enable capture), CC2P/CC2NP for edge select
TIMx_CR1     ← CEN = 1
Read TIMx_CCR2 on rising edge; toggle CCER edge polarity; read again on falling.
```

---

### 5.4 UART Alert Protocol

| ID | Requirement |
|----|-------------|
| F-UA-01 | Initialize USART2 at 115200 baud, 8N1, TX only (RX optional) |
| F-UA-02 | Send alerts as human-readable ASCII strings terminated with `\r\n` |
| F-UA-03 | Each alert includes: sensor name, event level, measured value, unit |
| F-UA-04 | Transmit character-by-character by polling `TXE` flag in `USART_SR` |
| F-UA-05 | Do not block other sensor reads for more than one UART frame duration |
| F-UA-06 | Define a startup banner sent once on init to confirm UART is alive |

**Alert string format**:
```
[ALERT] MOTION   | CRITICAL | Ax=+1.82g Ay=-0.12g Az=+0.03g\r\n
[ALERT] TEMP     | CRITICAL | T=73°C (threshold=60°C)\r\n
[ALERT] OBSTACLE | CRITICAL | d=14cm (threshold=20cm)\r\n
[INFO]  SYSTEM   | NORMAL   | All sensors OK\r\n
[INFO]  SYSTEM   | BOOT     | Vehicle Event Monitor v1.0\r\n
```

**Register sequence (UART)**:
```
RCC_APB1ENR  ← USART2EN = 1
RCC_AHB1ENR  ← GPIOAEN  = 1
GPIOA_MODER  ← TX pin = 10 (alternate function)
GPIOA_AFR    ← AF7 (USART2)
USART2_BRR   ← APB1_CLK / BAUD  (e.g. 42000000 / 115200 = 365 → 0x16D)
USART2_CR1   ← TE = 1, UE = 1
Transmit:   poll SR[TXE]; write to DR; poll SR[TC] before next char
```

---

## 6. Development Phases

---

### Phase 0 — Project Skeleton & UART Bringup
**Goal**: Establish project structure and prove UART is working before any sensor code.

**Deliverables**:
- `config.h` with all pin/threshold defines
- `systick.c` with `delay_ms()` and `get_tick()`
- `uart.c` with `uart_init()` and `uart_send_string()`
- `main.c` boots, prints banner, blinks LED every 1 s

**Test**: Open serial terminal at 115200. Confirm:
```
[INFO]  SYSTEM   | BOOT     | Vehicle Event Monitor v1.0
```
LED blinks at 1 Hz.

**Commit**: `feat: project skeleton, systick, uart bringup`

---

### Phase 1 — LM35 Temperature Sensing
**Goal**: ADC pipeline working, temperature read and classified.

**Deliverables**:
- `adc.c` — ADC1 init, single-conversion trigger/read
- `lm35.c` — `lm35_read_celsius()`, `lm35_classify()`
- UART prints temperature every 2 s
- UART prints CRITICAL alert when threshold exceeded

**Test** (`test_lm35.c`):
- [ ] Cold reading (room temp ~25°C) → `STATE_NORMAL`
- [ ] Warm sensor gently (finger/warm object) → reading increases
- [ ] Force `TEMP_THRESHOLD_C = 30` in config temporarily → confirm CRITICAL fires
- [ ] ADC count + converted value logged for calibration check

**Commit**: `feat: lm35 adc temperature sensing with uart alert`

---

### Phase 2 — MPU6050 Motion Sensing
**Goal**: I²C working, motion event classified.

**Deliverables**:
- `i2c.c` — I2C1 master init, `i2c_write_reg()`, `i2c_read_bytes()`
- `mpu6050.c` — `mpu6050_init()`, `mpu6050_read_accel()`, `mpu6050_classify()`
- UART prints accel X/Y/Z every 500 ms
- UART prints CRITICAL alert on sharp movement

**Test** (`test_mpu.c`):
- [ ] Stationary: all axes near 0 g (after baseline subtraction), state = NORMAL
- [ ] Read `WHO_AM_I` (reg `0x75`) → must return `0x68`; if not, I²C failed
- [ ] Tap/shake the board → CRITICAL fires within 1 reading
- [ ] Verify no false triggers on stationary baseline for 30 s
- [ ] Log raw counts + scaled values to verify scale factor

**Commit**: `feat: mpu6050 i2c motion sensing with uart alert`

---

### Phase 3 — HC-SR04 Obstacle Detection
**Goal**: Timer input capture working, distance measured and classified.

**Deliverables**:
- `hcsr04.c` — `hcsr04_trigger()`, `hcsr04_read_cm()`, `hcsr04_classify()`
- UART prints distance every 500 ms
- UART prints CRITICAL alert when obstacle within threshold
- Timeout handled: invalid reading reported, no false alert

**Test** (`test_hcsr04.c`):
- [ ] Object at known distance (e.g. 30 cm): measured value ≤ ±3 cm error
- [ ] Object at 10 cm → CRITICAL fires
- [ ] No object in range → reading > threshold, state = NORMAL
- [ ] Block echo pin (simulate timeout) → `[WARN] OBSTACLE | INVALID | no echo`
- [ ] Verify ≥60 ms between measurements (logic analyzer or SysTick log)

**Commit**: `feat: hcsr04 timer-capture obstacle detection with uart alert`

---

### Phase 4 — Integration & Full System Test
**Goal**: All sensors running together in the main loop, alerts non-conflicting.

**Deliverables**:
- `event.c` — unified event state struct, alert priority logic
- Main loop polls all three sensors, sends UART alerts
- LED/buzzer toggled on any CRITICAL event
- Startup self-test pings each sensor and reports status

**Integration Test Checklist**:
- [ ] All three sensors reporting NORMAL simultaneously for 60 s — no spurious alerts
- [ ] Trigger each sensor independently → correct alert, no cross-contamination
- [ ] Trigger all three simultaneously → three separate alerts sent in order
- [ ] Power cycle → banner printed, baseline re-established, no boot alerts
- [ ] Remove/block HC-SR04 → only obstacle reports invalid; temp and motion continue

**Commit**: `feat: full sensor integration, event routing, startup self-test`

---

## 7. Non-Functional Requirements

| ID | Requirement |
|----|-------------|
| NF-01 | Main loop period: ≤ 200 ms (all three sensors polled in one cycle) |
| NF-02 | UART transmit: non-blocking per-character poll; never delays sensor reads by > 1 ms |
| NF-03 | No dynamic memory allocation (`malloc`/`free`) — all state in static structs |
| NF-04 | No floating-point arithmetic — all sensor math uses integer scaling (×100, ×10 etc.) |
| NF-05 | ISRs (if used) are < 10 lines; no I²C or UART inside ISR |
| NF-06 | Every peripheral function returns a status code; caller must check it |
| NF-07 | Invalid/missing sensor readings must not produce an alert — silently mark as INVALID |
| NF-08 | Code compiles with 0 warnings in Keil at warning level 3 |

---

## 8. Code Modularity Rules

1. **`config.h` is the only file that knows pin numbers** — every driver accesses pins via macros defined there.
2. **Sensor drivers are self-contained** — `mpu6050.c` calls only `i2c.h` and `config.h`; it has no knowledge of UART or events.
3. **`event.c` owns all alert logic** — sensors return raw classified states; `event.c` decides what to transmit and when.
4. **`main.c` orchestrates, never implements** — it calls `init()` and `read()`/`classify()` functions; it contains no register writes.
5. **Every `.c` file has a matching `.h`** with only the public interface — no `extern` globals outside headers.
6. **Threshold logic lives in `config.h`** — no magic numbers inside `.c` files.

---

## 9. Test Harness Approach

Each `test_*.c` file is a standalone `main()` that can be flashed instead of the real `main.c`.  
Tests output pass/fail results over UART using the format:

```
[TEST] LM35       PASS  T=26°C  raw=1289
[TEST] MPU_WHO_AM_I PASS  got=0x68
[TEST] HCSR04     PASS  d=31cm  t_us=1798
[TEST] HCSR04     FAIL  timeout (echo not received)
```

This allows individual module validation before integration.

---

## 10. Success Criteria

| Sensor | Pass Condition |
|--------|---------------|
| LM35 | Temperature reads within ±2°C of reference; CRITICAL fires when T ≥ threshold |
| MPU6050 | `WHO_AM_I = 0x68`; stationary baseline stable; CRITICAL fires on deliberate impact |
| HC-SR04 | Distance within ±3 cm of reference; CRITICAL fires when d ≤ threshold; timeout handled |
| UART | All alerts arrive in correct format; no garbled characters at 115200 baud |
| Integration | All three sensors coexist; no false alerts in 60 s normal operation |
