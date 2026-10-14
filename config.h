#ifndef CONFIG_H
#define CONFIG_H

#include "stm32f411xe.h"
#include <stdint.h>

/* ============================================================================
 * config.h — Vehicle Event Logging and Response System Configuration Block
 * Course: 23CSE304 — Embedded Systems (Team 11)
 * Target: STM32F411CEU6 "Black Pill" (Bare-Metal CMSIS, No HAL)
 * ============================================================================
 */

/* --- System Clock ----------------------------------------------------------- */
#define SYS_CLOCK_MHZ           16U             /* 16 MHz HSI Internal Osc      */
#define HSI_HZ                  16000000U

/* --- Sensor 1: LM35 Temperature (ADC1) -------------------------------------- */
#define LM35_ADC_PORT           GPIOA
#define LM35_ADC_PIN            0U              /* PA0 = ADC1_IN0 (Left Pin 16) */
#define LM35_ADC_CHANNEL        0U
#define VREF_MV                 3300U           /* ADC Reference Voltage in mV  */

/* --- Sensor 2: HC-SR04 Ultrasonic Distance (PB0 TRIG, PB1 ECHO) ------------- */
#define HCSR04_PORT             GPIOB
#define HCSR04_TRIG_PIN         0U              /* PB0 (Left Header Pin 8, B0)  */
#define HCSR04_ECHO_PIN         1U              /* PB1 (Left Header Pin 7, B1)  */
#define HCSR04_TIMEOUT_US       35000U          /* ~6 m max range timeout       */

/* --- Sensor 3: MPU6050 Motion / Impact (I2C1) ------------------------------ */
#define MPU_GPIO_PORT           GPIOB
#define MPU_SCL_PIN             6U              /* PB6 (Right Header Pin 16, B6)*/
#define MPU_SDA_PIN             7U              /* PB7 (Right Header Pin 17, B7)*/
#define MPU_I2C_ADDR            0x68U           /* AD0 pulled to GND -> 0x68    */
#define MPU_BASELINE_SAMPLES    32U             /* Number of startup samples    */

/* --- UART Alert & Telemetry (Docklight / Android Auto) ---------------------- */
#define ALERT_BAUD              115200U         /* 115200 baud, 8-N-1           */
#define UART1_TX_PIN            9U              /* PA9 (USART1 TX, Left Pin 6)  */
#define UART2_TX_PIN            2U              /* PA2 (USART2 TX, Right Pin 8) */
#define SOFT_UART_PORT          GPIOB
#define SOFT_UART_TX_PIN        10U             /* PB10 (Right Pin 17, next to GND!) */
#define SOFT_UART_TX_PIN2       4U              /* PB4  (Left Pin 14)           */

/* --- Local Indicators & Emergency Interrupt (EXTI) -------------------------- */
#define LED_PORT                GPIOC
#define LED_WARN_PIN            13U             /* PC13 Onboard Blue LED        */
#define EMERGENCY_EXTI_PORT     GPIOB
#define EMERGENCY_EXTI_PIN      12U             /* PB12 Emergency Crash/SOS EXTI*/

/* --- Event Classification Thresholds (PRD Section 3) ----------------------- */
#define TEMP_THRESHOLD_C        60U             /* Critical if T >= 60 C        */
#define TEMP_WARN_C             45U             /* Warning if T >= 45 C         */

#define DIST_THRESHOLD_CM       20U             /* Critical if d <= 20 cm       */
#define DIST_WARN_CM            35U             /* Warning if d <= 35 cm        */

#define ACCEL_THRESHOLD_G_X100  150U            /* Critical if |Delta a| >= 1.50g */
#define ACCEL_WARN_G_X100       80U             /* Warning if |Delta a| >= 0.80g  */

/* --- Event Classification Enum (PRD Section 4.2) ---------------------------- */
typedef enum {
    STATE_NORMAL   = 0,                         /* Reading within safe range    */
    STATE_WARNING  = 1,                         /* Approaching threshold        */
    STATE_CRITICAL = 2,                         /* Threshold exceeded -> Alert  */
    STATE_INVALID  = 3                          /* Sensor out of range/offline  */
} EventState;

#endif /* CONFIG_H */
