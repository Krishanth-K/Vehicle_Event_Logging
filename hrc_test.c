#include "stm32f411xe.h"
#include <stdint.h>
#include <stdio.h>

/*
 * ============================================================================
 * STM32F411CEU6 HC-SR04 MULTI-PIN ULTRASONIC TEST ENGINE
 * Clock: 16 MHz HSI
 *
 * RECOMMENDED PINOUT (NEW 5V-TOLERANT PINS: PB0 & PB1):
 * - HC-SR04 VCC   -> STM32 5V  (Left Header Pin 1)
 * - HC-SR04 GND   -> STM32 GND (Left Header Pin 2)
 * - HC-SR04 ECHO  -> STM32 B1  (Left Header Pin 7)  [Direct wire, NO resistors]
 * - HC-SR04 TRIG  -> STM32 B0  (Left Header Pin 8)  [Direct wire, NO resistors]
 *
 * BACKUP PIN COMBINATIONS (Auto-scanned if PB0/PB1 has no echo):
 * - Pair 2: PA3 (TRIG) & PA4 (ECHO) [Left Header Pins 13 & 12]
 * - Pair 3: PA1 (TRIG) & PA2 (ECHO) [Left Header Pins 15 & 14]
 * - Pair 4: Swapped polarities
 *
 * ONBOARD LED (PC13):
 * - Toggles on EVERY valid ultrasonic distance reading!
 * ============================================================================
 */

#define HSI_HZ               16000000U

/* ============================================================================
 * KEIL WATCH 1 VARIABLES (Exact names for live monitoring)
 * ============================================================================
 */

/* --- Sensor 2: HC-SR04 Ultrasonic Distance --- */
volatile float distance = -1.0f;         /* Distance in cm */
volatile uint32_t echo_pulse_us = 0;     /* Flight time in microseconds */
volatile int echo_status = -1;           /* 0 = Valid Echo, -1 = Timeout / Error */

/* Diagnostic Indicators */
volatile int echo_diag_stage = 0;        /* 1=Trig sent, 2=Echo High, 3=Echo Low, 4=OK, -1=High Timeout, -2=Low Timeout */
volatile uint8_t echo_pin_live = 0;      /* Current live logic level of ECHO pin (0 or 1) */
volatile uint32_t valid_pings = 0;       /* Number of successful echo returns */
volatile uint32_t total_pings = 0;       /* Number of triggers fired */
volatile uint32_t loop_count = 0;
const char *active_pins_str = "TRIG=PB0, ECHO=PB1";

/* Backward compatibility variables for Keil Watch 1 */
volatile float temperature = 30.0f;
volatile float temperature_f = 86.0f;
volatile float adc_mv = 300.0f;
volatile float accel_x = 0.0f, accel_y = 0.0f, accel_z = 1.0f;
volatile float gyro_x = 0.0f, gyro_y = 0.0f, gyro_z = 0.0f;
volatile float mpu_temp_c = 30.0f;
volatile int mpu_init_result = 0;
volatile unsigned char mpu_id = 0x70;

/* ============================================================================
 * Pin Definition Table
 * ============================================================================
 */

typedef struct {
    GPIO_TypeDef *trig_port;
    uint8_t       trig_pin;
    GPIO_TypeDef *echo_port;
    uint8_t       echo_pin;
    const char   *name;
} PinPair;

static const PinPair CANDIDATE_PAIRS[] = {
    {GPIOB, 0U, GPIOB, 1U, "TRIG=PB0, ECHO=PB1 (Recommended)"},
    {GPIOB, 1U, GPIOB, 0U, "TRIG=PB1, ECHO=PB0 (Reversed)"},
    {GPIOA, 3U, GPIOA, 4U, "TRIG=PA3, ECHO=PA4 (Alternate)"},
    {GPIOA, 4U, GPIOA, 3U, "TRIG=PA4, ECHO=PA3 (Alternate Reversed)"},
    {GPIOA, 1U, GPIOA, 2U, "TRIG=PA1, ECHO=PA2 (Original)"},
    {GPIOA, 2U, GPIOA, 1U, "TRIG=PA2, ECHO=PA1 (Original Reversed)"}
};

#define NUM_CANDIDATE_PAIRS (sizeof(CANDIDATE_PAIRS) / sizeof(CANDIDATE_PAIRS[0]))
static int current_pair_idx = 0;

/* ============================================================================
 * Clock & Timing System
 * ============================================================================
 */

static void clock_init(void)
{
    RCC->CR |= RCC_CR_HSION;
    while ((RCC->CR & RCC_CR_HSIRDY) == 0U) {
    }

    RCC->CFGR &= ~RCC_CFGR_SW;
    while ((RCC->CFGR & RCC_CFGR_SWS) != 0U) {
    }

    RCC->CFGR &= ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2);
    FLASH->ACR &= ~FLASH_ACR_LATENCY;
}

static void dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static void delay_us(uint32_t us)
{
    uint32_t cycles = us * (HSI_HZ / 1000000U);
    uint32_t start = DWT->CYCCNT;

    while ((uint32_t)(DWT->CYCCNT - start) < cycles) {
    }
}

static void delay_ms(uint32_t ms)
{
    while (ms-- > 0U) {
        delay_us(1000U);
    }
}

/* ============================================================================
 * GPIO Configuration
 * ============================================================================
 */

static void configure_pair(const PinPair *pair)
{
    /* Configure TRIG pin: Output Push-Pull, High Speed, Initial LOW */
    pair->trig_port->MODER &= ~(3U << (pair->trig_pin * 2U));
    pair->trig_port->MODER |=  (1U << (pair->trig_pin * 2U));
    pair->trig_port->OTYPER &= ~(1U << pair->trig_pin);
    pair->trig_port->OSPEEDR |= (3U << (pair->trig_pin * 2U));
    pair->trig_port->PUPDR &= ~(3U << (pair->trig_pin * 2U));
    pair->trig_port->BSRR = (1U << (pair->trig_pin + 16U)); /* Hold LOW */

    /* Configure ECHO pin: Input with Pull-Down (guarantees 0V idle) */
    pair->echo_port->MODER &= ~(3U << (pair->echo_pin * 2U));
    pair->echo_port->PUPDR &= ~(3U << (pair->echo_pin * 2U));
    pair->echo_port->PUPDR |=  (2U << (pair->echo_pin * 2U)); /* Pull-Down */
}

static void gpio_init(void)
{
    /* Enable GPIOA, GPIOB, and GPIOC peripheral clocks */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;
    (void)RCC->AHB1ENR;

    /* PC13: Onboard Blue LED (Push-pull, Active LOW) */
    GPIOC->MODER &= ~(3U << (13U * 2U));
    GPIOC->MODER |=  (1U << (13U * 2U));
    GPIOC->OTYPER &= ~(1U << 13U);
    GPIOC->OSPEEDR |= (3U << (13U * 2U));
    GPIOC->BSRR = (1U << 13U); /* Off initially */

    /* PA9: USART1 TX (AF7) */
    GPIOA->MODER &= ~(3U << (9U * 2U));
    GPIOA->MODER |=  (2U << (9U * 2U));
    GPIOA->AFR[1] &= ~(0xFU << ((9U - 8U) * 4U));
    GPIOA->AFR[1] |=  (7U   << ((9U - 8U) * 4U));
    GPIOA->OSPEEDR |= (3U << (9U * 2U));

    /* Initialize default pair (PB0 TRIG, PB1 ECHO) */
    configure_pair(&CANDIDATE_PAIRS[0]);
    active_pins_str = CANDIDATE_PAIRS[0].name;
}

/* ============================================================================
 * USART1 Telemetry (115200 Baud, 8-N-1 on PA9)
 * ============================================================================
 */

static void uart_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    (void)RCC->APB2ENR;

    USART1->CR1 = 0U;
    USART1->CR2 = 0U;
    USART1->CR3 = 0U;
    USART1->BRR = 139U; /* 115200 baud @ 16 MHz HSI */
    USART1->CR1 = USART_CR1_TE | USART_CR1_UE;
}

static void uart_putc(char c)
{
    while ((USART1->SR & USART_SR_TXE) == 0U) {
    }
    USART1->DR = (uint8_t)c;
}

static void uart_print(const char *s)
{
    while (*s != '\0') {
        if (*s == '\n') {
            uart_putc('\r');
        }
        uart_putc(*s++);
    }
}

/* ============================================================================
 * HC-SR04 Core Measurement Routine
 * ============================================================================
 */

static float ping_pair(const PinPair *pair)
{
    uint32_t start;
    uint32_t timeout_cycles;

    total_pings++;

    echo_pin_live = (uint8_t)((pair->echo_port->IDR >> pair->echo_pin) & 1U);

    /* 1. Hold TRIG LOW for 10 us */
    pair->trig_port->BSRR = (1U << (pair->trig_pin + 16U));
    delay_us(10U);

    /* 2. Send 20 us HIGH Trigger Pulse */
    pair->trig_port->BSRR = (1U << pair->trig_pin);
    delay_us(20U);
    pair->trig_port->BSRR = (1U << (pair->trig_pin + 16U)); /* Back to LOW */
    echo_diag_stage = 1;

    /* 3. Wait for ECHO to transition HIGH (HC-SR04 emits ultrasound and pulls ECHO high) */
    start = DWT->CYCCNT;
    timeout_cycles = (HSI_HZ / 1000U) * 35U; /* 35 ms wait */

    while (((pair->echo_port->IDR >> pair->echo_pin) & 1U) == 0U) {
        if ((uint32_t)(DWT->CYCCNT - start) > timeout_cycles) {
            echo_diag_stage = -1; /* ECHO never went HIGH */
            return -1.0f;
        }
    }

    echo_diag_stage = 2; /* ECHO went HIGH! */

    /* 4. Measure duration that ECHO stays HIGH */
    uint32_t pulse_start = DWT->CYCCNT;
    timeout_cycles = (HSI_HZ / 1000U) * 40U; /* 40 ms max flight time */

    while (((pair->echo_port->IDR >> pair->echo_pin) & 1U) != 0U) {
        if ((uint32_t)(DWT->CYCCNT - pulse_start) > timeout_cycles) {
            echo_diag_stage = -2; /* ECHO stayed HIGH too long */
            return -1.0f;
        }
    }

    uint32_t elapsed = (uint32_t)(DWT->CYCCNT - pulse_start);
    echo_pulse_us = elapsed / (HSI_HZ / 1000000U);
    float dist = (float)echo_pulse_us / 58.0f;

    if (dist < 2.0f || dist > 400.0f) {
        echo_diag_stage = -3;
        return -1.0f;
    }

    echo_diag_stage = 4; /* SUCCESS! */
    valid_pings++;

    /* Toggle PC13 blue LED on valid ping */
    GPIOC->ODR ^= (1U << 13U);

    return dist;
}

static void update_distance(void)
{
    static int fail_streak = 0;

    float d = ping_pair(&CANDIDATE_PAIRS[current_pair_idx]);

    if (d >= 2.0f && d <= 400.0f) {
        distance = d;
        echo_status = 0;
        fail_streak = 0;
        active_pins_str = CANDIDATE_PAIRS[current_pair_idx].name;
        return;
    }

    fail_streak++;

    /*
     * If no valid pings yet, cycle through all candidate pairs every 3 failed attempts:
     * PB0/PB1 -> PA3/PA4 -> PA1/PA2 -> Swapped
     */
    if (valid_pings == 0 && fail_streak >= 3) {
        fail_streak = 0;
        current_pair_idx = (current_pair_idx + 1) % NUM_CANDIDATE_PAIRS;
        configure_pair(&CANDIDATE_PAIRS[current_pair_idx]);
        active_pins_str = CANDIDATE_PAIRS[current_pair_idx].name;
        delay_ms(25U);
    }

    if (fail_streak >= 3) {
        distance = -1.0f;
        echo_status = -1;
    }
}

/* ============================================================================
 * MAIN APPLICATION
 * ============================================================================
 */

int main(void)
{
    char line[160];

    clock_init();
    dwt_init();
    gpio_init();
    uart_init();

    delay_ms(250U);

    uart_print("\n\n");
    uart_print("===============================================================\n");
    uart_print("   STM32F411CEU6 HC-SR04 MULTI-PIN ULTRASONIC TEST ENGINE      \n");
    uart_print("   Target: PB0(TRIG) & PB1(ECHO) with Auto-Scan Fallbacks      \n");
    uart_print("   Console: USART1 115200 baud on PA9 TX                       \n");
    uart_print("===============================================================\n");

    while (1) {
        loop_count++;

        update_distance();

        /* Print telemetry to serial */
        if (distance >= 0.0f) {
            snprintf(line, sizeof(line),
                     "[OK] Distance: %6.2f cm | Echo: %5u us | Active: %s\n",
                     (double)distance, (unsigned int)echo_pulse_us, active_pins_str);
        } else {
            snprintf(line, sizeof(line),
                     "[--] Distance: OUT OF RANGE | Stage: %d | Probing: %s\n",
                     echo_diag_stage, active_pins_str);
        }
        uart_print(line);

        /* 80 ms acoustic dissipation delay */
        delay_ms(80U);
    }
}
