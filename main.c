#include "stm32f411xe.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "config.h"

/*
 * ============================================================================
 * VEHICLE EVENT LOGGING AND RESPONSE SYSTEM (PRD v1.0 IMPLEMENTATION)
 * Course: 23CSE304 — Embedded Systems (Team 11)
 * Platform: STM32F411CEU6 (ARM Cortex-M4, Bare-Metal Register Level, No HAL)
 *
 * HARDWARE CONFIGURATION:
 * 1. LM35 Temperature Sensor:
 *    - PA0 (ADC1_IN0, Left Header Pin 16, labeled A0)
 *
 * 2. HC-SR04 Obstacle Proximity Sensor:
 *    - PB0 (TRIG, Left Header Pin 8, labeled B0)
 *    - PB1 (ECHO, Left Header Pin 7, labeled B1, 5V-Tolerant Direct Wire)
 *
 * 3. MPU6050 Motion / Impact Sensor (GY-521):
 *    - PB6 (I2C1_SCL, Right Header Pin 16, labeled B6)
 *    - PB7 (I2C1_SDA, Right Header Pin 17, labeled B7)
 *    - AD0 -> GND (Address 0x68)
 *
 * 4. UART Telemetry & Alerts (Docklight / Android Auto):
 *    - Transmits simultaneously on PA9 (USART1 TX) AND PA2 (USART2 TX) @ 115200 baud
 *
 * 5. Local Indicators:
 *    - PC13: Onboard Blue LED (Active LOW)
 *      * Normal   : 1 Hz steady heartbeat
 *      * Critical : Fast 10 Hz strobe / solid alert
 * ============================================================================
 */

/* ============================================================================
 * GLOBAL WATCH VARIABLES (Visible live in Keil Watch 1 Window)
 * ============================================================================
 */

/* --- Sensor 1: LM35 Temperature --- */
volatile float temperature = 0.0f;       /* Temperature in Celsius */
volatile float temperature_f = 0.0f;     /* Temperature in Fahrenheit */
volatile float adc_mv = 0.0f;            /* Measured millivolts on PA0 */
volatile int temp_c = 0;                 /* Integer Celsius for threshold check */

/* --- Sensor 2: HC-SR04 Ultrasonic Distance --- */
volatile float distance = -1.0f;         /* Distance in cm (-1 = Invalid / Timeout) */
volatile uint32_t echo_pulse_us = 0;     /* Flight time in microseconds */
volatile int dist_cm = -1;               /* Integer distance in cm */

/* --- Sensor 3: MPU6050 Motion & Orientation --- */
volatile int16_t ax = 0, ay = 0, az = 0; /* Raw accelerometer counts */
volatile int16_t gx = 0, gy = 0, gz = 0; /* Raw gyroscope counts */
volatile float accel_x = 0.0f;           /* X-axis acceleration in g */
volatile float accel_y = 0.0f;           /* Y-axis acceleration in g */
volatile float accel_z = 0.0f;           /* Z-axis acceleration in g */
volatile float gyro_x = 0.0f;            /* Gyro X in deg/s */
volatile float gyro_y = 0.0f;            /* Gyro Y in deg/s */
volatile float gyro_z = 0.0f;            /* Gyro Z in deg/s */
volatile float mpu_temp_c = 0.0f;        /* Die temperature in C */

volatile int16_t baseline_ax = 0;        /* Stationary baseline X */
volatile int16_t baseline_ay = 0;        /* Stationary baseline Y */
volatile int16_t baseline_az = 0;        /* Stationary baseline Z */
volatile int max_delta_g_x100 = 0;       /* Impact deviation: max(|Delta a|) * 100 */

/* --- Event Classification States (PRD Section 4.2) --- */
volatile EventState state_motion   = STATE_NORMAL;
volatile EventState state_temp     = STATE_NORMAL;
volatile EventState state_obstacle = STATE_NORMAL;

volatile uint32_t critical_alert_count = 0;
volatile uint32_t loop_count = 0;
volatile uint8_t emergency_exti_triggered = 0; /* Set when hardware EXTI interrupt fires */
volatile int mpu_init_result = -1;       /* 0 = OK, -1 = Error */
volatile unsigned char mpu_id = 0x00;    /* WHO_AM_I register */

static uint8_t active_i2c_addr = MPU_I2C_ADDR;

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
 * Dual UART Driver (Simultaneous Output on USART1:PA9 & USART2:PA2 @ 115200)
 * ============================================================================
 */

static void uart_init(void)
{
    /* Enable GPIOA, GPIOB, USART1 (APB2), and USART2 (APB1) peripheral clocks */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    (void)RCC->AHB1ENR;
    (void)RCC->APB1ENR;
    (void)RCC->APB2ENR;

    /* 1. PA9 -> USART1 TX (AF7) */
    GPIOA->MODER &= ~(3U << (UART1_TX_PIN * 2U));
    GPIOA->MODER |=  (2U << (UART1_TX_PIN * 2U));
    GPIOA->AFR[1] &= ~(0xFU << ((UART1_TX_PIN - 8U) * 4U));
    GPIOA->AFR[1] |=  (7U   << ((UART1_TX_PIN - 8U) * 4U));
    GPIOA->OSPEEDR |= (3U << (UART1_TX_PIN * 2U));

    /* 2. PA2 -> USART2 TX (AF7) */
    GPIOA->MODER &= ~(3U << (UART2_TX_PIN * 2U));
    GPIOA->MODER |=  (2U << (UART2_TX_PIN * 2U));
    GPIOA->AFR[0] &= ~(0xFU << (UART2_TX_PIN * 4U));
    GPIOA->AFR[0] |=  (7U   << (UART2_TX_PIN * 4U));
    GPIOA->OSPEEDR |= (3U << (UART2_TX_PIN * 2U));

    /* 3. PB10 (Right Pin 17) & PB4 (Left Pin 14) -> Port B UART (Output, Push-Pull, High Speed, Idle HIGH) */
    GPIOB->MODER &= ~((3U << (SOFT_UART_TX_PIN * 2U)) | (3U << (SOFT_UART_TX_PIN2 * 2U)));
    GPIOB->MODER |=  ((1U << (SOFT_UART_TX_PIN * 2U)) | (1U << (SOFT_UART_TX_PIN2 * 2U)));
    GPIOB->OTYPER &= ~((1U << SOFT_UART_TX_PIN) | (1U << SOFT_UART_TX_PIN2));
    GPIOB->OSPEEDR |= ((3U << (SOFT_UART_TX_PIN * 2U)) | (3U << (SOFT_UART_TX_PIN2 * 2U)));
    GPIOB->PUPDR &= ~((3U << (SOFT_UART_TX_PIN * 2U)) | (3U << (SOFT_UART_TX_PIN2 * 2U)));
    GPIOB->PUPDR |=  ((1U << (SOFT_UART_TX_PIN * 2U)) | (1U << (SOFT_UART_TX_PIN2 * 2U)));
    GPIOB->BSRR = (1U << SOFT_UART_TX_PIN) | (1U << SOFT_UART_TX_PIN2); /* Idle HIGH */

    /* Configure USART1: 115200 baud @ 16 MHz HSI (BRR = 139) */
    USART1->CR1 = 0U;
    USART1->CR2 = 0U;
    USART1->CR3 = 0U;
    USART1->BRR = 139U;
    USART1->CR1 = USART_CR1_TE | USART_CR1_UE;

    /* Configure USART2: 115200 baud @ 16 MHz HSI (BRR = 139) */
    USART2->CR1 = 0U;
    USART2->CR2 = 0U;
    USART2->CR3 = 0U;
    USART2->BRR = 139U;
    USART2->CR1 = USART_CR1_TE | USART_CR1_UE;
}

static void uart_putc(char c)
{
    /* 1. Transmit to USART1 (PA9) with timeout to prevent hanging */
    uint32_t t1 = 5000U;
    while (((USART1->SR & USART_SR_TXE) == 0U) && (--t1 > 0U)) {
    }
    if (t1 > 0U) {
        USART1->DR = (uint8_t)c;
    }

    /* 2. Transmit to USART2 (PA2) with timeout to prevent hanging */
    uint32_t t2 = 5000U;
    while (((USART2->SR & USART_SR_TXE) == 0U) && (--t2 > 0U)) {
    }
    if (t2 > 0U) {
        USART2->DR = (uint8_t)c;
    }

    /* 3. Transmit simultaneously on PB10 & PB4 @ 115200 baud (139 cycles / bit) */
    const uint32_t bit_cycles = 139U;
    const uint32_t mask_set = (1U << SOFT_UART_TX_PIN) | (1U << SOFT_UART_TX_PIN2);
    const uint32_t mask_clr = (1U << (SOFT_UART_TX_PIN + 16U)) | (1U << (SOFT_UART_TX_PIN2 + 16U));

    uint32_t target = DWT->CYCCNT;

    /* Start bit: LOW */
    GPIOB->BSRR = mask_clr;
    target += bit_cycles;
    while ((int32_t)(DWT->CYCCNT - target) < 0) {
    }

    /* 8 Data bits (LSB first) */
    for (int i = 0; i < 8; i++) {
        if ((c >> i) & 1) {
            GPIOB->BSRR = mask_set;
        } else {
            GPIOB->BSRR = mask_clr;
        }
        target += bit_cycles;
        while ((int32_t)(DWT->CYCCNT - target) < 0) {
        }
    }

    /* Stop bit: HIGH */
    GPIOB->BSRR = mask_set;
    target += bit_cycles;
    while ((int32_t)(DWT->CYCCNT - target) < 0) {
    }
}


static void uart_send_string(const char *s)
{
    while (*s != '\0') {
        if (*s == '\n') {
            uart_putc('\r');
        }
        uart_putc(*s++);
    }
}

/* ============================================================================
 * GPIO Initialization
 * ============================================================================
 */

static void gpio_init(void)
{
    /* Enable GPIOA, GPIOB, and GPIOC clocks */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;
    (void)RCC->AHB1ENR;

    /* PC13: Onboard Blue LED (Push-Pull, Active LOW) */
    LED_PORT->MODER &= ~(3U << (LED_WARN_PIN * 2U));
    LED_PORT->MODER |=  (1U << (LED_WARN_PIN * 2U));
    LED_PORT->OTYPER &= ~(1U << LED_WARN_PIN);
    LED_PORT->OSPEEDR |= (3U << (LED_WARN_PIN * 2U));
    LED_PORT->BSRR = (1U << LED_WARN_PIN); /* LED OFF initially */

    /* PA0: LM35 Analog Input (Mode 11, No Pull) */
    LM35_ADC_PORT->MODER |= (3U << (LM35_ADC_PIN * 2U));
    LM35_ADC_PORT->PUPDR &= ~(3U << (LM35_ADC_PIN * 2U));

    /* PB0: HC-SR04 TRIG Output (Mode 01 = Push-Pull Output, Initial LOW) */
    HCSR04_PORT->MODER &= ~(3U << (HCSR04_TRIG_PIN * 2U));
    HCSR04_PORT->MODER |=  (1U << (HCSR04_TRIG_PIN * 2U));
    HCSR04_PORT->OTYPER &= ~(1U << HCSR04_TRIG_PIN);
    HCSR04_PORT->OSPEEDR |= (3U << (HCSR04_TRIG_PIN * 2U));
    HCSR04_PORT->PUPDR &= ~(3U << (HCSR04_TRIG_PIN * 2U));
    HCSR04_PORT->BSRR = (1U << (HCSR04_TRIG_PIN + 16U));

    /* PB1: HC-SR04 ECHO Input (Mode 00 = Input with Pull-Down) */
    HCSR04_PORT->MODER &= ~(3U << (HCSR04_ECHO_PIN * 2U));
    HCSR04_PORT->PUPDR &= ~(3U << (HCSR04_ECHO_PIN * 2U));
    HCSR04_PORT->PUPDR |=  (2U << (HCSR04_ECHO_PIN * 2U)); /* Pull-Down */

    /* PB6 (SCL) & PB7 (SDA): MPU6050 I2C (Open-Drain with Internal Pull-Up) */
    MPU_GPIO_PORT->MODER &= ~((3U << (MPU_SCL_PIN * 2U)) | (3U << (MPU_SDA_PIN * 2U)));
    MPU_GPIO_PORT->MODER |=  ((1U << (MPU_SCL_PIN * 2U)) | (1U << (MPU_SDA_PIN * 2U)));
    MPU_GPIO_PORT->OTYPER |= ((1U << MPU_SCL_PIN) | (1U << MPU_SDA_PIN));
    MPU_GPIO_PORT->OSPEEDR |= ((3U << (MPU_SCL_PIN * 2U)) | (3U << (MPU_SDA_PIN * 2U)));
    MPU_GPIO_PORT->PUPDR &= ~((3U << (MPU_SCL_PIN * 2U)) | (3U << (MPU_SDA_PIN * 2U)));
    MPU_GPIO_PORT->PUPDR |=  ((1U << (MPU_SCL_PIN * 2U)) | (1U << (MPU_SDA_PIN * 2U)));
    MPU_GPIO_PORT->BSRR = (1U << MPU_SCL_PIN) | (1U << MPU_SDA_PIN);
}

/* ============================================================================
 * GPIO External Interrupt (EXTI) Driver (PB12 Emergency Crash / SOS Trigger)
 * ============================================================================
 */

void EXTI15_10_IRQHandler(void)
{
    if ((EXTI->PR & (1U << EMERGENCY_EXTI_PIN)) != 0U) {
        EXTI->PR = (1U << EMERGENCY_EXTI_PIN); /* Clear interrupt flag (write 1 to clear) */
        (void)EXTI->PR;                         /* Drain Cortex-M4 write buffer to prevent re-entry */
        emergency_exti_triggered = 1U;
        state_motion = STATE_CRITICAL;
        critical_alert_count++;
        LED_PORT->ODR ^= (1U << LED_WARN_PIN); /* Emergency LED indicator */
    }
}

static void exti_init(void)
{
    /* 1. Enable SYSCFG Clock on APB2 */
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    (void)RCC->APB2ENR;

    /* 2. Configure PB12 as Input with Internal Pull-Up */
    EMERGENCY_EXTI_PORT->MODER &= ~(3U << (EMERGENCY_EXTI_PIN * 2U));
    EMERGENCY_EXTI_PORT->PUPDR &= ~(3U << (EMERGENCY_EXTI_PIN * 2U));
    EMERGENCY_EXTI_PORT->PUPDR |=  (1U << (EMERGENCY_EXTI_PIN * 2U)); /* 01 = Pull-Up */

    /* 3. Route EXTI12 to Port B (PB12) in SYSCFG_EXTICR4 */
    SYSCFG->EXTICR[3] &= ~(0xFU << 0U);
    SYSCFG->EXTICR[3] |=  (1U   << 0U); /* 1 = Port B */

    /* 4. Configure Falling-Edge Trigger on EXTI12 */
    EXTI->IMR  |= (1U << EMERGENCY_EXTI_PIN);  /* Unmask EXTI12 line */
    EXTI->FTSR |= (1U << EMERGENCY_EXTI_PIN);  /* Trigger on falling edge */
    EXTI->RTSR &= ~(1U << EMERGENCY_EXTI_PIN); /* Disable rising edge */

    /* 5. Enable EXTI15_10 Interrupt Vector in NVIC with Priority 2 */
    NVIC_SetPriority(EXTI15_10_IRQn, 2U);
    NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/* ============================================================================
 * LM35 Temperature Sensor Driver (ADC1)
 * ============================================================================
 */

static void adc_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    (void)RCC->APB2ENR;

    ADC->CCR &= ~(3U << 16U);

    ADC1->CR1 = 0U;
    ADC1->CR2 = 0U;
    ADC1->SQR1 = 0U;
    ADC1->SQR2 = 0U;
    ADC1->SQR3 = LM35_ADC_CHANNEL; /* Channel 0 = PA0 */

    /* 480 ADC clock cycles sample time for noise-free analog readings */
    ADC1->SMPR2 &= ~(7U << (LM35_ADC_CHANNEL * 3U));
    ADC1->SMPR2 |=  (7U << (LM35_ADC_CHANNEL * 3U));

    ADC1->CR2 |= ADC_CR2_ADON;
    delay_us(25U);
}

static uint16_t adc_read_single(void)
{
    ADC1->SR = 0U;
    ADC1->CR2 |= ADC_CR2_SWSTART;

    while ((ADC1->SR & ADC_SR_EOC) == 0U) {
    }

    return (uint16_t)(ADC1->DR & 0x0FFFU);
}

static void lm35_read_and_classify(void)
{
    uint32_t sum = 0;
    const int SAMPLES = 32;

    for (int i = 0; i < SAMPLES; i++) {
        sum += adc_read_single();
        delay_us(25U);
    }

    uint16_t adc_raw = (uint16_t)(sum / SAMPLES);
    adc_mv = ((float)adc_raw * (float)VREF_MV) / 4095.0f;

    /* LM35: 10 mV / degree Celsius */
    temperature = adc_mv / 10.0f;
    temperature_f = (temperature * 1.8f) + 32.0f;
    temp_c = (int)(temperature + 0.5f);

    /* PRD Event Classification (Section 5.2 F-LM-06) */
    if (temp_c >= (int)TEMP_THRESHOLD_C) {
        state_temp = STATE_CRITICAL;
    } else if (temp_c >= (int)TEMP_WARN_C) {
        state_temp = STATE_WARNING;
    } else {
        state_temp = STATE_NORMAL;
    }
}

/* ============================================================================
 * HC-SR04 Obstacle Distance Driver (PB0 TRIG, PB1 ECHO)
 * ============================================================================
 */

static void hcsr04_read_and_classify(void)
{
    uint32_t start;
    uint32_t timeout_cycles;

    /* 1. Send 15 us Trigger Pulse on PB0 */
    HCSR04_PORT->BSRR = (1U << HCSR04_TRIG_PIN);
    delay_us(15U);
    HCSR04_PORT->BSRR = (1U << (HCSR04_TRIG_PIN + 16U));

    /* 2. Wait for ECHO (PB1) to transition HIGH */
    start = DWT->CYCCNT;
    timeout_cycles = (HSI_HZ / 1000U) * 35U; /* 35 ms timeout */

    while (((HCSR04_PORT->IDR >> HCSR04_ECHO_PIN) & 1U) == 0U) {
        if ((uint32_t)(DWT->CYCCNT - start) > timeout_cycles) {
            distance = -1.0f;
            dist_cm = -1;
            state_obstacle = STATE_INVALID;
            return;
        }
    }

    /* 3. Measure duration that ECHO stays HIGH */
    uint32_t pulse_start = DWT->CYCCNT;
    timeout_cycles = (HSI_HZ / 1000U) * 40U; /* 40 ms max flight time */

    while (((HCSR04_PORT->IDR >> HCSR04_ECHO_PIN) & 1U) != 0U) {
        if ((uint32_t)(DWT->CYCCNT - pulse_start) > timeout_cycles) {
            distance = -1.0f;
            dist_cm = -1;
            state_obstacle = STATE_INVALID;
            return;
        }
    }

    uint32_t elapsed = (uint32_t)(DWT->CYCCNT - pulse_start);
    echo_pulse_us = elapsed / (HSI_HZ / 1000000U);
    float d = (float)echo_pulse_us / 58.0f;

    if (d >= 2.0f && d <= 400.0f) {
        distance = d;
        dist_cm = (int)(d + 0.5f);

        /* PRD Event Classification (Section 5.3 F-HC-07) */
        if (dist_cm <= (int)DIST_THRESHOLD_CM) {
            state_obstacle = STATE_CRITICAL;
        } else if (dist_cm <= (int)DIST_WARN_CM) {
            state_obstacle = STATE_WARNING;
        } else {
            state_obstacle = STATE_NORMAL;
        }
    } else {
        distance = -1.0f;
        dist_cm = -1;
        state_obstacle = STATE_INVALID;
    }
}

/* ============================================================================
 * Bit-Banged I2C Engine for MPU6050 (PB6 SCL, PB7 SDA)
 * ============================================================================
 */

static void i2c_clk_delay(void)
{
    delay_us(10U); /* 50 kHz I2C clock */
}

static void i2c_scl_hi(void)
{
    MPU_GPIO_PORT->BSRR = (1U << MPU_SCL_PIN);
    uint32_t timeout = 500U;
    while (((MPU_GPIO_PORT->IDR & (1U << MPU_SCL_PIN)) == 0U) && (--timeout > 0U)) {
        delay_us(1U);
    }
}

static void i2c_scl_lo(void)
{
    MPU_GPIO_PORT->BSRR = (1U << (MPU_SCL_PIN + 16U));
}

static void i2c_sda_hi(void)
{
    MPU_GPIO_PORT->BSRR = (1U << MPU_SDA_PIN);
}

static void i2c_sda_lo(void)
{
    MPU_GPIO_PORT->BSRR = (1U << (MPU_SDA_PIN + 16U));
}

static uint8_t i2c_sda_in(void)
{
    return (uint8_t)((MPU_GPIO_PORT->IDR >> MPU_SDA_PIN) & 1U);
}

static void i2c_bus_clear(void)
{
    i2c_sda_hi();
    for (int i = 0; i < 9; i++) {
        i2c_scl_lo();
        i2c_clk_delay();
        i2c_scl_hi();
        i2c_clk_delay();
    }
    i2c_sda_lo();
    i2c_clk_delay();
    i2c_scl_hi();
    i2c_clk_delay();
    i2c_sda_hi();
    i2c_clk_delay();
}

static void i2c_start(void)
{
    i2c_sda_hi();
    i2c_scl_hi();
    i2c_clk_delay();
    i2c_sda_lo();
    i2c_clk_delay();
    i2c_scl_lo();
    i2c_clk_delay();
}

static void i2c_stop(void)
{
    i2c_sda_lo();
    i2c_clk_delay();
    i2c_scl_hi();
    i2c_clk_delay();
    i2c_sda_hi();
    i2c_clk_delay();
}

static int i2c_write_byte(uint8_t byte)
{
    for (int i = 7; i >= 0; i--) {
        if ((byte >> i) & 1U) {
            i2c_sda_hi();
        } else {
            i2c_sda_lo();
        }
        i2c_clk_delay();
        i2c_scl_hi();
        i2c_clk_delay();
        i2c_scl_lo();
        i2c_clk_delay();
    }

    i2c_sda_hi();
    i2c_clk_delay();
    i2c_scl_hi();
    i2c_clk_delay();
    uint8_t ack = i2c_sda_in();
    i2c_scl_lo();
    i2c_clk_delay();

    return (ack == 0U) ? 0 : -1;
}

static uint8_t i2c_read_byte(int send_ack)
{
    uint8_t byte = 0U;
    i2c_sda_hi();

    for (int i = 7; i >= 0; i--) {
        i2c_clk_delay();
        i2c_scl_hi();
        i2c_clk_delay();
        if (i2c_sda_in()) {
            byte |= (1U << i);
        }
        i2c_scl_lo();
    }

    i2c_clk_delay();
    if (send_ack) {
        i2c_sda_lo();
    } else {
        i2c_sda_hi();
    }
    i2c_clk_delay();
    i2c_scl_hi();
    i2c_clk_delay();
    i2c_scl_lo();
    i2c_sda_hi();
    i2c_clk_delay();

    return byte;
}

static int i2c_write_reg(uint8_t reg, uint8_t data)
{
    i2c_start();
    if (i2c_write_byte((uint8_t)(active_i2c_addr << 1U)) != 0) {
        i2c_stop();
        return -1;
    }
    if (i2c_write_byte(reg) != 0) {
        i2c_stop();
        return -1;
    }
    if (i2c_write_byte(data) != 0) {
        i2c_stop();
        return -1;
    }
    i2c_stop();
    return 0;
}

static int i2c_read_reg(uint8_t reg, uint8_t *data)
{
    i2c_start();
    if (i2c_write_byte((uint8_t)(active_i2c_addr << 1U)) != 0) {
        i2c_stop();
        return -1;
    }
    if (i2c_write_byte(reg) != 0) {
        i2c_stop();
        return -1;
    }
    i2c_start();
    if (i2c_write_byte((uint8_t)((active_i2c_addr << 1U) | 1U)) != 0) {
        i2c_stop();
        return -1;
    }
    *data = i2c_read_byte(0);
    i2c_stop();
    return 0;
}

static int i2c_read_burst(uint8_t reg, uint8_t *buf, uint16_t len)
{
    if (len == 0U) return 0;

    i2c_start();
    if (i2c_write_byte((uint8_t)(active_i2c_addr << 1U)) != 0) {
        i2c_stop();
        return -1;
    }
    if (i2c_write_byte(reg) != 0) {
        i2c_stop();
        return -1;
    }
    i2c_start();
    if (i2c_write_byte((uint8_t)((active_i2c_addr << 1U) | 1U)) != 0) {
        i2c_stop();
        return -1;
    }
    for (uint16_t i = 0; i < len; i++) {
        buf[i] = i2c_read_byte(i < (len - 1U));
    }
    i2c_stop();
    return 0;
}

/* ============================================================================
 * MPU6050 Motion / Impact Driver & Baseline Calibrator
 * ============================================================================
 */

static int mpu6050_init(void)
{
    uint8_t addrs[2] = {0x68U, 0x69U};

    i2c_bus_clear();

    for (int a = 0; a < 2; a++) {
        active_i2c_addr = addrs[a];

        if (i2c_write_reg(0x6BU, 0x00U) == 0) {
            delay_ms(20U);

            uint8_t id = 0U;
            if (i2c_read_reg(0x75U, &id) == 0 && (id != 0x00U && id != 0xFFU)) {
                mpu_id = id;

                delay_ms(10U);
                i2c_write_reg(0x6BU, 0x01U); /* Clock: PLL with X gyro */
                delay_ms(5U);
                i2c_write_reg(0x6CU, 0x00U); /* Enable all 6 axes */
                i2c_write_reg(0x19U, 0x07U); /* 125 Hz sample rate */
                i2c_write_reg(0x1AU, 0x06U); /* DLPF ~5 Hz (PRD Section 5.1) */
                i2c_write_reg(0x1CU, 0x00U); /* Accelerometer +/- 2g */
                i2c_write_reg(0x1BU, 0x00U); /* Gyro +/- 250 deg/s */
                i2c_write_reg(0x38U, 0x00U); /* Disable interrupts */

                return 0;
            }
        }
    }

    return -1;
}

static int mpu6050_read_raw(volatile int16_t *out_ax, volatile int16_t *out_ay, volatile int16_t *out_az,
                            volatile int16_t *out_gx, volatile int16_t *out_gy, volatile int16_t *out_gz,
                            int16_t *out_temp)
{
    uint8_t raw[14];

    if (i2c_read_burst(0x3BU, raw, 14U) == 0) {
        *out_ax   = (int16_t)(((uint16_t)raw[0] << 8U) | raw[1]);
        *out_ay   = (int16_t)(((uint16_t)raw[2] << 8U) | raw[3]);
        *out_az   = (int16_t)(((uint16_t)raw[4] << 8U) | raw[5]);
        *out_temp = (int16_t)(((uint16_t)raw[6] << 8U) | raw[7]);
        *out_gx   = (int16_t)(((uint16_t)raw[8]  << 8U) | raw[9]);
        *out_gy   = (int16_t)(((uint16_t)raw[10] << 8U) | raw[11]);
        *out_gz   = (int16_t)(((uint16_t)raw[12] << 8U) | raw[13]);
        return 0;
    }

    return -1;
}

/* Establish stationary baseline on startup (PRD F-MPU-05) */
static void mpu6050_calibrate_baseline(void)
{
    int32_t sum_x = 0, sum_y = 0, sum_z = 0;
    int valid_count = 0;

    for (uint32_t i = 0; i < MPU_BASELINE_SAMPLES; i++) {
        int16_t x, y, z, gx_dummy, gy_dummy, gz_dummy, t_dummy;
        if (mpu6050_read_raw(&x, &y, &z, &gx_dummy, &gy_dummy, &gz_dummy, &t_dummy) == 0) {
            sum_x += x;
            sum_y += y;
            sum_z += z;
            valid_count++;
        }
        delay_ms(10U);
    }

    if (valid_count > 0) {
        baseline_ax = (int16_t)(sum_x / valid_count);
        baseline_ay = (int16_t)(sum_y / valid_count);
        baseline_az = (int16_t)(sum_z / valid_count);
    }
}

static void mpu6050_read_and_classify(void)
{
    int16_t raw_t = 0;

    if (mpu6050_read_raw(&ax, &ay, &az, &gx, &gy, &gz, &raw_t) == 0) {
        /* Convert to float g (+/- 2g: 16384 LSB/g) for Keil Watch & UART */
        accel_x = (float)ax / 16384.0f;
        accel_y = (float)ay / 16384.0f;
        accel_z = (float)az / 16384.0f;

        gyro_x = (float)gx / 131.0f;
        gyro_y = (float)gy / 131.0f;
        gyro_z = (float)gz / 131.0f;
        mpu_temp_c = ((float)raw_t / 340.0f) + 36.53f;

        /* Baseline Subtraction for Impact Detection (PRD F-MPU-06 & F-MPU-07) */
        int32_t delta_x = abs((int32_t)ax - (int32_t)baseline_ax);
        int32_t delta_y = abs((int32_t)ay - (int32_t)baseline_ay);
        int32_t delta_z = abs((int32_t)az - (int32_t)baseline_az);

        int32_t max_delta = delta_x;
        if (delta_y > max_delta) max_delta = delta_y;
        if (delta_z > max_delta) max_delta = delta_z;

        /* Scale to g x 100 using integer arithmetic (PRD F-MPU-08) */
        max_delta_g_x100 = (int)((max_delta * 100) / 16384);

        if (max_delta_g_x100 >= (int)ACCEL_THRESHOLD_G_X100) {
            state_motion = STATE_CRITICAL;
        } else if (max_delta_g_x100 >= (int)ACCEL_WARN_G_X100) {
            state_motion = STATE_WARNING;
        } else {
            state_motion = STATE_NORMAL;
        }

        mpu_init_result = 0;
    } else {
        state_motion = STATE_INVALID;
        i2c_bus_clear();
        mpu6050_init();
    }
}

/* ============================================================================
 * Event Manager & UART Alert Protocol (PRD Section 4.3 & 5.4)
 * ============================================================================
 */

static void transmit_alerts_and_telemetry(void)
{
    char msg[128];
    int any_critical = 0;

    /* 1. Motion / Impact Alert */
    if (state_motion == STATE_CRITICAL) {
        any_critical = 1;
        if ((loop_count % 5U) == 0U) {
            critical_alert_count++;
            if (emergency_exti_triggered) {
                uart_send_string("[ALERT] EXTI     | CRITICAL | Hardware Emergency Interrupt fired on PB12!\r\n");
                emergency_exti_triggered = 0U;
            }
            snprintf(msg, sizeof(msg),
                     "[ALERT] MOTION   | CRITICAL | Ax=%+5.2fg Ay=%+5.2fg Az=%+5.2fg\r\n",
                     (double)accel_x, (double)accel_y, (double)accel_z);
            uart_send_string(msg);
        }
    }

    /* 2. Temperature Overheat Alert */
    if (state_temp == STATE_CRITICAL) {
        any_critical = 1;
        if ((loop_count % 5U) == 0U) {
            critical_alert_count++;
            snprintf(msg, sizeof(msg),
                     "[ALERT] TEMP     | CRITICAL | T=%dC (threshold=%uC)\r\n",
                     temp_c, TEMP_THRESHOLD_C);
            uart_send_string(msg);
        }
    }

    /* 3. Obstacle Proximity Alert */
    if (state_obstacle == STATE_CRITICAL) {
        any_critical = 1;
        if ((loop_count % 5U) == 0U) {
            critical_alert_count++;
            snprintf(msg, sizeof(msg),
                     "[ALERT] OBSTACLE | CRITICAL | d=%dcm (threshold=%ucm)\r\n",
                     dist_cm, DIST_THRESHOLD_CM);
            uart_send_string(msg);
        }
    }

    /* 4. Normal System Status Heartbeat (every ~1 second = 10 loops) */
    if (!any_critical && (loop_count % 10U) == 0U) {
        uart_send_string("[INFO]  SYSTEM   | NORMAL   | All sensors OK\r\n");

        /* Send continuous telemetry block for Docklight / Android Auto dashboard */
        if (distance >= 0.0f) {
            snprintf(msg, sizeof(msg),
                     "[DATA]  T=%dC | d=%dcm | Ax=%+5.2fg Ay=%+5.2fg Az=%+5.2fg\r\n",
                     temp_c, dist_cm, (double)accel_x, (double)accel_y, (double)accel_z);
        } else {
            snprintf(msg, sizeof(msg),
                     "[DATA]  T=%dC | d=--cm | Ax=%+5.2fg Ay=%+5.2fg Az=%+5.2fg\r\n",
                     temp_c, (double)accel_x, (double)accel_y, (double)accel_z);
        }
        uart_send_string(msg);
    }

    /* 5. Local Indicator (PC13 Blue LED) */
    if (any_critical) {
        /* Rapid strobe on Critical alert */
        LED_PORT->ODR ^= (1U << LED_WARN_PIN);
    } else {
        /* Steady 1 Hz heartbeat blink when Normal */
        if ((loop_count % 5U) == 0U) {
            LED_PORT->ODR ^= (1U << LED_WARN_PIN);
        }
    }
}

/* ============================================================================
 * MAIN APPLICATION
 * ============================================================================
 */

int main(void)
{
    clock_init();
    dwt_init();
    gpio_init();
    exti_init();
    uart_init();
    adc_init();

    delay_ms(200U);

    /* PRD Startup Banner (F-UA-06) */
    uart_send_string("\r\n");
    uart_send_string("[INFO]  SYSTEM   | BOOT     | Vehicle Event Monitor v1.0\r\n");
    uart_send_string("[INFO]  SYSTEM   | INIT     | Calibrating sensors...\r\n");

    /* Initialize MPU6050 & establish baseline */
    mpu_init_result = mpu6050_init();
    if (mpu_init_result == 0) {
        mpu6050_calibrate_baseline();
        uart_send_string("[INFO]  SYSTEM   | INIT     | MPU6050 baseline calibrated\r\n");
    } else {
        uart_send_string("[WARN]  SYSTEM   | INIT     | MPU6050 not detected\r\n");
    }

    while (1) {
        loop_count++;

        /* 1. Poll LM35 Temperature Sensor */
        lm35_read_and_classify();

        /* 2. Poll HC-SR04 Obstacle Distance Sensor */
        hcsr04_read_and_classify();

        /* 3. Poll MPU6050 Motion Sensor */
        if (mpu_init_result == 0) {
            mpu6050_read_and_classify();
        } else {
            mpu_init_result = mpu6050_init();
        }

        /* 4. Transmit UART Alerts & Telemetry to Docklight / Android Auto */
        transmit_alerts_and_telemetry();

        /* 5. 100 ms main loop period (PRD Section 4.3 & NF-01) */
        delay_ms(100U);
    }
}
