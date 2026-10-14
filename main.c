#include "stm32f411xe.h"
#include <stdint.h>
#include <stdio.h>

/*
 * ============================================================================
 * STM32F411CEU6 ALL-IN-ONE 3-SENSOR MONITOR (BARE-METAL)
 * Clock: 16 MHz HSI
 *
 * SENSORS INTEGRATED:
 * 1. MPU6050 6-Axis Motion Sensor (Accelerometer + Gyroscope + Die Temp)
 *    - Bit-banged I2C Engine (Zero silicon errata, rock-solid breadboard timing)
 *    - Onboard Blue LED (PC13) blinks at 5 Hz when MPU6050 is reading live data!
 *
 * 2. LM35 Precision Centigrade Temperature Sensor:
 *    - ADC1 Channel 0 on PA0 (Analog input)
 *    - 64-sample oversampling for stable, noise-free room temperature readings
 *
 * 3. HC-SR04 Ultrasonic Distance Sensor:
 *    - PA1 : Trigger Output (25 us pulse)
 *    - PA2 : Echo Input (via 2-resistor voltage divider to protect STM32 3.3V pin)
 *    - Microsecond timing via DWT Cycle Counter
 *
 * 4. Serial Telemetry:
 *    - PA9 : USART1 TX (115200 baud, 8-N-1)
 * ============================================================================
 */

#define HSI_HZ               16000000U
#define ADC_VREF_MV          3300U

/* ============================================================================
 * GLOBAL WATCH VARIABLES (Visible in Keil Watch 1 Window)
 * ============================================================================
 */

/* --- Sensor 1: LM35 Temperature --- */
volatile float temperature = 0.0f;       /* Celsius */
volatile float temperature_f = 0.0f;     /* Fahrenheit */
volatile uint16_t adc_raw_val = 0;       /* Raw ADC conversion (0 - 4095) */
volatile float adc_mv = 0.0f;            /* Measured millivolts on PA0 */
volatile int lm35_warning = 0;           /* 1 = Overheating (>75C, check polarity!), 0 = Normal */

/* --- Sensor 2: HC-SR04 Ultrasonic Distance --- */
volatile float distance = -1.0f;         /* Distance in cm (-1 = Out of range / no echo) */
volatile uint32_t echo_pulse_us = 0;     /* Flight time in microseconds */
volatile int echo_status = -1;           /* 0 = Valid Echo, -1 = Timeout / Out of Range */

/* --- Sensor 3: MPU6050 Motion & Orientation --- */
volatile int16_t ax = 0, ay = 0, az = 0; /* Raw Accel counts (+/- 16384 per g) */
volatile int16_t gx = 0, gy = 0, gz = 0; /* Raw Gyro counts (+/- 131 per deg/s) */
volatile int16_t raw_temp = 0;
volatile float accel_x = 0.0f;           /* Accel X in g (at rest: ~0.00g) */
volatile float accel_y = 0.0f;           /* Accel Y in g (at rest: ~0.00g) */
volatile float accel_z = 0.0f;           /* Accel Z in g (flat on table: ~+1.00g) */
volatile float accel_magnitude = 0.0f;   /* Total gravity vector: sqrt(x^2+y^2+z^2) ~ 1.00g */
volatile float gyro_x = 0.0f;            /* Gyro X in deg/s (stationary: ~0.0 deg/s) */
volatile float gyro_y = 0.0f;            /* Gyro Y in deg/s (stationary: ~0.0 deg/s) */
volatile float gyro_z = 0.0f;            /* Gyro Z in deg/s (stationary: ~0.0 deg/s) */
volatile float mpu_temp_c = 0.0f;        /* MPU6050 internal silicon die temperature */

volatile int ok_ax = 0, ok_ay = 0, ok_az = 0;
volatile int ok_gx = 0, ok_gy = 0, ok_gz = 0;
volatile int mpu_init_result = -1;       /* 0 = MPU6050 OK, -1 = Error */
volatile unsigned char mpu_id = 0x00;    /* WHO_AM_I register (0x68 on MPU6050) */

volatile uint8_t i2c_active_addr = 0x68; /* I2C slave address (0x68 or 0x69) */
volatile uint8_t scl_pin_idle = 0;       /* 1 = SCL idle HIGH, 0 = Stuck LOW */
volatile uint8_t sda_pin_idle = 0;       /* 1 = SDA idle HIGH, 0 = Stuck LOW */
volatile uint32_t packets_read = 0;      /* Successful MPU readings counter */
volatile uint32_t loop_count = 0;        /* Main loop execution counter */

/* Pin configuration variables */
static uint8_t active_scl_pin = 6U;      /* PB6 default */
static uint8_t active_sda_pin = 7U;      /* PB7 default */
static uint8_t active_trig = 1U;         /* PA1 default */
static uint8_t active_echo = 2U;         /* PA2 default */

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

static float calc_sqrt(float val)
{
    if (val <= 0.0f) {
        return 0.0f;
    }
    float x = val;
    for (int i = 0; i < 8; i++) {
        x = 0.5f * (x + val / x);
    }
    return x;
}

/* ============================================================================
 * GPIO Initialization
 * ============================================================================
 */

static void gpio_init(void)
{
    /* Enable GPIOA, GPIOB, and GPIOC peripheral clocks */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;
    (void)RCC->AHB1ENR;

    /*
     * 1. PC13: Onboard Blue LED (Active LOW)
     * Output, Push-Pull, High Speed
     */
    GPIOC->MODER &= ~(3U << (13U * 2U));
    GPIOC->MODER |=  (1U << (13U * 2U));
    GPIOC->OTYPER &= ~(1U << 13U);
    GPIOC->OSPEEDR |= (3U << (13U * 2U));
    GPIOC->BSRR = (1U << 13U); /* LED OFF initially */

    /*
     * 2. PA0: Analog Input for LM35
     * Mode 11 = Analog, No Pull-Up/Down
     */
    GPIOA->MODER |= (3U << (0U * 2U));
    GPIOA->PUPDR &= ~(3U << (0U * 2U));

    /*
     * 3. PA1: HC-SR04 TRIG Output
     * Mode 01 = Output, Push-Pull, High Speed, Initial LOW
     */
    GPIOA->MODER &= ~(3U << (1U * 2U));
    GPIOA->MODER |=  (1U << (1U * 2U));
    GPIOA->OTYPER &= ~(1U << 1U);
    GPIOA->OSPEEDR |= (3U << (1U * 2U));
    GPIOA->BSRR = (1U << (1U + 16U));

    /*
     * 4. PA2: HC-SR04 ECHO Input
     * Mode 00 = Input, Floating (PUPDR = 00).
     * Resistor divider provides ground reference; floating prevents loading the divider.
     */
    GPIOA->MODER &= ~(3U << (2U * 2U));
    GPIOA->PUPDR &= ~(3U << (2U * 2U));

    /*
     * 5. PA9: USART1_TX (Alternate Function AF7)
     */
    GPIOA->MODER &= ~(3U << (9U * 2U));
    GPIOA->MODER |=  (2U << (9U * 2U));
    GPIOA->AFR[1] &= ~(0xFU << ((9U - 8U) * 4U));
    GPIOA->AFR[1] |=  (7U   << ((9U - 8U) * 4U));
    GPIOA->OSPEEDR |= (3U << (9U * 2U));

    /*
     * 6. Port B I2C Pins: PB6, PB7, PB8, PB9, PB10, PB11
     * Output Open-Drain with Internal Pull-Up enabled.
     */
    uint32_t i2c_pins = (1U << 6U) | (1U << 7U) | (1U << 8U) | (1U << 9U) | (1U << 10U) | (1U << 11U);

    for (int p = 0; p < 16; p++) {
        if (i2c_pins & (1U << p)) {
            GPIOB->MODER &= ~(3U << (p * 2U));
            GPIOB->MODER |=  (1U << (p * 2U)); /* Output */
            GPIOB->OTYPER |= (1U << p);         /* Open-Drain */
            GPIOB->OSPEEDR |= (3U << (p * 2U)); /* High Speed */
            GPIOB->PUPDR &= ~(3U << (p * 2U));
            GPIOB->PUPDR |=  (1U << (p * 2U)); /* Internal Pull-Up */
            GPIOB->BSRR = (1U << p);            /* Release High */
        }
    }
}

/* ============================================================================
 * USART1 Driver (115200 Baud, 8-N-1 on PA9)
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
 * ADC Driver for LM35 Temperature Sensor
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
    ADC1->SQR3 = 0U; /* Channel 0 = PA0 */

    /* 480 ADC clock cycles sample time for highest precision & low noise */
    ADC1->SMPR2 &= ~(7U << (0U * 3U));
    ADC1->SMPR2 |=  (7U << (0U * 3U));

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

static void update_temperature(void)
{
    uint32_t sum = 0;
    const int SAMPLES = 64;

    for (int i = 0; i < SAMPLES; i++) {
        sum += adc_read_single();
        delay_us(50U);
    }

    adc_raw_val = (uint16_t)(sum / SAMPLES);
    adc_mv = ((float)adc_raw_val * (float)ADC_VREF_MV) / 4095.0f;

    /* LM35: 10 mV / degree Celsius */
    temperature = adc_mv / 10.0f;
    temperature_f = (temperature * 1.8f) + 32.0f;

    /* Overheating detection: If temp > 75C at room temp, Pin 1 and 3 are reversed */
    lm35_warning = (temperature > 75.0f) ? 1 : 0;
}

/* ============================================================================
 * HC-SR04 Ultrasonic Distance Driver
 * ============================================================================
 */

static float ping_sensor(uint8_t trig, uint8_t echo)
{
    uint32_t start;
    uint32_t elapsed;
    uint32_t timeout_cycles;

    /* 1. Ensure ECHO line is idle (LOW) before triggering */
    start = DWT->CYCCNT;
    timeout_cycles = (HSI_HZ / 1000U) * 15U; /* 15 ms wait */
    while ((GPIOA->IDR & (1U << echo)) != 0U) {
        if ((uint32_t)(DWT->CYCCNT - start) > timeout_cycles) {
            return -1.0f;
        }
    }

    /* 2. Hold TRIG LOW for 5 us */
    GPIOA->BSRR = (1U << (trig + 16U));
    delay_us(5U);

    /* 3. Send 25 us Trigger Pulse */
    GPIOA->BSRR = (1U << trig);
    delay_us(25U);
    GPIOA->BSRR = (1U << (trig + 16U));

    /* 4. Wait for ECHO to transition HIGH */
    start = DWT->CYCCNT;
    timeout_cycles = (HSI_HZ / 1000U) * 25U; /* 25 ms timeout */
    while ((GPIOA->IDR & (1U << echo)) == 0U) {
        if ((uint32_t)(DWT->CYCCNT - start) > timeout_cycles) {
            return -1.0f;
        }
    }

    /* 5. Measure HIGH pulse duration */
    start = DWT->CYCCNT;
    timeout_cycles = (HSI_HZ / 1000U) * 35U; /* 35 ms max (~6 meters) */
    while ((GPIOA->IDR & (1U << echo)) != 0U) {
        if ((uint32_t)(DWT->CYCCNT - start) > timeout_cycles) {
            return -1.0f;
        }
    }

    elapsed = (uint32_t)(DWT->CYCCNT - start);

    echo_pulse_us = elapsed / (HSI_HZ / 1000000U);
    float dist_cm = (float)echo_pulse_us / 58.0f;

    if (dist_cm < 2.0f || dist_cm > 400.0f) {
        return -1.0f;
    }

    return dist_cm;
}

static void update_distance(void)
{
    float d = ping_sensor(active_trig, active_echo);

    if (d >= 0.0f) {
        distance = d;
        echo_status = 0;
        return;
    }

    /* Quick retry with 10 ms pause */
    delay_ms(10U);
    d = ping_sensor(active_trig, active_echo);
    if (d >= 0.0f) {
        distance = d;
        echo_status = 0;
        return;
    }

    /* Check if TRIG (PA1) and ECHO (PA2) wires were accidentally reversed */
    GPIOA->MODER &= ~(3U << (1U * 2U)); /* PA1 as Input */
    GPIOA->MODER &= ~(3U << (2U * 2U));
    GPIOA->MODER |=  (1U << (2U * 2U)); /* PA2 as Output */

    d = ping_sensor(2U, 1U);
    if (d >= 0.0f) {
        active_trig = 2U;
        active_echo = 1U;
        distance = d;
        echo_status = 0;
        uart_print("[HC-SR04] Auto-detected reversed wiring: PA2=TRIG, PA1=ECHO\n");
        return;
    }

    /* Restore default pins */
    GPIOA->MODER &= ~(3U << (1U * 2U));
    GPIOA->MODER |=  (1U << (1U * 2U)); /* PA1 as Output */
    GPIOA->MODER &= ~(3U << (2U * 2U)); /* PA2 as Input */
    active_trig = 1U;
    active_echo = 2U;

    distance = -1.0f;
    echo_status = -1;
}

/* ============================================================================
 * Bit-Banged I2C Engine for MPU6050 (Zero Errata, Breadboard 50 kHz Clock)
 * ============================================================================
 */

static void i2c_clk_delay(void)
{
    delay_us(10U); /* 50 kHz I2C clock */
}

static void i2c_scl_hi(void)
{
    GPIOB->BSRR = (1U << active_scl_pin);
    uint32_t timeout = 500U;
    while (((GPIOB->IDR & (1U << active_scl_pin)) == 0U) && (--timeout > 0U)) {
        delay_us(1U);
    }
}

static void i2c_scl_lo(void)
{
    GPIOB->BSRR = (1U << (active_scl_pin + 16U));
}

static void i2c_sda_hi(void)
{
    GPIOB->BSRR = (1U << active_sda_pin);
}

static void i2c_sda_lo(void)
{
    GPIOB->BSRR = (1U << (active_sda_pin + 16U));
}

static uint8_t i2c_sda_in(void)
{
    return (uint8_t)((GPIOB->IDR >> active_sda_pin) & 1U);
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

    /* 9th clock: Receive ACK from Slave */
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
    if (i2c_write_byte((uint8_t)(i2c_active_addr << 1U)) != 0) {
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
    if (i2c_write_byte((uint8_t)(i2c_active_addr << 1U)) != 0) {
        i2c_stop();
        return -1;
    }
    if (i2c_write_byte(reg) != 0) {
        i2c_stop();
        return -1;
    }
    i2c_start();
    if (i2c_write_byte((uint8_t)((i2c_active_addr << 1U) | 1U)) != 0) {
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
    if (i2c_write_byte((uint8_t)(i2c_active_addr << 1U)) != 0) {
        i2c_stop();
        return -1;
    }
    if (i2c_write_byte(reg) != 0) {
        i2c_stop();
        return -1;
    }
    i2c_start();
    if (i2c_write_byte((uint8_t)((i2c_active_addr << 1U) | 1U)) != 0) {
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
 * MPU6050 Auto-Probing & Reading Engine
 * ============================================================================
 */

static int mpu6050_try_pin_and_addr(uint8_t scl_pin, uint8_t sda_pin, uint8_t addr)
{
    active_scl_pin = scl_pin;
    active_sda_pin = sda_pin;
    i2c_active_addr = addr;

    i2c_bus_clear();

    scl_pin_idle = (uint8_t)((GPIOB->IDR >> active_scl_pin) & 1U);
    sda_pin_idle = (uint8_t)((GPIOB->IDR >> active_sda_pin) & 1U);

    if (!scl_pin_idle || !sda_pin_idle) {
        return -1;
    }

    if (i2c_write_reg(0x6BU, 0x00U) != 0) {
        return -2;
    }

    delay_ms(20U);

    uint8_t id = 0U;
    if (i2c_read_reg(0x75U, &id) != 0) {
        return -3;
    }

    if (id == 0x00U || id == 0xFFU) {
        return -4;
    }

    mpu_id = id;

    delay_ms(10U);
    i2c_write_reg(0x6BU, 0x01U); /* Clock: PLL with X gyro */
    delay_ms(5U);
    i2c_write_reg(0x6CU, 0x00U); /* Enable all 6 axes */
    i2c_write_reg(0x19U, 0x07U); /* 125 Hz sample rate */
    i2c_write_reg(0x1AU, 0x03U); /* DLPF 42 Hz */
    i2c_write_reg(0x1BU, 0x00U); /* Gyro +/- 250 deg/s */
    i2c_write_reg(0x1CU, 0x00U); /* Accel +/- 2g */
    i2c_write_reg(0x38U, 0x00U); /* Disable interrupts */

    return 0;
}

static int mpu6050_auto_probe(void)
{
    struct {
        uint8_t scl;
        uint8_t sda;
        const char *desc;
    } pairs[] = {
        {6U, 7U, "PB6 (SCL) & PB7 (SDA) [Standard]"},
        {7U, 6U, "PB7 (SCL) & PB6 (SDA) [Reversed]"},
        {8U, 9U, "PB8 (SCL) & PB9 (SDA) [Alternate]"},
        {9U, 8U, "PB9 (SCL) & PB8 (SDA) [Alternate Reversed]"},
        {10U, 11U, "PB10 (SCL) & PB11 (SDA) [I2C2]"}
    };

    const int NUM_PAIRS = sizeof(pairs) / sizeof(pairs[0]);
    uint8_t candidate_addrs[2] = {0x68U, 0x69U};
    char log[128];

    for (int p = 0; p < NUM_PAIRS; p++) {
        for (int a = 0; a < 2; a++) {
            if (mpu6050_try_pin_and_addr(pairs[p].scl, pairs[p].sda, candidate_addrs[a]) == 0) {
                snprintf(log, sizeof(log),
                         "[MPU6050] Connected on %s, Addr 0x%02X, ID = 0x%02X\n",
                         pairs[p].desc, candidate_addrs[a], mpu_id);
                uart_print(log);
                return 0;
            }
        }
    }

    active_scl_pin = 6U;
    active_sda_pin = 7U;
    i2c_active_addr = 0x68U;
    scl_pin_idle = (uint8_t)((GPIOB->IDR >> active_scl_pin) & 1U);
    sda_pin_idle = (uint8_t)((GPIOB->IDR >> active_sda_pin) & 1U);

    return -1;
}

static void update_mpu6050(void)
{
    uint8_t raw[14];

    if (i2c_read_burst(0x3BU, raw, 14U) == 0) {
        ax = (int16_t)(((uint16_t)raw[0] << 8U) | raw[1]);
        ay = (int16_t)(((uint16_t)raw[2] << 8U) | raw[3]);
        az = (int16_t)(((uint16_t)raw[4] << 8U) | raw[5]);

        raw_temp = (int16_t)(((uint16_t)raw[6] << 8U) | raw[7]);

        gx = (int16_t)(((uint16_t)raw[8]  << 8U) | raw[9]);
        gy = (int16_t)(((uint16_t)raw[10] << 8U) | raw[11]);
        gz = (int16_t)(((uint16_t)raw[12] << 8U) | raw[13]);

        /* Accelerometer in g (+/- 2g: 16384 LSB/g) */
        accel_x = (float)ax / 16384.0f;
        accel_y = (float)ay / 16384.0f;
        accel_z = (float)az / 16384.0f;
        accel_magnitude = calc_sqrt((accel_x * accel_x) + (accel_y * accel_y) + (accel_z * accel_z));

        /* Gyroscope in deg/s (+/- 250 deg/s: 131 LSB/(deg/s)) */
        gyro_x = (float)gx / 131.0f;
        gyro_y = (float)gy / 131.0f;
        gyro_z = (float)gz / 131.0f;

        /* MPU6050 internal die temperature */
        mpu_temp_c = ((float)raw_temp / 340.0f) + 36.53f;

        ok_ax = ok_ay = ok_az = 1;
        ok_gx = ok_gy = ok_gz = 1;
        mpu_init_result = 0;
        packets_read++;

        /*
         * USER FEATURE REQUEST: "keep the led blinking for the mpu6050"
         * Toggle Onboard Blue LED (PC13) every reading when MPU6050 is active!
         */
        GPIOC->ODR ^= (1U << 13U);
    } else {
        ok_ax = ok_ay = ok_az = 0;
        ok_gx = ok_gy = ok_gz = 0;

        i2c_bus_clear();
        mpu6050_auto_probe();

        /* If MPU is offline, turn LED OFF */
        GPIOC->BSRR = (1U << 13U);
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
    adc_init();

    delay_ms(250U);

    uart_print("\n\n");
    uart_print("===============================================================\n");
    uart_print("   STM32F411CEU6 ALL 3 SENSORS LIVE MONITOR (BARE-METAL)       \n");
    uart_print("   MPU6050 Motion + LM35 Temp + HC-SR04 Ultrasonic Distance   \n");
    uart_print("   Serial Telemetry: USART1 115200 baud on PA9 TX              \n");
    uart_print("===============================================================\n");

    mpu_init_result = mpu6050_auto_probe();

    if (mpu_init_result == 0) {
        uart_print("[STATUS] MPU6050 INITIALIZED AND STREAMING!\n");
    } else {
        uart_print("[WARNING] MPU6050 not detected. Retrying in background...\n");
    }

    while (1) {
        loop_count++;

        /* 1. Read LM35 Temperature Sensor */
        update_temperature();

        /* 2. Read HC-SR04 Ultrasonic Distance Sensor */
        update_distance();

        /* 3. Read MPU6050 (Blinks PC13 LED on each valid packet) */
        if (mpu_init_result == 0) {
            update_mpu6050();
        } else {
            mpu_init_result = mpu6050_auto_probe();
            /* Slow blink while searching */
            if ((loop_count % 4U) == 0U) {
                GPIOC->ODR ^= (1U << 13U);
            }
        }

        /* 4. Telemetry Output to USART1 Serial Console */
        uart_print("---------------------------------------------------------------\n");

        /* LM35 Status & Reading */
        if (lm35_warning) {
            snprintf(line, sizeof(line),
                     "[1] LM35 TEMP     : !! OVERHEATING %.1f C !! (Pin 1 and Pin 3 REVERSED!)\n",
                     (double)temperature);
        } else {
            snprintf(line, sizeof(line),
                     "[1] LM35 TEMP     : %.2f C  (%.2f F)  [Raw: %u, %.1f mV]\n",
                     (double)temperature, (double)temperature_f,
                     (unsigned int)adc_raw_val, (double)adc_mv);
        }
        uart_print(line);

        /* HC-SR04 Status & Reading */
        if (distance >= 0.0f) {
            snprintf(line, sizeof(line),
                     "[2] HC-SR04 DIST  : %.2f cm  [Echo Pulse: %u us]\n",
                     (double)distance, (unsigned int)echo_pulse_us);
        } else {
            snprintf(line, sizeof(line),
                     "[2] HC-SR04 DIST  : OUT OF RANGE / NO ECHO\n");
        }
        uart_print(line);

        /* MPU6050 Status & Reading */
        if (ok_ax && ok_ay && ok_az) {
            snprintf(line, sizeof(line),
                     "[3] MPU6050 ACCEL : X=%+6.3fg  Y=%+6.3fg  Z=%+6.3fg  [Total: %.2fg]\n",
                     (double)accel_x, (double)accel_y, (double)accel_z,
                     (double)accel_magnitude);
            uart_print(line);

            snprintf(line, sizeof(line),
                     "    MPU6050 GYRO  : X=%+6.2fd/s Y=%+6.2fd/s Z=%+6.2fd/s [Die: %.1fC]\n",
                     (double)gyro_x, (double)gyro_y, (double)gyro_z,
                     (double)mpu_temp_c);
            uart_print(line);
        } else {
            snprintf(line, sizeof(line),
                     "[3] MPU6050       : SEARCHING ON I2C (SCL=%d, SDA=%d)...\n",
                     scl_pin_idle, sda_pin_idle);
            uart_print(line);
        }

        delay_ms(150U);
    }
}
