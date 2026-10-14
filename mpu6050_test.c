#include "stm32f411xe.h"
#include <stdint.h>
#include <stdio.h>

/*
 * ============================================================================
 * STM32F411CEU6 MPU6050 (GY-521) DEDICATED TEST PROGRAM
 * Clock: 16 MHz HSI (Internal High-Speed Oscillator)
 *
 * PURPOSE:
 * Focused, standalone diagnostic test for MPU6050 6-Axis Motion Sensor
 * (Accelerometer + Gyroscope + Die Temperature).
 * All other sensors (LM35, HC-SR04) have been removed as requested.
 *
 * HARDWARE CONNECTIONS (MPU6050 / GY-521 to STM32F411 Black Pill):
 * ----------------------------------------------------------------------------
 * | GY-521 Pin | STM32 Black Pill Pin | Physical Location    | Description   |
 * |------------|----------------------|----------------------|---------------|
 * | 1. VCC     | 5V  (or 3.3V)        | Left Header Pin 18   | Power Supply  |
 * | 2. GND     | GND                  | Left Header Pin 19   | Ground        |
 * | 3. SCL     | PB6                  | Left Header Pin 14   | I2C Clock     |
 * | 4. SDA     | PB7                  | Left Header Pin 15   | I2C Data      |
 * | 5. XDA     | NC                   | DO NOT CONNECT       | Leave Empty   |
 * | 6. XCL     | NC                   | DO NOT CONNECT       | Leave Empty   |
 * | 7. AD0     | GND                  | Left Header Pin 19   | I2C Addr 0x68 |
 * | 8. INT     | NC                   | DO NOT CONNECT       | Leave Empty   |
 * ----------------------------------------------------------------------------
 *
 * SERIAL TELEMETRY (Optional USB-to-UART dongle):
 * - PA9 (Left Header Pin 6) : USART1 TX (115200 baud, 8-N-1)
 *
 * ONBOARD LED (PC13):
 * - Fast toggle (10 Hz): MPU6050 actively communicating and streaming data!
 * - Slow toggle (1 Hz) : Waiting / Searching for MPU6050 on I2C bus.
 * ============================================================================
 */

#define HSI_HZ               16000000U

/* ============================================================================
 * KEIL WATCH 1 VARIABLES (Exact names matching your Watch window)
 * ============================================================================
 */

/* MPU-6050 Processed Physical Readings */
volatile float accel_x = 0.0f;           /* Accel X in g  (at rest: ~0.00 g) */
volatile float accel_y = 0.0f;           /* Accel Y in g  (at rest: ~0.00 g) */
volatile float accel_z = 0.0f;           /* Accel Z in g  (flat on table: ~+1.00 g) */
volatile float accel_magnitude = 0.0f;   /* Total gravity vector: sqrt(x^2+y^2+z^2) ~ 1.00 g */

volatile float gyro_x = 0.0f;            /* Gyro X in deg/s (stationary: ~0.0 deg/s) */
volatile float gyro_y = 0.0f;            /* Gyro Y in deg/s (stationary: ~0.0 deg/s) */
volatile float gyro_z = 0.0f;            /* Gyro Z in deg/s (stationary: ~0.0 deg/s) */

volatile float mpu_temp_c = 0.0f;        /* MPU6050 internal silicon die temperature in C */

/* Watch Window Compatibility Aliases */
volatile float temperature = 0.0f;       /* Set to mpu_temp_c for watch window */
volatile float temperature_f = 0.0f;     /* Die temp in Fahrenheit */
volatile float adc_mv = 0.0f;            /* 0.0 mV (LM35 unplugged) */
volatile float distance = 0.0f;          /* 0.0 cm (Ultrasonic unplugged) */
volatile uint32_t echo_pulse_us = 0;     /* 0 us (Ultrasonic unplugged) */

/* MPU-6050 Raw Register Values */
volatile int16_t ax = 0, ay = 0, az = 0; /* Raw Accel counts (+/- 16384 per g) */
volatile int16_t gx = 0, gy = 0, gz = 0; /* Raw Gyro counts (+/- 131 per deg/s) */
volatile int16_t raw_temp = 0;

/* Status & Diagnostic Indicators */
volatile int ok_ax = 0, ok_ay = 0, ok_az = 0;
volatile int ok_gx = 0, ok_gy = 0, ok_gz = 0;
volatile int mpu_init_result = -1;       /* 0 = MPU6050 OK, -1 = NOT INITIALIZED */
volatile unsigned char mpu_id = 0x00;    /* WHO_AM_I register (0x68 on MPU6050) */

volatile uint8_t i2c_active_addr = 0x68; /* Current I2C slave address (0x68 or 0x69) */
volatile uint8_t scl_pin_idle = 0;       /* 1 = SCL idle HIGH (Good), 0 = Stuck LOW (Wiring issue) */
volatile uint8_t sda_pin_idle = 0;       /* 1 = SDA idle HIGH (Good), 0 = Stuck LOW (Wiring issue) */
volatile uint32_t packets_read = 0;      /* Increments every successful sample read */
volatile uint32_t read_errors = 0;       /* Increments if an I2C transaction fails */
volatile uint32_t loop_count = 0;        /* Execution heartbeat counter */

/* Active Pin Pair Index */
volatile uint8_t active_scl_pin = 6U;    /* Default PB6 */
volatile uint8_t active_sda_pin = 7U;    /* Default PB7 */
const char *active_pin_desc = "PB6(SCL), PB7(SDA)";

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
     * Configure Onboard Blue LED on PC13
     * PC13: Output, Push-Pull, High Speed, Active LOW
     */
    GPIOC->MODER &= ~(3U << (13U * 2U));
    GPIOC->MODER |=  (1U << (13U * 2U));
    GPIOC->OTYPER &= ~(1U << 13U);
    GPIOC->OSPEEDR |= (3U << (13U * 2U));
    GPIOC->BSRR = (1U << 13U); /* LED OFF initially */

    /*
     * Configure USART1_TX on PA9 (Alternate Function AF7)
     */
    GPIOA->MODER &= ~(3U << (9U * 2U));
    GPIOA->MODER |=  (2U << (9U * 2U));
    GPIOA->AFR[1] &= ~(0xFU << ((9U - 8U) * 4U));
    GPIOA->AFR[1] |=  (7U   << ((9U - 8U) * 4U));
    GPIOA->OSPEEDR |= (3U << (9U * 2U));

    /*
     * Configure candidate I2C pins on Port B:
     * PB6, PB7, PB8, PB9, PB10, PB11
     * Set to Output Open-Drain with internal Pull-Up enabled.
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
 * USART1 Serial Telemetry Driver (115200 Baud, 8-N-1 on PA9)
 * ============================================================================
 */

static void uart_init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
    (void)RCC->APB2ENR;

    USART1->CR1 = 0U;
    USART1->CR2 = 0U;
    USART1->CR3 = 0U;
    USART1->BRR = 139U; /* 115200 baud @ 16 MHz HSI (16000000 / (16 * 115200) = 8.6875 -> Mantissa=8, Frac=11 -> 0x8B = 139) */
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
 * Bit-Banged I2C Engine (Zero Errata, Breadboard-Optimized 50 kHz Clock)
 * ============================================================================
 */

static void i2c_clk_delay(void)
{
    /* 10 us delay = 50 kHz I2C clock (ideal for jumper wires & breadboard) */
    delay_us(10U);
}

static void i2c_scl_hi(void)
{
    GPIOB->BSRR = (1U << active_scl_pin);
    /* Clock stretching check with timeout */
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

/*
 * 9-Clock Bus Unfreeze / Recovery
 * If MPU6050 was reset mid-read, it might be holding SDA low waiting for clocks.
 * Sending 9 clocks with SDA released safely unhangs the I2C slave.
 */
static void i2c_bus_clear(void)
{
    i2c_sda_hi();
    for (int i = 0; i < 9; i++) {
        i2c_scl_lo();
        i2c_clk_delay();
        i2c_scl_hi();
        i2c_clk_delay();
    }
    /* Generate STOP condition */
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

    /* 9th clock: Master sends ACK (0) or NACK (1) */
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
    i2c_start(); /* Repeated START */
    if (i2c_write_byte((uint8_t)((i2c_active_addr << 1U) | 1U)) != 0) {
        i2c_stop();
        return -1;
    }
    *data = i2c_read_byte(0); /* NACK on single byte read */
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
    i2c_start(); /* Repeated START */
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
 * MPU6050 Initialization and Multi-Pin Auto-Prober
 * ============================================================================
 */

static int mpu6050_try_pin_and_addr(uint8_t scl_pin, uint8_t sda_pin, uint8_t addr)
{
    active_scl_pin = scl_pin;
    active_sda_pin = sda_pin;
    i2c_active_addr = addr;

    i2c_bus_clear();

    /* Check idle line voltages */
    scl_pin_idle = (uint8_t)((GPIOB->IDR >> active_scl_pin) & 1U);
    sda_pin_idle = (uint8_t)((GPIOB->IDR >> active_sda_pin) & 1U);

    if (!scl_pin_idle || !sda_pin_idle) {
        return -1; /* Line is stuck low (shorted to GND or floating) */
    }

    /* Wake MPU-6050 up: Write 0x00 to PWR_MGMT_1 (0x6B) */
    if (i2c_write_reg(0x6BU, 0x00U) != 0) {
        return -2; /* No I2C ACK from slave address */
    }

    delay_ms(20U);

    /* Read WHO_AM_I register (0x75) */
    uint8_t id = 0U;
    if (i2c_read_reg(0x75U, &id) != 0) {
        return -3; /* Failed reading WHO_AM_I */
    }

    if (id == 0x00U || id == 0xFFU) {
        return -4; /* Invalid ID byte returned */
    }

    mpu_id = id;

    /* Device responded and confirmed ID! Now fully configure MPU6050: */
    delay_ms(10U);

    /* 1. Set Clock Source: PLL with X-Axis Gyro Reference */
    i2c_write_reg(0x6BU, 0x01U);
    delay_ms(5U);

    /* 2. Enable All 6 Axes (Accel X/Y/Z + Gyro X/Y/Z) */
    i2c_write_reg(0x6CU, 0x00U);

    /* 3. Sample Rate Divider: 1 kHz / (1 + 7) = 125 Hz */
    i2c_write_reg(0x19U, 0x07U);

    /* 4. Digital Low Pass Filter (DLPF): 42 Hz Bandwidth */
    i2c_write_reg(0x1AU, 0x03U);

    /* 5. Gyroscope Full Scale: +/- 250 deg/s (131 LSB / deg/s) */
    i2c_write_reg(0x1BU, 0x00U);

    /* 6. Accelerometer Full Scale: +/- 2g (16384 LSB / g) */
    i2c_write_reg(0x1CU, 0x00U);

    /* 7. Disable Interrupts (polling mode) */
    i2c_write_reg(0x38U, 0x00U);

    return 0; /* SUCCESS! */
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
            int res = mpu6050_try_pin_and_addr(pairs[p].scl, pairs[p].sda, candidate_addrs[a]);
            if (res == 0) {
                active_pin_desc = pairs[p].desc;
                snprintf(log, sizeof(log),
                         "[MPU6050] Connected on %s, Address 0x%02X, WHO_AM_I = 0x%02X\n",
                         pairs[p].desc, candidate_addrs[a], mpu_id);
                uart_print(log);
                return 0;
            }
        }
    }

    /* Fallback to default PB6/PB7 @ 0x68 for diagnostics */
    active_scl_pin = 6U;
    active_sda_pin = 7U;
    i2c_active_addr = 0x68U;
    active_pin_desc = pairs[0].desc;
    scl_pin_idle = (uint8_t)((GPIOB->IDR >> active_scl_pin) & 1U);
    sda_pin_idle = (uint8_t)((GPIOB->IDR >> active_sda_pin) & 1U);

    return -1;
}

/* ============================================================================
 * Read 14 Registers and Convert to Physical Units
 * ============================================================================
 */

static void mpu6050_read_all(void)
{
    uint8_t raw[14];

    if (i2c_read_burst(0x3BU, raw, 14U) == 0) {
        /* Parse 16-bit Big-Endian values */
        ax = (int16_t)(((uint16_t)raw[0] << 8U) | raw[1]);
        ay = (int16_t)(((uint16_t)raw[2] << 8U) | raw[3]);
        az = (int16_t)(((uint16_t)raw[4] << 8U) | raw[5]);

        raw_temp = (int16_t)(((uint16_t)raw[6] << 8U) | raw[7]);

        gx = (int16_t)(((uint16_t)raw[8]  << 8U) | raw[9]);
        gy = (int16_t)(((uint16_t)raw[10] << 8U) | raw[11]);
        gz = (int16_t)(((uint16_t)raw[12] << 8U) | raw[13]);

        /* Convert Accelerometer to g (+/- 2g range: 16384 LSB/g) */
        accel_x = (float)ax / 16384.0f;
        accel_y = (float)ay / 16384.0f;
        accel_z = (float)az / 16384.0f;

        /* Total acceleration magnitude: sqrt(X^2 + Y^2 + Z^2) */
        accel_magnitude = calc_sqrt((accel_x * accel_x) + (accel_y * accel_y) + (accel_z * accel_z));

        /* Convert Gyroscope to deg/s (+/- 250 deg/s range: 131 LSB/(deg/s)) */
        gyro_x = (float)gx / 131.0f;
        gyro_y = (float)gy / 131.0f;
        gyro_z = (float)gz / 131.0f;

        /* Convert Silicon Die Temperature (Datasheet formula: Temp_degC = (raw / 340) + 36.53) */
        mpu_temp_c = ((float)raw_temp / 340.0f) + 36.53f;
        temperature = mpu_temp_c;
        temperature_f = (temperature * 1.8f) + 32.0f;

        ok_ax = ok_ay = ok_az = 1;
        ok_gx = ok_gy = ok_gz = 1;
        mpu_init_result = 0;
        packets_read++;

        /* Toggle PC13 Onboard LED at 5 Hz to give visible heartbeat */
        if ((packets_read % 2U) == 0U) {
            GPIOC->ODR ^= (1U << 13U);
        }
    } else {
        ok_ax = ok_ay = ok_az = 0;
        ok_gx = ok_gy = ok_gz = 0;
        read_errors++;

        /* Bus error recovery */
        i2c_bus_clear();
        mpu6050_auto_probe();
    }
}

/* ============================================================================
 * MAIN APPLICATION ENTRY
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
    uart_print("   STM32F411CEU6 MPU6050 (GY-521) DEDICATED SENSOR TEST       \n");
    uart_print("   Console: USART1 115200 baud, 8-N-1 on PA9 TX               \n");
    uart_print("===============================================================\n");

    /* Probe for MPU-6050 across all pin combinations */
    mpu_init_result = mpu6050_auto_probe();

    if (mpu_init_result == 0) {
        uart_print("[STATUS] MPU6050 INITIALIZED AND READY!\n");
        uart_print("---------------------------------------------------------------\n");
    } else {
        uart_print("[WARNING] MPU6050 NOT DETECTED. Diagnostics:\n");
        snprintf(line, sizeof(line),
                 "  -> SCL Pin (PB6) Level: %s (Must be HIGH when idle)\n",
                 scl_pin_idle ? "HIGH (OK)" : "LOW (ERROR - wire grounded or missing pull-up)");
        uart_print(line);
        snprintf(line, sizeof(line),
                 "  -> SDA Pin (PB7) Level: %s (Must be HIGH when idle)\n",
                 sda_pin_idle ? "HIGH (OK)" : "LOW (ERROR - wire grounded or missing pull-up)");
        uart_print(line);
        uart_print("  -> Please verify VCC is plugged to 5V (or 3.3V) and GND to GND!\n");
        uart_print("---------------------------------------------------------------\n");
    }

    while (1) {
        loop_count++;

        if (mpu_init_result == 0) {
            mpu6050_read_all();

            /* Print Live Sensor Readings every 100 ms */
            snprintf(line, sizeof(line),
                     "ACCEL (g)  : X=%+6.3f  Y=%+6.3f  Z=%+6.3f  | Total = %.2fg\n",
                     (double)accel_x, (double)accel_y, (double)accel_z,
                     (double)accel_magnitude);
            uart_print(line);

            snprintf(line, sizeof(line),
                     "GYRO (d/s) : X=%+6.2f  Y=%+6.2f  Z=%+6.2f  | Die Temp = %.1f C\n",
                     (double)gyro_x, (double)gyro_y, (double)gyro_z,
                     (double)mpu_temp_c);
            uart_print(line);

            snprintf(line, sizeof(line),
                     "[OK] Packets: %u | Raw Counts: [AX:%d AY:%d AZ:%d] [GX:%d GY:%d GZ:%d]\n",
                     (unsigned int)packets_read, (int)ax, (int)ay, (int)az, (int)gx, (int)gy, (int)gz);
            uart_print(line);
            uart_print("---------------------------------------------------------------\n");
        } else {
            /* Retry probing every 500 ms */
            mpu_init_result = mpu6050_auto_probe();

            scl_pin_idle = (uint8_t)((GPIOB->IDR >> active_scl_pin) & 1U);
            sda_pin_idle = (uint8_t)((GPIOB->IDR >> active_sda_pin) & 1U);

            snprintf(line, sizeof(line),
                     "[SCANNING] Looking for MPU6050... SCL(PB6)=%d, SDA(PB7)=%d\n",
                     scl_pin_idle, sda_pin_idle);
            uart_print(line);

            /* Slow blink PC13 LED while searching */
            GPIOC->ODR ^= (1U << 13U);
        }

        delay_ms(100U);
    }
}
