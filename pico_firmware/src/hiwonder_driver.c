#include "hiwonder_driver.h"
#include "config.h"

#include "pico/stdlib.h"
#include "hardware/i2c.h"

#include <string.h>
#include <stdio.h>

/* ═══════════════════════════════════════════════════════════════
 * Hiwonder 4-Channel Motor Driver — I2C Register Map
 *
 * Confirmed against official Hiwonder MentorPi source code and
 * the YX-4055AM driver board protocol.
 *
 * Register layout:
 *   0x14 (20): Motor type config      (W, 4 bytes M1-M4, 3=JGB)
 *   0x15 (21): Encoder polarity       (W, 4 bytes M1-M4, 0=default)
 *   0x1F (31): Open-loop PWM          (W, 4×int8 M1-M4, -100..+100)
 *   0x33 (51): Closed-loop speed      (W, 4×int8 M1-M4, pulses/10ms)
 *   0x3C (60): Encoder total read     (R, 16 bytes, 4×int32 LE)
 *
 * Closed-loop speed mode (0x33) uses the board's onboard PID with
 * encoder feedback to maintain the commanded speed. Preferred over
 * open-loop PWM (0x1F) which varies with battery voltage and load.
 * ═══════════════════════════════════════════════════════════════ */

/* Register addresses (Hiwonder YX-4055AM 4-Channel Controller) */
#define REG_MOTOR_TYPE       0x14
#define REG_ENCODER_POLARITY 0x15
#define REG_OPEN_LOOP_PWM    0x1F    /* 4 bytes (M1, M2, M3, M4) -100 to 100 */
#define REG_CLOSED_LOOP_SPD  0x33    /* 4 bytes (M1, M2, M3, M4) speed cmd */
#define REG_ENCODER_READ     0x3C    /* 16 bytes (4x 32-bit little-endian) */

/* I2C timeout */
#define I2C_TIMEOUT_US       10000

/* ── Internal state ──────────────────────────────────────────── */
static bool driver_connected = false;
static int8_t motor_cmd_cache[4] = {0, 0, 0, 0};

/* ── I2C helpers ─────────────────────────────────────────────── */

static bool i2c_write_reg(uint8_t reg, const uint8_t *data, uint8_t len)
{
    uint8_t buf[5];  /* max: 1 reg + 4 data bytes */
    buf[0] = reg;
    if (len > 4) len = 4;
    memcpy(&buf[1], data, len);

    int ret = i2c_write_timeout_us(I2C_PORT, MOTOR_DRIVER_ADDR,
                                    buf, 1 + len, false, I2C_TIMEOUT_US);
    return (ret == (int)(1 + len));
}

static bool i2c_read_reg(uint8_t reg, uint8_t *data, uint8_t len)
{
    /* Write register address */
    int ret = i2c_write_timeout_us(I2C_PORT, MOTOR_DRIVER_ADDR,
                                    &reg, 1, true, I2C_TIMEOUT_US);
    if (ret != 1) return false;

    /* Read data */
    ret = i2c_read_timeout_us(I2C_PORT, MOTOR_DRIVER_ADDR,
                               data, len, false, I2C_TIMEOUT_US);
    return (ret == (int)len);
}

static int8_t clamp_speed(int8_t speed)
{
    if (speed > MOTOR_SPEED_MAX) return MOTOR_SPEED_MAX;
    if (speed < MOTOR_SPEED_MIN) return MOTOR_SPEED_MIN;
    return speed;
}

static int32_t decode_i32_le(const uint8_t *buf, int idx)
{
    uint32_t raw = (uint32_t)buf[idx]
                 | ((uint32_t)buf[idx + 1] << 8)
                 | ((uint32_t)buf[idx + 2] << 16)
                 | ((uint32_t)buf[idx + 3] << 24);
    return (int32_t)raw;
}

static bool write_motor_cache(void)
{
    uint8_t payload[4];
    for (int i = 0; i < 4; i++) {
        payload[i] = (uint8_t)motor_cmd_cache[i];
    }
    return i2c_write_reg(REG_CLOSED_LOOP_SPD, payload, 4);
}

/* ── Public API ──────────────────────────────────────────────── */

bool hiwonder_driver_init(void)
{
    /* Init I2C pins */
    i2c_init(I2C_PORT, I2C_FREQ_HZ);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    /* Check if driver board responds */
    driver_connected = hiwonder_is_connected();

    if (driver_connected) {
        printf("[HW_DRV] Motor driver board detected at 0x%02X\n",
               MOTOR_DRIVER_ADDR);

        /* Configure motor type: JGB37-520 series with Hall encoder */
        uint8_t motor_type[4] = {
            MOTOR_TYPE_JGB37, MOTOR_TYPE_JGB37,
            MOTOR_TYPE_JGB37, MOTOR_TYPE_JGB37
        };
        i2c_write_reg(REG_MOTOR_TYPE, motor_type, 4);

        /* Configure encoder polarity: default direction */
        uint8_t enc_polarity[4] = {
            ENCODER_POLARITY_DEFAULT, ENCODER_POLARITY_DEFAULT,
            ENCODER_POLARITY_DEFAULT, ENCODER_POLARITY_DEFAULT
        };
        i2c_write_reg(REG_ENCODER_POLARITY, enc_polarity, 4);

        /* Stop all motors on init */
        hiwonder_stop_all();

        printf("[HW_DRV] Motor type=JGB37, encoder polarity=default\n");
    } else {
        printf("[HW_DRV] WARNING: Motor driver board NOT detected at 0x%02X\n",
               MOTOR_DRIVER_ADDR);
    }

    memset(motor_cmd_cache, 0, sizeof(motor_cmd_cache));

    return driver_connected;
}

bool hiwonder_set_speed(uint8_t channel, int8_t speed)
{
    if (channel < 1 || channel > 4) {
        return false;
    }

    motor_cmd_cache[channel - 1] = clamp_speed(speed);
    return write_motor_cache();
}

bool hiwonder_set_speeds(int8_t speed_left, int8_t speed_right)
{
    for (int i = 0; i < 4; i++) {
        motor_cmd_cache[i] = 0;
    }
    motor_cmd_cache[MOTOR_CHANNEL_LEFT - 1]  = clamp_speed(speed_left);
    motor_cmd_cache[MOTOR_CHANNEL_RIGHT - 1] = clamp_speed(speed_right);

    return write_motor_cache();
}

bool hiwonder_read_encoder(uint8_t channel, int32_t *count)
{
    if (channel < 1 || channel > 4 || count == NULL) {
        return false;
    }

    uint8_t buf[16] = {0};
    if (!i2c_read_reg(REG_ENCODER_READ, buf, 16)) {
        return false;
    }

    int idx = (int)(channel - 1) * 4;
    *count = decode_i32_le(buf, idx);
    return true;
}

bool hiwonder_read_encoders(int32_t *left_count, int32_t *right_count)
{
    uint8_t buf[16] = {0};
    if (!i2c_read_reg(REG_ENCODER_READ, buf, 16)) {
        return false;
    }

    /* 16 bytes = 4x 32-bit little-endian sequences */
    /* Encoder slots follow M1..M4, four bytes per channel. */
    
    int l_idx = (MOTOR_CHANNEL_LEFT - 1) * 4;
    int r_idx = (MOTOR_CHANNEL_RIGHT - 1) * 4;

    *left_count = decode_i32_le(buf, l_idx);
    *right_count = decode_i32_le(buf, r_idx);
                             
    return true;
}

bool hiwonder_stop_all(void)
{
    memset(motor_cmd_cache, 0, sizeof(motor_cmd_cache));
    return write_motor_cache();
}

bool hiwonder_is_connected(void)
{
    /*
     * Attempt a zero-length write to the driver address.
     * If the device ACKs, it's present on the bus.
     */
    uint8_t dummy;
    int ret = i2c_read_timeout_us(I2C_PORT, MOTOR_DRIVER_ADDR,
                                   &dummy, 1, false, I2C_TIMEOUT_US);
    return (ret >= 0);
}
