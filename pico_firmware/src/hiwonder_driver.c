#include "hiwonder_driver.h"
#include "config.h"

#include "pico/stdlib.h"
#include "hardware/i2c.h"

#include <string.h>
#include <stdio.h>

/* ═══════════════════════════════════════════════════════════════
 * Hiwonder 4-Channel Motor Driver — I2C Register Map
 *
 * Based on reverse-engineering the Hiwonder chassis_control ROS1
 * package and the YX-4055AM driver board protocol.
 *
 * Register layout (write):
 *   0x01 : Motor 1 speed (int8_t, -100 to +100)
 *   0x02 : Motor 2 speed
 *   0x03 : Motor 3 speed
 *   0x04 : Motor 4 speed
 *   0x10 : Set all speeds to 0 (any value)
 *
 * Register layout (read):
 *   0x11 : Motor 1 encoder count (4 bytes, little-endian int32)
 *   0x12 : Motor 2 encoder count
 *   0x13 : Motor 3 encoder count
 *   0x14 : Motor 4 encoder count
 *
 * NOTE: These register addresses are based on Hiwonder's protocol.
 * If the actual board uses different registers, update the defines
 * below after testing with an I2C scan and protocol analysis.
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
    return i2c_write_reg(REG_OPEN_LOOP_PWM, payload, 4);
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
        /* Stop all motors on init */
        hiwonder_stop_all();
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
    /* Index for M2: (2-1)*4 = 4 */
    /* Index for M4: (4-1)*4 = 12 */
    
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
