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
static int32_t prev_encoder_left  = 0;
static int32_t prev_encoder_right = 0;

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

    prev_encoder_left  = 0;
    prev_encoder_right = 0;

    return driver_connected;
}

bool hiwonder_set_speed(uint8_t channel, int8_t speed)
{
    /* Not used directly anymore, use set_speeds for bulk transfer */
    return false;
}

bool hiwonder_set_speeds(int8_t speed_left, int8_t speed_right)
{
    /* Clamp speeds */
    if (speed_left > MOTOR_SPEED_MAX)  speed_left = MOTOR_SPEED_MAX;
    if (speed_left < MOTOR_SPEED_MIN)  speed_left = MOTOR_SPEED_MIN;
    if (speed_right > MOTOR_SPEED_MAX) speed_right = MOTOR_SPEED_MAX;
    if (speed_right < MOTOR_SPEED_MIN) speed_right = MOTOR_SPEED_MIN;

    /* The payload expects 4 bytes for M1, M2, M3, M4 */
    /* Hardware wiring: M2 = Left, M4 = Right */
    uint8_t p[4] = {0, 0, 0, 0};
    p[MOTOR_CHANNEL_LEFT - 1]  = (uint8_t)speed_left;
    p[MOTOR_CHANNEL_RIGHT - 1] = (uint8_t)speed_right;

    return i2c_write_reg(REG_OPEN_LOOP_PWM, p, 4);
}

bool hiwonder_read_encoder(uint8_t channel, int32_t *count)
{
    /* Not used directly anymore, use read_encoders for bulk transfer */
    return false;
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

    *left_count = (int32_t)(buf[l_idx] | (buf[l_idx+1] << 8) | 
                            (buf[l_idx+2] << 16) | (buf[l_idx+3] << 24));
                            
    *right_count = (int32_t)(buf[r_idx] | (buf[r_idx+1] << 8) | 
                             (buf[r_idx+2] << 16) | (buf[r_idx+3] << 24));
                             
    return true;
}

bool hiwonder_stop_all(void)
{
    uint8_t p[4] = {0, 0, 0, 0};
    return i2c_write_reg(REG_OPEN_LOOP_PWM, p, 4);
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
