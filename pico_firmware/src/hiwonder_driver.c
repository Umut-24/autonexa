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

/* Register addresses (may need calibration) */
#define REG_MOTOR1_SPEED     0x01
#define REG_MOTOR2_SPEED     0x02
#define REG_MOTOR3_SPEED     0x03
#define REG_MOTOR4_SPEED     0x04
#define REG_STOP_ALL         0x10

#define REG_ENCODER1_COUNT   0x11
#define REG_ENCODER2_COUNT   0x12
#define REG_ENCODER3_COUNT   0x13
#define REG_ENCODER4_COUNT   0x14

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

static uint8_t channel_to_speed_reg(uint8_t channel)
{
    switch (channel) {
        case 1: return REG_MOTOR1_SPEED;
        case 2: return REG_MOTOR2_SPEED;
        case 3: return REG_MOTOR3_SPEED;
        case 4: return REG_MOTOR4_SPEED;
        default: return REG_MOTOR1_SPEED;
    }
}

static uint8_t channel_to_encoder_reg(uint8_t channel)
{
    switch (channel) {
        case 1: return REG_ENCODER1_COUNT;
        case 2: return REG_ENCODER2_COUNT;
        case 3: return REG_ENCODER3_COUNT;
        case 4: return REG_ENCODER4_COUNT;
        default: return REG_ENCODER1_COUNT;
    }
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
    /* Clamp */
    if (speed > MOTOR_SPEED_MAX)  speed = MOTOR_SPEED_MAX;
    if (speed < MOTOR_SPEED_MIN)  speed = MOTOR_SPEED_MIN;

    uint8_t reg = channel_to_speed_reg(channel);
    uint8_t data = (uint8_t)speed; /* signed → unsigned byte for I2C */
    return i2c_write_reg(reg, &data, 1);
}

bool hiwonder_set_speeds(int8_t speed_left, int8_t speed_right)
{
    bool ok1 = hiwonder_set_speed(MOTOR_CHANNEL_LEFT,  speed_left);
    bool ok2 = hiwonder_set_speed(MOTOR_CHANNEL_RIGHT, speed_right);
    return ok1 && ok2;
}

bool hiwonder_read_encoder(uint8_t channel, int32_t *count)
{
    uint8_t reg = channel_to_encoder_reg(channel);
    uint8_t buf[4] = {0};

    if (!i2c_read_reg(reg, buf, 4)) {
        return false;
    }

    /* Little-endian int32 */
    *count = (int32_t)(buf[0] | (buf[1] << 8) | (buf[2] << 16) | (buf[3] << 24));
    return true;
}

bool hiwonder_read_encoders(int32_t *left_count, int32_t *right_count)
{
    bool ok1 = hiwonder_read_encoder(MOTOR_CHANNEL_LEFT,  left_count);
    bool ok2 = hiwonder_read_encoder(MOTOR_CHANNEL_RIGHT, right_count);
    return ok1 && ok2;
}

bool hiwonder_stop_all(void)
{
    bool ok = true;
    for (uint8_t ch = 1; ch <= 4; ch++) {
        if (!hiwonder_set_speed(ch, 0)) {
            ok = false;
        }
    }
    return ok;
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
