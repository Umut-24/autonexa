#ifndef HIWONDER_DRIVER_H
#define HIWONDER_DRIVER_H

#include <stdint.h>
#include <stdbool.h>

/**
 * Hiwonder 4-Channel Encoder Motor Driver — I2C Interface
 *
 * The Hiwonder motor driver board has its own MCU (YX-4055AM) that
 * handles motor PID and encoder counting internally. We communicate
 * via I2C to:
 *   - Configure motor type and encoder polarity (registers 0x14, 0x15)
 *   - Set motor speed for each channel M1–M4 (closed-loop: register 0x33)
 *   - Read accumulated encoder counts (register 0x3C, 4×int32 LE)
 *   - Stop all motors
 *
 * Motor channel mapping:
 *   M2 = left rear wheel
 *   M4 = right rear wheel
 *
 * Speed units: encoder pulses per 10ms (closed-loop PID on the board).
 * Clamped to MOTOR_SPEED_MIN..MOTOR_SPEED_MAX (see config.h).
 */

/** Initialise I2C bus and verify communication with driver board. */
bool hiwonder_driver_init(void);

/**
 * Set motor speed on a specific channel (closed-loop PID).
 * @param channel  Motor channel (1–4)
 * @param speed    Speed in pulses/10ms [MOTOR_SPEED_MIN..MOTOR_SPEED_MAX]. 0 = stop.
 * @return         true if I2C write succeeded.
 */
bool hiwonder_set_speed(uint8_t channel, int8_t speed);

/**
 * Set left and right rear motor speeds simultaneously (closed-loop PID).
 * @param speed_left   Left motor speed in pulses/10ms
 * @param speed_right  Right motor speed in pulses/10ms
 * @return             true if both I2C writes succeeded.
 */
bool hiwonder_set_speeds(int8_t speed_left, int8_t speed_right);

/**
 * Read encoder count from a specific channel.
 * The driver board maintains accumulated encoder ticks.
 * @param channel   Motor channel (1–4)
 * @param[out] count  Encoder tick count (signed 32-bit).
 * @return          true if I2C read succeeded.
 */
bool hiwonder_read_encoder(uint8_t channel, int32_t *count);

/**
 * Read both left and right encoder counts.
 * @param[out] left_count   Left wheel encoder ticks.
 * @param[out] right_count  Right wheel encoder ticks.
 * @return                  true if both reads succeeded.
 */
bool hiwonder_read_encoders(int32_t *left_count, int32_t *right_count);

/** Stop all motors immediately (speed = 0 for all channels). */
bool hiwonder_stop_all(void);

/** Check if the driver board is responding on I2C. */
bool hiwonder_is_connected(void);

#endif /* HIWONDER_DRIVER_H */
