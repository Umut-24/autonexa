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
 *   - Set motor speed for each channel (M1–M4)
 *   - Read encoder counts from each channel
 *   - Stop all motors
 *
 * Motor channel mapping:
 *   M2 = left rear wheel
 *   M4 = right rear wheel
 *
 * Speed range: -100 to +100 (negative = reverse)
 */

/** Initialise I2C bus and verify communication with driver board. */
bool hiwonder_driver_init(void);

/**
 * Set motor speed on a specific channel.
 * @param channel  Motor channel (1–4)
 * @param speed    Speed value [-100 .. +100]. 0 = stop.
 * @return         true if I2C write succeeded.
 */
bool hiwonder_set_speed(uint8_t channel, int8_t speed);

/**
 * Set left and right rear motor speeds simultaneously.
 * @param speed_left   Left motor speed [-100 .. +100]
 * @param speed_right  Right motor speed [-100 .. +100]
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
