/*
 * encoder.h — quadrature wheel-encoder reader for the AutoNexa chassis.
 *
 * Two Hiwonder Hall encoders, one per rear-drive motor, decoded with full
 * 4x quadrature in the shared GPIO interrupt. Pins and forward-sign
 * correction come from config.h (ENCODER_LEFT_A_PIN, ENCODER_RIGHT_A_PIN,
 * ENCODER_LEFT_SIGN, ENCODER_RIGHT_SIGN); channel B is always A+1.
 *
 * Edge-rate budget: at the chassis top speed (~0.3 m/s) the motor shaft
 * turns ~43 rev/s, so each channel sees ~2 kedges/s — trivial for a GPIO
 * IRQ. The decoder used here is the same one verified by test/encoder_test.c.
 */
#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

/* Initialise the four encoder GPIOs (input + pull-up) and register the
 * shared quadrature IRQ. Call once during startup, before the control loop. */
void encoder_init(void);

/* Signed cumulative 4x-quadrature counts, forward-sign corrected.
 * 32-bit aligned reads are atomic on the Cortex-M0+, so these are
 * safe to call from the control loop while the IRQ updates them. */
int32_t encoder_get_left(void);
int32_t encoder_get_right(void);

/* Zero both counts (e.g. on an odometry reset). */
void encoder_reset(void);

#endif /* ENCODER_H */
