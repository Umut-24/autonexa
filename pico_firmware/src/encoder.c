/*
 * encoder.c — 4x quadrature decoder for the two rear-wheel encoders.
 *
 * Decodes in the shared GPIO interrupt: on any edge of either channel we
 * read both A/B lines, form a 2-bit state, and look up the transition
 * (+1 / -1 / 0-on-invalid) against the previous state. This is the same
 * decoder proven by test/encoder_test.c, promoted to a firmware module.
 *
 * Pins / signs are from config.h. Channel B is A+1 for each encoder.
 */
#include "encoder.h"
#include "config.h"

#include "pico/stdlib.h"
#include "hardware/gpio.h"

/* Resolve A/B pins from the config A-pin defines. */
#define ENC_L_A  ENCODER_LEFT_A_PIN
#define ENC_L_B  (ENCODER_LEFT_A_PIN + 1)
#define ENC_R_A  ENCODER_RIGHT_A_PIN
#define ENC_R_B  (ENCODER_RIGHT_A_PIN + 1)

/* Quadrature transition table. State = (A << 1) | B, index =
 * (prev << 2) | curr. +1 / -1 for valid single steps, 0 for no-change
 * and for the "both bits flipped" case (a missed edge / glitch). */
static const int8_t QTABLE[16] = {
    0, +1, -1,  0,
   -1,  0,  0, +1,
   +1,  0,  0, -1,
    0, -1, +1,  0,
};

/* Raw counts — written by the IRQ, read by the control loop. 32-bit
 * aligned loads/stores are atomic on the Cortex-M0+. */
static volatile int32_t raw_left  = 0;
static volatile int32_t raw_right = 0;

static uint8_t prev_left  = 0;
static uint8_t prev_right = 0;

static inline uint8_t read_state(uint a_pin, uint b_pin)
{
    return (uint8_t)((gpio_get(a_pin) << 1) | gpio_get(b_pin));
}

/* Shared GPIO interrupt — fires on either edge of any of the 4 lines. */
static void encoder_gpio_callback(uint gpio, uint32_t events)
{
    (void)events;
    if (gpio == ENC_L_A || gpio == ENC_L_B) {
        uint8_t curr = read_state(ENC_L_A, ENC_L_B);
        raw_left += QTABLE[(prev_left << 2) | curr];
        prev_left = curr;
    } else if (gpio == ENC_R_A || gpio == ENC_R_B) {
        uint8_t curr = read_state(ENC_R_A, ENC_R_B);
        raw_right += QTABLE[(prev_right << 2) | curr];
        prev_right = curr;
    }
}

static void init_pin(uint pin)
{
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);
    /* Internal pull-up: harmless for push-pull Hall outputs, required
     * if the encoder happens to be open-drain. */
    gpio_pull_up(pin);
}

void encoder_init(void)
{
    init_pin(ENC_L_A);
    init_pin(ENC_L_B);
    init_pin(ENC_R_A);
    init_pin(ENC_R_B);

    /* Seed previous-state with the resting line levels so the first real
     * edge produces a valid transition, not a spurious glitch. */
    prev_left  = read_state(ENC_L_A, ENC_L_B);
    prev_right = read_state(ENC_R_A, ENC_R_B);

    /* One callback serves the whole GPIO IRQ bank; register it with the
     * first pin, enable the IRQ on the remaining three. */
    gpio_set_irq_enabled_with_callback(
        ENC_L_A, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true,
        &encoder_gpio_callback);
    gpio_set_irq_enabled(ENC_L_B, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(ENC_R_A, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(ENC_R_B, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
}

int32_t encoder_get_left(void)
{
    return (int32_t)ENCODER_LEFT_SIGN * raw_left;
}

int32_t encoder_get_right(void)
{
    return (int32_t)ENCODER_RIGHT_SIGN * raw_right;
}

void encoder_reset(void)
{
    raw_left  = 0;
    raw_right = 0;
}
