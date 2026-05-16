/*
 * encoder_test.c — standalone Hiwonder motor-encoder bench test.
 *
 * Purpose: verify the two quadrature encoders are wired correctly and
 * counting cleanly, BEFORE touching the production firmware. This is a
 * self-contained program — it does not drive the motors, talk micro-ROS,
 * or use the ASCII CLI. Flash it, open the USB serial port, spin each
 * wheel by hand, and watch the counts.
 *
 * Wiring (matches the agreed pinout — see config.h once the defines land):
 *     Left  encoder  A -> GPIO 10   B -> GPIO 11
 *     Right encoder  A -> GPIO 12   B -> GPIO 13
 *     Both encoders  VCC -> 3V3(OUT) pin 36   GND -> any GND
 *
 * Decoding: full 4x quadrature in the shared GPIO IRQ. On every edge of
 * either channel we read both lines, form a 2-bit state, and look up the
 * transition (+1 / -1 / 0-on-invalid) against the previous state. At
 * hand-spin and low-RPM speeds the interrupt path will not miss edges;
 * for the production firmware the same decode should move to PIO so it
 * stays exact at full motor speed.
 *
 * Build:  cd pico_firmware/build && cmake .. && make encoder_test
 * Flash:  hold BOOTSEL, plug in, then
 *         cp encoder_test.uf2 /media/$USER/RPI-RP2/
 * View:   screen /dev/ttyACM0 115200      (or any serial monitor)
 *
 * Expected: spin LEFT wheel forward -> LEFT count climbs steadily.
 *           Spin it back -> count falls. Same for RIGHT. If a count
 *           jumps erratically or never moves, see the checklist printed
 *           at the bottom of the boot banner.
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"

/* ── Encoder pins ─────────────────────────────────────────────
 * Kept local to this test. Once verified, promote these to config.h
 * (ENCODER_LEFT_A_PIN, ...) for the production firmware.
 */
#define ENC_L_A 10
#define ENC_L_B 11
#define ENC_R_A 12
#define ENC_R_B 13

/* Counts per output-shaft revolution. Mirrors config.h:
 *   ENCODER_CPR (11) * 4 (quadrature) * MOTOR_GEAR_RATIO (30) = 1320
 */
#define EDGES_PER_REV 1320

/* ── Quadrature transition table ──────────────────────────────
 * State = (A << 1) | B, range 0..3. Index = (prev << 2) | curr.
 * +1 / -1 for valid single-step transitions, 0 for no-change and for
 * the "both bits flipped" case (a missed edge / glitch).
 */
static const int8_t QTABLE[16] = {
    0, +1, -1,  0,
   -1,  0,  0, +1,
   +1,  0,  0, -1,
    0, -1, +1,  0,
};

/* Counts are written by the IRQ, read by main(). 32-bit aligned loads
 * are atomic on the Cortex-M0+, so a plain read in main() is safe. */
static volatile int32_t count_l = 0;
static volatile int32_t count_r = 0;

/* Invalid-transition counters — a healthy encoder should keep these at
 * (or very near) zero. A climbing value means electrical noise, a loose
 * wire, or a too-slow IRQ missing edges. */
static volatile uint32_t glitch_l = 0;
static volatile uint32_t glitch_r = 0;

static uint8_t prev_l = 0;
static uint8_t prev_r = 0;

static inline uint8_t read_state(uint a_pin, uint b_pin)
{
    return (uint8_t)((gpio_get(a_pin) << 1) | gpio_get(b_pin));
}

/* Shared GPIO interrupt — fires on either edge of any of the 4 lines. */
static void gpio_callback(uint gpio, uint32_t events)
{
    (void)events;
    if (gpio == ENC_L_A || gpio == ENC_L_B) {
        uint8_t curr = read_state(ENC_L_A, ENC_L_B);
        int8_t  step = QTABLE[(prev_l << 2) | curr];
        if (step == 0 && curr != prev_l) {
            glitch_l++;            /* both bits changed = missed edge */
        }
        count_l += step;
        prev_l = curr;
    } else if (gpio == ENC_R_A || gpio == ENC_R_B) {
        uint8_t curr = read_state(ENC_R_A, ENC_R_B);
        int8_t  step = QTABLE[(prev_r << 2) | curr];
        if (step == 0 && curr != prev_r) {
            glitch_r++;
        }
        count_r += step;
        prev_r = curr;
    }
}

static void init_encoder_pin(uint pin)
{
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);
    /* Internal pull-up: harmless for push-pull Hall outputs, and
     * required if your encoder happens to be open-drain. */
    gpio_pull_up(pin);
}

int main(void)
{
    stdio_init_all();
    sleep_ms(2000);          /* give the USB CDC port time to enumerate */

    init_encoder_pin(ENC_L_A);
    init_encoder_pin(ENC_L_B);
    init_encoder_pin(ENC_R_A);
    init_encoder_pin(ENC_R_B);

    /* Seed the previous-state with the lines' resting levels so the
     * first real edge produces a valid transition, not a fake glitch. */
    prev_l = read_state(ENC_L_A, ENC_L_B);
    prev_r = read_state(ENC_R_A, ENC_R_B);

    /* One callback serves every GPIO IRQ. Register it with the first
     * pin, then just enable the IRQ on the remaining three. */
    gpio_set_irq_enabled_with_callback(
        ENC_L_A, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
    gpio_set_irq_enabled(ENC_L_B, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(ENC_R_A, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(ENC_R_B, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);

    printf("\n=== AutoNexa encoder bench test ===\n");
    printf("Left  enc: A=GPIO%d B=GPIO%d\n", ENC_L_A, ENC_L_B);
    printf("Right enc: A=GPIO%d B=GPIO%d\n", ENC_R_A, ENC_R_B);
    printf("%d edges per wheel revolution.\n", EDGES_PER_REV);
    printf("Spin a wheel forward -> its count should climb steadily.\n");
    printf("If a count never moves or glitches climb fast, check:\n");
    printf("  - encoder VCC is on 3V3 (NOT 5V), GND shared with Pico\n");
    printf("  - A/B wires on the right GPIOs, not swapped with VCC/GND\n");
    printf("  - A and B not shorted together\n");
    printf("-----------------------------------\n");

    int32_t last_l = 0, last_r = 0;

    while (true) {
        sleep_ms(250);

        int32_t cl = count_l;          /* atomic 32-bit reads */
        int32_t cr = count_r;
        uint32_t gl = glitch_l;
        uint32_t gr = glitch_r;

        int32_t dl = cl - last_l;
        int32_t dr = cr - last_r;
        last_l = cl;
        last_r = cr;

        /* Live A/B levels — useful to confirm the lines are alive even
         * before you spin anything (both should read 0 or 1, not float). */
        printf("L: cnt=%-8ld d=%-5ld rev=%+.3f AB=%d%d glitch=%lu  |  "
               "R: cnt=%-8ld d=%-5ld rev=%+.3f AB=%d%d glitch=%lu\n",
               (long)cl, (long)dl, (double)cl / EDGES_PER_REV,
               gpio_get(ENC_L_A), gpio_get(ENC_L_B), (unsigned long)gl,
               (long)cr, (long)dr, (double)cr / EDGES_PER_REV,
               gpio_get(ENC_R_A), gpio_get(ENC_R_B), (unsigned long)gr);
    }
    return 0;
}
