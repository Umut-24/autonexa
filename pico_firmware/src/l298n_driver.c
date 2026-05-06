#include "l298n_driver.h"
#include "config.h"

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"

/* ── Internal state ──────────────────────────────────────────── */

typedef struct {
    uint pin_in_a;
    uint pin_in_b;
    uint pin_en;
    uint pwm_slice;
    uint pwm_chan;
    bool reversed;   /* if true, flip the sign of duty before applying */
} l298n_chan_t;

/* Index 0 = channel 1 (LEFT motor on OUT3-OUT4),
 * Index 1 = channel 2 (RIGHT motor on OUT1-OUT2). */
static l298n_chan_t channels[2];
static uint16_t pwm_wrap = 99;  /* set in init based on PWM_FREQ_HZ */

/* ── Helpers ─────────────────────────────────────────────────── */

static int8_t clamp_speed(int8_t s)
{
    if (s > MOTOR_SPEED_MAX) return MOTOR_SPEED_MAX;
    if (s < MOTOR_SPEED_MIN) return MOTOR_SPEED_MIN;
    return s;
}

static int clamp_int(int v, int lo, int hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* Drive one channel with a duty in [-100, +100]. Direction pins set per
 * sign; PWM duty applied on the enable pin. duty=0 → coast. */
static void apply_duty(int idx, int duty_pct)
{
    duty_pct = clamp_int(duty_pct, -100, 100);

    /* Apply per-channel polarity (set in init from config.h). Lets the
     * caller's "positive = forward" stay consistent when a motor's two
     * output leads happen to be wired the opposite way at the L298N. */
    if (channels[idx].reversed) duty_pct = -duty_pct;

    if (duty_pct > 0) {
        gpio_put(channels[idx].pin_in_a, 1);
        gpio_put(channels[idx].pin_in_b, 0);
    } else if (duty_pct < 0) {
        gpio_put(channels[idx].pin_in_a, 0);
        gpio_put(channels[idx].pin_in_b, 1);
        duty_pct = -duty_pct;
    } else {
        /* Coast — both inputs low, PWM 0. */
        gpio_put(channels[idx].pin_in_a, 0);
        gpio_put(channels[idx].pin_in_b, 0);
    }

    /* Map duty% to PWM level. wrap = 99 → level 0..99 = 0..~99% duty.
     * For duty=100 the level reaches wrap (counter never below) which
     * the SDK treats as full-on. */
    uint16_t level = (uint16_t)((duty_pct * (int)pwm_wrap) / 100);
    if (duty_pct >= 100) level = pwm_wrap + 1;  /* fully high */
    pwm_set_chan_level(channels[idx].pwm_slice, channels[idx].pwm_chan, level);
}

/* ── Public API ──────────────────────────────────────────────── */

bool l298n_driver_init(void)
{
    /* Channel 1 = LEFT (OUT3-OUT4) */
    channels[0].pin_in_a = L298N_LEFT_IN3_PIN;
    channels[0].pin_in_b = L298N_LEFT_IN4_PIN;
    channels[0].pin_en   = L298N_LEFT_EN_PIN;
    channels[0].reversed = (L298N_LEFT_REVERSED != 0);

    /* Channel 2 = RIGHT (OUT1-OUT2) */
    channels[1].pin_in_a = L298N_RIGHT_IN1_PIN;
    channels[1].pin_in_b = L298N_RIGHT_IN2_PIN;
    channels[1].pin_en   = L298N_RIGHT_EN_PIN;
    channels[1].reversed = (L298N_RIGHT_REVERSED != 0);

    /* PWM: 1 MHz tick → wrap=99 → 100 ticks/period → 10 kHz. */
    pwm_wrap = 99;
    const float clkdiv = 125.0f;

    for (int i = 0; i < 2; i++) {
        /* Direction pins as plain GPIO output, default low. */
        gpio_init(channels[i].pin_in_a);
        gpio_set_dir(channels[i].pin_in_a, GPIO_OUT);
        gpio_put(channels[i].pin_in_a, 0);
        gpio_init(channels[i].pin_in_b);
        gpio_set_dir(channels[i].pin_in_b, GPIO_OUT);
        gpio_put(channels[i].pin_in_b, 0);

        /* Enable pin as PWM. */
        gpio_set_function(channels[i].pin_en, GPIO_FUNC_PWM);
        channels[i].pwm_slice = pwm_gpio_to_slice_num(channels[i].pin_en);
        channels[i].pwm_chan  = pwm_gpio_to_channel(channels[i].pin_en);
        pwm_set_clkdiv(channels[i].pwm_slice, clkdiv);
        pwm_set_wrap(channels[i].pwm_slice, pwm_wrap);
        pwm_set_chan_level(channels[i].pwm_slice, channels[i].pwm_chan, 0);
        pwm_set_enabled(channels[i].pwm_slice, true);
    }

    printf("[L298N] init: LEFT=ch1 (IN%d/IN%d/EN%d)  RIGHT=ch2 (IN%d/IN%d/EN%d)  PWM=%dHz\n",
           L298N_LEFT_IN3_PIN, L298N_LEFT_IN4_PIN, L298N_LEFT_EN_PIN,
           L298N_RIGHT_IN1_PIN, L298N_RIGHT_IN2_PIN, L298N_RIGHT_EN_PIN,
           L298N_PWM_FREQ_HZ);
    return true;
}

bool l298n_set_speed(uint8_t channel, int8_t speed)
{
    if (channel < 1 || channel > 2) return false;
    int idx = channel - 1;
    int8_t s = clamp_speed(speed);
    /* Map [-MOTOR_SPEED_MAX, +MOTOR_SPEED_MAX] → [-100, +100] %. */
    int duty = ((int)s * 100) / MOTOR_SPEED_MAX;
    apply_duty(idx, duty);
    return true;
}

bool l298n_set_speeds(int8_t speed_left, int8_t speed_right)
{
    l298n_set_speed(MOTOR_CHANNEL_LEFT,  speed_left);
    l298n_set_speed(MOTOR_CHANNEL_RIGHT, speed_right);
    return true;
}

bool l298n_set_raw_pwm(int8_t m1_pct, int8_t m2_pct)
{
    apply_duty(0, (int)m1_pct);
    apply_duty(1, (int)m2_pct);
    return true;
}

bool l298n_stop_all(void)
{
    for (int i = 0; i < 2; i++) {
        gpio_put(channels[i].pin_in_a, 0);
        gpio_put(channels[i].pin_in_b, 0);
        pwm_set_chan_level(channels[i].pwm_slice, channels[i].pwm_chan, 0);
    }
    return true;
}
