#include "safety.h"
#include "config.h"
#include "hiwonder_driver.h"
#include "motor_control.h"
#include "servo.h"

#include "pico/stdlib.h"

/* ── Internal state ──────────────────────────────────────────── */
static bool     estop_active   = false;
static bool     timed_out      = false;
static uint32_t last_cmd_ms    = 0;
static bool     led_state      = false;
static uint32_t last_blink_ms  = 0;

/* ── Public API ──────────────────────────────────────────────── */

void safety_init(void)
{
    /* Heartbeat LED */
    gpio_init(HEARTBEAT_LED_PIN);
    gpio_set_dir(HEARTBEAT_LED_PIN, GPIO_OUT);
    gpio_put(HEARTBEAT_LED_PIN, 0);

    last_cmd_ms   = to_ms_since_boot(get_absolute_time());
    last_blink_ms = last_cmd_ms;
    estop_active  = false;
    timed_out     = false;
}

void safety_update(void)
{
    uint32_t now_ms = to_ms_since_boot(get_absolute_time());

    /* ── Check command timeout ──────────────────────────────── */
    if (!estop_active) {
        if ((now_ms - last_cmd_ms) > CMD_TIMEOUT_MS) {
            if (!timed_out) {
                /* Transition: running → timed out */
                motor_control_emergency_stop();
                timed_out = true;
            }
        } else {
            timed_out = false;
        }
    }

    /* ── E-STOP enforcement ─────────────────────────────────── */
    if (estop_active) {
        motor_control_emergency_stop();
    }

    /* ── Heartbeat LED ──────────────────────────────────────── */
    uint32_t blink_period_ms = estop_active ? 100 : 500;  /* 5 Hz / 1 Hz */

    if ((now_ms - last_blink_ms) >= blink_period_ms) {
        led_state = !led_state;
        gpio_put(HEARTBEAT_LED_PIN, led_state);
        last_blink_ms = now_ms;
    }
}

void safety_feed_watchdog(void)
{
    last_cmd_ms = to_ms_since_boot(get_absolute_time());
}

void safety_estop_activate(void)
{
    estop_active = true;
    motor_control_emergency_stop();
}

void safety_estop_clear(void)
{
    estop_active = false;
    timed_out    = false;
    last_cmd_ms  = to_ms_since_boot(get_absolute_time());
}

bool safety_is_estopped(void)
{
    return estop_active;
}

bool safety_is_timed_out(void)
{
    return timed_out;
}

bool safety_is_ok(void)
{
    return !estop_active && !timed_out;
}
