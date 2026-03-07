/**
 * Autonexa — Pico Firmware Main
 *
 * Hiwonder Ackermann Steering Chassis
 *
 * Phase 1: Bench test via serial CLI
 *   - I2C communication with Hiwonder motor driver board
 *   - LD-1501MG servo steering on GPIO 12
 *   - Encoder reading from driver board
 *   - Safety watchdog with command timeout and E-STOP
 *
 * Phase 2 (future): micro-ROS executor replaces serial CLI
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "pico/stdlib.h"
#include "hardware/timer.h"
#include "hardware/i2c.h"

#include "config.h"
#include "servo.h"
#include "hiwonder_driver.h"
#include "safety.h"
#include "ackermann.h"

/* ── State ───────────────────────────────────────────────────── */
static float target_steer_rad = 0.0f;
static int8_t speed_left      = 0;
static int8_t speed_right     = 0;

static int32_t encoder_left   = 0;
static int32_t encoder_right  = 0;
static int32_t prev_enc_left  = 0;
static int32_t prev_enc_right = 0;

/* Odometry */
static odom_state_t odom;
static bool motors_enabled = false;

/* Telemetry counter */
static uint32_t loop_count = 0;

/* ── Serial command buffer ───────────────────────────────────── */
#define SERIAL_BUF_SIZE 128
static char serial_buf[SERIAL_BUF_SIZE];
static uint8_t serial_idx = 0;

/* ── Process text commands (Phase 1 bench testing) ───────────── */
static void process_serial_command(const char *cmd)
{
    /* === SERVO === */
    if (strncmp(cmd, "SERVO_PWM ", 10) == 0) {
        uint16_t pwm = (uint16_t)atoi(cmd + 10);
        servo_set_pwm_us(pwm);
        printf("OK SERVO_PWM %u\n", pwm);
    }
    else if (strncmp(cmd, "SERVO_ANGLE ", 12) == 0) {
        float angle = strtof(cmd + 12, NULL);
        servo_set_angle(angle);
        printf("OK SERVO_ANGLE %.3f\n", angle);
    }
    else if (strcmp(cmd, "SERVO_CENTER") == 0) {
        servo_center();
        printf("OK SERVO_CENTER\n");
    }

    /* === MOTOR (I2C via Hiwonder driver board) === */
    else if (strncmp(cmd, "SPEED_L ", 8) == 0) {
        speed_left = (int8_t)atoi(cmd + 8);
        if (motors_enabled && safety_is_ok()) {
            hiwonder_set_speed(MOTOR_CHANNEL_LEFT, speed_left);
        }
        safety_feed_watchdog();
        printf("OK SPEED_L %d\n", speed_left);
    }
    else if (strncmp(cmd, "SPEED_R ", 8) == 0) {
        speed_right = (int8_t)atoi(cmd + 8);
        if (motors_enabled && safety_is_ok()) {
            hiwonder_set_speed(MOTOR_CHANNEL_RIGHT, speed_right);
        }
        safety_feed_watchdog();
        printf("OK SPEED_R %d\n", speed_right);
    }
    else if (strncmp(cmd, "SPEED ", 6) == 0) {
        int8_t spd = (int8_t)atoi(cmd + 6);
        speed_left  = spd;
        speed_right = spd;
        if (motors_enabled && safety_is_ok()) {
            hiwonder_set_speeds(speed_left, speed_right);
        }
        safety_feed_watchdog();
        printf("OK SPEED %d\n", spd);
    }
    else if (strncmp(cmd, "STEER ", 6) == 0) {
        target_steer_rad = strtof(cmd + 6, NULL);
        servo_set_angle(target_steer_rad);
        safety_feed_watchdog();
        printf("OK STEER %.3f\n", target_steer_rad);
    }

    /* === ACKERMANN (vx, wz → speed + steer) === */
    else if (strncmp(cmd, "VEL ", 4) == 0) {
        float vx, wz;
        if (sscanf(cmd + 4, "%f %f", &vx, &wz) == 2) {
            float steer, v_l, v_r;
            ackermann_inverse(vx, wz, &steer, &v_l, &v_r);

            /* Convert m/s to driver board speed units (approx) */
            /* max speed ≈ 0.3 m/s → scale to 100 */
            float scale = 100.0f / 0.3f;
            speed_left  = (int8_t)(v_l * scale);
            speed_right = (int8_t)(v_r * scale);
            target_steer_rad = steer;

            if (speed_left  >  100) speed_left  =  100;
            if (speed_left  < -100) speed_left  = -100;
            if (speed_right >  100) speed_right =  100;
            if (speed_right < -100) speed_right = -100;

            servo_set_angle(steer);
            if (motors_enabled && safety_is_ok()) {
                hiwonder_set_speeds(speed_left, speed_right);
            }
            safety_feed_watchdog();
            printf("OK VEL vx=%.3f wz=%.3f → steer=%.3f L=%d R=%d\n",
                   vx, wz, steer, speed_left, speed_right);
        } else {
            printf("ERR VEL usage: VEL <vx_mps> <wz_radps>\n");
        }
    }

    /* === ENCODER === */
    else if (strcmp(cmd, "ENC_READ") == 0) {
        int32_t l = 0, r = 0;
        hiwonder_read_encoders(&l, &r);
        printf("ENC L=%ld R=%ld\n", (long)l, (long)r);
    }

    /* === I2C SCAN === */
    else if (strcmp(cmd, "I2C_SCAN") == 0) {
        printf("I2C scan on bus 0:\n");
        for (uint8_t addr = 0x08; addr < 0x78; addr++) {
            uint8_t dummy;
            int ret = i2c_read_timeout_us(I2C_PORT, addr,
                                          &dummy, 1, false, 5000);
            if (ret >= 0) {
                printf("  Found device at 0x%02X\n", addr);
            }
        }
        printf("Scan complete.\n");
    }

    /* === SAFETY === */
    else if (strcmp(cmd, "ESTOP") == 0) {
        safety_estop_activate();
        speed_left  = 0;
        speed_right = 0;
        printf("OK ESTOP\n");
    }
    else if (strcmp(cmd, "ESTOP_CLEAR") == 0) {
        safety_estop_clear();
        printf("OK ESTOP_CLEAR\n");
    }

    /* === ENABLE / DISABLE === */
    else if (strcmp(cmd, "ENABLE") == 0) {
        motors_enabled = true;
        safety_feed_watchdog();
        printf("OK ENABLE\n");
    }
    else if (strcmp(cmd, "DISABLE") == 0) {
        motors_enabled = false;
        speed_left  = 0;
        speed_right = 0;
        hiwonder_stop_all();
        printf("OK DISABLE\n");
    }

    /* === STOP === */
    else if (strcmp(cmd, "STOP") == 0) {
        speed_left  = 0;
        speed_right = 0;
        hiwonder_stop_all();
        servo_center();
        printf("OK STOP\n");
    }

    /* === STATUS === */
    else if (strcmp(cmd, "STATUS") == 0) {
        printf("STATUS enabled=%d estop=%d timeout=%d "
               "speed_L=%d speed_R=%d steer=%.3f "
               "enc_L=%ld enc_R=%ld "
               "odom x=%.3f y=%.3f yaw=%.2f\n",
               motors_enabled,
               safety_is_estopped(),
               safety_is_timed_out(),
               speed_left,
               speed_right,
               servo_get_angle(),
               (long)encoder_left,
               (long)encoder_right,
               odom.x, odom.y, odom.yaw);
    }

    /* === HELP === */
    else if (strcmp(cmd, "HELP") == 0) {
        printf("Commands:\n");
        printf("  SERVO_PWM <us>       - Raw servo PWM (500-2500)\n");
        printf("  SERVO_ANGLE <rad>    - Servo angle\n");
        printf("  SERVO_CENTER         - Center steering\n");
        printf("  SPEED <-100..100>    - Both motors speed\n");
        printf("  SPEED_L <-100..100>  - Left motor speed\n");
        printf("  SPEED_R <-100..100>  - Right motor speed\n");
        printf("  STEER <rad>          - Steering angle\n");
        printf("  VEL <vx> <wz>        - Ackermann velocity cmd\n");
        printf("  ENC_READ             - Read encoder counts\n");
        printf("  I2C_SCAN             - Scan I2C bus\n");
        printf("  ENABLE               - Enable motors\n");
        printf("  DISABLE              - Disable motors\n");
        printf("  STOP                 - Stop all + center\n");
        printf("  ESTOP                - Emergency stop\n");
        printf("  ESTOP_CLEAR          - Clear E-STOP\n");
        printf("  STATUS               - Print state\n");
    }
    else {
        printf("ERR unknown: %s (type HELP)\n", cmd);
    }
}

/* ── Poll serial for text commands ───────────────────────────── */
static void poll_serial(void)
{
    int ch;
    while ((ch = getchar_timeout_us(0)) != PICO_ERROR_TIMEOUT) {
        if (ch == '\n' || ch == '\r') {
            if (serial_idx > 0) {
                serial_buf[serial_idx] = '\0';
                process_serial_command(serial_buf);
                serial_idx = 0;
            }
        } else if (serial_idx < SERIAL_BUF_SIZE - 1) {
            serial_buf[serial_idx++] = (char)ch;
        }
    }
}

/* ── Telemetry output ────────────────────────────────────────── */
static void print_telemetry(void)
{
    /* Print every 5th loop (10 Hz at 50 Hz loop) */
    if (loop_count % 5 != 0) return;

    printf("TEL %lu,%d,%d,%.3f,%ld,%ld,%.3f,%.3f,%.2f,%d,%d\n",
           (unsigned long)to_ms_since_boot(get_absolute_time()),
           speed_left,
           speed_right,
           servo_get_angle(),
           (long)encoder_left,
           (long)encoder_right,
           odom.x, odom.y, odom.yaw,
           safety_is_estopped(),
           safety_is_timed_out());
}

/* ══════════════════════════════════════════════════════════════
 * MAIN
 * ══════════════════════════════════════════════════════════════ */

int main(void)
{
    /* ── Init Pico stdlib (USB serial) ───────────────────────── */
    stdio_init_all();
    sleep_ms(2000);  /* wait for USB serial */

    printf("\n");
    printf("╔═══════════════════════════════════════════╗\n");
    printf("║  AUTONEXA Pico Firmware v2.0              ║\n");
    printf("║  Hiwonder Ackermann Chassis               ║\n");
    printf("║  Phase 1 — Bench Test (Serial CLI)        ║\n");
    printf("║  Control freq: %d Hz                      ║\n", CONTROL_FREQ_HZ);
    printf("╚═══════════════════════════════════════════╝\n");
    printf("Type HELP for commands.\n\n");

    /* ── Init subsystems ─────────────────────────────────────── */
    servo_init();
    printf("[INIT] Servo OK (GPIO %d, center=%d µs)\n",
           SERVO_PIN, SERVO_PWM_CENTER_US);

    bool driver_ok = hiwonder_driver_init();
    printf("[INIT] Motor driver: %s\n", driver_ok ? "OK" : "NOT FOUND");

    safety_init();
    printf("[INIT] Safety watchdog OK (timeout=%d ms)\n", CMD_TIMEOUT_MS);

    ackermann_odom_reset(&odom);
    printf("[INIT] Ackermann kinematics ready\n");
    printf("[INIT] Wheelbase=%.2fm Track=%.2fm MaxSteer=%.1f°\n",
           WHEELBASE_M, TRACK_WIDTH_M, MAX_STEERING_RAD * 180.0f / 3.14159f);
    printf("\n");

    /* ── Main control loop ───────────────────────────────────── */
    absolute_time_t next_tick = get_absolute_time();

    while (true) {
        /* Wait for next control period */
        next_tick = delayed_by_us(next_tick, CONTROL_PERIOD_US);
        while (absolute_time_diff_us(get_absolute_time(), next_tick) > 0) {
            tight_loop_contents();
        }

        loop_count++;

        /* 1) Read encoders from driver board */
        hiwonder_read_encoders(&encoder_left, &encoder_right);

        /* 2) Compute odometry from encoder deltas */
        int32_t delta_l = encoder_left  - prev_enc_left;
        int32_t delta_r = encoder_right - prev_enc_right;
        prev_enc_left  = encoder_left;
        prev_enc_right = encoder_right;

        if (delta_l != 0 || delta_r != 0) {
            /* Convert encoder ticks to distance */
            float dist_l = (float)delta_l / (float)ENCODER_EDGES_PER_REV
                           * 2.0f * 3.14159f * WHEEL_RADIUS_M;
            float dist_r = (float)delta_r / (float)ENCODER_EDGES_PER_REV
                           * 2.0f * 3.14159f * WHEEL_RADIUS_M;

            float v_l = dist_l / CONTROL_DT_S;
            float v_r = dist_r / CONTROL_DT_S;

            ackermann_forward(v_l, v_r, servo_get_angle(),
                              CONTROL_DT_S, &odom);
        }

        /* 3) Safety check */
        safety_update();

        /* 4) Apply motor commands if enabled */
        if (motors_enabled && safety_is_ok()) {
            hiwonder_set_speeds(speed_left, speed_right);
        }

        /* 5) Poll serial */
        poll_serial();

        /* 6) Telemetry */
        print_telemetry();
    }

    return 0;
}
