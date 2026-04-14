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
 * Phase 2: micro-ROS executor replaces serial CLI (compile with -DUSE_MICRO_ROS)
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
#include "motor_control.h"

#ifdef USE_MICRO_ROS
#include "uros_transport.h"
#endif

/* ── State ───────────────────────────────────────────────────── */
static int32_t encoder_left   = 0;
static int32_t encoder_right  = 0;
static int32_t prev_enc_left  = 0;
static int32_t prev_enc_right = 0;

/* Odometry */
static odom_state_t odom;

/* Telemetry / loop counter */
static uint32_t loop_count = 0;

#ifndef USE_MICRO_ROS
/* ── Serial-only state (bench-test raw speed commands) ──────── */
static float   target_steer_rad = 0.0f;
static int8_t  speed_left       = 0;
static int8_t  speed_right      = 0;

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
        if (motor_control_is_enabled() && safety_is_ok()) {
            hiwonder_set_speed(MOTOR_CHANNEL_LEFT, speed_left);
        }
        safety_feed_watchdog();
        printf("OK SPEED_L %d\n", speed_left);
    }
    else if (strncmp(cmd, "SPEED_R ", 8) == 0) {
        speed_right = (int8_t)atoi(cmd + 8);
        if (motor_control_is_enabled() && safety_is_ok()) {
            hiwonder_set_speed(MOTOR_CHANNEL_RIGHT, speed_right);
        }
        safety_feed_watchdog();
        printf("OK SPEED_R %d\n", speed_right);
    }
    else if (strncmp(cmd, "SPEED ", 6) == 0) {
        int8_t spd = (int8_t)atoi(cmd + 6);
        speed_left  = spd;
        speed_right = spd;
        if (motor_control_is_enabled() && safety_is_ok()) {
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
            motor_control_set_velocity(vx, wz);
            safety_feed_watchdog();
            printf("OK VEL vx=%.3f wz=%.3f → steer=%.3f L=%d R=%d\n",
                   vx, wz, motor_control_get_steer_rad(),
                   motor_control_get_speed_left(),
                   motor_control_get_speed_right());
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
    
    /* === RAW PWM INJECTION (M1, M2, M3, M4) === */
    else if (strncmp(cmd, "RAW_PWM ", 8) == 0) {
        int m1, m2, m3, m4;
        if (sscanf(cmd + 8, "%d %d %d %d", &m1, &m2, &m3, &m4) == 4) {
            uint8_t buf[5] = {0x1F, (uint8_t)m1, (uint8_t)m2, (uint8_t)m3, (uint8_t)m4};
            int ret = i2c_write_timeout_us(I2C_PORT, MOTOR_DRIVER_ADDR, 
                                           buf, 5, false, 5000);
            printf("OK RAW_PWM %d %d %d %d (ret=%d)\n", m1, m2, m3, m4, ret);
        } else {
            printf("ERR RAW_PWM usage: RAW_PWM <m1> <m2> <m3> <m4>\n");
        }
    }

    /* === RAW PID INJECTION (M1, M2, M3, M4) === */
    else if (strncmp(cmd, "RAW_PID ", 8) == 0) {
        int m1, m2, m3, m4;
        if (sscanf(cmd + 8, "%d %d %d %d", &m1, &m2, &m3, &m4) == 4) {
            uint8_t buf[5] = {0x33, (uint8_t)m1, (uint8_t)m2, (uint8_t)m3, (uint8_t)m4};
            int ret = i2c_write_timeout_us(I2C_PORT, MOTOR_DRIVER_ADDR, 
                                           buf, 5, false, 5000);
            printf("OK RAW_PID %d %d %d %d (ret=%d)\n", m1, m2, m3, m4, ret);
        } else {
            printf("ERR RAW_PID usage: RAW_PID <m1> <m2> <m3> <m4>\n");
        }
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
    /* === I2C RAW DEBUG === */
    else if (strncmp(cmd, "I2C_WRITE ", 10) == 0) {
        int reg, val;
        if (sscanf(cmd + 10, "%d %d", &reg, &val) == 2) {
            uint8_t buf[2] = {(uint8_t)reg, (uint8_t)val};
            int ret = i2c_write_timeout_us(I2C_PORT, MOTOR_DRIVER_ADDR, buf, 2, false, 5000);
            printf("I2C_WRITE reg=%d val=%d ret=%d\n", reg, val, ret);
        }
    }
    else if (strncmp(cmd, "I2C_READ ", 9) == 0) {
        int reg, len;
        if (sscanf(cmd + 9, "%d %d", &reg, &len) == 2) {
            uint8_t reg_u8 = (uint8_t)reg;
            i2c_write_timeout_us(I2C_PORT, MOTOR_DRIVER_ADDR, &reg_u8, 1, true, 5000);
            uint8_t buf[16] = {0};
            if (len > 16) len = 16;
            int ret = i2c_read_timeout_us(I2C_PORT, MOTOR_DRIVER_ADDR, buf, len, false, 5000);
            printf("I2C_READ reg=%d len=%d ret=%d data=", reg, len, ret);
            for(int i=0; i<len; i++) printf("%02X ", buf[i]);
            printf("\n");
        }
    }

    /* === SAFETY === */
    else if (strcmp(cmd, "ESTOP") == 0) {
        safety_estop_activate();
        motor_control_stop();
        printf("OK ESTOP\n");
    }
    else if (strcmp(cmd, "ESTOP_CLEAR") == 0) {
        safety_estop_clear();
        printf("OK ESTOP_CLEAR\n");
    }

    /* === ENABLE / DISABLE === */
    else if (strcmp(cmd, "ENABLE") == 0) {
        motor_control_enable(true);
        printf("OK ENABLE\n");
    }
    else if (strcmp(cmd, "DISABLE") == 0) {
        motor_control_enable(false);
        printf("OK DISABLE\n");
    }

    /* === STOP === */
    else if (strcmp(cmd, "STOP") == 0) {
        motor_control_stop();
        printf("OK STOP\n");
    }

    /* === STATUS === */
    else if (strcmp(cmd, "STATUS") == 0) {
        printf("STATUS enabled=%d estop=%d timeout=%d "
               "speed_L=%d speed_R=%d steer=%.3f "
               "enc_L=%ld enc_R=%ld "
               "odom x=%.3f y=%.3f yaw=%.2f\n",
               motor_control_is_enabled(),
               safety_is_estopped(),
               safety_is_timed_out(),
               motor_control_get_speed_left(),
               motor_control_get_speed_right(),
               servo_get_angle(),
               (long)encoder_left,
               (long)encoder_right,
               odom.x, odom.y, odom.yaw);
    }

    /* === HELP === */
    else if (strcmp(cmd, "HELP") == 0) {
        printf("Commands:\n");
        printf("  SERVO_PWM <us>       - Raw servo PWM (500-2500)\n");
        printf("  SERVO_ANGLE <rad>    - Servo angle (±0.52 rad = ±30 deg)\n");
        printf("  SERVO_CENTER         - Center steering\n");
        printf("  SPEED <-30..30>      - Both motors (closed-loop, pulses/10ms)\n");
        printf("  SPEED_L <-30..30>    - Left motor speed\n");
        printf("  SPEED_R <-30..30>    - Right motor speed\n");
        printf("  STEER <rad>          - Steering angle\n");
        printf("  VEL <vx> <wz>        - Ackermann velocity cmd (m/s, rad/s)\n");
        printf("  ENC_READ             - Read encoder counts\n");
        printf("  RAW_PWM <m1..m4>     - Open-loop PWM bypass (reg 0x1F)\n");
        printf("  RAW_PID <m1..m4>     - Closed-loop bypass (reg 0x33)\n");
        printf("  I2C_SCAN             - Scan I2C bus\n");
        printf("  I2C_WRITE <reg> <val> - Raw I2C write\n");
        printf("  I2C_READ <reg> <len> - Raw I2C read\n");
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
           motor_control_get_speed_left(),
           motor_control_get_speed_right(),
           servo_get_angle(),
           (long)encoder_left,
           (long)encoder_right,
           odom.x, odom.y, odom.yaw,
           safety_is_estopped(),
           safety_is_timed_out());
}

#endif /* !USE_MICRO_ROS */

/* ══════════════════════════════════════════════════════════════
 * MAIN
 * ══════════════════════════════════════════════════════════════ */

int main(void)
{
#ifndef USE_MICRO_ROS
    /* ── Init Pico stdlib (USB serial) ───────────────────────── */
    stdio_init_all();
#endif

    /* Pre-init Heartbeat LED to verify power and boot */
    gpio_init(HEARTBEAT_LED_PIN);
    gpio_set_dir(HEARTBEAT_LED_PIN, GPIO_OUT);
    for(int i=0; i<4; i++) {
        gpio_put(HEARTBEAT_LED_PIN, 1);
        sleep_ms(100);
        gpio_put(HEARTBEAT_LED_PIN, 0);
        sleep_ms(100);
    }

#ifndef USE_MICRO_ROS
    sleep_ms(2000);  /* wait for USB serial */

    printf("\n");
    printf("╔═══════════════════════════════════════════╗\n");
    printf("║  AUTONEXA Pico Firmware v2.0              ║\n");
    printf("║  Hiwonder Ackermann Chassis               ║\n");
    printf("║  Phase 1 — Bench Test (Serial CLI)        ║\n");
    printf("║  Control freq: %d Hz                      ║\n", CONTROL_FREQ_HZ);
    printf("╚═══════════════════════════════════════════╝\n");
    printf("Type HELP for commands.\n\n");
#endif

    /* ── Init subsystems ─────────────────────────────────────── */
    servo_init();
    hiwonder_driver_init();
    safety_init();
    ackermann_odom_reset(&odom);

#ifdef USE_MICRO_ROS
    /* micro-ROS transport init — blocks until agent is found */
    while (!uros_init()) {
        /* Blink LED fast while waiting for agent */
        gpio_put(HEARTBEAT_LED_PIN, 1);
        sleep_ms(50);
        gpio_put(HEARTBEAT_LED_PIN, 0);
        sleep_ms(50);
    }
#endif

    /* ── Rate dividers for micro-ROS publishing ──────────────── */
#ifdef USE_MICRO_ROS
    const uint32_t odom_divider  = CONTROL_FREQ_HZ / UROS_ODOM_PUB_RATE_HZ;   /* 50/20 = every 2-3 loops */
    const uint32_t joint_divider = CONTROL_FREQ_HZ / UROS_JOINT_PUB_RATE_HZ;   /* 50/10 = every 5 loops  */
#endif

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

        /* Convert encoder ticks to wheel linear velocities each cycle.
           Always run forward kinematics so odom.vx/odom.wz are refreshed
           to zero when the robot is stationary. */
        float dist_l = (float)delta_l / (float)ENCODER_EDGES_PER_REV
                       * 2.0f * 3.14159f * WHEEL_RADIUS_M;
        float dist_r = (float)delta_r / (float)ENCODER_EDGES_PER_REV
                       * 2.0f * 3.14159f * WHEEL_RADIUS_M;

        float v_l = dist_l / CONTROL_DT_S;
        float v_r = dist_r / CONTROL_DT_S;

        ackermann_forward(v_l, v_r, servo_get_angle(),
                          CONTROL_DT_S, &odom);

        /* 3) Safety check */
        safety_update();

        /* 4) Apply motor commands */
        motor_control_apply();

#ifdef USE_MICRO_ROS
        /* 5a) micro-ROS: spin executor + publish */
        uros_spin_some();

        if (loop_count % odom_divider == 0) {
            uros_publish_odom(&odom);
        }
        if (loop_count % joint_divider == 0) {
            /* Convert wheel linear speed to angular speed (rad/s) */
            float wl_rads = (WHEEL_RADIUS_M > 0.001f) ? v_l / WHEEL_RADIUS_M : 0.0f;
            float wr_rads = (WHEEL_RADIUS_M > 0.001f) ? v_r / WHEEL_RADIUS_M : 0.0f;
            uros_publish_joint_state(wl_rads, wr_rads, servo_get_angle());
        }
#else
        /* 5b) ASCII serial: poll commands + print telemetry */
        poll_serial();
        print_telemetry();
#endif
    }

    return 0;
}
