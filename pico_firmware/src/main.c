/**
 * Autonexa — Pico Firmware Main
 *
 * Ackermann Steering Chassis with L298N H-bridge
 *
 * Phase 1: Bench test via serial CLI
 *   - L298N dual H-bridge (open-loop PWM, no on-board encoders)
 *   - LD-1501MG servo steering on configured PWM GPIO
 *   - Safety watchdog with command timeout and E-STOP
 *
 * Phase 2: micro-ROS executor replaces serial CLI (compile with -DUSE_MICRO_ROS)
 *
 * Hardware migrated from Hiwonder I2C smart driver to L298N on 2026-05-06
 * after the Hiwonder board's MCU burned. External quadrature encoders are
 * now wired (GP10/11 left, GP12/13 right), so ENC_READ and the odometry
 * fields in TEL/STATUS carry real data, and motor_control_apply() uses the
 * measured wheel motion for its encoder-aware kick-start.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "pico/stdlib.h"
#include "hardware/timer.h"

#include "config.h"
#include "servo.h"
#include "l298n_driver.h"
#include "safety.h"
#include "ackermann.h"
#include "motor_control.h"
#include "encoder.h"

#ifdef USE_MICRO_ROS
#include "uros_transport.h"
#include <rmw_microros/rmw_microros.h>
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
    else if (strcmp(cmd, "SERVO_SWEEP") == 0) {
        const uint16_t pulses[] = {1000, 1500, 2000, 1500};
        printf("OK SERVO_SWEEP pin=GP%d pulses=1000,1500,2000,1500\n", SERVO_PIN);
        for (int repeat = 0; repeat < 3; repeat++) {
            for (int i = 0; i < 4; i++) {
                servo_set_pwm_us(pulses[i]);
                printf("SERVO pulse_us=%u\n", pulses[i]);
                sleep_ms(700);
            }
        }
        printf("OK SERVO_SWEEP done\n");
    }
    else if (strcmp(cmd, "SERVO_PIN_TEST") == 0) {
        printf("OK SERVO_PIN_TEST GP%d toggling 0V/3V3\n", SERVO_PIN);
        gpio_init(SERVO_PIN);
        gpio_set_dir(SERVO_PIN, GPIO_OUT);
        for (int i = 0; i < 10; i++) {
            gpio_put(SERVO_PIN, 1);
            printf("SERVO_PIN_TEST high\n");
            sleep_ms(500);
            gpio_put(SERVO_PIN, 0);
            printf("SERVO_PIN_TEST low\n");
            sleep_ms(500);
        }
        servo_init();
        printf("OK SERVO_PIN_TEST done pwm_restored\n");
    }
    else if (strcmp(cmd, "SERVO_PIN_HOLD") == 0) {
        printf("OK SERVO_PIN_HOLD GP%d high 10s then low 10s\n", SERVO_PIN);
        gpio_init(SERVO_PIN);
        gpio_set_dir(SERVO_PIN, GPIO_OUT);
        gpio_put(SERVO_PIN, 1);
        printf("SERVO_PIN_HOLD high_now measure GP%d to GND\n", SERVO_PIN);
        sleep_ms(10000);
        gpio_put(SERVO_PIN, 0);
        printf("SERVO_PIN_HOLD low_now measure GP%d to GND\n", SERVO_PIN);
        sleep_ms(10000);
        servo_init();
        printf("OK SERVO_PIN_HOLD done pwm_restored\n");
    }

    /* === MOTOR (L298N H-bridge) ===
     * Important: persist via motor_control_set_speeds() so the next
     * motor_control_apply() tick picks them up. Writing only once via
     * l298n_set_speed() here would be overridden 20 ms later when the
     * control loop calls motor_control_apply() with its own (stale,
     * zero) state. */
    else if (strncmp(cmd, "SPEED_L ", 8) == 0) {
        speed_left = (int8_t)atoi(cmd + 8);
        motor_control_set_speed_left(speed_left);
        safety_feed_watchdog();
        printf("OK SPEED_L %d\n", speed_left);
    }
    else if (strncmp(cmd, "SPEED_R ", 8) == 0) {
        speed_right = (int8_t)atoi(cmd + 8);
        motor_control_set_speed_right(speed_right);
        safety_feed_watchdog();
        printf("OK SPEED_R %d\n", speed_right);
    }
    else if (strncmp(cmd, "SPEEDS ", 7) == 0) {
        int left, right;
        if (sscanf(cmd + 7, "%d %d", &left, &right) == 2) {
            speed_left  = (int8_t)left;
            speed_right = (int8_t)right;
            motor_control_set_speeds(speed_left, speed_right);
            safety_feed_watchdog();
            printf("OK SPEEDS %d %d\n",
                   motor_control_get_speed_left(),
                   motor_control_get_speed_right());
        } else {
            printf("ERR SPEEDS usage: SPEEDS <left> <right>\n");
        }
    }
    else if (strncmp(cmd, "SPEED ", 6) == 0) {
        int8_t spd = (int8_t)atoi(cmd + 6);
        speed_left  = spd;
        speed_right = spd;
        motor_control_set_speeds(spd, spd);
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

    /* === ENCODER (external quadrature on GP10/11 + GP12/13) === */
    else if (strcmp(cmd, "ENC_READ") == 0) {
        printf("ENC L=%ld R=%ld\n",
               (long)encoder_get_left(), (long)encoder_get_right());
    }

    /* === RAW PWM INJECTION (open-loop, bypasses motor_control) ===
     * Format kept as 4 channels for protocol compatibility with the GUI's
     * bench panel. m3/m4 are accepted but ignored — L298N has only 2
     * channels. m1 = LEFT motor, m2 = RIGHT motor (matches GUI bench
     * "M1+/M2+" buttons). Each value is a duty percentage in [-100, 100]. */
    else if (strncmp(cmd, "RAW_PWM ", 8) == 0) {
        int m1, m2, m3, m4;
        if (sscanf(cmd + 8, "%d %d %d %d", &m1, &m2, &m3, &m4) == 4) {
            l298n_set_raw_pwm((int8_t)m1, (int8_t)m2);
            safety_feed_watchdog();
            printf("OK RAW_PWM %d %d %d %d (m3/m4 ignored on L298N)\n",
                   m1, m2, m3, m4);
        } else {
            printf("ERR RAW_PWM usage: RAW_PWM <m1> <m2> <m3> <m4>\n");
        }
    }

    /* === Closed-loop velocity-PI live tuning (bench) ===
     * "PI <kp> <ki> <ff>" sets gains; "PI" alone reports them. Lets the PI be
     * tuned without reflashing; bake the winners into config.h. */
    else if (strncmp(cmd, "PI ", 3) == 0) {
        float kp, ki, ff;
        if (sscanf(cmd + 3, "%f %f %f", &kp, &ki, &ff) == 3) {
            motor_control_set_pi(kp, ki, ff);
            printf("OK PI kp=%.1f ki=%.1f ff=%.1f\n", kp, ki, ff);
        } else {
            printf("ERR PI usage: PI <kp> <ki> <ff_offset_pct>\n");
        }
    }
    else if (strcmp(cmd, "PI") == 0) {
        float kp, ki, ff;
        motor_control_get_pi(&kp, &ki, &ff);
        printf("PI kp=%.1f ki=%.1f ff=%.1f\n", kp, ki, ff);
    }

    /* === RAW_PID and I2C debug verbs are not supported on L298N === */
    else if (strncmp(cmd, "RAW_PID ", 8) == 0 ||
             strcmp(cmd, "I2C_SCAN") == 0 ||
             strncmp(cmd, "I2C_WRITE ", 10) == 0 ||
             strncmp(cmd, "I2C_READ ", 9) == 0) {
        printf("ERR not supported on L298N (use RAW_PWM for open-loop)\n");
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
        printf("Commands (L298N H-bridge build):\n");
        printf("  SERVO_PWM <us>       - Raw servo PWM (500-2500)\n");
        printf("  SERVO_ANGLE <rad>    - Servo angle (±0.52 rad = ±30 deg)\n");
        printf("  SERVO_CENTER         - Center steering\n");
        printf("  SERVO_SWEEP          - Sweep GP%d through 1000/1500/2000 us\n", SERVO_PIN);
        printf("  SERVO_PIN_TEST       - Toggle GP%d high/low for electrical debug\n", SERVO_PIN);
        printf("  SERVO_PIN_HOLD       - Hold GP%d high 10s, then low 10s\n", SERVO_PIN);
        printf("  SPEED <-30..30>      - Both motors (closed-loop target speed)\n");
        printf("  SPEEDS <L> <R>       - Atomic per-wheel speed targets\n");
        printf("  SPEED_L <-30..30>    - Left motor speed\n");
        printf("  SPEED_R <-30..30>    - Right motor speed\n");
        printf("  STEER <rad>          - Steering angle\n");
        printf("  VEL <vx> <wz>        - Ackermann velocity cmd (m/s, rad/s)\n");
        printf("  ENC_READ             - Read quadrature encoder counts (L/R)\n");
        printf("  RAW_PWM <m1..m4>     - Open-loop PWM duty %% (m1=L, m2=R, m3/m4 ignored)\n");
        printf("  RAW_PID/I2C_*        - Not supported on L298N\n");
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

    /* TEL <ms>,<spdL>,<spdR>,<steer>,<encL>,<encR>,
     *     <x>,<y>,<yaw>,<vx>,<wz>,<estop>,<timeout>
     * vx/wz added 2026-05-16 so the RPi5 bridge can publish a full
     * Odometry twist. Keep test/pico_gui.py + docs in sync with the
     * field count. */
    printf("TEL %lu,%d,%d,%.3f,%ld,%ld,%.3f,%.3f,%.3f,%.3f,%.3f,%d,%d\n",
           (unsigned long)to_ms_since_boot(get_absolute_time()),
           motor_control_get_speed_left(),
           motor_control_get_speed_right(),
           servo_get_angle(),
           (long)encoder_left,
           (long)encoder_right,
           odom.x, odom.y, odom.yaw,
           odom.vx, odom.wz,
           safety_is_estopped(),
           safety_is_timed_out());

    motor_debug_t dbg;
    motor_control_get_debug(&dbg);
    /* MOT <ms>,<targetL>,<targetR>,<measL>,<measR>,<dutyL>,<dutyR>,
     *     <startedL>,<startedR>,<stallL>,<stallR>,<cutoffL>,<cutoffR>
     * This is the floor-test truth source for distinguishing PI saturation,
     * stall cutoff, and power/torque-limited starts. */
    printf("MOT %lu,%.3f,%.3f,%.3f,%.3f,%d,%d,%d,%d,%d,%d,%d,%d\n",
           (unsigned long)to_ms_since_boot(get_absolute_time()),
           dbg.target_left_mps,
           dbg.target_right_mps,
           dbg.measured_left_mps,
           dbg.measured_right_mps,
           dbg.duty_left_pct,
           dbg.duty_right_pct,
           dbg.started_left,
           dbg.started_right,
           dbg.stall_left,
           dbg.stall_right,
           dbg.cutoff_left,
           dbg.cutoff_right);
}

#endif /* !USE_MICRO_ROS */

/* ══════════════════════════════════════════════════════════════
 * MAIN
 * ══════════════════════════════════════════════════════════════ */

int main(void)
{
    /* ── Init Pico stdlib (USB serial) ───────────────────────── */
    /* Must be called early in both modes: serial CLI needs it for printf,
     * micro-ROS needs USB CDC enumerated before the XRCE-DDS ping loop. */
    stdio_init_all();
#ifdef USE_MICRO_ROS
    sleep_ms(2000);  /* allow USB CDC to fully enumerate before first ping */
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
    sleep_ms(2000);  /* wait for USB serial (serial CLI only) */

    printf("\n");
    printf("╔═══════════════════════════════════════════╗\n");
    printf("║  AUTONEXA Pico Firmware v3.0              ║\n");
    printf("║  Ackermann + L298N H-bridge               ║\n");
    printf("║  Phase 1 — Bench Test (Serial CLI)        ║\n");
    printf("║  Control freq: %d Hz                      ║\n", CONTROL_FREQ_HZ);
    printf("╚═══════════════════════════════════════════╝\n");
    printf("Type HELP for commands.\n\n");
#endif

    /* ── Init subsystems ─────────────────────────────────────── */
    servo_init();
    l298n_driver_init();
    safety_init();
    encoder_init();
    ackermann_odom_reset(&odom);

#ifdef USE_MICRO_ROS
    /* Set up the custom serial transport exactly once. */
    uros_transport_setup();

    /* Wait for micro-ROS agent: ping with 1s timeout, retry indefinitely.
     * The Pico LED blinks fast while waiting so you can tell it's alive. */
    while (rmw_uros_ping_agent(1000, 1) != RMW_RET_OK) {
        gpio_put(HEARTBEAT_LED_PIN, 1);
        sleep_ms(50);
        gpio_put(HEARTBEAT_LED_PIN, 0);
        sleep_ms(50);
    }

    /* Agent found — initialize node, subscribers, publishers, executor. */
    while (!uros_init()) {
        sleep_ms(100);
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

        /* 1) Read the two quadrature encoders and integrate odometry.
         *    Counts → per-tick wheel distance → wheel linear speed. */
        encoder_left  = encoder_get_left();
        encoder_right = encoder_get_right();
        int32_t d_l = encoder_left  - prev_enc_left;
        int32_t d_r = encoder_right - prev_enc_right;
        prev_enc_left  = encoder_left;
        prev_enc_right = encoder_right;

        /* Metres travelled per quadrature edge: wheel circumference
         * divided by edges-per-wheel-rev. */
        const float m_per_edge =
            (2.0f * (float)M_PI * WHEEL_RADIUS_M) / (float)ENCODER_EDGES_PER_REV;
        float v_l = ((float)d_l * m_per_edge) / CONTROL_DT_S;
        float v_r = ((float)d_r * m_per_edge) / CONTROL_DT_S;

        /* 2) Differential-drive odometry — measured yaw rate from the
         *    rear-wheel differential (see ackermann_odom_diff). */
        ackermann_odom_diff(v_l, v_r, CONTROL_DT_S, &odom);

        /* 3) Safety check */
        safety_update();

        /* 4) Feed measured per-wheel speed to the closed-loop velocity PI,
         *    then apply motor commands. The feedback call must precede apply()
         *    so the PI regulates on this tick's measured speed. */
        motor_control_update_feedback(v_l, v_r);
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
