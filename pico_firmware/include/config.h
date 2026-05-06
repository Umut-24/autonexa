#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>

/* ============================================================
 * AUTONEXA — Pico Firmware Configuration
 * Ackermann Steering Chassis with L298N H-bridge
 * ============================================================ */

/* ---------- Vehicle Geometry ---------- */
#define WHEELBASE_M          0.25f   /* front-to-rear axle distance [m]    */
#define WHEEL_RADIUS_M       0.033f  /* wheel radius [m] (66 mm diameter)  */
#define MAX_STEERING_RAD     0.5236f /* ±30 degrees in radians             */
#define TRACK_WIDTH_M        0.20f   /* left-to-right wheel distance [m]   */

/* ---------- Motor Info ---------- */
/* JGB37-520R30-12: 12V DC, 1:30 gear ratio                                */
/* Encoder constants kept for future use; no encoders connected currently. */
#define MOTOR_GEAR_RATIO     30
#define ENCODER_CPR          11      /* encoder disk slots (per motor rev)  */
#define ENCODER_EDGES_PER_REV (ENCODER_CPR * 4 * MOTOR_GEAR_RATIO)
                                     /* 11 * 4 * 30 = 1320 edges/wheel rev */

/* ---------- L298N H-bridge motor driver ---------- */
/* Switched from Hiwonder I2C smart driver on 2026-05-06 — Hiwonder MCU
 * burned. L298N is a dumb dual H-bridge: 2 direction pins + 1 PWM enable
 * per motor. No on-board encoder counting; odometry is unavailable until
 * external encoder hardware is wired in.
 *
 *   Right motor (OUT1-OUT2):  IN1 = GP2, IN2 = GP3, ENA = GP4
 *   Left  motor (OUT3-OUT4):  IN3 = GP6, IN4 = GP7, ENB = GP8
 */
#define L298N_RIGHT_IN1_PIN  2
#define L298N_RIGHT_IN2_PIN  3
#define L298N_RIGHT_EN_PIN   4
#define L298N_LEFT_IN3_PIN   6
#define L298N_LEFT_IN4_PIN   7
#define L298N_LEFT_EN_PIN    8

#define L298N_PWM_FREQ_HZ    10000   /* 10 kHz PWM (above audible)         */

/* Per-channel direction polarity. 1 = swap "forward" / "reverse" so that
 * positive PWM commands produce physical forward rotation. Verified
 * empirically 2026-05-06: with the natural mapping (=0), the GUI's
 * `M1 +fwd` and `M2 +fwd` both spun the wheels backward, so both
 * channels live with this flipped. Re-zero a flag if the motor wires get
 * physically swapped on that channel. */
#define L298N_LEFT_REVERSED   1
#define L298N_RIGHT_REVERSED  1

/* Logical channel numbering — matches CLI verbs SPEED_L/SPEED_R + the
 * GUI's bench panel "M1/M2" buttons. M1 = left, M2 = right. */
#define MOTOR_CHANNEL_LEFT   1
#define MOTOR_CHANNEL_RIGHT  2

/* CLI speed range (signed integer for SPEED/SPEED_L/SPEED_R verbs).
 * Internally maps to PWM duty 0..100% via (value * 100 / MOTOR_SPEED_MAX). */
#define MOTOR_SPEED_MAX      30      /* SPEED 30  -> 100% PWM duty         */
#define MOTOR_SPEED_MIN     -30      /* SPEED -30 -> 100% reverse duty     */

/* Static-friction kick-start: motors on this chassis don't actually rotate
 * below ~70% duty cycle. Any non-zero SPEED command is snapped up to this
 * floor so the GUI's W/S keys (and Nav2 vx commands) produce real motion
 * even at low values. SPEED 0 still produces 0% duty (full stop).
 *
 * RAW_PWM bypasses this — it's a diagnostic verb where the literal duty
 * matters (e.g. for hunting the deadband threshold). Set to 0 to disable
 * the kick-start entirely. */
#define MOTOR_DEADBAND_PCT   70

/* ---------- Servo (Steering) — LD-1501MG ---------- */
#define SERVO_PIN            15      /* GPIO 15 (servo debug wiring)       */

#define SERVO_PWM_CENTER_US  1500    /* straight ahead                     */
#define SERVO_PWM_MIN_US     500     /* full one direction (0°)            */
#define SERVO_PWM_MAX_US     2500    /* full other direction (180°)        */
#define SERVO_PERIOD_US      20000   /* 50 Hz PWM period                   */

/* ---------- Control Loop ---------- */
#define CONTROL_FREQ_HZ      50      /* main loop frequency [Hz]           */
#define CONTROL_PERIOD_US    (1000000 / CONTROL_FREQ_HZ)  /* 20 000 µs    */
#define CONTROL_DT_S         (1.0f / CONTROL_FREQ_HZ)     /* 0.02 s       */

/* ---------- Safety ---------- */
#define CMD_TIMEOUT_MS       200     /* go to brake if no command for this  */
#define HEARTBEAT_LED_PIN    25      /* on-board LED (Pico / Pico W)        */

/* ---------- Serial ---------- */
#define SERIAL_BAUD          115200

/* ---------- micro-ROS ---------- */
#ifdef USE_MICRO_ROS
#define UROS_ODOM_PUB_RATE_HZ    20
#define UROS_JOINT_PUB_RATE_HZ   10
#define UROS_NODE_NAME            "pico_controller"
#define UROS_DOMAIN_ID            0
#endif

/* ---------- Operating Modes ---------- */
typedef enum {
    MODE_MANUAL  = 0,
    MODE_AUTO    = 1,
    MODE_ESTOP   = 2
} control_mode_t;

#endif /* CONFIG_H */
