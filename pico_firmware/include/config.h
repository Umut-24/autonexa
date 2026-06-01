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
#define MOTOR_GEAR_RATIO     30
#define ENCODER_CPR          11      /* encoder disk slots (per motor rev)  */
#define ENCODER_EDGES_PER_REV (ENCODER_CPR * 4 * MOTOR_GEAR_RATIO)
                                     /* 11 * 4 * 30 = 1320 edges/wheel rev */

/* ---------- Quadrature encoders ---------- */
/* Hiwonder Hall encoders on both rear-drive motors. A/B must be on
 * consecutive GPIOs (the PIO quadrature program reads A as base, B = A+1).
 * VCC -> Pico 3V3(OUT), GND -> Pico GND.                                  */
#define ENCODER_LEFT_A_PIN   10      /* left  B = GPIO 11                   */
#define ENCODER_RIGHT_A_PIN  12      /* right B = GPIO 13                   */
/* Forward-motion sign correction. The two encoders are mirror-mounted
 * (left/right motors face opposite ways), so forward motion spins them
 * in opposite electrical directions. Verified 2026-05-16 by a forward
 * hand-push: left read negative, right positive → left is inverted. */
#define ENCODER_LEFT_SIGN    (-1)
#define ENCODER_RIGHT_SIGN   (+1)

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

/* ── Closed-loop wheel-velocity control (encoder PI) ─────────────────
 * The drive is now CLOSED-LOOP (motor_control.c velocity_pi). Every prior
 * open-loop scheme (fixed deadband floor, then an adaptive run-duty ramp) was a
 * kludge around the missing feedback: PWM duty has no fixed relationship to real
 * m/s under load, so a commanded speed was never actually achieved. With the
 * quadrature encoders wired, each wheel is now regulated to a TARGET SPEED:
 *
 *   target_mps = (SPEED / MOTOR_SPEED_MAX) * MOTOR_V_MAX_MPS   (signed)
 *   err        = target_mps - measured_wheel_mps   (from encoders)
 *   duty       = MOTOR_VEL_KP*err + integral(MOTOR_VEL_KI*err)  -> clamp +-100%
 *
 * Per-wheel loops with a common target also equalize the L/R motor asymmetry
 * (left is stronger) -> no veer. Integral anti-windup freezes the integrator
 * when the duty saturates. SPEED 0 coasts and resets the loop.
 *
 * Two open-loop helpers are kept around the PI:
 *   - START FEEDFORWARD: a one-shot MOTOR_KICK_PCT pulse for up to MOTOR_KICK_MS
 *     to break static friction from rest before the PI takes over (the encoder
 *     ends it early once moving). Static friction can exceed what the PI ramps
 *     to in one tick, so the kick guarantees a clean break-away.
 *   - STALL CUTOFF: if commanded but the encoder shows not-moving past the kick
 *     for > MOTOR_STALL_CUTOFF_MS (the PI will have wound duty to saturation),
 *     CUT to 0 (coast). A blocked / over-loaded wheel must not sit energized:
 *     stall current through the current-limitless L298N overheats the motor +
 *     bridge (the audible "beep"). Re-arms on the next SPEED 0.
 *
 * RAW_PWM still bypasses all of this — it applies the literal duty.
 *
 * Tuning: MOTOR_V_MAX_MPS = bench-measured top wheel speed at 100% duty ON THE
 * FLOOR (loaded) so SPEED 30 == the real top speed. Kp/Ki tuned for a fast,
 * non-oscillating step. Starting values below are first guesses pending the
 * on-hardware step-response tune. */
#define MOTOR_V_MAX_MPS        0.25f /* loaded top wheel speed @100% duty [m/s] (MEASURE) */
/* Static deadband feedforward: a base duty (sign of target) applied whenever a
 * non-zero speed is commanded, so the duty starts near the sustain instead of 0.
 * Without it a pure PI has to wind the integral up through the whole dead zone
 * (slow start, low-speed hunting, false stall cutoffs). The PI then only trims
 * the residual; the integral can go negative to slow below the offset. */
#define MOTOR_FF_OFFSET_PCT    45.0f /* floor-load deadband feedforward duty %           */
#define MOTOR_VEL_KP           180.0f /* P gain: duty% per (m/s) error                    */
#define MOTOR_VEL_KI           900.0f /* I gain: duty% per (m/s*s) error                  */
#define MOTOR_VEL_MOVING_MPS   0.02f /* |wheel m/s| above this => "moving"               */
#define MOTOR_VEL_LPF_ALPHA    0.4f  /* EWMA on measured wheel speed (0=slow,1=raw)      */
#define MOTOR_KICK_PCT         100  /* start-feedforward duty % to break stiction        */
#define MOTOR_KICK_MS          400  /* one-shot start-feedforward max duration [ms]       */
#define MOTOR_START_MIN_DUTY_PCT 75 /* min duty during loaded start-assist after kick     */
#define MOTOR_START_ASSIST_MS  1200 /* keep start duty floor while still below move speed */
#define MOTOR_STALL_CUTOFF_MS  1600 /* commanded+not-moving longer than this -> cut to 0  */

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
