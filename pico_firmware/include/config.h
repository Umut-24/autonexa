#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>

/* ============================================================
 * AUTONEXA — Pico Firmware Configuration
 * Hiwonder Ackermann Steering Chassis
 * ============================================================ */

/* ---------- Vehicle Geometry ---------- */
#define WHEELBASE_M          0.25f   /* front-to-rear axle distance [m]    */
#define WHEEL_RADIUS_M       0.033f  /* wheel radius [m] (66 mm diameter)  */
#define MAX_STEERING_RAD     0.5236f /* ±30 degrees in radians             */
#define TRACK_WIDTH_M        0.20f   /* left-to-right wheel distance [m]   */

/* ---------- Motor Info ---------- */
/* JGB37-520R30-12: 12V DC, 1:30 gear ratio, quadrature encoder            */
#define MOTOR_GEAR_RATIO     30
#define ENCODER_CPR          11      /* encoder disk slots (per motor rev)  */
#define ENCODER_EDGES_PER_REV (ENCODER_CPR * 4 * MOTOR_GEAR_RATIO)
                                     /* 11 * 4 * 30 = 1320 edges/wheel rev */

/* ---------- Hiwonder Motor Driver Board (I2C) ---------- */
#define I2C_PORT             i2c0
#define I2C_SDA_PIN          0
#define I2C_SCL_PIN          1
#define I2C_FREQ_HZ          100000  /* 100 kHz standard mode              */
#define MOTOR_DRIVER_ADDR    0x34    /* Hiwonder 4-ch driver I2C address   */

/* Motor channel mapping on driver board */
#define MOTOR_CHANNEL_LEFT   2       /* M2 = left rear                     */
#define MOTOR_CHANNEL_RIGHT  4       /* M4 = right rear                    */

/* Motor type for driver board init (register 0x14) */
#define MOTOR_TYPE_JGB37     3       /* JGB37-520 series with Hall encoder */
#define ENCODER_POLARITY_DEFAULT 0   /* default encoder counting direction */

/* Speed range for driver board commands (closed-loop: pulses per 10ms) */
#define MOTOR_SPEED_MAX      30      /* max forward speed value            */
#define MOTOR_SPEED_MIN     -30      /* max reverse speed value            */

/* ---------- Servo (Steering) — LD-1501MG ---------- */
#define SERVO_PIN            12      /* GPIO 12 (Hiwonder standard)        */

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
