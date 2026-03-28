/**
 * @file    can_messages.h
 * @brief   CAN Message ID definitions and payload structures
 *          for Dual-Drone Communication System
 *
 * @author  Arya
 * @date    2025
 *
 * CAN ID Priority Map (lower ID = higher priority):
 *
 *   0x001  EMERGENCY       <- highest priority, always wins arbitration
 *   0x010  ARM / DISARM
 *   0x020  COMMAND
 *   0x101  POSITION
 *   0x102  VELOCITY
 *   0x103  ATTITUDE (roll/pitch/yaw)
 *   0x200  BATTERY
 *   0x201  SENSOR_STATUS
 *   0x300  HEARTBEAT       <- lowest priority, sent every 500ms
 */

#ifndef CAN_MESSAGES_H
#define CAN_MESSAGES_H

#include <stdint.h>

/* ================================================================
 * MESSAGE IDs
 * ================================================================ */

#define CAN_ID_EMERGENCY        0x001U   /* Emergency land / kill */
#define CAN_ID_ARM              0x010U   /* Arm or disarm motors  */
#define CAN_ID_COMMAND          0x020U   /* Velocity setpoint cmd */
#define CAN_ID_POSITION         0x101U   /* GPS / estimated pos   */
#define CAN_ID_VELOCITY         0x102U   /* Current velocity      */
#define CAN_ID_ATTITUDE         0x103U   /* Roll, pitch, yaw      */
#define CAN_ID_BATTERY          0x200U   /* Battery voltage / %   */
#define CAN_ID_SENSOR_STATUS    0x201U   /* Sensor health flags   */
#define CAN_ID_HEARTBEAT        0x300U   /* Alive ping            */

/* ================================================================
 * NODE IDs (used inside payloads to identify sender)
 * ================================================================ */

#define DRONE_A_NODE_ID         0x01U
#define DRONE_B_NODE_ID         0x02U
#define GCS_NODE_ID             0x10U    /* Ground Control Station */

/* ================================================================
 * PAYLOAD STRUCTURES
 * All structs are packed to fit within 8-byte CAN data field.
 * Use can_pack_* / can_unpack_* functions — never cast pointers
 * directly (alignment issues on Cortex-M).
 * ================================================================ */

/* CAN_ID_POSITION  (8 bytes)
 * x, y as int16 in centimetres from home point
 * z as int16 in centimetres (altitude)
 * source_id identifies which drone sent it
 */
typedef struct __attribute__((packed)) {
    int16_t  x_cm;          /* East  offset from home, cm  */
    int16_t  y_cm;          /* North offset from home, cm  */
    int16_t  z_cm;          /* Altitude above home,    cm  */
    uint8_t  source_id;     /* DRONE_A_NODE_ID / DRONE_B   */
    uint8_t  fix_type;      /* 0=none 1=2D 2=3D 3=DGPS    */
} CAN_Position_t;           /* sizeof = 8 bytes             */

/* CAN_ID_VELOCITY  (8 bytes) */
typedef struct __attribute__((packed)) {
    int16_t  vx_cms;        /* cm/s, body frame forward    */
    int16_t  vy_cms;        /* cm/s, body frame right      */
    int16_t  vz_cms;        /* cm/s, down positive         */
    uint8_t  source_id;
    uint8_t  reserved;
} CAN_Velocity_t;           /* sizeof = 8 bytes             */

/* CAN_ID_ATTITUDE  (8 bytes)
 * Angles in units of 0.1 degrees (int16), range -1800..1800
 */
typedef struct __attribute__((packed)) {
    int16_t  roll_ddeg;     /* 0.1 deg units, -1800..1800  */
    int16_t  pitch_ddeg;
    int16_t  yaw_ddeg;      /* 0..3600 (0..360.0 deg)      */
    uint8_t  source_id;
    uint8_t  reserved;
} CAN_Attitude_t;           /* sizeof = 8 bytes             */

/* CAN_ID_BATTERY  (6 bytes, DLC=6) */
typedef struct __attribute__((packed)) {
    uint16_t voltage_mv;    /* millivolts                  */
    uint8_t  percent;       /* 0..100                      */
    uint8_t  source_id;
    uint8_t  cell_count;
    uint8_t  flags;         /* bit0=charging bit1=critical */
} CAN_Battery_t;            /* sizeof = 6 bytes             */

/* CAN_ID_COMMAND  (7 bytes, DLC=7)
 * Velocity setpoint from GCS or lead drone
 */
typedef struct __attribute__((packed)) {
    int16_t  vx_cms;
    int16_t  vy_cms;
    int16_t  vz_cms;
    uint8_t  target_id;     /* which drone should act      */
} CAN_Command_t;            /* sizeof = 7 bytes             */

/* CAN_ID_ARM  (2 bytes, DLC=2) */
typedef struct __attribute__((packed)) {
    uint8_t  target_id;     /* 0xFF = broadcast to all     */
    uint8_t  arm;           /* 1=arm, 0=disarm             */
} CAN_Arm_t;

/* CAN_ID_EMERGENCY  (2 bytes, DLC=2) */
typedef struct __attribute__((packed)) {
    uint8_t  source_id;
    uint8_t  reason;        /* 0=manual 1=low_bat 2=failsafe 3=collision */
} CAN_Emergency_t;

/* CAN_ID_HEARTBEAT  (3 bytes, DLC=3) */
typedef struct __attribute__((packed)) {
    uint8_t  source_id;
    uint8_t  state;         /* 0=idle 1=armed 2=flying 3=land 4=error */
    uint8_t  seq;           /* rolling counter 0..255      */
} CAN_Heartbeat_t;

/* CAN_ID_SENSOR_STATUS  (4 bytes, DLC=4) */
typedef struct __attribute__((packed)) {
    uint8_t  source_id;
    uint8_t  gps_ok    : 1;
    uint8_t  imu_ok    : 1;
    uint8_t  baro_ok   : 1;
    uint8_t  mag_ok    : 1;
    uint8_t  reserved  : 4;
    uint16_t error_code;
} CAN_SensorStatus_t;

/* ================================================================
 * DRONE STATE ENUM
 * ================================================================ */

typedef enum {
    DRONE_STATE_IDLE      = 0,
    DRONE_STATE_ARMED     = 1,
    DRONE_STATE_FLYING    = 2,
    DRONE_STATE_LANDING   = 3,
    DRONE_STATE_ERROR     = 4,
} DroneState_t;

/* ================================================================
 * BATTERY FLAGS
 * ================================================================ */

#define BAT_FLAG_CHARGING   (1 << 0)
#define BAT_FLAG_CRITICAL   (1 << 1)   /* < 10%, land now         */
#define BAT_FLAG_WARNING    (1 << 2)   /* < 20%, return home      */

/* ================================================================
 * EMERGENCY REASONS
 * ================================================================ */

#define EMRG_MANUAL         0x00
#define EMRG_LOW_BATTERY    0x01
#define EMRG_FAILSAFE       0x02
#define EMRG_COLLISION      0x03
#define EMRG_SENSOR_FAIL    0x04

#endif /* CAN_MESSAGES_H */
