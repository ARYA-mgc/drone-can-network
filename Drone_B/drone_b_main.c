/**
 * @file    drone_b_main.c
 * @brief   Drone B — Follower drone application
 *          Node ID: 0x02
 *
 * Drone B responsibilities:
 *   - Follows Drone A's position (offset formation flying)
 *   - Sends its own position, attitude, battery on CAN bus
 *   - If Drone A goes silent → autonomous hover / land
 *   - Broadcasts emergency if own battery drops critical
 *
 * Formation offset: Drone B holds 2m east, 0m north of Drone A.
 * Modify FORMATION_OFFSET_X_CM / _Y_CM to change shape.
 *
 * @author  Arya
 * @date    2025
 */

#include "drone_b_main.h"
#include "can_driver.h"
#include "can_messages.h"
#include "stm32f4xx_hal.h"
#include <string.h>

/* ================================================================
 * HARDWARE HANDLES
 * ================================================================ */

extern CAN_HandleTypeDef hcan1;

/* ================================================================
 * FORMATION CONFIG
 * ================================================================ */

#define FORMATION_OFFSET_X_CM    200    /* 2m east of Drone A      */
#define FORMATION_OFFSET_Y_CM      0    /* same north position      */
#define FORMATION_OFFSET_Z_CM      0    /* same altitude            */

/* ================================================================
 * TIMING
 * ================================================================ */

#define TELEMETRY_INTERVAL_MS    20U
#define ATTITUDE_INTERVAL_MS     20U
#define BATTERY_INTERVAL_MS      500U

/* ================================================================
 * DRONE B STATE
 * ================================================================ */

static DroneState_t   s_state       = DRONE_STATE_IDLE;
static CAN_Position_t s_my_pos      = {0};
static CAN_Attitude_t s_my_att      = {0};
static CAN_Battery_t  s_my_bat      = {0};
static CAN_Velocity_t s_my_vel      = {0};

/* Latest data received from Drone A */
static CAN_Position_t s_leader_pos  = {0};
static uint8_t        s_leader_pos_fresh = 0;

static uint32_t s_last_telem_tick   = 0;
static uint32_t s_last_att_tick     = 0;
static uint32_t s_last_bat_tick     = 0;
static uint8_t  s_leader_was_alive  = 0;

/* ================================================================
 * SENSOR READ STUBS (replace with your driver calls)
 * ================================================================ */

static void drone_b_read_gps(CAN_Position_t *out)
{
    out->source_id = DRONE_B_NODE_ID;
    out->fix_type  = 2;
    /* out->x_cm = gps_get_x_cm(); */
    /* out->y_cm = gps_get_y_cm(); */
    /* out->z_cm = gps_get_altitude_cm(); */
}

static void drone_b_read_imu(CAN_Attitude_t *out)
{
    out->source_id = DRONE_B_NODE_ID;
    /* out->roll_ddeg  = (int16_t)(imu_roll_deg()  * 10); */
    /* out->pitch_ddeg = (int16_t)(imu_pitch_deg() * 10); */
    /* out->yaw_ddeg   = (int16_t)(imu_yaw_deg()   * 10); */
}

static void drone_b_read_battery(CAN_Battery_t *out)
{
    out->source_id  = DRONE_B_NODE_ID;
    out->cell_count = 4;
    out->flags      = 0;
    /* out->voltage_mv = adc_read_battery_mv(); */
    /* out->percent    = battery_estimate_soc(); */

    if (out->percent < 10) out->flags |= BAT_FLAG_CRITICAL;
    if (out->percent < 20) out->flags |= BAT_FLAG_WARNING;
}

/* ================================================================
 * FORMATION CONTROL
 * Called when we have a fresh leader position.
 * Computes where Drone B should go and sends a velocity command
 * to our own flight controller.
 * ================================================================ */

static void drone_b_update_formation(void)
{
    if (!s_leader_pos_fresh) return;
    s_leader_pos_fresh = 0;

    /* Target = leader position + formation offset */
    int16_t target_x = s_leader_pos.x_cm + FORMATION_OFFSET_X_CM;
    int16_t target_y = s_leader_pos.y_cm + FORMATION_OFFSET_Y_CM;
    int16_t target_z = s_leader_pos.z_cm + FORMATION_OFFSET_Z_CM;

    /* Simple P-controller — gains in cm/s per cm error */
    /* In production: use a full position PID controller */
    int16_t err_x = target_x - s_my_pos.x_cm;
    int16_t err_y = target_y - s_my_pos.y_cm;
    int16_t err_z = target_z - s_my_pos.z_cm;

    const int16_t Kp = 3;   /* 3 cm/s per cm of error — tune this */

    s_my_vel.vx_cms    = (int16_t)(err_x * Kp / 10);
    s_my_vel.vy_cms    = (int16_t)(err_y * Kp / 10);
    s_my_vel.vz_cms    = (int16_t)(err_z * Kp / 10);
    s_my_vel.source_id = DRONE_B_NODE_ID;

    /* Clamp to safe velocity limits (100 cm/s = 1 m/s max) */
    if (s_my_vel.vx_cms >  100) s_my_vel.vx_cms =  100;
    if (s_my_vel.vx_cms < -100) s_my_vel.vx_cms = -100;
    if (s_my_vel.vy_cms >  100) s_my_vel.vy_cms =  100;
    if (s_my_vel.vy_cms < -100) s_my_vel.vy_cms = -100;
    if (s_my_vel.vz_cms >   50) s_my_vel.vz_cms =   50;
    if (s_my_vel.vz_cms <  -50) s_my_vel.vz_cms =  -50;

    /* TODO: pass velocity setpoint to flight controller */
    /* fc_set_velocity_setpoint(s_my_vel.vx_cms,
                                s_my_vel.vy_cms,
                                s_my_vel.vz_cms); */
}

/* ================================================================
 * RX CALLBACKS
 * ================================================================ */

static void on_position_received(const CAN_Position_t *msg)
{
    /* Only track Drone A (the leader) */
    if (msg->source_id == DRONE_A_NODE_ID) {
        memcpy(&s_leader_pos, msg, sizeof(CAN_Position_t));
        s_leader_pos_fresh = 1;
    }
}

static void on_arm_received(const CAN_Arm_t *msg)
{
    if (msg->arm) {
        s_state = DRONE_STATE_ARMED;
        /* TODO: arm_motors(); */
    } else {
        s_state = DRONE_STATE_IDLE;
        /* TODO: disarm_motors(); */
    }
}

static void on_emergency_received(const CAN_Emergency_t *msg)
{
    (void)msg;
    s_state = DRONE_STATE_LANDING;
    /* TODO: flight_controller_emergency_land(); */
    CAN_Driver_SendEmergency(EMRG_FAILSAFE);
}

static void on_command_received(const CAN_Command_t *msg)
{
    /* Direct velocity override from GCS */
    s_my_vel.vx_cms = msg->vx_cms;
    s_my_vel.vy_cms = msg->vy_cms;
    s_my_vel.vz_cms = msg->vz_cms;
    /* TODO: fc_set_velocity_setpoint(...); */
}

/* ================================================================
 * INIT
 * ================================================================ */

void DroneB_Init(void)
{
    CAN_DrvStatus_t status = CAN_Driver_Init(&hcan1, DRONE_B_NODE_ID);

    if (status != CAN_DRV_OK) {
        while (1);  /* CAN init failed — halt */
    }

    CAN_Callbacks_t cbs = {
        .on_position  = on_position_received,
        .on_velocity  = NULL,
        .on_attitude  = NULL,
        .on_battery   = NULL,
        .on_command   = on_command_received,
        .on_arm       = on_arm_received,
        .on_emergency = on_emergency_received,
        .on_heartbeat = NULL,
    };
    CAN_Driver_RegisterCallbacks(&cbs);

    CAN_Driver_SendHeartbeat(DRONE_STATE_IDLE);
}

/* ================================================================
 * MAIN PROCESS LOOP
 * ================================================================ */

void DroneB_Process(void)
{
    uint32_t now = HAL_GetTick();

    CAN_Driver_Process();

    /* -- Formation update whenever we have new leader position -- */
    if (s_state == DRONE_STATE_FLYING) {
        drone_b_update_formation();
    }

    /* -- Position + Velocity TX @ 50Hz -- */
    if ((now - s_last_telem_tick) >= TELEMETRY_INTERVAL_MS) {
        s_last_telem_tick = now;
        drone_b_read_gps(&s_my_pos);
        CAN_Driver_SendPosition(&s_my_pos);

        CAN_Driver_SendVelocity(&s_my_vel);
    }

    /* -- Attitude TX @ 50Hz -- */
    if ((now - s_last_att_tick) >= ATTITUDE_INTERVAL_MS) {
        s_last_att_tick = now;
        drone_b_read_imu(&s_my_att);
        CAN_Driver_SendAttitude(&s_my_att);
    }

    /* -- Battery TX @ 2Hz -- */
    if ((now - s_last_bat_tick) >= BATTERY_INTERVAL_MS) {
        s_last_bat_tick = now;
        drone_b_read_battery(&s_my_bat);
        CAN_Driver_SendBattery(&s_my_bat);

        if (s_my_bat.flags & BAT_FLAG_CRITICAL) {
            if (s_state == DRONE_STATE_FLYING) {
                s_state = DRONE_STATE_LANDING;
                CAN_Driver_SendEmergency(EMRG_LOW_BATTERY);
            }
        }
    }

    /* -- Leader (Drone A) watchdog -- */
    uint8_t a_alive = CAN_Driver_IsPeerAlive(DRONE_A_NODE_ID);

    if (s_leader_was_alive && !a_alive) {
        /* Leader went silent — hover or land */
        s_my_vel.vx_cms = 0;
        s_my_vel.vy_cms = 0;
        s_my_vel.vz_cms = 0;
        /* TODO: fc_hold_position(); or fc_land(); */
    }
    s_leader_was_alive = a_alive;
}
