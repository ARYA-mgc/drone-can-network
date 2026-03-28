/**
 * @file    drone_a_main.c
 * @brief   Drone A — Lead drone application
 *          Node ID: 0x01
 *
 * Drone A responsibilities:
 *   - Sends position, velocity, attitude at 50Hz
 *   - Sends battery status at 2Hz
 *   - Monitors Drone B heartbeat — triggers emergency if B goes silent
 *   - Accepts velocity commands from GCS
 *   - Broadcasts emergency on critical battery
 *
 * Wiring:
 *   STM32 CAN1_TX -> TJA1050 TXD -> CAN-H/CAN-L bus
 *   STM32 CAN1_RX -> TJA1050 RXD
 *   120 ohm termination resistor at each bus end
 *
 * @author  Arya
 * @date    2025
 */

#include "drone_a_main.h"
#include "can_driver.h"
#include "can_messages.h"
#include "stm32f4xx_hal.h"
#include <string.h>

/* ================================================================
 * HARDWARE HANDLES (defined in main.c by CubeMX)
 * ================================================================ */

extern CAN_HandleTypeDef hcan1;

/* ================================================================
 * TIMING
 * ================================================================ */

#define TELEMETRY_INTERVAL_MS    20U    /* 50 Hz position/velocity  */
#define ATTITUDE_INTERVAL_MS     20U    /* 50 Hz attitude           */
#define BATTERY_INTERVAL_MS      500U   /* 2 Hz battery             */

/* ================================================================
 * DRONE A STATE
 * ================================================================ */

static DroneState_t     s_state          = DRONE_STATE_IDLE;
static CAN_Position_t   s_my_pos         = {0};
static CAN_Velocity_t   s_my_vel         = {0};
static CAN_Attitude_t   s_my_att         = {0};
static CAN_Battery_t    s_my_bat         = {0};

static uint32_t s_last_telem_tick  = 0;
static uint32_t s_last_att_tick    = 0;
static uint32_t s_last_bat_tick    = 0;

static uint8_t  s_peer_was_alive   = 0;

/* ================================================================
 * SENSOR READ STUBS
 * Replace these with your actual sensor driver calls.
 * ================================================================ */

static void drone_a_read_gps(CAN_Position_t *out)
{
    /* TODO: read your GPS module here */
    /* Example: read from u-blox via UART, convert to cm offsets */
    out->source_id = DRONE_A_NODE_ID;
    out->fix_type  = 2;             /* 3D fix                     */
    /* out->x_cm = gps_get_x_cm(); */
    /* out->y_cm = gps_get_y_cm(); */
    /* out->z_cm = gps_get_altitude_cm(); */
}

static void drone_a_read_imu(CAN_Attitude_t *out)
{
    /* TODO: read your IMU (e.g. MPU6050, ICM42688) */
    out->source_id = DRONE_A_NODE_ID;
    /* out->roll_ddeg  = (int16_t)(imu_roll_deg()  * 10); */
    /* out->pitch_ddeg = (int16_t)(imu_pitch_deg() * 10); */
    /* out->yaw_ddeg   = (int16_t)(imu_yaw_deg()   * 10); */
}

static void drone_a_read_battery(CAN_Battery_t *out)
{
    /* TODO: read ADC for battery voltage */
    out->source_id  = DRONE_A_NODE_ID;
    out->cell_count = 4;
    /* out->voltage_mv = adc_read_battery_mv(); */
    /* out->percent    = battery_estimate_soc(); */
    out->flags      = 0;

    if (out->percent < 10) out->flags |= BAT_FLAG_CRITICAL;
    if (out->percent < 20) out->flags |= BAT_FLAG_WARNING;
}

/* ================================================================
 * RX CALLBACKS (from CAN bus)
 * ================================================================ */

static void on_command_received(const CAN_Command_t *msg)
{
    /* GCS or Drone B sent us a velocity setpoint */
    s_my_vel.vx_cms = msg->vx_cms;
    s_my_vel.vy_cms = msg->vy_cms;
    s_my_vel.vz_cms = msg->vz_cms;

    /* TODO: pass to your flight controller */
    /* fc_set_velocity_setpoint(msg->vx_cms, msg->vy_cms, msg->vz_cms); */
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
    /* Another node declared emergency — we land too */
    (void)msg;  /* suppress unused warning if not logging reason */
    s_state = DRONE_STATE_LANDING;

    /* TODO: trigger your emergency landing routine */
    /* flight_controller_emergency_land(); */

    /* Broadcast our own emergency so GCS knows */
    CAN_Driver_SendEmergency(EMRG_FAILSAFE);
}

static void on_heartbeat_received(const CAN_Heartbeat_t *msg)
{
    (void)msg;
    /* Peer alive tracking is handled inside can_driver.c.
       s_peer_was_alive is checked in the main process loop. */
}

/* ================================================================
 * INIT
 * ================================================================ */

void DroneA_Init(void)
{
    /* Initialise CAN peripheral for Drone A */
    CAN_DrvStatus_t status = CAN_Driver_Init(&hcan1, DRONE_A_NODE_ID);

    if (status != CAN_DRV_OK) {
        /* TODO: signal hardware fault (LED, buzzer) */
        while (1);  /* halt — CAN init failed */
    }

    /* Register message callbacks */
    CAN_Callbacks_t cbs = {
        .on_position  = NULL,               /* we don't react to others' pos */
        .on_velocity  = NULL,
        .on_attitude  = NULL,
        .on_battery   = NULL,
        .on_command   = on_command_received,
        .on_arm       = on_arm_received,
        .on_emergency = on_emergency_received,
        .on_heartbeat = on_heartbeat_received,
    };
    CAN_Driver_RegisterCallbacks(&cbs);

    /* Announce presence */
    CAN_Driver_SendHeartbeat(DRONE_STATE_IDLE);
}

/* ================================================================
 * MAIN PROCESS LOOP
 * Call from main() while(1) or a FreeRTOS task at ~100Hz
 * ================================================================ */

void DroneA_Process(void)
{
    uint32_t now = HAL_GetTick();

    /* -- CAN driver internal tasks (heartbeat TX, peer timeout) -- */
    CAN_Driver_Process();

    /* -- Position + Velocity TX @ 50Hz -- */
    if ((now - s_last_telem_tick) >= TELEMETRY_INTERVAL_MS) {
        s_last_telem_tick = now;

        drone_a_read_gps(&s_my_pos);
        CAN_Driver_SendPosition(&s_my_pos);

        s_my_vel.source_id = DRONE_A_NODE_ID;
        CAN_Driver_SendVelocity(&s_my_vel);
    }

    /* -- Attitude TX @ 50Hz -- */
    if ((now - s_last_att_tick) >= ATTITUDE_INTERVAL_MS) {
        s_last_att_tick = now;
        drone_a_read_imu(&s_my_att);
        CAN_Driver_SendAttitude(&s_my_att);
    }

    /* -- Battery TX @ 2Hz -- */
    if ((now - s_last_bat_tick) >= BATTERY_INTERVAL_MS) {
        s_last_bat_tick = now;
        drone_a_read_battery(&s_my_bat);
        CAN_Driver_SendBattery(&s_my_bat);

        /* Trigger emergency if battery is critical */
        if (s_my_bat.flags & BAT_FLAG_CRITICAL) {
            if (s_state == DRONE_STATE_FLYING) {
                s_state = DRONE_STATE_LANDING;
                CAN_Driver_SendEmergency(EMRG_LOW_BATTERY);
            }
        }
    }

    /* -- Peer (Drone B) watchdog -- */
    uint8_t b_alive = CAN_Driver_IsPeerAlive(DRONE_B_NODE_ID);

    if (s_peer_was_alive && !b_alive) {
        /* Drone B just went silent — emergency */
        CAN_Driver_SendEmergency(EMRG_FAILSAFE);
        s_state = DRONE_STATE_LANDING;
    }
    s_peer_was_alive = b_alive;
}
