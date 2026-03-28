/**
 * @file    can_driver.h
 * @brief   CAN bus driver — hardware abstraction layer
 *          Wraps STM32 HAL CAN for drone communication
 *
 * @author  Arya
 * @date    2025
 *
 * Usage:
 *   CAN_Driver_Init(&hcan1);
 *   CAN_Driver_SendPosition(&pos_data);
 *   // Received frames handled via CAN_Driver_RxCallback()
 */

#ifndef CAN_DRIVER_H
#define CAN_DRIVER_H

#include "can_messages.h"
#include "stm32f4xx_hal.h"   /* swap for your STM32 series */

/* ================================================================
 * CONFIG
 * ================================================================ */

#define CAN_TX_TIMEOUT_MS       10U
#define CAN_HEARTBEAT_INTERVAL  500U    /* ms                     */
#define CAN_NODE_TIMEOUT_MS     1500U   /* peer dead if silent >1.5s */
#define CAN_BAUD_RATE_KBPS      500U    /* 500 Kbps                */

/* ================================================================
 * RESULT CODES
 * ================================================================ */

typedef enum {
    CAN_DRV_OK            =  0,
    CAN_DRV_ERR_TX_BUSY   = -1,
    CAN_DRV_ERR_TX_FAIL   = -2,
    CAN_DRV_ERR_INVALID   = -3,
    CAN_DRV_ERR_TIMEOUT   = -4,
} CAN_DrvStatus_t;

/* ================================================================
 * PEER STATE
 * Tracks the last known data from the other drone
 * ================================================================ */

typedef struct {
    uint8_t       node_id;
    DroneState_t  state;
    CAN_Position_t  last_pos;
    CAN_Battery_t   last_bat;
    CAN_Attitude_t  last_att;
    uint32_t        last_seen_ms;   /* HAL_GetTick() timestamp    */
    uint8_t         is_alive;       /* 1 if within timeout window */
} PeerNode_t;

/* ================================================================
 * CALLBACK FUNCTION TYPES
 * Register your own handlers via CAN_Driver_RegisterCallbacks()
 * ================================================================ */

typedef void (*CAN_PositionCb_t)  (const CAN_Position_t  *msg);
typedef void (*CAN_VelocityCb_t)  (const CAN_Velocity_t  *msg);
typedef void (*CAN_AttitudeCb_t)  (const CAN_Attitude_t  *msg);
typedef void (*CAN_BatteryCb_t)   (const CAN_Battery_t   *msg);
typedef void (*CAN_CommandCb_t)   (const CAN_Command_t   *msg);
typedef void (*CAN_ArmCb_t)       (const CAN_Arm_t       *msg);
typedef void (*CAN_EmergencyCb_t) (const CAN_Emergency_t *msg);
typedef void (*CAN_HeartbeatCb_t) (const CAN_Heartbeat_t *msg);

typedef struct {
    CAN_PositionCb_t   on_position;
    CAN_VelocityCb_t   on_velocity;
    CAN_AttitudeCb_t   on_attitude;
    CAN_BatteryCb_t    on_battery;
    CAN_CommandCb_t    on_command;
    CAN_ArmCb_t        on_arm;
    CAN_EmergencyCb_t  on_emergency;
    CAN_HeartbeatCb_t  on_heartbeat;
} CAN_Callbacks_t;

/* ================================================================
 * PUBLIC API
 * ================================================================ */

/**
 * @brief  Initialise CAN peripheral and configure hardware filters.
 * @param  hcan     Pointer to HAL CAN handle (e.g. &hcan1)
 * @param  node_id  This drone's node ID (DRONE_A_NODE_ID / DRONE_B_NODE_ID)
 * @retval CAN_DRV_OK on success
 */
CAN_DrvStatus_t CAN_Driver_Init(CAN_HandleTypeDef *hcan, uint8_t node_id);

/**
 * @brief  Register application-level message callbacks.
 */
void CAN_Driver_RegisterCallbacks(const CAN_Callbacks_t *cbs);

/**
 * @brief  Must be called from main loop (or task). Handles:
 *         - Heartbeat transmission every CAN_HEARTBEAT_INTERVAL ms
 *         - Peer timeout detection
 */
void CAN_Driver_Process(void);

/**
 * @brief  Returns 1 if peer node is still sending heartbeats.
 */
uint8_t CAN_Driver_IsPeerAlive(uint8_t peer_node_id);

/**
 * @brief  Get last received peer data.
 */
const PeerNode_t *CAN_Driver_GetPeer(uint8_t peer_node_id);

/* --- Transmit helpers ------------------------------------------ */

CAN_DrvStatus_t CAN_Driver_SendPosition (const CAN_Position_t  *msg);
CAN_DrvStatus_t CAN_Driver_SendVelocity (const CAN_Velocity_t  *msg);
CAN_DrvStatus_t CAN_Driver_SendAttitude (const CAN_Attitude_t  *msg);
CAN_DrvStatus_t CAN_Driver_SendBattery  (const CAN_Battery_t   *msg);
CAN_DrvStatus_t CAN_Driver_SendCommand  (const CAN_Command_t   *msg);
CAN_DrvStatus_t CAN_Driver_SendArm      (uint8_t target_id, uint8_t arm);
CAN_DrvStatus_t CAN_Driver_SendEmergency(uint8_t reason);
CAN_DrvStatus_t CAN_Driver_SendHeartbeat(DroneState_t state);

/* --- HAL callback (call this from your HAL_CAN_RxFifo0MsgPendingCallback) */
void CAN_Driver_RxCallback(CAN_HandleTypeDef *hcan);

#endif /* CAN_DRIVER_H */
