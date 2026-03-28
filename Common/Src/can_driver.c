/**
 * @file    can_driver.c
 * @brief   CAN bus driver implementation
 *          Handles TX, RX dispatch, heartbeat, peer tracking
 *
 * @author  Arya
 * @date    2025
 */

#include "can_driver.h"
#include <string.h>
#include <stddef.h>

/* ================================================================
 * PRIVATE STATE
 * ================================================================ */

static CAN_HandleTypeDef *s_hcan     = NULL;
static uint8_t            s_node_id  = 0;
static CAN_Callbacks_t    s_cbs      = {0};
static uint8_t            s_hb_seq   = 0;
static uint32_t           s_last_hb_tick = 0;

/* Up to 2 peer drones tracked */
static PeerNode_t s_peers[2] = {0};
static uint8_t    s_peer_count = 0;

/* ================================================================
 * PRIVATE HELPERS
 * ================================================================ */

static PeerNode_t *prv_get_or_create_peer(uint8_t node_id)
{
    for (uint8_t i = 0; i < s_peer_count; i++) {
        if (s_peers[i].node_id == node_id) {
            return &s_peers[i];
        }
    }
    if (s_peer_count < 2) {
        s_peers[s_peer_count].node_id = node_id;
        s_peer_count++;
        return &s_peers[s_peer_count - 1];
    }
    return NULL;
}

/**
 * @brief  Raw CAN transmit — packs header and sends.
 *         Waits for a free mailbox up to CAN_TX_TIMEOUT_MS.
 */
static CAN_DrvStatus_t prv_transmit(uint32_t std_id,
                                     const uint8_t *data,
                                     uint8_t dlc)
{
    if (s_hcan == NULL || data == NULL || dlc > 8) {
        return CAN_DRV_ERR_INVALID;
    }

    CAN_TxHeaderTypeDef header = {
        .StdId              = std_id,
        .ExtId              = 0,
        .IDE                = CAN_ID_STD,
        .RTR                = CAN_RTR_DATA,
        .DLC                = dlc,
        .TransmitGlobalTime = DISABLE,
    };

    uint32_t mailbox;
    uint32_t deadline = HAL_GetTick() + CAN_TX_TIMEOUT_MS;

    /* Spin-wait for a free mailbox */
    while (HAL_CAN_GetTxMailboxesFreeLevel(s_hcan) == 0) {
        if (HAL_GetTick() > deadline) {
            return CAN_DRV_ERR_TIMEOUT;
        }
    }

    if (HAL_CAN_AddTxMessage(s_hcan, &header, data, &mailbox) != HAL_OK) {
        return CAN_DRV_ERR_TX_FAIL;
    }

    return CAN_DRV_OK;
}

/* ================================================================
 * INIT
 * ================================================================ */

CAN_DrvStatus_t CAN_Driver_Init(CAN_HandleTypeDef *hcan, uint8_t node_id)
{
    if (hcan == NULL) return CAN_DRV_ERR_INVALID;

    s_hcan    = hcan;
    s_node_id = node_id;
    memset(&s_cbs,   0, sizeof(s_cbs));
    memset(s_peers,  0, sizeof(s_peers));
    s_peer_count = 0;
    s_hb_seq     = 0;

    /*
     * Hardware filter — accept all standard frames into FIFO0.
     * In production: configure a mask filter per node to reduce
     * CPU wakeups. Example below accepts everything (mask=0).
     */
    CAN_FilterTypeDef filter = {
        .FilterIdHigh         = 0x0000,
        .FilterIdLow          = 0x0000,
        .FilterMaskIdHigh     = 0x0000,
        .FilterMaskIdLow      = 0x0000,
        .FilterFIFOAssignment = CAN_RX_FIFO0,
        .FilterBank           = 0,
        .FilterMode           = CAN_FILTERMODE_IDMASK,
        .FilterScale          = CAN_FILTERSCALE_32BIT,
        .FilterActivation     = CAN_FILTER_ENABLE,
        .SlaveStartFilterBank = 14,
    };

    if (HAL_CAN_ConfigFilter(s_hcan, &filter) != HAL_OK) {
        return CAN_DRV_ERR_TX_FAIL;
    }

    if (HAL_CAN_Start(s_hcan) != HAL_OK) {
        return CAN_DRV_ERR_TX_FAIL;
    }

    /* Enable Rx FIFO0 interrupt */
    if (HAL_CAN_ActivateNotification(s_hcan,
            CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
        return CAN_DRV_ERR_TX_FAIL;
    }

    return CAN_DRV_OK;
}

void CAN_Driver_RegisterCallbacks(const CAN_Callbacks_t *cbs)
{
    if (cbs != NULL) {
        memcpy(&s_cbs, cbs, sizeof(CAN_Callbacks_t));
    }
}

/* ================================================================
 * PROCESS (call from main loop or RTOS task every ~10ms)
 * ================================================================ */

void CAN_Driver_Process(void)
{
    uint32_t now = HAL_GetTick();

    /* Heartbeat TX */
    if ((now - s_last_hb_tick) >= CAN_HEARTBEAT_INTERVAL) {
        s_last_hb_tick = now;
        /* State tracking is application responsibility;
           pass DRONE_STATE_IDLE here or wire up your state machine */
        CAN_Driver_SendHeartbeat(DRONE_STATE_FLYING);
    }

    /* Peer timeout check */
    for (uint8_t i = 0; i < s_peer_count; i++) {
        if (s_peers[i].is_alive &&
            (now - s_peers[i].last_seen_ms) > CAN_NODE_TIMEOUT_MS) {
            s_peers[i].is_alive = 0;
            /* Application should detect this via IsPeerAlive() and react */
        }
    }
}

/* ================================================================
 * PEER QUERY
 * ================================================================ */

uint8_t CAN_Driver_IsPeerAlive(uint8_t peer_node_id)
{
    for (uint8_t i = 0; i < s_peer_count; i++) {
        if (s_peers[i].node_id == peer_node_id) {
            return s_peers[i].is_alive;
        }
    }
    return 0;
}

const PeerNode_t *CAN_Driver_GetPeer(uint8_t peer_node_id)
{
    for (uint8_t i = 0; i < s_peer_count; i++) {
        if (s_peers[i].node_id == peer_node_id) {
            return &s_peers[i];
        }
    }
    return NULL;
}

/* ================================================================
 * TRANSMIT API
 * ================================================================ */

CAN_DrvStatus_t CAN_Driver_SendPosition(const CAN_Position_t *msg)
{
    uint8_t buf[8];
    memcpy(buf, msg, sizeof(CAN_Position_t));
    return prv_transmit(CAN_ID_POSITION, buf, sizeof(CAN_Position_t));
}

CAN_DrvStatus_t CAN_Driver_SendVelocity(const CAN_Velocity_t *msg)
{
    uint8_t buf[8];
    memcpy(buf, msg, sizeof(CAN_Velocity_t));
    return prv_transmit(CAN_ID_VELOCITY, buf, sizeof(CAN_Velocity_t));
}

CAN_DrvStatus_t CAN_Driver_SendAttitude(const CAN_Attitude_t *msg)
{
    uint8_t buf[8];
    memcpy(buf, msg, sizeof(CAN_Attitude_t));
    return prv_transmit(CAN_ID_ATTITUDE, buf, sizeof(CAN_Attitude_t));
}

CAN_DrvStatus_t CAN_Driver_SendBattery(const CAN_Battery_t *msg)
{
    uint8_t buf[8];
    memcpy(buf, msg, sizeof(CAN_Battery_t));
    return prv_transmit(CAN_ID_BATTERY, buf, sizeof(CAN_Battery_t));
}

CAN_DrvStatus_t CAN_Driver_SendCommand(const CAN_Command_t *msg)
{
    uint8_t buf[8];
    memcpy(buf, msg, sizeof(CAN_Command_t));
    return prv_transmit(CAN_ID_COMMAND, buf, sizeof(CAN_Command_t));
}

CAN_DrvStatus_t CAN_Driver_SendArm(uint8_t target_id, uint8_t arm)
{
    CAN_Arm_t msg = { .target_id = target_id, .arm = arm };
    uint8_t buf[2];
    memcpy(buf, &msg, sizeof(CAN_Arm_t));
    return prv_transmit(CAN_ID_ARM, buf, sizeof(CAN_Arm_t));
}

CAN_DrvStatus_t CAN_Driver_SendEmergency(uint8_t reason)
{
    CAN_Emergency_t msg = { .source_id = s_node_id, .reason = reason };
    uint8_t buf[2];
    memcpy(buf, &msg, sizeof(CAN_Emergency_t));
    return prv_transmit(CAN_ID_EMERGENCY, buf, sizeof(CAN_Emergency_t));
}

CAN_DrvStatus_t CAN_Driver_SendHeartbeat(DroneState_t state)
{
    CAN_Heartbeat_t msg = {
        .source_id = s_node_id,
        .state     = (uint8_t)state,
        .seq       = s_hb_seq++,
    };
    uint8_t buf[3];
    memcpy(buf, &msg, sizeof(CAN_Heartbeat_t));
    return prv_transmit(CAN_ID_HEARTBEAT, buf, sizeof(CAN_Heartbeat_t));
}

/* ================================================================
 * RX CALLBACK
 * Call from: HAL_CAN_RxFifo0MsgPendingCallback()
 * ================================================================ */

void CAN_Driver_RxCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0,
                              &rx_header, rx_data) != HAL_OK) {
        return;
    }

    /* Ignore own transmissions (source_id in byte 0 or 6) */
    /* (optional — hardware loopback should be disabled)   */

    uint32_t now = HAL_GetTick();

    switch (rx_header.StdId) {

        case CAN_ID_POSITION: {
            CAN_Position_t msg;
            memcpy(&msg, rx_data, sizeof(msg));
            if (msg.source_id != s_node_id) {
                PeerNode_t *p = prv_get_or_create_peer(msg.source_id);
                if (p) {
                    memcpy(&p->last_pos, &msg, sizeof(msg));
                    p->last_seen_ms = now;
                    p->is_alive     = 1;
                }
            }
            if (s_cbs.on_position) s_cbs.on_position(&msg);
            break;
        }

        case CAN_ID_VELOCITY: {
            CAN_Velocity_t msg;
            memcpy(&msg, rx_data, sizeof(msg));
            if (s_cbs.on_velocity) s_cbs.on_velocity(&msg);
            break;
        }

        case CAN_ID_ATTITUDE: {
            CAN_Attitude_t msg;
            memcpy(&msg, rx_data, sizeof(msg));
            if (msg.source_id != s_node_id) {
                PeerNode_t *p = prv_get_or_create_peer(msg.source_id);
                if (p) {
                    memcpy(&p->last_att, &msg, sizeof(msg));
                    p->last_seen_ms = now;
                    p->is_alive     = 1;
                }
            }
            if (s_cbs.on_attitude) s_cbs.on_attitude(&msg);
            break;
        }

        case CAN_ID_BATTERY: {
            CAN_Battery_t msg;
            memcpy(&msg, rx_data, sizeof(msg));
            if (msg.source_id != s_node_id) {
                PeerNode_t *p = prv_get_or_create_peer(msg.source_id);
                if (p) {
                    memcpy(&p->last_bat, &msg, sizeof(msg));
                    p->last_seen_ms = now;
                    p->is_alive     = 1;
                }
            }
            if (s_cbs.on_battery) s_cbs.on_battery(&msg);
            break;
        }

        case CAN_ID_COMMAND: {
            CAN_Command_t msg;
            memcpy(&msg, rx_data, sizeof(msg));
            /* Only process if targeted at this drone or broadcast */
            if (msg.target_id == s_node_id || msg.target_id == 0xFF) {
                if (s_cbs.on_command) s_cbs.on_command(&msg);
            }
            break;
        }

        case CAN_ID_ARM: {
            CAN_Arm_t msg;
            memcpy(&msg, rx_data, sizeof(msg));
            if (msg.target_id == s_node_id || msg.target_id == 0xFF) {
                if (s_cbs.on_arm) s_cbs.on_arm(&msg);
            }
            break;
        }

        case CAN_ID_EMERGENCY: {
            CAN_Emergency_t msg;
            memcpy(&msg, rx_data, sizeof(msg));
            /* Emergency is always acted on regardless of target */
            if (s_cbs.on_emergency) s_cbs.on_emergency(&msg);
            break;
        }

        case CAN_ID_HEARTBEAT: {
            CAN_Heartbeat_t msg;
            memcpy(&msg, rx_data, sizeof(msg));
            if (msg.source_id != s_node_id) {
                PeerNode_t *p = prv_get_or_create_peer(msg.source_id);
                if (p) {
                    p->state        = (DroneState_t)msg.state;
                    p->last_seen_ms = now;
                    p->is_alive     = 1;
                }
            }
            if (s_cbs.on_heartbeat) s_cbs.on_heartbeat(&msg);
            break;
        }

        default:
            /* Unknown ID — silently ignore */
            break;
    }
}

/* ================================================================
 * HAL WEAK OVERRIDE
 * If not using a separate RTOS task for RX, override HAL callback here.
 * Comment out if you handle it in your own stm32f4xx_it.c.
 * ================================================================ */

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_Driver_RxCallback(hcan);
}
