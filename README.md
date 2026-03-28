# Drone CAN Network
### Dual-drone communication over CAN bus — STM32 + HAL

Two STM32-based drones on a shared CAN bus. Drone A leads. Drone B follows in formation.
Real-time position sharing, priority-based messaging, automatic fault detection.

---

## What's in here

```
drone-can-network/
├── Common/
│   ├── Inc/
│   │   ├── can_messages.h     # All CAN message IDs and payload structs
│   │   └── can_driver.h       # Driver API — init, send, callbacks
│   └── Src/
│       └── can_driver.c       # Full driver implementation
├── Drone_A/
│   ├── drone_a_main.h
│   └── drone_a_main.c         # Lead drone — broadcasts position, monitors B
├── Drone_B/
│   ├── drone_b_main.h
│   └── drone_b_main.c         # Follower drone — formation control from A's pos
└── docs/
    └── message_table.md       # CAN ID reference
```

---

## CAN Message Priority Map

| CAN ID | Message | Priority | Rate |
|--------|---------|----------|------|
| `0x001` | EMERGENCY | Highest | On event |
| `0x010` | ARM / DISARM | High | On command |
| `0x020` | VELOCITY COMMAND | High | On command |
| `0x101` | POSITION | Medium | 50 Hz |
| `0x102` | VELOCITY | Medium | 50 Hz |
| `0x103` | ATTITUDE | Medium | 50 Hz |
| `0x200` | BATTERY | Low | 2 Hz |
| `0x201` | SENSOR STATUS | Low | 1 Hz |
| `0x300` | HEARTBEAT | Lowest | 2 Hz |

Lower ID = higher priority. CAN arbitration ensures emergency frames
always win without collision or data loss.

---

## Hardware Setup

```
STM32 (Drone A)                     STM32 (Drone B)
   CAN1_TX ──► TJA1050 ──┐   ┌── TJA1050 ◄── CAN1_TX
   CAN1_RX ◄── TJA1050 ──┤   ├── TJA1050 ──► CAN1_RX
                          │   │
                    ┌─────┴───┴─────┐
             [120Ω] │  CAN-H CAN-L  │ [120Ω]
                    └───────────────┘
                      twisted pair
```

**Required components per drone:**
- STM32F4 (or F1/F3 — any with CAN peripheral)
- TJA1050 or MCP2551 CAN transceiver
- 120Ω termination resistor at each end of the bus
- Twisted pair wire (any length up to ~40m at 500Kbps)

---

## How to integrate

**Step 1 — copy Common/ into your STM32CubeIDE project**

**Step 2 — initialise CAN in CubeMX**
- Mode: Normal
- Baud: 500 Kbps (Prescaler=6, BS1=11, BS2=4 at 84MHz APB1)
- Enable CAN global interrupt

**Step 3 — call init and process**

For Drone A:
```c
// in main.c
#include "drone_a_main.h"

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_CAN1_Init();

    DroneA_Init();

    while (1) {
        DroneA_Process();
        HAL_Delay(10);  // or remove if using RTOS
    }
}
```

For Drone B — same pattern but call `DroneB_Init()` and `DroneB_Process()`.

**Step 4 — fill in sensor stubs**

Look for `/* TODO: */` comments in `drone_a_main.c` and `drone_b_main.c`.
Replace the stub functions with your actual GPS, IMU, and ADC reads.

---

## Formation flying

Drone B holds a fixed offset from Drone A using a simple P-controller.
Default: 2m east, same altitude.

Change offset in `drone_b_main.c`:
```c
#define FORMATION_OFFSET_X_CM    200   // 2m east
#define FORMATION_OFFSET_Y_CM      0
#define FORMATION_OFFSET_Z_CM      0
```

Tune the proportional gain:
```c
const int16_t Kp = 3;   // cm/s per cm of position error
```

For real flight: replace the P-controller with a full PID loop and
feed the output into your flight controller's velocity setpoint input.

---

## Fault handling

| Scenario | What happens |
|----------|-------------|
| Drone B battery < 10% | Drone B broadcasts EMERGENCY, lands autonomously |
| Drone A goes silent (>1.5s) | Drone B detects peer timeout, holds position / lands |
| Any node sends EMERGENCY | All drones transition to LANDING state |
| CAN wire disconnects | Both nodes detect timeout independently and land |

---

## Extending this

- **CAN FD** — increase DLC to 64 bytes if you need larger payloads (sensor fusion data, config frames)
- **More drones** — add `DRONE_C_NODE_ID = 0x03` in `can_messages.h`, copy Drone B, change formation offset
- **GCS link** — connect a USB-CAN adapter (e.g. PEAK PCAN) to the bus, use python-can to read/send from PC
- **Logging** — send raw CAN frames to SD card using the heartbeat seq number as a timestamp reference

---

## Tested on

- STM32F411 Nucleo (500 Kbps, TJA1050 transceiver)
- STM32F103 Blue Pill (same config)

> Hardware not available? Test with two STM32 dev boards connected by
> 3 wires (CAN-H, CAN-L, GND) with a 120Ω resistor across H and L at each end.

---

**Author:** Arya · CIT Chennai  
**Target:** STM32 HAL · CAN 2.0A (11-bit ID) · ISO 11898
