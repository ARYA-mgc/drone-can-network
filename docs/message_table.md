# CAN Message Reference

## POSITION (0x101) — 8 bytes

| Byte | Field | Type | Unit | Notes |
|------|-------|------|------|-------|
| 0–1 | x_cm | int16 | cm | East offset from home |
| 2–3 | y_cm | int16 | cm | North offset from home |
| 4–5 | z_cm | int16 | cm | Altitude above home |
| 6 | source_id | uint8 | — | 0x01=DroneA 0x02=DroneB |
| 7 | fix_type | uint8 | — | 0=none 1=2D 2=3D 3=DGPS |

## VELOCITY (0x102) — 8 bytes

| Byte | Field | Type | Unit | Notes |
|------|-------|------|------|-------|
| 0–1 | vx_cms | int16 | cm/s | Forward (body frame) |
| 2–3 | vy_cms | int16 | cm/s | Right |
| 4–5 | vz_cms | int16 | cm/s | Down positive |
| 6 | source_id | uint8 | — | |
| 7 | reserved | uint8 | — | 0x00 |

## ATTITUDE (0x103) — 8 bytes

| Byte | Field | Type | Unit | Notes |
|------|-------|------|------|-------|
| 0–1 | roll_ddeg | int16 | 0.1° | -1800..1800 |
| 2–3 | pitch_ddeg | int16 | 0.1° | -1800..1800 |
| 4–5 | yaw_ddeg | int16 | 0.1° | 0..3600 |
| 6 | source_id | uint8 | — | |
| 7 | reserved | uint8 | — | 0x00 |

## BATTERY (0x200) — 6 bytes

| Byte | Field | Type | Unit | Notes |
|------|-------|------|------|-------|
| 0–1 | voltage_mv | uint16 | mV | e.g. 14800 = 14.8V |
| 2 | percent | uint8 | % | 0..100 |
| 3 | source_id | uint8 | — | |
| 4 | cell_count | uint8 | — | e.g. 4 for 4S |
| 5 | flags | uint8 | — | bit0=charging bit1=critical bit2=warning |

## EMERGENCY (0x001) — 2 bytes

| Byte | Field | Type | Notes |
|------|-------|------|-------|
| 0 | source_id | uint8 | Node declaring emergency |
| 1 | reason | uint8 | 0=manual 1=low_bat 2=failsafe 3=collision 4=sensor |

## HEARTBEAT (0x300) — 3 bytes

| Byte | Field | Type | Notes |
|------|-------|------|-------|
| 0 | source_id | uint8 | |
| 1 | state | uint8 | 0=idle 1=armed 2=flying 3=landing 4=error |
| 2 | seq | uint8 | Rolling 0..255 — detect missed frames |

## ARM (0x010) — 2 bytes

| Byte | Field | Type | Notes |
|------|-------|------|-------|
| 0 | target_id | uint8 | 0xFF = all drones |
| 1 | arm | uint8 | 1=arm 0=disarm |

## COMMAND (0x020) — 7 bytes

| Byte | Field | Type | Unit | Notes |
|------|-------|------|------|-------|
| 0–1 | vx_cms | int16 | cm/s | |
| 2–3 | vy_cms | int16 | cm/s | |
| 4–5 | vz_cms | int16 | cm/s | |
| 6 | target_id | uint8 | — | 0xFF = broadcast |
