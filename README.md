# Corolla CAN Emulator

CAN bus emulator for a stripped 2005 Toyota Corolla/Matrix retrofit running FrogPilot (0.9.7) on a Comma 3. Spoofs the ECU messages that openpilot expects from a 2016 Toyota Corolla so it can control a junkyard EPS motor and Comma Pedal.

Originally based on [wocsor's corolla_emulator](https://github.com/wocsor/corolla_emulator), heavily rewritten.

## Hardware

| Device | Board | Role |
|--------|-------|------|
| **Emulator** | Arduino Uno + Seeed CAN Shield v2 (CS=D9) | Sends all spoofed ECU messages (0xAA, 0xB4, 0x1D2, 0x1D3, etc.) |
| **VSS Speed** | Longan CANBed V1 (CS=D17) | Reads vehicle speed sensor pulses, sends 0xAA + 0xB4 |
| **Cruise Stalk** | Longan CANBed V1 (CS=D17) | Reads Toyota cruise stalk resistance on A0, sends 0x69 |

All devices share a single 500 kbps CAN bus with the EPS, SAS, Comma Pedal, and Comma 3 (via 2x Ocelot).

## Directory Layout

```
corolla_emulator/
├── fakeecu/corolla_emulator/   Main emulator sketch (Arduino Uno + Seeed CAN Shield)
├── vss_speed/                  Standalone VSS speed device (CANBed V1)
├── cruise_stalk/               Standalone cruise stalk reader (CANBed V1)
├── vss_test/                   VSS pulse counter test sketch
├── canbed_test/                CANBed basic CAN send/receive test
├── interrupt_test/             Hardware interrupt test for VSS pin
└── pics/                       Original wocsor schematic images
```

## Auto-Detection

The emulator auto-detects external devices on the CAN bus:

- **VSS Speed (0xAA)**: When seen, emulator stops sending its own 0xAA/0xB4 and uses the received speed. Falls back to local speed source if device disappears (1s timeout).
- **Cruise Stalk (0x69)**: When seen, emulator uses CAN stalk events for the cruise state machine instead of digital buttons on pins 3/4. Falls back to digital buttons if device disappears (1s timeout). **Safety: cruise is auto-cancelled if the stalk device is lost while engaged.**

On first detection, the emulator syncs its MAIN state to the stalk device's reported state to prevent boot desync.

## Cruise State Machine

Four states: `OFF → MAIN_ON → ENGAGED ↔ CANCELLED`

| Input | From State | Action |
|-------|-----------|--------|
| MAIN ON | OFF | → MAIN_ON |
| MAIN OFF | any | → OFF, cruiseSetSpeed = 0 (full reset) |
| SET | MAIN_ON or CANCELLED | Capture current speed, → ENGAGED |
| SET | ENGAGED | Decrement set speed by 1 km/h (min 1) |
| RES | CANCELLED | Resume at preserved speed, → ENGAGED |
| RES | ENGAGED | Increment set speed by 1 km/h (max 255) |
| CANCEL | ENGAGED | → CANCELLED (set speed preserved) |

CAN messages affected:
- **0x1D2 PCM_CRUISE**: `CRUISE_ACTIVE` (bit 5 of byte 0) — true when ENGAGED
- **0x1D3 PCM_CRUISE_2**: `MAIN_ON` (bit 7 of byte 1), `SET_SPEED` (byte 2, 1 km/h units)

## Cruise Stalk Protocol (0x69)

2 bytes at 20 Hz from the stalk device:

| Byte | Field | Values |
|------|-------|--------|
| 0 | Button event | 0=NONE, 1=SET, 2=RES, 3=CANCEL, 4=MAIN_TOGGLE |
| 1 | MAIN state | 0=OFF, 1=ON (toggle tracked on stalk device) |

Button events are pulse-style: non-NONE for one frame, then NONE. MAIN state (byte 1) is sent on every frame, allowing the emulator to recover from lost event frames.

## Digital Button Fallback (bench testing)

When no stalk device is detected, physical buttons on the emulator work:

| Pin | Function |
|-----|----------|
| D3 | SET (if not engaged) or CANCEL (if engaged) |
| D4 | MAIN on/off toggle |
| D5 | Gear: Drive ↔ Park |

## Speed Sources

Configured via `SPEED_SOURCE` define in the emulator:

| Value | Source | Notes |
|-------|--------|-------|
| 0 | Serial | Type mph + enter in serial monitor (default) |
| 1 | Potentiometer | Analog read on A0 |
| 2 | VSS pulse counter | Hardware interrupt on D7 |

When `AUTO_DETECT_SPEED_DEVICE` is true, CAN speed from the VSS device overrides the local source.

## Libraries

- **Emulator** (Seeed CAN Shield): `mcp_can.h` from Seeed — 3-arg `begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ)`
- **CANBed V1 sketches**: `mcp_can.h` from Longan Labs — same API, different underlying lib. Both are installed; Arduino resolves by board.

Both use `MCP_16MHZ` (16 MHz crystal) and `MCP_NORMAL` mode.
