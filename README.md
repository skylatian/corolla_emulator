# Corolla CAN Emulator

CAN bus emulator for a stripped 2005 Toyota Corolla/Matrix retrofit running FrogPilot (0.9.7) on a Comma 3. Spoofs the ECU messages that openpilot expects from a 2016 Toyota Corolla so it can control a junkyard EPS motor and Comma Pedal.

Originally based on [wocsor's corolla_emulator](https://github.com/wocsor/corolla_emulator), heavily rewritten.

## Hardware

| Device | Board | Role |
|--------|-------|------|
| **Emulator** | Arduino Uno + Seeed CAN Shield v2 (CS=D9) | Sends all spoofed ECU messages (0xAA, 0xB4, 0x1D2, 0x1D3, etc.) |
| **VSS Speed** | Longan CANBed V1 (CS=D17) | Reads vehicle speed sensor pulses, sends 0xAA + 0xB4 |
| **Cruise Stalk** | Longan CANBed V1 (CS=D17) | Reads Toyota cruise stalk resistance on A0, sends 0x69; detects C3/onroad by filtered 0x2E4 RX |

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

3 bytes at 20 Hz from the stalk device:

| Byte | Field | Values |
|------|-------|--------|
| 0 | Button event | 0=NONE, 1=SET, 2=RES, 3=CANCEL, 4=MAIN_TOGGLE |
| 1 | MAIN state | 0=OFF, 1=ON (toggle tracked on stalk device) |
| 2 | Checksum | `toyota_checksum(0x69, data, 3)` — emulator rejects frames that fail |

Button events are pulse-style: non-NONE for one frame, then NONE. MAIN state (byte 1) is sent on every frame, allowing the emulator to recover from lost event frames.

## Cruise Stalk C3 Detection

The cruise stalk device uses `0x2E4` (`STEERING_LKA`) as the C3/onroad signal. That message is
sent by openpilot through panda while onroad, and disappears when OP goes offroad or panda TX is
blocked.

The stalk CANBed V1 uses MCP2515 hardware masks/filters to accept only standard ID `0x2E4`.
This is required because the MCP2515 only has two RX buffers; accepting all traffic on the busy
retrofit bus can drop `0x2E4` before the sketch drains it.

Library details for the Longan/Seeed `mcp_can` library:

- Use `CAN.begin(MCP_STDEXT, CAN_500KBPS, MCP_16MHZ)`.
- Do not use `MCP_ANY` for this sketch; it disables masks/filters by configuring RX buffers to
  accept all messages.
- Do not use `MCP_STD`; that mode is commented out in the library and fails initialization.
- Standard 11-bit filter encoding is `id << 16`; exact-match mask for all standard ID bits is
  `0x07FF0000`.

The sketch also performs software validation (`rxId == 0x2E4`, length `5`) and prints `2E4/s`
in its serial heartbeat.

## Digital Button Fallback (bench testing)

When no stalk device is detected, physical buttons on the emulator work:

| Pin | Function |
|-----|----------|
| D3 | SET: engage cruise / decrement set speed |
| D4 | RES: resume cruise / increment set speed |
| D5 | CANCEL: disengage cruise (set speed preserved) |
| D6 | MAIN: toggle cruise on/off |
| D8 | Gear: Drive ↔ Park |
| A1 | Brake pedal switch (NC pair, INPUT_PULLUP) |

## Brake Pedal Sensor

The emulator reads the car's brake pedal switch (Stop Light Switch S7) to report `BRAKE_PRESSED` in CAN message 0x224 (`BRAKE_MODULE`).

**Wiring**: Uses the **NC (Normally Closed) pair** (pins 1-2 on the 4-pin switch). One terminal to emulator pin A1, other terminal to GND.

**Fail-safe design**: The NC pair is closed when the brake is NOT pressed and opens when pressed. With `INPUT_PULLUP`:
- Brake not pressed → switch closed → pin reads LOW → `brakePressed = false`
- Brake pressed → switch opens → pin reads HIGH → `brakePressed = true`
- Wire break → pin reads HIGH (pull-up) → `brakePressed = true` (safe default)

Wire break is indistinguishable from "brake pressed" — OP will not accelerate. This is the desired failure mode.

**Roadmap**: Upgrade to dual-channel redundancy using both NC (pins 1-2) and NO (pins 3-4) pairs from the switch. Cross-check that the two channels always read opposite states; any agreement = sensor fault = report brake pressed. This catches additional failure modes (shorts, switch mechanism failure) beyond wire breaks.

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

Cruise stalk exception: initialize with `MCP_STDEXT` so the 0x2E4 hardware filters remain
enabled. `MCP_ANY` is fine for simple send/receive test sketches but is not safe for filtered
C3 detection.
