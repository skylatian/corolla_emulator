// ============================================================================
// Corolla CAN Emulator for FrogPilot Retrofit
// Spoofs ECU messages so openpilot (FrogPilot 0.9.7) recognizes a Toyota Corolla
// and allows steering control via the 2016 Corolla EPS.
//
// Hardware: Arduino + Seeed Studio CAN Bus Shield v2 (MCP2515, 16MHz crystal, CS=D9)
// Bus: 500kbps CAN, shared with EPS + SAS + Comma 3
// Library: mcp_can 1.5.1 (coryjfowler) — NOT Seeed's CAN_BUS_Shield library. The
//          shield is Seeed hardware, but this code only compiles against coryjfowler's
//          API (3-arg begin with MCP_ANY/MCP_STDEXT); Seeed's 2.x MCP_CAN is abstract.
//
// Also sends 0x500 RETROFIT_EMULATOR_DIAG at 2 Hz (and on every cruise transition):
// boot count, cruise state + why it last changed, stalk presence, CAN health.
// Layout in the DIAGNOSTICS section; decoded by openpilot's DBC.
// ============================================================================

#include <SPI.h>
#include <EEPROM.h>
#include <avr/pgmspace.h>
#include <mcp_can.h>  // coryjfowler mcp_can 1.5.1

#define CAN_CS_PIN 9
MCP_CAN CAN(CAN_CS_PIN);  // Seeed CAN Bus Shield v2: CS on D9 (not D10)

// ============================================================================
// FLAG-BASED CONFIGURATION
// Toggle these to control which messages the emulator sends.
// ============================================================================

// Set to true when real hardware is connected and provides these messages.
// Set to false to have the emulator spoof them (bench testing without EPS/SAS).
#define REAL_EPS_CONNECTED    true  // EPS provides 0x260 (STEER_TORQUE_SENSOR), 0x262 (EPS_STATUS)
#define REAL_SAS_CONNECTED    true  // SAS provides 0x25 (STEER_ANGLE_SENSOR)
#define REAL_PEDAL_CONNECTED  false  // Comma Pedal provides 0x201 (GAS_SENSOR) at 50 Hz

// Messages the emulator can send. Defaults are what FrogPilot needs.
#define SEND_ACC_CONTROL      false  // 0x343 — WARNING: triggers relay_malfunction if true!
#define SEND_BRAKE_MODULE     true   // 0x224 — required by panda safety RX checks
#define SEND_PCM_CRUISE_2     true   // 0x1D3 — MAIN_ON flag (cruise availability)
#define SEND_GEAR_PACKET      true   // 0x3BC — gear position (fallback when VSS device absent)
#define SEND_BODY_CONTROL     true   // 0x620 — doors/seatbelt state
#define SEND_WHEEL_SPEEDS     true   // 0xAA  — required by panda safety RX checks (83 Hz)
#define SEND_SPEED            true   // 0xB4  — vehicle speed (also EPS init message)
#define SEND_PCM_CRUISE       true   // 0x1D2 — cruise state (required by panda safety)
#define SEND_ENGINE_RPM       true   // 0x1C4 — EPS init message (42 Hz required by carstate)
#define SEND_EPS_INIT_4CB     true   // 0x4CB — EPS init message
#define SEND_PRE_COLLISION    true   // 0x283 — required by carstate for non-TSS2 without DSU
#define SEND_VSC1S07          true   // 0x320 — vehicle stability control (20 Hz required)

// External VSS speed device.
// When true (production): the external VSS device owns speed (0xAA/0xB4) and gear
// (0x3BC). The emulator NEVER sends them — it only listens to 0xAA to capture the
// cruise set-speed and for heartbeat display. If the VSS device goes silent, 0xAA
// stops, openpilot sees a CAN timeout and disengages (fail-safe).
// When false (bench, no VSS device): the emulator reads speed from the serial
// monitor (type mph + enter) and sends 0xAA/0xB4/0x3BC itself.
#define VSS_DEVICE_INSTALLED      true
#define SPEED_DEVICE_TIMEOUT_MS   100  // 1 second without 0xAA = device gone (heartbeat display)

// External cruise stalk device (0x69). The emulator always expects the CAN stalk —
// there is no digital-button fallback. If the stalk device goes silent for
// STALK_DEVICE_TIMEOUT_MS, cruise turns fully OFF (MAIN off too), whatever state it
// was in: without a working stalk the driver can't turn MAIN / Always-On Lateral off,
// so neither may stay on. When the stalk comes back, cruise stays OFF until the
// driver presses MAIN — a returning stalk never turns cruise back on by itself.
#define STALK_DEVICE_TIMEOUT_MS   1000  // 1 second without 0x69 = stalk device gone

// Set true to always report brake NOT pressed regardless of switch state (testing override)
#define FORCE_BRAKE_OFF        true

// Send the bulk "background" messages from the original emulator (0x367, 0x394, etc.)
// These fill out the CAN bus to look more like a real car. Required for OP to fingerprint the car, at least in its current state
#define SEND_BACKGROUND_MSGS  true

// ============================================================================
// PIN ASSIGNMENTS
// ============================================================================

// The brake pedal switch is the emulator's ONLY physical input. Everything else
// (cruise stalk, speed, gear) arrives over CAN from dedicated devices.
#define BRAKE_PIN          A1  // Brake pedal switch (NC pair: closed=not pressed, open=pressed)
                               // Uses INPUT_PULLUP — wire break reads as "pressed" (fail-safe)
// Freed pins: D3-D6 (cruise buttons), A0 (speed pot), A2-A4 (spare inputs), D7 (VSS pulse), D8.

// ============================================================================
// TIMING (micros-based, non-blocking)
// ============================================================================

// Interval in microseconds for each frequency tier
#define INTERVAL_83HZ   12048   // ~83 Hz (12.048 ms) — WHEEL_SPEEDS, STEER_ANGLE_SENSOR
#define INTERVAL_50HZ   20000   // 50 Hz (20 ms) — STEER_TORQUE_SENSOR, SPEED, ENGINE_RPM, VSC1S07
#define INTERVAL_40HZ   25000   // 40 Hz (25 ms) — BRAKE_MODULE
#define INTERVAL_33HZ   30303   // ~33 Hz (30.3 ms) — PCM_CRUISE, PCM_CRUISE_2, PRE_COLLISION
#define INTERVAL_25HZ   40000   // 25 Hz (40 ms) — EPS_STATUS
#define INTERVAL_3HZ    333333  // ~3 Hz — BODY_CONTROL_STATE
#define INTERVAL_1HZ    1000000 // 1 Hz — GEAR_PACKET (fallback)

unsigned long lastMicros_83hz = 0;
unsigned long lastMicros_50hz = 0;
unsigned long lastMicros_40hz = 0;
unsigned long lastMicros_33hz = 0;
unsigned long lastMicros_25hz = 0;
unsigned long lastMicros_3hz  = 0;
unsigned long lastMicros_1hz  = 0;
unsigned long lastMillis_hb   = 0;  // heartbeat
#define HEARTBEAT_MS 500

// Background messages go out a few per slot instead of all at once. Sending the
// whole ~68-frame burst in one pass blocked the loop for ~20 ms, long enough for
// the 2 RX buffers to overflow and drop stalk (0x69) frames — one-frame SET/RES
// events were lost that way. Each ID still goes out at ~10 Hz.
#define INTERVAL_BG_SLOT  10000  // 10 ms per slot, 10 slots per 10 Hz cycle
unsigned long lastMicros_bg = 0;

// ============================================================================
// DIAGNOSTICS + CAN HEALTH
//
// 0x500 RETROFIT_EMULATOR_DIAG (8 bytes, 2 Hz, plus immediately on every cruise
// transition):
//   byte 0      BOOT_COUNT       increments every boot (EEPROM), wraps at 255
//   byte 1 hi   ROLLING_COUNTER  4-bit (deliberately not named COUNTER — openpilot's
//                                parser would count a jump after a reboot as a CAN error)
//   byte 1 lo   RESET_KIND       see RESET_* (the bootloader clears MCUSR, so usually 0)
//   byte 2 7:6  CRUISE_STATE     CRUISE_OFF / MAIN_ON / ENGAGED / CANCELLED
//   byte 2 b5   STALK_PRESENT    0x69 seen within STALK_DEVICE_TIMEOUT_MS
//   byte 2 b4   STALK_MAIN_ON    MAIN byte of the last 0x69
//   byte 2 3:0  LAST_TRANSITION  what last changed the cruise state (TR_*)
//   byte 3      CAN_ERROR_FLAGS  MCP2515 EFLG, OR-latched since the last frame.
//                                RX0OVR (bit 6) = a stalk frame was dropped.
//   byte 4 hi   CAN_RECOVERIES   controller re-inits since boot (saturates at 15)
//   byte 4 lo   STALK_LOSSES     stalk timeouts since boot (saturates at 15)
//   byte 5      TX_FAILS         failed sends since the last frame (saturates)
//   byte 6      LOOP_MAX_MS      slowest loop() since the last frame (saturates)
//   byte 7      CHECKSUM         toyota_checksum
// ============================================================================

#define DIAG_CAN_ID          0x500
#define DIAG_INTERVAL_MS     500
#define HEALTH_INTERVAL_MS   100
#define TX_STUCK_MS          500   // no successful send for this long → re-init controller
#define RECOVERY_RETRY_MS    500

#define EE_BOOT_COUNT   0

// RESET_KIND values (shared with the stalk and VSS devices)
#define RESET_UNKNOWN     0
#define RESET_POWER_ON    1
#define RESET_BROWN_OUT   2
#define RESET_EXTERNAL    3
#define RESET_WATCHDOG    4

// LAST_TRANSITION values
#define TR_BOOT           0
#define TR_SET            1   // SET engaged cruise
#define TR_RES            2   // RES resumed cruise
#define TR_CANCEL         3   // stalk CANCEL
#define TR_MAIN_ON        4   // stalk MAIN_TOGGLE event
#define TR_MAIN_OFF       5
#define TR_MAIN_SYNC_ON   6   // MAIN byte changed without an event frame (event frame lost)
#define TR_MAIN_SYNC_OFF  7   // (also what a stalk reboot looks like: it boots with MAIN off)
#define TR_STALK_LOST     8   // no 0x69 for STALK_DEVICE_TIMEOUT_MS
#define TR_OP_CANCEL      9   // openpilot asked to cancel (0x343 CANCEL_REQ)
#define TR_FIRST_SYNC     10  // first stalk frame after emulator boot
#define TR_SET_DEC        11  // SET while engaged (set speed -1)
#define TR_RES_INC        12  // RES while engaged (set speed +1)

// MCP2515 registers not exposed by the library's public API
#define REG_CANSTAT      0x0E
#define REG_EFLG         0x2D
#define CANSTAT_OPMOD    0xE0  // bits 7:5, 000 = normal
#define EFLG_RX_OVERFLOW 0xC0  // RX1OVR | RX0OVR (latched until cleared)

// ============================================================================
// STATE
// ============================================================================

// Cruise state machine
#define CRUISE_OFF        0  // MAIN off, no cruise
#define CRUISE_MAIN_ON    1  // MAIN on, cruise not engaged
#define CRUISE_ENGAGED    2  // Cruise actively controlling speed
#define CRUISE_CANCELLED  3  // Cruise cancelled, speed preserved for resume

uint8_t cruiseState = CRUISE_OFF;  // Always start OFF — stalk device sets MAIN on first 0x69
bool cruiseEngaged = false;    // CRUISE_ACTIVE in 0x1D2 (derived from state)
bool cruiseMain = false;       // MAIN_ON in 0x1D3 (derived from state)
bool gearInDrive = true;       // true = Drive, false = Park (fallback when VSS device absent)
bool brakePressed = false;     // Brake pedal state from NC switch (fail-safe: true on wire break)
uint8_t cruiseSetSpeed = 0;    // Captured speed at SET moment (km/h)
uint8_t msg_counter_1d2 = 0;  // Unused — kept for future rolling counter if needed
uint8_t msg_counter_201 = 0;  // Counter for 0x201 (GAS_SENSOR, 4-bit rolling)

// TX error tracking
uint32_t txOk    = 0;
uint32_t txError = 0;

// Serial-controlled speed — only used in bench mode (no external VSS device).
// Type a number in mph into the serial monitor and hit enter.
#if !VSS_DEVICE_INSTALLED
uint16_t serialSpeedKmh100 = 0;
char serialBuf[16];
uint8_t serialBufIdx = 0;
#endif

// CAN-received speed from the external VSS device (0xAA on bus)
#if VSS_DEVICE_INSTALLED
uint16_t canSpeedKmh100 = 0;       // speed in km/h * 100, read from 0xAA on bus
unsigned long lastCanSpeedMs = 0;   // millis() of last 0xAA received
bool speedDevicePresent = false;    // true when 0xAA seen within timeout
#endif

// Stalk button event codes (must match cruise_stalk.ino)
#define STALK_BTN_NONE         0
#define STALK_BTN_SET          1
#define STALK_BTN_RES          2
#define STALK_BTN_CANCEL       3
#define STALK_BTN_MAIN_TOGGLE  4

// CAN-received stalk input from external cruise stalk device
uint8_t stalkButtonEvent = STALK_BTN_NONE;  // Latest button event from stalk
bool stalkMainOn = false;                    // MAIN state from stalk device
unsigned long lastStalkMs = 0;               // millis() of last 0x69 received
bool stalkDevicePresent = false;             // true when 0x69 seen within timeout
bool stalkMainChanged = false;               // true when stalkMainOn changes between frames
bool prevStalkMainOn = false;                // previous frame's MAIN state for edge detection
bool stalkSeenSinceBoot = false;             // first detection syncs MAIN; later returns don't
bool opCancelRequested = false;              // 0x343 CANCEL_REQ seen since last loop

// Diagnostics state
uint8_t bootCount = 0;
uint8_t resetKind = RESET_UNKNOWN;
uint8_t diagRolling = 0;
uint8_t lastTransition = TR_BOOT;
uint8_t eflgLatched = 0;
uint8_t txFailsSinceDiag = 0;
uint8_t canRecoveries = 0;
uint8_t stalkLosses = 0;
unsigned long loopMaxUs = 0;
unsigned long lastTxOkMs = 0;
unsigned long lastHealthMs = 0;
unsigned long lastRecoveryMs = 0;
unsigned long lastDiagMs = 0;
bool diagDue = true;

void note_transition(uint8_t reason) {
  lastTransition = reason;
  diagDue = true;  // report cruise changes right away, not up to 500 ms later
}

void update_cruise_outputs();

// Turn MAIN on/off. OFF is a full reset (set speed cleared).
void set_main(bool on, uint8_t reason) {
  if (on) {
    if (cruiseState == CRUISE_OFF) {
      cruiseState = CRUISE_MAIN_ON;
      note_transition(reason);
      Serial.println(F("Cruise: MAIN ON"));
    }
  } else {
    if (cruiseState != CRUISE_OFF) {
      note_transition(reason);
      Serial.println(F("Cruise: MAIN OFF — full reset"));
    }
    cruiseState = CRUISE_OFF;
    cruiseSetSpeed = 0;
  }
  update_cruise_outputs();
}

// Read incoming CAN messages. Non-blocking — drains all available frames from
// the MCP2515 RX buffers (2 deep) to prevent overflow. Handles:
//   0xAA (WHEEL_SPEEDS) from external VSS device
//   0x69 (CRUISE_STALK) from external stalk device
void check_can_receive() {
  unsigned long nowMs = millis();

  // Drain all available frames (MCP2515 has only 2 RX buffers — reading just
  // one per loop risks dropping frames under heavy bus traffic from EPS/SAS/Pedal)
  while (CAN.checkReceive() == CAN_MSGAVAIL) {
    unsigned long rxId;
    uint8_t rxLen;
    uint8_t rxBuf[8];
    CAN.readMsgBuf(&rxId, &rxLen, rxBuf);

    #if VSS_DEVICE_INSTALLED
    if (rxId == 0xAA && rxLen >= 2) {
      uint16_t raw = ((uint16_t)rxBuf[0] << 8) | rxBuf[1];
      canSpeedKmh100 = (raw >= 6767) ? (raw - 6767) : 0;
      lastCanSpeedMs = nowMs;

      if (!speedDevicePresent) {
        speedDevicePresent = true;
        Serial.println(F("VSS device DETECTED — using CAN speed"));
      }
    }
    #endif

    if (rxId == 0x69 && rxLen >= 3) {
      // Validate toyota_checksum (byte 2) — reject corrupted frames
      if (rxBuf[2] != toyota_checksum(0x69, rxBuf, 3)) continue;

      // Validate event code (0-4 are valid, reject garbage)
      uint8_t evt = rxBuf[0];
      if (evt != STALK_BTN_NONE && evt <= STALK_BTN_MAIN_TOGGLE) {
        stalkButtonEvent = evt;  // Latch valid non-NONE events
      }

      // Track MAIN state from every frame (not just event frames).
      // This fixes desync when a MAIN_TOGGLE event frame is lost —
      // the next NONE frame still carries the correct MAIN state.
      bool newMainOn = (rxBuf[1] != 0);
      lastStalkMs = nowMs;

      if (!stalkDevicePresent) {
        stalkDevicePresent = true;
        // Take the stalk's MAIN state as the baseline — no edge on this frame.
        prevStalkMainOn = newMainOn;
        stalkMainOn = newMainOn;
        stalkMainChanged = false;

        if (!stalkSeenSinceBoot) {
          // First detection since emulator boot: sync to the stalk's MAIN state.
          // Fixes boot desync (emulator boots OFF, stalk may already be ON).
          stalkSeenSinceBoot = true;
          set_main(stalkMainOn, TR_FIRST_SYNC);
          Serial.println(F("Stalk device DETECTED — using CAN stalk input"));
          Serial.print(F("  Synced MAIN state: "));
          Serial.println(stalkMainOn ? "ON" : "OFF");
        } else {
          // Returning after a loss: cruise was turned OFF and stays OFF until the
          // driver presses MAIN, even if the stalk still believes MAIN is on.
          Serial.println(F("Stalk device BACK — cruise stays OFF until MAIN is pressed"));
        }
      } else {
        if (newMainOn != prevStalkMainOn) {
          stalkMainChanged = true;  // Edge detected — process in handle_stalk_input()
        }
        prevStalkMainOn = newMainOn;
        stalkMainOn = newMainOn;
      }
    }

    // 0x343 ACC_CONTROL from openpilot. openpilot sends CANCEL_REQ whenever it is
    // disengaged while cruise reports active (e.g. after a CAN fault disengage). A
    // real Toyota PCM drops cruise on it; without this the emulator stayed ENGAGED,
    // openpilot saw no rising edge, and SET could not re-engage (it only lowered
    // the set speed). CANCEL_REQ: DBC 24|1@0+ = byte 3, bit 0.
    if (rxId == 0x343 && rxLen == 8) {
      if (rxBuf[7] != toyota_checksum(0x343, rxBuf, 8)) continue;
      if (rxBuf[3] & 0x01) {
        opCancelRequested = true;
      }
    }
  }

  // Timeouts
  #if VSS_DEVICE_INSTALLED
  if (speedDevicePresent && (nowMs - lastCanSpeedMs) >= SPEED_DEVICE_TIMEOUT_MS) {
    speedDevicePresent = false;
    canSpeedKmh100 = 0;
    Serial.println(F("VSS device LOST — openpilot will time out 0xAA and disengage"));
  }
  #endif

  // Stalk device loss (always active; the stalk is the only cruise input). Without a
  // stalk the driver can't press CANCEL or MAIN, so turn cruise fully OFF — MAIN too.
  // (This used to go to CANCELLED with MAIN on, which left Always-On Lateral running
  // with no way to turn it off until the stalk was power-cycled.)
  if (stalkDevicePresent && (nowMs - lastStalkMs) >= STALK_DEVICE_TIMEOUT_MS) {
    stalkDevicePresent = false;
    stalkButtonEvent = STALK_BTN_NONE;
    stalkMainChanged = false;
    if (stalkLosses < 255) stalkLosses++;
    set_main(false, TR_STALK_LOST);
    diagDue = true;
    Serial.println(F("Stalk device LOST — cruise OFF"));
  }
}

// ============================================================================
// TOYOTA CHECKSUM
// From opendbc/can/common.cc:
//   sum = data_length + (address & 0xFF) + (address >> 8) + data[0..n-2]
//   checksum = sum & 0xFF
//   Goes in last byte of message.
// ============================================================================

uint8_t toyota_checksum(uint16_t address, uint8_t *data, uint8_t len) {
  uint8_t sum = len;
  sum += (address & 0xFF);
  sum += (address >> 8);
  for (uint8_t i = 0; i < len - 1; i++) {
    sum += data[i];
  }
  return sum;  // & 0xFF implicit for uint8_t
}

// ============================================================================
// CAN SEND HELPERS
// ============================================================================

// Reusable zero buffer
uint8_t zeros[8] = {0, 0, 0, 0, 0, 0, 0, 0};

void sendMsg(uint16_t id, uint8_t *data, uint8_t len) {
  byte result = CAN.sendMsgBuf(id, 0, len, data);
  if (result == CAN_OK) {
    txOk++;
    lastTxOkMs = millis();
  } else {
    txError++;
    if (txFailsSinceDiag < 255) txFailsSinceDiag++;
    // Uncomment for verbose TX error logging (noisy at high rates):
    // Serial.print("TX ERR 0x"); Serial.print(id, HEX);
    // Serial.print(" code="); Serial.println(result);
  }
}

void print_heartbeat() {
  unsigned long now = millis();
  if (now - lastMillis_hb < HEARTBEAT_MS) return;
  lastMillis_hb = now;

  #if VSS_DEVICE_INSTALLED
    uint16_t spd = canSpeedKmh100;
  #else
    uint16_t spd = serialSpeedKmh100;
  #endif

  static const char* stateNames[] = {"OFF", "MAIN_ON", "ENGAGED", "CANCELLED"};
  Serial.println("--- Status ---");
  Serial.print("  State:  "); Serial.println(stateNames[cruiseState]);
  Serial.print("  Cruise: "); Serial.println(cruiseEngaged ? "ENGAGED" : "off");
  Serial.print("  Main:   "); Serial.println(cruiseMain    ? "ON"       : "off");
  Serial.print("  Gear:   "); Serial.println(gearInDrive   ? "Drive"    : "Park");
  Serial.print("  Brake:  "); Serial.println(brakePressed  ? "PRESSED"  : "off");
  Serial.print("  Speed:  "); Serial.println(spd);
  Serial.print("  SetSpd: "); Serial.print(cruiseSetSpeed); Serial.println(" km/h");
  #if VSS_DEVICE_INSTALLED
  Serial.print("  VSSdev: "); Serial.println(speedDevicePresent ? "PRESENT" : "absent");
  Serial.print("  CAN spd:"); Serial.println(canSpeedKmh100);
  #endif
  Serial.print("  Stalk:  "); Serial.println(stalkDevicePresent ? "PRESENT" : "absent");
  Serial.print("  TX ok:  "); Serial.println(txOk);
  Serial.print("  TX err: "); Serial.println(txError);
  Serial.print(F("  Boot:   ")); Serial.print(bootCount);
  Serial.print(F("  recov: ")); Serial.print(canRecoveries);
  Serial.print(F("  stalk lost: ")); Serial.print(stalkLosses);
  Serial.print(F("  last tr: ")); Serial.println(lastTransition);
  Serial.println("--------------");
}

// ============================================================================
// MESSAGE BUILDERS
// ============================================================================

// 0xAA — WHEEL_SPEEDS (8 bytes, no checksum)
// DBC: (0.01, -67.67) → physical = raw * 0.01 - 67.67
// So raw = (km/h + 67.67) * 100
// spd_kmh is speed in km/h * 100 (e.g., 0 km/h = 0, 30 km/h = 3000)
void send_wheel_speeds(uint16_t spd_kmh_x100) {
  uint16_t raw = spd_kmh_x100 + 6767;  // offset: 67.67 * 100
  uint8_t w1 = (raw >> 8);
  uint8_t w2 = raw & 0xFF;
  uint8_t data[8] = {w1, w2, w1, w2, w1, w2, w1, w2};
  sendMsg(0xAA, data, 8);
}

// 0xB4 — SPEED (8 bytes, toyota_checksum)
// DBC: SPEED signal at (47|16@0+, 0.01, 0) → physical = raw * 0.01
// So raw = km/h * 100
// Byte layout: signal bit 47|16@0+ means bytes 5-6 (big endian)
void send_speed(uint16_t spd_kmh_x100) {
  uint8_t data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  data[5] = (spd_kmh_x100 >> 8);
  data[6] = spd_kmh_x100 & 0xFF;
  data[7] = toyota_checksum(0xB4, data, 8);
  sendMsg(0xB4, data, 8);
}

// 0x1D2 — PCM_CRUISE (8 bytes, toyota_checksum)
// Byte 0 bits: GAS_RELEASED (bit 4), CRUISE_ACTIVE (bit 5)
// Panda checks: checksum, and pcm_cruise_check() uses bit 5 for controls_allowed
// controls_allowed triggers on RISING EDGE of bit 5 (0->1 transition)
void send_pcm_cruise(bool cruise_active) {
  uint8_t data[8] = {0, 0, 0, 0, 0, 0, 0, 0};

  // Byte 0: GAS_RELEASED=1 (bit 4), CRUISE_ACTIVE (bit 5)
  data[0] = 0x10;  // GAS_RELEASED = 1
  if (cruise_active) {
    data[0] |= 0x20;  // CRUISE_ACTIVE = 1 (bit 5)
  }

  // Checksum in last byte
  data[7] = toyota_checksum(0x1D2, data, 8);
  sendMsg(0x1D2, data, 8);
}

// 0x1D3 — PCM_CRUISE_2 (8 bytes, toyota_checksum)
// DBC: MAIN_ON at 15|1@0+ (Motorola) = byte 1, bit 7
// DBC: SET_SPEED at 23|8@0+ (Motorola) = byte 2, 8 bits, 1 km/h, unsigned
// DBC: LOW_SPEED_LOCKOUT at 14|2@0+ = byte 1, bits 6-5 (0 = unlocked)
void send_pcm_cruise_2(bool main_on, uint8_t set_speed_kmh) {
  uint8_t data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  if (main_on) {
    data[1] = 0x80;  // MAIN_ON = byte 1, bit 7 (DBC bit 15 Motorola)
  }
  // LOW_SPEED_LOCKOUT = 0 (bits 6-5 of byte 1, already 0)
  data[2] = set_speed_kmh;  // SET_SPEED
  data[7] = toyota_checksum(0x1D3, data, 8);
  sendMsg(0x1D3, data, 8);
}

// 0x224 — BRAKE_MODULE (8 bytes, no checksum in DBC)
// Byte 0 bit 5: BRAKE_PRESSED
void send_brake_module() {
  uint8_t data[8] = {0};
  if (brakePressed) {
    data[0] = (1 << 5);  // Set bit 5
  }
  sendMsg(0x224, data, 8);
}

// 0x3BC — GEAR_PACKET (8 bytes, no checksum in DBC)
// Byte 1: gear position. 0x00 = Drive, 0x20 = not-in-drive (Park/Neutral)
// Only sent as fallback when VSS speed device is absent.
void send_gear_packet(bool in_drive) {
  uint8_t data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  if (!in_drive) {
    data[1] = 0x20;  // Not in drive
  }
  sendMsg(0x3BC, data, 8);
}

// 0x620 — BODY_CONTROL_STATE (8 bytes, no checksum in DBC)
// Doors closed, seatbelt on — all zeros is safe defaults
void send_body_control() {
  sendMsg(0x620, zeros, 8);
}

// 0x1C4 — ENGINE_RPM (8 bytes, no checksum in DBC)
// EPS requires specific data bytes for initialization (not zeros)
void send_engine_rpm() {
  uint8_t data[8] = {0x05, 0xEA, 0x1B, 0x08, 0x00, 0x00, 0xC0, 0x9F};
  sendMsg(0x1C4, data, 8);
}

// 0x4CB — EPS init message (must be 40+ Hz with specific data)
void send_eps_init_4cb() {
  uint8_t data[8] = {0x66, 0x06, 0x08, 0x0A, 0x02, 0x00, 0x00, 0x00};
  sendMsg(0x4CB, data, 8);
}

// 0x260 — STEER_TORQUE_SENSOR (8 bytes, toyota_checksum)
// Only sent when REAL_EPS_CONNECTED is false (bench testing)
void send_steer_torque_sensor() {
  uint8_t data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  // STEER_TORQUE_DRIVER=0, STEER_TORQUE_EPS=0, STEER_ANGLE=0
  // STEER_ANGLE_INITIALIZING=0 (must be 0 for valid angle)
  // STEER_OVERRIDE=0
  data[7] = toyota_checksum(0x260, data, 8);
  sendMsg(0x260, data, 8);
}

// 0x25 — STEER_ANGLE_SENSOR (8 bytes, no checksum in DBC)
// Only sent when REAL_SAS_CONNECTED is false (bench testing)
void send_steer_angle_sensor() {
  sendMsg(0x25, zeros, 8);
}

// 0x201 — GAS_SENSOR (6 bytes, CRC8 checksum)
// Only sent when REAL_PEDAL_CONNECTED is false (bench testing without pedal)
// Panda checks: counter (4-bit, max 15), frequency (50 Hz), no checksum validation
// Carstate checks: counter + frequency via CAN parser
void send_gas_sensor() {
  uint8_t data[6] = {0, 0, 0, 0, 0, 0};
  // INTERCEPTOR_GAS=0, INTERCEPTOR_GAS2=0 (no pedal input)
  // STATE=0 (NO_FAULT) — bits 39:4 = byte 4, bits 7-4
  // COUNTER_PEDAL — bits 35:4 = byte 4, bits 3-0
  data[4] = (0 << 4) | (msg_counter_201 & 0x0F);  // STATE=0 | COUNTER
  // CHECKSUM_PEDAL — byte 5 (CRC8, poly 0xD5, init 0xFF, over bytes 0-4 reversed)
  // Panda doesn't validate checksum, but pedal firmware convention uses CRC8
  uint8_t crc = 0xFF;
  for (int i = 4; i >= 0; i--) {
    crc ^= data[i];
    for (int j = 0; j < 8; j++) {
      crc = (crc & 0x80) ? ((crc << 1) ^ 0xD5) : (crc << 1);
    }
  }
  data[5] = crc;
  sendMsg(0x201, data, 6);
  msg_counter_201 = (msg_counter_201 + 1) & 0x0F;
}

// 0x262 — EPS_STATUS (5 bytes, toyota_checksum)
// Only sent when REAL_EPS_CONNECTED is false (bench testing)
// DBC: LKA_STATE at 31|7@0+ (Motorola) = byte 3, bits 7-1
// LKA_STATE=0 is in TEMP_STEER_FAULTS → "steering temporarily unavailable"
// LKA_STATE=1 means "ready" (not in any fault list)
void send_eps_status() {
  uint8_t data[5] = {0, 0, 0, 0, 0};
  data[3] = 0x02;  // LKA_STATE = 1 (ready) — value 1 shifted into bits 7-1
  data[4] = toyota_checksum(0x262, data, 5);
  sendMsg(0x262, data, 5);
}

// 0x283 — PRE_COLLISION (7 bytes, toyota_checksum)
// Required by carstate for non-TSS2 cars without DSU and without DISABLE_RADAR
// PRECOLLISION_ACTIVE=0, FORCE=0 (no AEB active)
void send_pre_collision() {
  uint8_t data[7] = {0, 0, 0, 0, 0, 0, 0};
  data[6] = toyota_checksum(0x283, data, 7);
  sendMsg(0x283, data, 7);
}

// 0x320 — VSC1S07 (8 bytes, no checksum in DBC)
// Vehicle stability control grade — required by carstate at 20 Hz
void send_vsc1s07() {
  sendMsg(0x320, zeros, 8);
}

// 0x343 — ACC_CONTROL (8 bytes)
// WARNING: Sending this triggers stock ECU detection in panda → relay_malfunction!
void send_acc_control() {
  sendMsg(0x343, zeros, 8);
}

// ============================================================================
// BACKGROUND MESSAGES
// These are from the original emulator — fill out the bus to look like a real car.
// Sent at ~10 Hz. Not required for steering, but may help with canValid.
// ============================================================================

// Original IDs from wocsor's emulator (various data lengths, all-zero payloads).
// Kept in flash (PROGMEM) — the Uno has 2 KB of RAM.
struct BgMsg {
  uint16_t id;
  uint8_t len;
};

// 0x2E4 removed — that's STEERING_LKA (steering command), only openpilot should send it.
// Sending it from the emulator triggers relay_malfunction detection.
// 0x283 removed — now has dedicated sender with checksum at 33 Hz.
// 8-byte list filtered to remove IDs with dedicated senders:
// 0x25 (SAS), 0x260 (EPS), 0x320 (VSC1S07), 0x343 (stock ECU detection) —
// these are gated by their respective flags below.
const BgMsg bgMsgs[] PROGMEM = {
  {0x423, 1},
  {0x367, 2}, {0x394, 2}, {0x3D3, 2},
  {0x351, 4}, {0x3BB, 4}, {0xBA, 4},
  {0x3E6, 5},
  {0x1AA, 6}, {0x384, 6}, {0x386, 6},
  {0x365, 7}, {0x366, 7}, {0x3E7, 7},
  {0x24, 8}, {0x1D0, 8}, {0x2C1, 8}, {0x344, 8},
  {0x380, 8}, {0x381, 8}, {0x389, 8}, {0x38F, 8}, {0x399, 8}, {0x3A5, 8}, {0x3B0, 8}, {0x3B1, 8}, {0x3B7, 8},
  {0x3E8, 8}, {0x3E9, 8}, {0x3F9, 8}, {0x411, 8}, {0x413, 8}, {0x414, 8}, {0x420, 8},
  {0x45A, 8}, {0x489, 8}, {0x48A, 8}, {0x48B, 8}, {0x4AC, 8}, {0x4D3, 8}, {0x4FF, 8},
  {0x610, 8}, {0x611, 8}, {0x615, 8}, {0x619, 8}, {0x61A, 8},  // 0x614 (BLINKERS_STATE) moved to cruise stalk device
  {0x621, 8}, {0x622, 8}, {0x623, 8}, {0x624, 8},
  {0x638, 8}, {0x63C, 8}, {0x63D, 8}, {0x640, 8}, {0x680, 8}, {0x6F3, 8},
  {0x770, 8}, {0x778, 8}, {0x7C6, 8}, {0x7CE, 8},
  {0x7E0, 8}, {0x7E1, 8}, {0x7E2, 8}, {0x7E3, 8}, {0x7E4, 8}, {0x7E5, 8}, {0x7E6, 8}, {0x7E7, 8}, {0x7E8, 8},
#if !REAL_SAS_CONNECTED
  {0x25, 8},
#endif
  // 0x260 NOT sent here — dedicated send_steer_torque_sensor() at 50 Hz handles it with correct checksum
#if SEND_ACC_CONTROL
  {0x343, 8},
#endif
};
const uint8_t BG_COUNT = sizeof(bgMsgs) / sizeof(bgMsgs[0]);
const uint8_t BG_PER_SLOT = (BG_COUNT + 9) / 10;  // 10 slots per 100 ms cycle
uint8_t bgIndex = 0;

// Send the next BG_PER_SLOT background frames. Called every INTERVAL_BG_SLOT, so each
// ID repeats at ~10 Hz without one long blocking burst.
void send_background_slot() {
  for (uint8_t i = 0; i < BG_PER_SLOT; i++) {
    uint16_t id = pgm_read_word(&bgMsgs[bgIndex].id);
    uint8_t len = pgm_read_byte(&bgMsgs[bgIndex].len);
    sendMsg(id, zeros, len);
    if (++bgIndex >= BG_COUNT) bgIndex = 0;
  }
}

// ============================================================================
// SERIAL SPEED INPUT (bench only — when no external VSS device)
// Type a speed in mph and press enter. E.g., "25" → 25 mph → ~40 km/h
// ============================================================================

#if !VSS_DEVICE_INSTALLED
void check_serial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialBufIdx > 0) {
        serialBuf[serialBufIdx] = '\0';
        uint16_t mph = atoi(serialBuf);
        serialSpeedKmh100 = (uint32_t)mph * 16093 / 100;  // mph to km/h * 100
        Serial.print("Speed set: ");
        Serial.print(mph);
        Serial.print(" mph (");
        Serial.print(serialSpeedKmh100 / 100);
        Serial.println(" km/h)");
        serialBufIdx = 0;
      }
    } else if (serialBufIdx < sizeof(serialBuf) - 1) {
      serialBuf[serialBufIdx++] = c;
    }
  }
}
#endif  // !VSS_DEVICE_INSTALLED

// ============================================================================
// CRUISE STATE MACHINE
// ============================================================================

// Get current speed in km/h (integer) from the best available source
uint8_t get_current_speed_kmh() {
  #if VSS_DEVICE_INSTALLED
    // 0 when the VSS device is absent — SET then captures "no target" (safe).
    return (uint8_t)(canSpeedKmh100 / 100);
  #else
    return (uint8_t)(serialSpeedKmh100 / 100);
  #endif
}

// Update derived outputs from cruise state
void update_cruise_outputs() {
  cruiseMain = (cruiseState != CRUISE_OFF);
  cruiseEngaged = (cruiseState == CRUISE_ENGAGED);
}

// Process a cruise button event through the state machine.
// event: 1=SET, 2=RES, 3=CANCEL, 4=MAIN_TOGGLE
// main_on: current MAIN state (from stalk device or internal toggle)
void process_cruise_event(uint8_t event, bool main_on) {
  // MAIN_TOGGLE: transition based on main_on state (OFF = full reset)
  if (event == 4) {  // MAIN_TOGGLE
    set_main(main_on, main_on ? TR_MAIN_ON : TR_MAIN_OFF);
    return;
  }

  // All other events require MAIN to be on
  if (cruiseState == CRUISE_OFF) return;

  switch (event) {
    case 1:  // SET
      if (cruiseState == CRUISE_MAIN_ON || cruiseState == CRUISE_CANCELLED) {
        // Engage: capture current speed as new set speed
        cruiseSetSpeed = get_current_speed_kmh();
        cruiseState = CRUISE_ENGAGED;
        note_transition(TR_SET);
        Serial.print(F("Cruise: SET — engaged at "));
        Serial.print(cruiseSetSpeed);
        Serial.println(F(" km/h"));
      } else if (cruiseState == CRUISE_ENGAGED) {
        // Decrement set speed (clamp to 1, never 0 while engaged)
        if (cruiseSetSpeed > 1) cruiseSetSpeed--;
        note_transition(TR_SET_DEC);
        Serial.print(F("Cruise: SET- → "));
        Serial.print(cruiseSetSpeed);
        Serial.println(F(" km/h"));
      }
      break;

    case 2:  // RES
      if (cruiseState == CRUISE_CANCELLED) {
        // Resume: re-engage with preserved speed
        cruiseState = CRUISE_ENGAGED;
        note_transition(TR_RES);
        Serial.print(F("Cruise: RESUME at "));
        Serial.print(cruiseSetSpeed);
        Serial.println(F(" km/h"));
      } else if (cruiseState == CRUISE_ENGAGED) {
        // Increment set speed (clamp to 255)
        if (cruiseSetSpeed < 255) cruiseSetSpeed++;
        note_transition(TR_RES_INC);
        Serial.print(F("Cruise: RES+ → "));
        Serial.print(cruiseSetSpeed);
        Serial.println(F(" km/h"));
      }
      break;

    case 3:  // CANCEL
      if (cruiseState == CRUISE_ENGAGED) {
        cruiseState = CRUISE_CANCELLED;
        note_transition(TR_CANCEL);
        Serial.print(F("Cruise: CANCELLED (speed preserved: "));
        Serial.print(cruiseSetSpeed);
        Serial.println(F(" km/h)"));
      }
      break;
  }

  update_cruise_outputs();
}

// ============================================================================
// CRUISE STALK INPUT
// ============================================================================

// Apply the latest CAN stalk (0x69) input to the cruise state machine.
// The emulator always expects the CAN stalk — there is no digital-button fallback.
void handle_stalk_input() {
  // Process button events (SET, RES, CANCEL, MAIN_TOGGLE)
  if (stalkButtonEvent != STALK_BTN_NONE) {
    process_cruise_event(stalkButtonEvent, stalkMainOn);
    stalkButtonEvent = STALK_BTN_NONE;  // Consume the event
  }

  // Detect MAIN state changes from non-event frames.
  // Fixes desync when a MAIN_TOGGLE event frame is lost: the MAIN state
  // (byte 1) still arrives correctly on subsequent NONE frames. By
  // detecting the edge here, we recover even without the event code.
  if (stalkMainChanged) {
    stalkMainChanged = false;
    // Only act if the stalk's MAIN state disagrees with emulator state
    bool emulatorMainOn = (cruiseState != CRUISE_OFF);
    if (stalkMainOn != emulatorMainOn) {
      set_main(stalkMainOn, stalkMainOn ? TR_MAIN_SYNC_ON : TR_MAIN_SYNC_OFF);
    }
  }
}

// openpilot asked the "PCM" to cancel (0x343 CANCEL_REQ). Same effect as the stalk's
// CANCEL: MAIN stays on and the set speed is kept, so SET/RES can re-engage.
void handle_op_cancel() {
  if (!opCancelRequested) return;
  opCancelRequested = false;
  if (cruiseState == CRUISE_ENGAGED) {
    cruiseState = CRUISE_CANCELLED;
    note_transition(TR_OP_CANCEL);
    update_cruise_outputs();
    Serial.println(F("Cruise: CANCELLED by openpilot (0x343 CANCEL_REQ)"));
  }
}

// ============================================================================
// DIAGNOSTICS + CAN CONTROLLER HEALTH
// ============================================================================

// Why we booted. Optiboot clears MCUSR before the sketch runs, so this is usually
// RESET_UNKNOWN; the boot counter is what reliably shows a restart.
void diag_boot() {
  uint8_t mcusr = MCUSR;
  MCUSR = 0;
  if (mcusr & _BV(WDRF)) resetKind = RESET_WATCHDOG;
  else if (mcusr & _BV(BORF)) resetKind = RESET_BROWN_OUT;
  else if (mcusr & _BV(EXTRF)) resetKind = RESET_EXTERNAL;
  else if (mcusr & _BV(PORF)) resetKind = RESET_POWER_ON;
  else resetKind = RESET_UNKNOWN;
  bootCount = EEPROM.read(EE_BOOT_COUNT) + 1;
  EEPROM.update(EE_BOOT_COUNT, bootCount);
}

uint8_t mcp_read_register(uint8_t address) {
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
  digitalWrite(CAN_CS_PIN, LOW);
  SPI.transfer(0x03);  // READ
  SPI.transfer(address);
  uint8_t value = SPI.transfer(0x00);
  digitalWrite(CAN_CS_PIN, HIGH);
  SPI.endTransaction();
  return value;
}

void mcp_modify_register(uint8_t address, uint8_t mask, uint8_t data) {
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
  digitalWrite(CAN_CS_PIN, LOW);
  SPI.transfer(0x05);  // BIT MODIFY
  SPI.transfer(address);
  SPI.transfer(mask);
  SPI.transfer(data);
  digitalWrite(CAN_CS_PIN, HIGH);
  SPI.endTransaction();
}

// RX hardware filters. The emulator only reads three IDs; accepting everything
// (MCP_ANY) made the 2 RX buffers compete with the whole bus.
//   RXB0 (mask 0, filters 0-1): 0x69 only — the stalk gets its own buffer, so a
//        burst of 0xAA frames can't push a one-frame SET/RES event out.
//   RXB1 (mask 1, filters 2-5): 0xAA (VSS speed) and 0x343 (openpilot cancel).
//        0x343 repeats every frame while openpilot wants a cancel, so an
//        occasional drop here is harmless.
// With rollover enabled (the library's default for MCP_STDEXT) a 0x69 that finds
// RXB0 full still lands in RXB1. Filter encoding: standard 11-bit ID << 16.
#define RX_MASK_EXACT 0x07FF0000UL
#define RX_FILTER(id) ((unsigned long)(id) << 16)

bool setup_rx_filters() {
  if (CAN.init_Mask(0, 0, RX_MASK_EXACT) != MCP2515_OK ||
      CAN.init_Filt(0, 0, RX_FILTER(0x69)) != MCP2515_OK ||
      CAN.init_Filt(1, 0, RX_FILTER(0x69)) != MCP2515_OK ||
      CAN.init_Mask(1, 0, RX_MASK_EXACT) != MCP2515_OK ||
      CAN.init_Filt(2, 0, RX_FILTER(0xAA)) != MCP2515_OK ||
      CAN.init_Filt(3, 0, RX_FILTER(0x343)) != MCP2515_OK ||
      CAN.init_Filt(4, 0, RX_FILTER(0xAA)) != MCP2515_OK ||
      CAN.init_Filt(5, 0, RX_FILTER(0x343)) != MCP2515_OK) {
    Serial.println(F("CAN filter setup failed"));
    return false;
  }
  return true;
}

// Bring the MCP2515 up in normal mode with the RX filters. Used at boot and for
// recovery. Same sequence as the stalk device: begin() may report a failure on a
// live bus (its final loopback step), so check NORMAL mode explicitly instead.
bool can_init() {
  byte beginResult = CAN.begin(MCP_STDEXT, CAN_500KBPS, MCP_16MHZ);
  if (beginResult != CAN_OK) {
    Serial.print(F("CAN begin warning/result: "));
    Serial.println(beginResult);
  }
  // Normal mode BEFORE filter setup so init_Mask returns to normal, not loopback.
  if (CAN.setMode(MCP_NORMAL) != MCP2515_OK) {
    Serial.println(F("CAN setMode(NORMAL) failed"));
    return false;
  }
  if (!setup_rx_filters()) {
    return false;
  }
  lastTxOkMs = millis();
  return true;
}

// Re-initialize a stuck or reset controller. Cruise state is kept: while the
// emulator was silent openpilot timed out its messages and disengaged, and it
// sends 0x343 CANCEL_REQ afterwards if cruise is still active.
void can_health_check(unsigned long nowMs) {
  uint8_t eflg = mcp_read_register(REG_EFLG);
  eflgLatched |= eflg;
  if (eflg & EFLG_RX_OVERFLOW) {
    mcp_modify_register(REG_EFLG, EFLG_RX_OVERFLOW, 0x00);  // overflow flags latch until cleared
  }

  bool modeLost = (mcp_read_register(REG_CANSTAT) & CANSTAT_OPMOD) != 0;
  bool txStuck = (nowMs - lastTxOkMs) >= TX_STUCK_MS;
  if ((modeLost || txStuck) && nowMs - lastRecoveryMs >= RECOVERY_RETRY_MS) {
    lastRecoveryMs = nowMs;
    if (canRecoveries < 255) canRecoveries++;
    Serial.print(F("CAN recovery: "));
    Serial.println(modeLost ? F("controller left normal mode") : F("TX stuck"));
    if (can_init()) diagDue = true;
  }
}

void send_diag() {
  unsigned long loopMaxMs = loopMaxUs / 1000;
  uint8_t data[8];
  data[0] = bootCount;
  data[1] = (uint8_t)((diagRolling & 0x0F) << 4) | (resetKind & 0x0F);
  data[2] = (uint8_t)((cruiseState & 0x03) << 6) | (stalkDevicePresent ? 0x20 : 0x00) |
            (stalkMainOn ? 0x10 : 0x00) | (lastTransition & 0x0F);
  data[3] = eflgLatched;
  data[4] = (uint8_t)(min(canRecoveries, (uint8_t)15) << 4) | min(stalkLosses, (uint8_t)15);
  data[5] = txFailsSinceDiag;
  data[6] = loopMaxMs > 255 ? 255 : (uint8_t)loopMaxMs;
  data[7] = toyota_checksum(DIAG_CAN_ID, data, 8);
  sendMsg(DIAG_CAN_ID, data, 8);

  diagRolling = (diagRolling + 1) & 0x0F;
  eflgLatched = 0;
  txFailsSinceDiag = 0;
  loopMaxUs = 0;
}

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  diag_boot();
  Serial.begin(115200);
  delay(1000);
  Serial.println("Corolla CAN Emulator starting...");
  Serial.print(F("Boot #"));
  Serial.println(bootCount);

  // Brake pedal switch — the emulator's only physical input.
  // Gear defaults to Drive (gearInDrive=true); the VSS device owns gear (0x3BC).
  pinMode(BRAKE_PIN, INPUT_PULLUP);  // NC switch: LOW=not pressed, HIGH=pressed (or wire break — fail-safe)

  #if VSS_DEVICE_INSTALLED
  Serial.println(F("Speed source: external VSS device (CAN 0xAA)"));
  #else
  Serial.println(F("Speed source: serial (type mph + enter) — emulator sends 0xAA/0xB4"));
  #endif

  // Init MCP2515
  while (!can_init()) {
    Serial.println(F("CAN init failed, retrying..."));
    delay(500);
  }
  Serial.println(F("CAN BUS init ok — RX filters: 0x69 (RXB0), 0xAA + 0x343 (RXB1)"));

  Serial.println("--- Config ---");
  Serial.print("REAL_EPS_CONNECTED:   "); Serial.println(REAL_EPS_CONNECTED ? "true" : "false");
  Serial.print("REAL_SAS_CONNECTED:   "); Serial.println(REAL_SAS_CONNECTED ? "true" : "false");
  Serial.print("REAL_PEDAL_CONNECTED: "); Serial.println(REAL_PEDAL_CONNECTED ? "true" : "false");
  Serial.print("VSS_DEVICE_INSTALLED: "); Serial.println(VSS_DEVICE_INSTALLED ? "true (listen for 0xAA)" : "false (serial speed; send 0xAA/0xB4)");
  Serial.print("SEND_ACC_CONTROL:     "); Serial.println(SEND_ACC_CONTROL ? "true (WARNING!)" : "false");
  Serial.println("--------------");
}

// ============================================================================
// MAIN LOOP (non-blocking, micros-based timing)
// ============================================================================

void loop() {
  unsigned long now = micros();

  // Local speed source — only present in bench mode (no external VSS device).
  // In production the VSS device owns 0xAA/0xB4, so the emulator sends neither.
  #if !VSS_DEVICE_INSTALLED
    uint16_t spd = serialSpeedKmh100;
  #endif

  // Brake pedal: read every loop iteration (safety-critical, no debounce)
  // NC switch with INPUT_PULLUP: HIGH = pressed (or wire break — fail-safe), LOW = not pressed
  #if FORCE_BRAKE_OFF
    brakePressed = false;
  #else
    brakePressed = (digitalRead(BRAKE_PIN) == HIGH);
  #endif

  // --- 83 Hz tier: WHEEL_SPEEDS, STEER_ANGLE_SENSOR ---
  // STEER_ANGLE_SENSOR needs 80 Hz per carstate parser
  if (now - lastMicros_83hz >= INTERVAL_83HZ) {
    lastMicros_83hz = now;

    // 0xAA only when no external VSS device (bench). In production the VSS device owns it.
    #if SEND_WHEEL_SPEEDS && !VSS_DEVICE_INSTALLED
      send_wheel_speeds(spd);
    #endif

    #if !REAL_SAS_CONNECTED
      send_steer_angle_sensor();
    #endif
  }

  // --- 50 Hz tier: STEER_TORQUE_SENSOR, SPEED, ENGINE_RPM, VSC1S07 ---
  // ENGINE_RPM needs 42 Hz per carstate, VSC1S07 needs 20 Hz — 50 Hz covers both
  if (now - lastMicros_50hz >= INTERVAL_50HZ) {
    lastMicros_50hz = now;

    #if !REAL_EPS_CONNECTED
      send_steer_torque_sensor();
    #endif

    #if !REAL_PEDAL_CONNECTED
      send_gas_sensor();
    #endif

    // 0xB4 only when no external VSS device (bench).
    #if SEND_SPEED && !VSS_DEVICE_INSTALLED
      send_speed(spd);
    #endif

    #if SEND_ENGINE_RPM
      send_engine_rpm();
    #endif

    #if SEND_EPS_INIT_4CB
      send_eps_init_4cb();
    #endif

    #if SEND_VSC1S07
      send_vsc1s07();
    #endif
  }

  // --- 40 Hz tier: BRAKE_MODULE ---
  if (now - lastMicros_40hz >= INTERVAL_40HZ) {
    lastMicros_40hz = now;

    #if SEND_BRAKE_MODULE
      send_brake_module();
    #endif
  }

  // --- 33 Hz tier: PCM_CRUISE, PCM_CRUISE_2, PRE_COLLISION ---
  if (now - lastMicros_33hz >= INTERVAL_33HZ) {
    lastMicros_33hz = now;

    #if SEND_PCM_CRUISE
      send_pcm_cruise(cruiseEngaged);
    #endif

    #if SEND_PCM_CRUISE_2
      send_pcm_cruise_2(cruiseMain, cruiseSetSpeed);
    #endif

    #if SEND_PRE_COLLISION
      send_pre_collision();
    #endif
  }

  // --- 25 Hz tier: EPS_STATUS ---
  if (now - lastMicros_25hz >= INTERVAL_25HZ) {
    lastMicros_25hz = now;

    #if !REAL_EPS_CONNECTED
      send_eps_status();
    #endif
  }

  // --- Background messages: a few per 10 ms slot (each ID ~10 Hz) ---
  if (now - lastMicros_bg >= INTERVAL_BG_SLOT) {
    lastMicros_bg = now;

    #if SEND_BACKGROUND_MSGS
      send_background_slot();
    #endif
  }

  // --- 3 Hz tier: BODY_CONTROL_STATE ---
  if (now - lastMicros_3hz >= INTERVAL_3HZ) {
    lastMicros_3hz = now;

    #if SEND_BODY_CONTROL
      send_body_control();
    #endif
  }

  // --- 1 Hz tier: GEAR_PACKET (fallback when VSS device absent) ---
  if (now - lastMicros_1hz >= INTERVAL_1HZ) {
    lastMicros_1hz = now;

    // 0x3BC only when no external VSS device (bench). The VSS device owns gear.
    #if SEND_GEAR_PACKET && !VSS_DEVICE_INSTALLED
      send_gear_packet(gearInDrive);
    #endif
  }

  // --- CAN receive: external VSS speed (0xAA) + cruise stalk (0x69) ---
  check_can_receive();

  // --- Serial speed input (bench only) + cruise stalk handling ---
  #if !VSS_DEVICE_INSTALLED
    check_serial();
  #endif
  handle_stalk_input();
  handle_op_cancel();

  // --- CAN controller health + diagnostics frame ---
  unsigned long nowMs = millis();
  if (nowMs - lastHealthMs >= HEALTH_INTERVAL_MS) {
    lastHealthMs = nowMs;
    can_health_check(nowMs);
  }
  if (diagDue || nowMs - lastDiagMs >= DIAG_INTERVAL_MS) {
    diagDue = false;
    lastDiagMs = nowMs;
    send_diag();
  }

  // --- Heartbeat: print status every 500 ms ---
  print_heartbeat();

  unsigned long loopUs = micros() - now;
  if (loopUs > loopMaxUs) loopMaxUs = loopUs;
}
