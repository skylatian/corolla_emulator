// ============================================================================
// Cruise Stalk CAN Device — Standalone Stalk + Blinker Reader
// Reads Toyota cruise stalk resistance on analog pin and blinker switch
// state on two digital pins. Sends CAN messages at 20 Hz.
//
// Hardware: Longan CANBed V1 (ATmega32U4, Leonardo bootloader, 16MHz)
//           Onboard MCP2515 CAN controller, CS=D17
//           Stalk signal → A0 (resistance-based, all buttons on one wire)
//           DPDT relay on D4 — switches stalk wiring between stock ECU and A0
//           Left blinker sense → BLINKER_LEFT_PIN (12V via divider, LOW = active)
//           Right blinker sense → BLINKER_RIGHT_PIN (12V via divider, LOW = active)
//
// Library:  mcp_can 1.5.1 (coryjfowler) — this is what `#include <mcp_can.h>`
//           resolves to for the Leonardo build (checked with arduino-cli -v).
//
// Sends:
//   0x69  CRUISE_STALK   at 20 Hz — 3 bytes: button event + MAIN state + checksum
//   0x614 BLINKERS_STATE at 20 Hz — 8 bytes: TURN_SIGNALS (1=left, 2=right, 3=none)
//   0x501 RETROFIT_STALK_DIAG at 2 Hz — boot count, reset cause, CAN health
//         (layout in the DIAGNOSTICS section; decoded by openpilot's DBC)
//
// Fault handling:
//   - Hardware watchdog (500 ms). A hung loop or a hung library call resets the
//     board; the watchdog interrupt records which phase it hung in (EEPROM) so the
//     next 0x501 frame reports it.
//   - CAN controller recovery. If nothing has been sent successfully for
//     TX_STUCK_MS, or the MCP2515 has left normal mode, the controller is
//     re-initialized. Any recovery or reboot comes back with MAIN off, so the
//     emulator turns cruise fully off — a stalk fault never leaves cruise or
//     Always-On Lateral running without a working stalk.
//
// Relay behavior:
//   The stalk's 2 analog wires are routed through a DPDT relay.
//   - Relay de-energized (NC): stalk wired to stock ECU (normal cruise)
//   - Relay energized (NO):   stalk wired to Arduino A0 (OP cruise via CAN)
//   The relay is energized when 0x2E4 (STEERING_LKA) is seen on the CAN bus,
//   indicating openpilot is actively running (onroad). When 0x2E4 disappears
//   (C3 goes offroad or disconnects), the relay de-energizes after a timeout,
//   returning the stalk to the stock ECU. A CANCEL event is sent on timeout
//   so the emulator disengages cruise cleanly.
//
// The stalk is a 2-wire resistance-based device. Each position (SET/-,
// RES/+, CANCEL, MAIN) produces a different resistance value on A0.
// All positions are momentary (spring-return). MAIN is tracked as a
// toggle internally: each MAIN pulse flips the mainOn bool.
//
// Blinker sense lines are 12V from the car's blinker ECU, pulled to ground
// by the blinker stalk. Read via voltage dividers. Signal is steady (not
// flashing like the bulbs). Hazards are handled by a separate flasher relay
// and won't pull both lines low simultaneously.
// ============================================================================

#include <SPI.h>
#include <EEPROM.h>
#include <avr/wdt.h>
#include <mcp_can.h>  // coryjfowler mcp_can 1.5.1

// ============================================================================
// FIRMWARE VERSION
// Bump FW_VERSION in every commit that changes this sketch: MMDDYY plus one
// character, a-z then 0-9 (e.g. "092726a", "092726b"). The pre-commit hook in
// .githooks/ enforces the bump and the format. Never used for fingerprinting.
//
// VERSION_CAN_ID (8 bytes, alternating pages, each page every ~1 s):
//   byte 0      PAGE   0 = version tag, 1 = build time
//   page 0      bytes 1-7: FW_VERSION as ASCII
//   page 1      bytes 1-6: build year-2000, month, day, hour, minute, second
//               (compiler __DATE__/__TIME__, so a flash of an edited but
//               un-bumped sketch still shows up as a different build)
// ============================================================================

#define FW_VERSION      "092726a"
#define VERSION_CAN_ID  0x504
static_assert(sizeof(FW_VERSION) - 1 == 7, "FW_VERSION must be 7 characters: MMDDYY + one of a-z/0-9");

// ============================================================================
// CONFIGURATION
// ============================================================================

#define CAN_CS_PIN  17  // CANBed V1 onboard MCP2515 CS pin
#define STALK_PIN   A3  // Analog input for stalk resistance
#define RELAY_PIN   4   // DPDT relay control: HIGH = stalk to Arduino, LOW = stalk to stock ECU

// Blinker sense pins — INPUT_PULLUP, LOW = blinker active
// 12V sense lines through voltage dividers to 5V logic.
// Assign actual pin numbers when wiring.
#define BLINKER_LEFT_PIN   5   // Left blinker sense (LOW = left turn active)
#define BLINKER_RIGHT_PIN  6   // Right blinker sense (LOW = right turn active)

// CAN message ID for stalk data (not used by any Toyota DBC message)
#define STALK_CAN_ID  0x69

// Set true to force relay ON (stalk to Arduino) regardless of C3 detection (testing override)
#define FORCE_RELAY_ON  false

// CAN message ID to detect openpilot onroad (only OP sends this via panda)
#define C3_DETECT_ID  0x2E4

// Timeout: how long without 0x2E4 before we consider C3 gone and flip relay back.
// OP sends STEERING_LKA at ~50 Hz, so 500ms = ~25 missed frames.
#define C3_TIMEOUT_MS  500

// What to do on C3 timeout: send a CANCEL event so emulator disengages cruise.
// Set to false to just flip the relay without sending CANCEL.
#define C3_TIMEOUT_SEND_CANCEL  true

// ============================================================================
// RESISTANCE THRESHOLDS (ADC values, 0-1023)
// Each button produces a different resistance on the same 2-wire circuit.
// Wire a voltage divider: 5V → 10k pullup → A0 → stalk → GND.
// Then read ADC values for each button position.
// ============================================================================

// note: these values are all with a ~470 ohm resistor (measured about 466)

#define STALK_RANGE        75   // Half-width around each midpoint

#define STALK_SET_MID      430
#define STALK_SET_MIN      (STALK_SET_MID - STALK_RANGE)
#define STALK_SET_MAX      (STALK_SET_MID + STALK_RANGE)

#define STALK_RES_MID      675
#define STALK_RES_MIN      (STALK_RES_MID - STALK_RANGE)
#define STALK_RES_MAX      (STALK_RES_MID + STALK_RANGE)

#define STALK_CANCEL_MID   235
#define STALK_CANCEL_MIN   (STALK_CANCEL_MID - STALK_RANGE)
#define STALK_CANCEL_MAX   (STALK_CANCEL_MID + STALK_RANGE)

#define STALK_MAIN_MIN     900
#define STALK_MAIN_MAX     1023

static_assert(STALK_CANCEL_MAX < STALK_SET_MIN,  "CANCEL and SET ranges overlap");
static_assert(STALK_SET_MAX    < STALK_RES_MIN,  "SET and RES ranges overlap");
static_assert(STALK_RES_MAX    < STALK_MAIN_MIN, "RES and MAIN ranges overlap");

// ============================================================================
// BUTTON EVENT CODES (byte 0 of CAN message)
// ============================================================================

#define BTN_NONE         0
#define BTN_SET          1
#define BTN_RES          2
#define BTN_CANCEL       3
#define BTN_MAIN_TOGGLE  4

// ============================================================================
// TIMING
// ============================================================================

#define INTERVAL_20HZ    50000  // 50ms = 20 Hz
#define HEARTBEAT_MS     2000

// ============================================================================
// DIAGNOSTICS + FAULT RECOVERY
//
// 0x501 RETROFIT_STALK_DIAG (8 bytes, 2 Hz + once right after boot):
//   byte 0      BOOT_COUNT       increments every boot (EEPROM), wraps at 255
//   byte 1 hi   ROLLING_COUNTER  4-bit, increments per frame (not named COUNTER on
//                                purpose: openpilot's parser would treat a jump
//                                after a reboot as a CAN error)
//   byte 1 lo   RESET_KIND       why the board last started (RESET_* below)
//   byte 2 hi   RESET_PHASE      loop phase the watchdog caught it in (PHASE_*)
//   byte 2 3:2  RECOVERY_REASON  last CAN controller recovery (RECOVER_*)
//   byte 2 b1   C3_PRESENT       0x2E4 seen (relay energized)
//   byte 2 b0   MAIN_ON
//   byte 3      CAN_ERROR_FLAGS  MCP2515 EFLG, OR-latched since the last frame
//   byte 4      TX_FAILS         failed sends since the last frame (saturates)
//   byte 5 hi   INIT_RETRIES     failed CAN init attempts at boot (saturates at 15)
//   byte 5 lo   CAN_RECOVERIES   controller re-inits since boot (saturates at 15)
//   byte 6      LOOP_MAX_MS      slowest loop() since the last frame (saturates)
//   byte 7      CHECKSUM         toyota_checksum
// ============================================================================

#define DIAG_CAN_ID          0x501
#define DIAG_INTERVAL_MS     500
#define HEALTH_INTERVAL_MS   100   // CAN controller health check period
#define TX_STUCK_MS          500   // no successful send for this long → re-init controller
#define RECOVERY_RETRY_MS    500   // minimum spacing between re-init attempts
#define RECOVERY_MAX_FAILS   5     // consecutive failed re-inits → deliberate reset

// EEPROM layout
#define EE_BOOT_COUNT   0
#define EE_RESET_CAUSE  1   // written just before a watchdog/deliberate reset
#define EE_RESET_PHASE  2

// RESET_KIND values (shared by all three retrofit ECUs)
#define RESET_UNKNOWN     0  // power-on or brown-out: the bootloader clears MCUSR, so these
                             // can't be told apart here. On the CANBed (Caterina) a
                             // brown-out also holds the board in the bootloader ~8 s,
                             // so the gap length in the log separates the two.
#define RESET_POWER_ON    1  // from MCUSR (only when no bootloader cleared it)
#define RESET_BROWN_OUT   2
#define RESET_EXTERNAL    3
#define RESET_WATCHDOG    4
#define RESET_DELIBERATE  5  // gave up recovering the CAN controller

// RESET_PHASE values: what the loop was doing (recorded by the watchdog interrupt)
#define PHASE_SETUP      1
#define PHASE_CAN_INIT   2
#define PHASE_CAN_RX     3
#define PHASE_STALK      4
#define PHASE_CAN_TX     5
#define PHASE_HEALTH     6
#define PHASE_SERIAL     7
#define PHASE_RECOVERY   8
#define PHASE_LOOP       9

// RECOVERY_REASON values
#define RECOVER_NONE      0
#define RECOVER_TX_STUCK  1  // nothing sent successfully for TX_STUCK_MS
#define RECOVER_MODE      2  // MCP2515 left normal mode (e.g. it reset on its own)
#define RECOVER_BUS_OFF   3  // TX stuck while the controller reported bus-off

// MCP2515 registers not exposed by the library's public API
#define REG_CANSTAT      0x0E
#define REG_EFLG         0x2D
#define CANSTAT_OPMOD    0xE0  // bits 7:5, 000 = normal
#define EFLG_RX_OVERFLOW 0xC0  // RX1OVR | RX0OVR (latched until cleared)
#define EFLG_TXBO        0x20

// SET/RES auto-repeat: after holding for REPEAT_DELAY_MS, repeat every REPEAT_INTERVAL_MS
#define REPEAT_DELAY_MS     500   // Initial hold before repeating starts
#define REPEAT_INTERVAL_MS  200   // Repeat rate once held (5 km/h per second)
#define DEBOUNCE_MS          20   // ADC must read stable for this long before accepting

unsigned long lastMicros_20hz = 0;
unsigned long lastMillis_hb   = 0;

// ============================================================================
// STATE
// ============================================================================

MCP_CAN CAN(CAN_CS_PIN);

bool mainOn = false;              // MAIN toggle state (flips on each MAIN press)
uint8_t pendingEvent = BTN_NONE;  // Button event to send (cleared after one frame)
uint8_t lastButton = BTN_NONE;   // Previous loop's decoded button (for edge detection)
unsigned long holdStartMs = 0;   // When the current button was first pressed
unsigned long lastRepeatMs = 0;  // Last auto-repeat timestamp

// Debounce state
uint8_t candidateBtn = BTN_NONE;     // Button being debounced
unsigned long candidateStartMs = 0;  // When candidate first appeared
uint8_t stableBtn = BTN_NONE;       // Last confirmed-stable button
uint32_t debounceRejects = 0;       // Noise metric: candidates that didn't settle
uint32_t debounceRejectsLastHb = 0;

// C3/relay state
bool c3Present = false;           // true when 0x2E4 seen within timeout
unsigned long lastC3Ms = 0;       // millis() of last 0x2E4 received
uint32_t c3RxCount = 0;           // 0x2E4 frames received since boot (debug)
uint32_t c3RxCountLastHb = 0;     // snapshot for per-heartbeat rate

// Blinker state: DBC TURN_SIGNALS value (1=left, 2=right, 3=none)
uint8_t turnSignals = 3;  // default: none

// TX stats
uint32_t txOk    = 0;
uint32_t txError = 0;

// Diagnostics state
uint8_t bootCount = 0;
uint8_t resetKind = RESET_UNKNOWN;
uint8_t resetPhase = 0;
volatile uint8_t currentPhase = PHASE_SETUP;  // read by the watchdog interrupt
uint8_t diagRolling = 0;
uint8_t eflgLatched = 0;
uint8_t txFailsSinceDiag = 0;
uint8_t initRetries = 0;
uint8_t canRecoveries = 0;
uint8_t lastRecoveryReason = RECOVER_NONE;
uint8_t recoveryFailStreak = 0;
unsigned long loopMaxUs = 0;
unsigned long lastTxOkMs = 0;
unsigned long lastHealthMs = 0;
unsigned long lastRecoveryMs = 0;
unsigned long lastDiagMs = 0;
bool diagDue = true;  // send one right after boot

// ============================================================================
// STALK READING
// ============================================================================

// Decode ADC value into button. No state tracking — pure analog read.
uint8_t decode_stalk_adc() {
  int adc = analogRead(STALK_PIN);

  if (adc >= STALK_SET_MIN && adc <= STALK_SET_MAX)       return BTN_SET;
  if (adc >= STALK_RES_MIN && adc <= STALK_RES_MAX)       return BTN_RES;
  if (adc >= STALK_CANCEL_MIN && adc <= STALK_CANCEL_MAX) return BTN_CANCEL;
  if (adc >= STALK_MAIN_MIN && adc <= STALK_MAIN_MAX)     return BTN_MAIN_TOGGLE;
  return BTN_NONE;
}

// Read stalk with edge detection + auto-repeat for SET/RES.
// MAIN and CANCEL: fire once per press (edge-detected).
// SET and RES: fire on initial press, then auto-repeat after hold delay.
// Returns BTN_NONE if no event to send.
uint8_t read_stalk() {
  uint8_t rawBtn = decode_stalk_adc();
  unsigned long now = millis();

  // Debounce: require stable reading for DEBOUNCE_MS before accepting
  if (rawBtn != candidateBtn) {
    if (candidateBtn != stableBtn) debounceRejects++;
    candidateBtn = rawBtn;
    candidateStartMs = now;
  }
  if (candidateBtn != stableBtn && (now - candidateStartMs) >= DEBOUNCE_MS) {
    stableBtn = candidateBtn;
  }
  uint8_t btn = stableBtn;

  // --- New press (rising edge: NONE or different button → this button) ---
  if (btn != BTN_NONE && btn != lastButton) {
    lastButton = btn;
    holdStartMs = now;
    lastRepeatMs = now;

    if (btn == BTN_MAIN_TOGGLE) {
      mainOn = !mainOn;
      Serial.print(F("MAIN toggled: "));
      Serial.println(mainOn ? "ON" : "OFF");
    } else {
      Serial.print(F("Button: "));
      switch (btn) {
        case BTN_SET:    Serial.println(F("SET")); break;
        case BTN_RES:    Serial.println(F("RES")); break;
        case BTN_CANCEL: Serial.println(F("CANCEL")); break;
      }
    }
    return btn;
  }

  // --- Button held (same button as last loop) ---
  if (btn != BTN_NONE && btn == lastButton) {
    // Only auto-repeat SET and RES
    if ((btn == BTN_SET || btn == BTN_RES) &&
        (now - holdStartMs >= REPEAT_DELAY_MS) &&
        (now - lastRepeatMs >= REPEAT_INTERVAL_MS)) {
      lastRepeatMs = now;
      return btn;
    }
    return BTN_NONE;  // MAIN/CANCEL held or repeat not yet due
  }

  // --- Button released ---
  lastButton = BTN_NONE;
  return BTN_NONE;
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
  return sum;
}

// ============================================================================
// CAN SEND
// ============================================================================

bool sendMsg(uint16_t id, uint8_t *data, uint8_t len) {
  currentPhase = PHASE_CAN_TX;
  byte result = CAN.sendMsgBuf(id, 0, len, data);
  if (result == CAN_OK) {
    txOk++;
    lastTxOkMs = millis();
    return true;
  } else {
    txError++;
    if (txFailsSinceDiag < 255) txFailsSinceDiag++;
    return false;
  }
}

// Send 0x69: byte 0 = button event, byte 1 = MAIN state, byte 2 = toyota_checksum
bool send_stalk_msg(uint8_t button_event) {
  uint8_t data[3];
  data[0] = button_event;
  data[1] = mainOn ? 1 : 0;
  data[2] = toyota_checksum(STALK_CAN_ID, data, 3);
  return sendMsg(STALK_CAN_ID, data, 3);
}

// 0x614 — BLINKERS_STATE (8 bytes, no checksum)
// DBC: TURN_SIGNALS at 29|2@0+ — byte 3, bits 5:4
// Values: 1=left, 2=right, 3=none
void send_blinkers_state(uint8_t turn_signals) {
  uint8_t data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  data[3] = (turn_signals & 0x03) << 4;
  sendMsg(0x614, data, 8);
}

// Read blinker switches and return DBC TURN_SIGNALS value
uint8_t read_blinkers() {
  if (digitalRead(BLINKER_LEFT_PIN) == LOW)  return 1;  // left
  if (digitalRead(BLINKER_RIGHT_PIN) == LOW) return 2;  // right
  return 3;                                               // none
}

// ============================================================================
// CAN RECEIVE — detect C3 presence via 0x2E4
// ============================================================================

// MCP2515 has only 2 RX hardware buffers. With MCP_ANY the bus flood from the
// emulator, EPS, and VSS device can fill both slots between loop() calls, so
// 0x2E4 frames get dropped before software ever reads them. Hardware filters
// discard everything except 0x2E4 at the controller — RX buffers stay free.
//
// Filter encoding (Longan/Seeed mcp_can library): standard 11-bit ID << 16.
// Mask 0x07FF0000 = match all ID bits exactly. Filter 0x02E40000 = ID 0x2E4.
#define C3_RX_MASK   0x07FF0000UL
#define C3_RX_FILTER ((unsigned long)C3_DETECT_ID << 16)

bool setup_c3_rx_filter() {
  // Must run after setMode(MCP_NORMAL) — init_Mask returns to mcpMode, and begin()
  // leaves mcpMode as LOOPBACK which fails on a live bus.
  if (CAN.init_Mask(0, 0, C3_RX_MASK) != MCP2515_OK ||
      CAN.init_Filt(0, 0, C3_RX_FILTER) != MCP2515_OK ||
      CAN.init_Filt(1, 0, C3_RX_FILTER) != MCP2515_OK ||
      CAN.init_Mask(1, 0, C3_RX_MASK) != MCP2515_OK ||
      CAN.init_Filt(2, 0, C3_RX_FILTER) != MCP2515_OK ||
      CAN.init_Filt(3, 0, C3_RX_FILTER) != MCP2515_OK ||
      CAN.init_Filt(4, 0, C3_RX_FILTER) != MCP2515_OK ||
      CAN.init_Filt(5, 0, C3_RX_FILTER) != MCP2515_OK) {
    Serial.println(F("CAN filter setup failed"));
    return false;
  }
  return true;
}

void check_can_receive() {
  #if FORCE_RELAY_ON
    // Still drain RX buffers to prevent overflow, but ignore C3 detection
    while (CAN.checkReceive() == CAN_MSGAVAIL) {
      unsigned long rxId;
      uint8_t rxLen;
      uint8_t rxBuf[8];
      CAN.readMsgBuf(&rxId, &rxLen, rxBuf);
    }
    return;
  #endif

  unsigned long nowMs = millis();

  currentPhase = PHASE_CAN_RX;

  // Hardware filter guarantees every received frame is 0x2E4
  while (CAN.checkReceive() == CAN_MSGAVAIL) {
    unsigned long rxId;
    uint8_t rxLen;
    uint8_t rxBuf[8];
    CAN.readMsgBuf(&rxId, &rxLen, rxBuf);

    // Belt-and-suspenders: verify ID and expected STEERING_LKA length (5 bytes)
    if (rxId != C3_DETECT_ID || rxLen != 5) {
      continue;
    }

    c3RxCount++;
    lastC3Ms = nowMs;

    if (!c3Present) {
      c3Present = true;
      digitalWrite(RELAY_PIN, HIGH);  // Route stalk to Arduino
      lastButton = BTN_NONE;
      candidateBtn = BTN_NONE;
      stableBtn = BTN_NONE;
      candidateStartMs = nowMs;
      Serial.println(F("C3 DETECTED (0x2E4) — relay ON, stalk to Arduino"));
    }
  }

  // Timeout: C3 gone
  if (c3Present && (nowMs - lastC3Ms) >= C3_TIMEOUT_MS) {
    c3Present = false;
    digitalWrite(RELAY_PIN, LOW);  // Route stalk back to stock ECU
    lastButton = BTN_NONE;
    candidateBtn = BTN_NONE;
    stableBtn = BTN_NONE;
    Serial.println(F("C3 LOST — relay OFF, stalk to stock ECU"));

    #if C3_TIMEOUT_SEND_CANCEL
      // Send CANCEL so emulator disengages cruise cleanly
      pendingEvent = BTN_CANCEL;
      send_stalk_msg(pendingEvent);
      pendingEvent = BTN_NONE;
      Serial.println(F("  Sent CANCEL to emulator"));
    #endif
  }
}

// ============================================================================
// WATCHDOG
// Interrupt-then-reset mode: the first 500 ms timeout runs WDT_vect, which records
// the phase the loop was stuck in, then forces the reset. The bootloader clears
// MCUSR, so EEPROM is the only record of a watchdog reset that survives it.
// ============================================================================

ISR(WDT_vect) {
  eeprom_write_byte((uint8_t *)EE_RESET_CAUSE, RESET_WATCHDOG);
  eeprom_write_byte((uint8_t *)EE_RESET_PHASE, currentPhase);
  wdt_enable(WDTO_15MS);  // plain reset mode, fires almost immediately
  while (true) {}
}

void watchdog_arm() {
  cli();
  wdt_reset();
  WDTCSR = _BV(WDCE) | _BV(WDE);
  WDTCSR = _BV(WDIE) | _BV(WDE) | _BV(WDP2) | _BV(WDP0);  // 500 ms, interrupt then reset
  sei();
}

// Reset on purpose (CAN controller could not be recovered). Recorded like a watchdog
// reset so the next boot reports it.
void deliberate_reset() {
  Serial.println(F("CAN recovery failed repeatedly — resetting board"));
  eeprom_write_byte((uint8_t *)EE_RESET_CAUSE, RESET_DELIBERATE);
  eeprom_write_byte((uint8_t *)EE_RESET_PHASE, PHASE_RECOVERY);
  cli();
  wdt_enable(WDTO_15MS);
  while (true) {}
}

// Work out why we booted, then clear the record for next time.
void diag_boot() {
  uint8_t mcusr = MCUSR;
  MCUSR = 0;
  uint8_t cause = eeprom_read_byte((uint8_t *)EE_RESET_CAUSE);
  if (cause == RESET_WATCHDOG || cause == RESET_DELIBERATE) {
    resetKind = cause;
    resetPhase = eeprom_read_byte((uint8_t *)EE_RESET_PHASE) & 0x0F;
  } else if (mcusr & _BV(WDRF)) {
    resetKind = RESET_WATCHDOG;
  } else if (mcusr & _BV(BORF)) {
    resetKind = RESET_BROWN_OUT;
  } else if (mcusr & _BV(EXTRF)) {
    resetKind = RESET_EXTERNAL;
  } else if (mcusr & _BV(PORF)) {
    resetKind = RESET_POWER_ON;
  } else {
    resetKind = RESET_UNKNOWN;
  }
  if (cause != 0) {
    EEPROM.update(EE_RESET_CAUSE, 0);
    EEPROM.update(EE_RESET_PHASE, 0);
  }
  bootCount = EEPROM.read(EE_BOOT_COUNT) + 1;
  EEPROM.update(EE_BOOT_COUNT, bootCount);
}

// ============================================================================
// CAN CONTROLLER INIT + HEALTH
// ============================================================================

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

// Bring the MCP2515 up in normal mode with the 0x2E4 filter. Used at boot and for
// recovery. Returns false if any step failed (caller retries).
bool can_init() {
  currentPhase = PHASE_CAN_INIT;

  // Use MCP_STDEXT so MCP2515 masks/filters stay enabled. MCP_ANY disables
  // hardware filtering, and MCP_STD is broken/commented out in this library.
  //
  // begin() configures baud rate then tries to enter loopback mode before returning.
  // On a live bus that loopback step can fail ("Returning to Previous Mode Failure")
  // even though config/baud succeeded, so continue to an explicit NORMAL mode check.
  byte beginResult = CAN.begin(MCP_STDEXT, CAN_500KBPS, MCP_16MHZ);
  if (beginResult != CAN_OK) {
    Serial.print(F("CAN begin warning/result: "));
    Serial.println(beginResult);
  }
  wdt_reset();

  // Switch to normal mode BEFORE filter setup so init_Mask returns here, not loopback.
  if (CAN.setMode(MCP_NORMAL) != MCP2515_OK) {
    Serial.println(F("CAN setMode(NORMAL) failed — check MCP2515 wiring/SPI"));
    return false;
  }
  if (!setup_c3_rx_filter()) {
    return false;
  }
  lastTxOkMs = millis();  // give the fresh controller a full TX_STUCK_MS window
  return true;
}

// Re-initialize the controller after a fault. Cruise always comes back with MAIN
// off: during the outage the emulator either saw no frames (and turned cruise off
// after 1 s) or is about to see MAIN fall — either way both sides agree on OFF.
void can_recover(uint8_t reason) {
  currentPhase = PHASE_RECOVERY;
  lastRecoveryMs = millis();
  lastRecoveryReason = reason;
  if (canRecoveries < 255) canRecoveries++;

  Serial.print(F("CAN recovery, reason "));
  Serial.println(reason);

  mainOn = false;
  pendingEvent = BTN_NONE;

  if (can_init()) {
    recoveryFailStreak = 0;
    diagDue = true;
  } else if (++recoveryFailStreak >= RECOVERY_MAX_FAILS) {
    deliberate_reset();
  }
}

void can_health_check(unsigned long nowMs) {
  currentPhase = PHASE_HEALTH;
  uint8_t eflg = mcp_read_register(REG_EFLG);
  eflgLatched |= eflg;
  if (eflg & EFLG_RX_OVERFLOW) {
    mcp_modify_register(REG_EFLG, EFLG_RX_OVERFLOW, 0x00);  // overflow flags latch until cleared
  }

  uint8_t reason = RECOVER_NONE;
  if ((mcp_read_register(REG_CANSTAT) & CANSTAT_OPMOD) != 0) {
    reason = RECOVER_MODE;
  } else if (nowMs - lastTxOkMs >= TX_STUCK_MS) {
    reason = (eflg & EFLG_TXBO) ? RECOVER_BUS_OFF : RECOVER_TX_STUCK;
  }

  if (reason != RECOVER_NONE && nowMs - lastRecoveryMs >= RECOVERY_RETRY_MS) {
    can_recover(reason);
  }
}

void send_diag() {
  unsigned long loopMaxMs = loopMaxUs / 1000;
  uint8_t data[8];
  data[0] = bootCount;
  data[1] = (uint8_t)((diagRolling & 0x0F) << 4) | (resetKind & 0x0F);
  data[2] = (uint8_t)((resetPhase & 0x0F) << 4) | ((lastRecoveryReason & 0x03) << 2) |
            (c3Present ? 0x02 : 0x00) | (mainOn ? 0x01 : 0x00);
  data[3] = eflgLatched;
  data[4] = txFailsSinceDiag;
  data[5] = (uint8_t)(min(initRetries, (uint8_t)15) << 4) | min(canRecoveries, (uint8_t)15);
  data[6] = loopMaxMs > 255 ? 255 : (uint8_t)loopMaxMs;
  data[7] = toyota_checksum(DIAG_CAN_ID, data, 8);
  sendMsg(DIAG_CAN_ID, data, 8);

  diagRolling = (diagRolling + 1) & 0x0F;
  eflgLatched = 0;
  txFailsSinceDiag = 0;
  loopMaxUs = 0;
}

// ============================================================================
// FIRMWARE VERSION FRAME
// ============================================================================

uint8_t buildTime[6];  // year-2000, month, day, hour, minute, second
uint8_t versionPage = 0;

uint8_t two_digits(const char *s) {
  return (uint8_t)((s[0] == ' ' ? 0 : s[0] - '0') * 10 + (s[1] - '0'));
}

// __DATE__ is "Sep 27 2026" (day space-padded), __TIME__ is "14:03:22".
void parse_build_time() {
  static const char months[] = "JanFebMarAprMayJunJulAugSepOctNovDec";
  const char *date = __DATE__;
  const char *time = __TIME__;
  buildTime[1] = 0;
  for (uint8_t i = 0; i < 12; i++) {
    if (strncmp(date, months + 3 * i, 3) == 0) buildTime[1] = i + 1;
  }
  buildTime[0] = two_digits(date + 9);
  buildTime[2] = two_digits(date + 4);
  buildTime[3] = two_digits(time);
  buildTime[4] = two_digits(time + 3);
  buildTime[5] = two_digits(time + 6);
}

void print_version() {
  Serial.print(F("Firmware " FW_VERSION ", built " __DATE__ " " __TIME__));
  Serial.println();
}

// Called with every diagnostics frame: alternates the version tag and build time.
void send_version_page() {
  uint8_t data[8] = {versionPage, 0, 0, 0, 0, 0, 0, 0};
  if (versionPage == 0) {
    memcpy(data + 1, FW_VERSION, 7);
  } else {
    memcpy(data + 1, buildTime, 6);
  }
  sendMsg(VERSION_CAN_ID, data, 8);
  versionPage ^= 1;
}

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  diag_boot();
  watchdog_arm();

  Serial.begin(115200);
  Serial.println(F("Cruise Stalk Device starting..."));

  // Relay pin
  pinMode(RELAY_PIN, OUTPUT);
  #if FORCE_RELAY_ON
    digitalWrite(RELAY_PIN, HIGH);  // Force relay ON (testing override)
    c3Present = true;
  #else
    digitalWrite(RELAY_PIN, LOW);   // Start de-energized (NC = stalk to stock ECU)
  #endif

  // Blinker sense pins
  pinMode(BLINKER_LEFT_PIN, INPUT_PULLUP);
  pinMode(BLINKER_RIGHT_PIN, INPUT_PULLUP);

  parse_build_time();
  print_version();
  Serial.print(F("Boot #"));
  Serial.print(bootCount);
  Serial.print(F("  reset kind "));
  Serial.print(resetKind);
  Serial.print(F("  phase "));
  Serial.println(resetPhase);
  Serial.print(F("Stalk pin: A"));
  Serial.println(STALK_PIN - A0);
  Serial.print(F("Relay pin: D"));
  Serial.println(RELAY_PIN);
  Serial.print(F("CAN ID: 0x"));
  Serial.println(STALK_CAN_ID, HEX);
  Serial.print(F("C3 detect: 0x"));
  Serial.println(C3_DETECT_ID, HEX);
  Serial.print(F("C3 timeout: "));
  Serial.print(C3_TIMEOUT_MS);
  Serial.println(F("ms"));

  // Keep retrying instead of halting: a halt here used to leave the board silent
  // until power-cycled. The watchdog still catches a call that never returns.
  while (!can_init()) {
    if (initRetries < 255) initRetries++;
    Serial.println(F("CAN init failed, retrying..."));
    for (uint8_t i = 0; i < 4; i++) {
      wdt_reset();
      delay(50);
    }
  }
  Serial.println(F("CAN BUS init ok — RX filter 0x2E4 only, sending 0x69 at 20 Hz"));
  currentPhase = PHASE_LOOP;
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  wdt_reset();
  unsigned long now = micros();
  unsigned long nowMs = now / 1000;
  currentPhase = PHASE_LOOP;

  // Check for C3 presence and manage relay
  check_can_receive();

  // Only read stalk when relay is energized (stalk wired to A0)
  if (c3Present) {
    currentPhase = PHASE_STALK;
    uint8_t btn = read_stalk();
    if (btn != BTN_NONE) {
      pendingEvent = btn;  // Latch event for next CAN frame
    }
  }

  // --- Read blinkers (every loop — digitalRead is ~4µs) ---
  turnSignals = read_blinkers();

  // --- 20 Hz: send stalk + blinker CAN messages ---
  if (now - lastMicros_20hz >= INTERVAL_20HZ) {
    lastMicros_20hz = now;
    if (send_stalk_msg(pendingEvent)) {
      pendingEvent = BTN_NONE;
    }
    send_blinkers_state(turnSignals);
  }

  // --- CAN controller health (recovers a stuck or reset MCP2515) ---
  if (nowMs - lastHealthMs >= HEALTH_INTERVAL_MS) {
    lastHealthMs = nowMs;
    can_health_check(nowMs);
  }

  // --- 2 Hz: diagnostics frame ---
  if (diagDue || nowMs - lastDiagMs >= DIAG_INTERVAL_MS) {
    diagDue = false;
    lastDiagMs = nowMs;
    send_diag();
    send_version_page();
  }

  // --- Heartbeat ---
  if (nowMs - lastMillis_hb >= HEARTBEAT_MS) {
    currentPhase = PHASE_SERIAL;
    lastMillis_hb = nowMs;
    Serial.print(F("MAIN: "));
    Serial.print(mainOn ? "ON" : "OFF");
    Serial.print(F("  C3: "));
    Serial.print(c3Present ? "ON" : "off");
    Serial.print(F("  2E4/s: "));
    Serial.print((c3RxCount - c3RxCountLastHb) / 2);  // heartbeat every 2s
    c3RxCountLastHb = c3RxCount;
    if (c3Present) {
      Serial.print(F("  ADC: "));
      Serial.print(analogRead(STALK_PIN));
      Serial.print(F("  noise: "));
      Serial.print(debounceRejects - debounceRejectsLastHb);
      debounceRejectsLastHb = debounceRejects;
    }
    Serial.print(F("  Blink: "));
    switch (turnSignals) {
      case 1:  Serial.print(F("L")); break;
      case 2:  Serial.print(F("R")); break;
      default: Serial.print(F("-")); break;
    }
    Serial.print(F("  TX ok: "));
    Serial.print(txOk);
    Serial.print(F("  err: "));
    Serial.print(txError);
    Serial.print(F("  boot: "));
    Serial.print(bootCount);
    Serial.print(F("  recov: "));
    Serial.println(canRecoveries);
  }

  unsigned long loopUs = micros() - now;
  if (loopUs > loopMaxUs) loopMaxUs = loopUs;
}
