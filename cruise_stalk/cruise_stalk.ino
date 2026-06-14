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
// Library:  Longan Labs Arduino_CAN_BUS_MCP2515
//           https://github.com/Longan-Labs/Aruino_CAN_BUS_MCP2515
//
// Sends:
//   0x69  CRUISE_STALK   at 20 Hz — 3 bytes: button event + MAIN state + checksum
//   0x614 BLINKERS_STATE at 20 Hz — 8 bytes: TURN_SIGNALS (1=left, 2=right, 3=none)
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
#include <mcp_can.h>  // Longan Labs MCP2515 library

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
#define FORCE_RELAY_ON  true

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

#define STALK_SET_MIN      380 // mid ~430
#define STALK_SET_MAX      490

#define STALK_RES_MIN      625 // mid ~675
#define STALK_RES_MAX      925

#define STALK_CANCEL_MIN   185 // mid ~235
#define STALK_CANCEL_MAX   295

#define STALK_MAIN_MIN     900 // max
#define STALK_MAIN_MAX     1023

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

// SET/RES auto-repeat: after holding for REPEAT_DELAY_MS, repeat every REPEAT_INTERVAL_MS
#define REPEAT_DELAY_MS     500   // Initial hold before repeating starts
#define REPEAT_INTERVAL_MS  200   // Repeat rate once held (5 km/h per second)

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
  uint8_t btn = decode_stalk_adc();
  unsigned long now = millis();

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

void sendMsg(uint16_t id, uint8_t *data, uint8_t len) {
  byte result = CAN.sendMsgBuf(id, 0, len, data);
  if (result == CAN_OK) {
    txOk++;
  } else {
    txError++;
  }
}

// Send 0x69: byte 0 = button event, byte 1 = MAIN state, byte 2 = toyota_checksum
void send_stalk_msg(uint8_t button_event) {
  uint8_t data[3];
  data[0] = button_event;
  data[1] = mainOn ? 1 : 0;
  data[2] = toyota_checksum(STALK_CAN_ID, data, 3);
  sendMsg(STALK_CAN_ID, data, 3);
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
      Serial.println(F("C3 DETECTED (0x2E4) — relay ON, stalk to Arduino"));
    }
  }

  // Timeout: C3 gone
  if (c3Present && (nowMs - lastC3Ms) >= C3_TIMEOUT_MS) {
    c3Present = false;
    digitalWrite(RELAY_PIN, LOW);  // Route stalk back to stock ECU
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
// SETUP
// ============================================================================

void setup() {
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

  // Switch to normal mode BEFORE filter setup so init_Mask returns here, not loopback.
  if (CAN.setMode(MCP_NORMAL) != MCP2515_OK) {
    Serial.println(F("CAN setMode(NORMAL) failed — check MCP2515 wiring/SPI"));
    while (true) { delay(1000); }
  }
  if (!setup_c3_rx_filter()) {
    Serial.println(F("Halting: 0x2E4 hardware filter is required for robust C3 detection"));
    while (true) { delay(1000); }
  }
  Serial.println(F("CAN BUS init ok — RX filter 0x2E4 only, sending 0x69 at 20 Hz"));
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  unsigned long now = micros();
  unsigned long nowMs = now / 1000;

  // Check for C3 presence and manage relay
  check_can_receive();

  // Only read stalk when relay is energized (stalk wired to A0)
  if (c3Present) {
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
    send_stalk_msg(pendingEvent);
    pendingEvent = BTN_NONE;  // Clear after sending (pulse behavior)
    send_blinkers_state(turnSignals);
  }

  // --- Heartbeat ---
  if (nowMs - lastMillis_hb >= HEARTBEAT_MS) {
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
    Serial.println(txError);
  }
}
