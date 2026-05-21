// ============================================================================
// Cruise Stalk CAN Device — Standalone Stalk Reader
// Reads Toyota cruise stalk resistance on analog pin, decodes button press,
// and sends CAN message 0x69 at 20 Hz.
//
// Hardware: Longan CANBed V1 (ATmega32U4, Leonardo bootloader, 16MHz)
//           Onboard MCP2515 CAN controller, CS=D17
//           Stalk signal → A0 (resistance-based, all buttons on one wire)
//
// Library:  Longan Labs Arduino_CAN_BUS_MCP2515
//           https://github.com/Longan-Labs/Aruino_CAN_BUS_MCP2515
//
// Sends:
//   0x69 CRUISE_STALK at 20 Hz — 2 bytes: button event + MAIN state
//
// The stalk is a 2-wire resistance-based device. Each position (SET/-,
// RES/+, CANCEL, MAIN) produces a different resistance value on A0.
// All positions are momentary (spring-return). MAIN is tracked as a
// toggle internally: each MAIN pulse flips the mainOn bool.
// ============================================================================

#include <SPI.h>
#include <mcp_can.h>  // Longan Labs MCP2515 library

// ============================================================================
// CONFIGURATION
// ============================================================================

#define CAN_CS_PIN  17  // CANBed V1 onboard MCP2515 CS pin
#define STALK_PIN   A0  // Analog input for stalk resistance

// CAN message ID for stalk data (not used by any Toyota DBC message)
#define STALK_CAN_ID  0x69

// ============================================================================
// RESISTANCE THRESHOLDS (ADC values, 0-1023)
// Placeholder values — measure actual stalk resistances and tune these.
// Each button produces a different resistance on the same 2-wire circuit.
// Wire a voltage divider: 5V → 10k pullup → A0 → stalk → GND.
// Then read ADC values for each button position.
// ============================================================================

#define STALK_SET_MIN      100
#define STALK_SET_MAX      200

#define STALK_RES_MIN      250
#define STALK_RES_MAX      350

#define STALK_CANCEL_MIN   400
#define STALK_CANCEL_MAX   500

#define STALK_MAIN_MIN     550
#define STALK_MAIN_MAX     650

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
#define DEBOUNCE_MS      200    // Lockout per press
#define HEARTBEAT_MS     2000

unsigned long lastMicros_20hz = 0;
unsigned long lastMillis_hb   = 0;

// ============================================================================
// STATE
// ============================================================================

MCP_CAN CAN(CAN_CS_PIN);

bool mainOn = false;              // MAIN toggle state (flips on each MAIN press)
uint8_t pendingEvent = BTN_NONE;  // Button event to send (cleared after one frame)
unsigned long lastPressMs = 0;    // Debounce timestamp

// TX stats
uint32_t txOk    = 0;
uint32_t txError = 0;

// ============================================================================
// STALK READING
// ============================================================================

// Decode ADC value into button press. Returns BTN_NONE if no button or
// if within debounce lockout.
uint8_t read_stalk() {
  int adc = analogRead(STALK_PIN);
  uint8_t btn = BTN_NONE;

  if (adc >= STALK_SET_MIN && adc <= STALK_SET_MAX) {
    btn = BTN_SET;
  } else if (adc >= STALK_RES_MIN && adc <= STALK_RES_MAX) {
    btn = BTN_RES;
  } else if (adc >= STALK_CANCEL_MIN && adc <= STALK_CANCEL_MAX) {
    btn = BTN_CANCEL;
  } else if (adc >= STALK_MAIN_MIN && adc <= STALK_MAIN_MAX) {
    btn = BTN_MAIN_TOGGLE;
  }

  // Debounce: ignore presses within lockout window
  if (btn != BTN_NONE) {
    unsigned long now = millis();
    if (now - lastPressMs < DEBOUNCE_MS) {
      return BTN_NONE;  // Still in lockout
    }
    lastPressMs = now;

    // MAIN is a toggle — flip state on each press
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
  }

  return btn;
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

// Send 0x69: byte 0 = button event, byte 1 = MAIN state
void send_stalk_msg(uint8_t button_event) {
  uint8_t data[2];
  data[0] = button_event;
  data[1] = mainOn ? 1 : 0;
  sendMsg(STALK_CAN_ID, data, 2);
}

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  Serial.begin(115200);
  Serial.println(F("Cruise Stalk Device starting..."));

  Serial.print(F("Stalk pin: A"));
  Serial.println(STALK_PIN - A0);
  Serial.print(F("CAN ID: 0x"));
  Serial.println(STALK_CAN_ID, HEX);

  // Init onboard MCP2515
  while (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ) != CAN_OK) {
    Serial.println(F("CAN init failed, retrying..."));
    delay(500);
  }
  CAN.setMode(MCP_NORMAL);
  Serial.println(F("CAN BUS init ok — sending 0x69 at 20 Hz"));
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  unsigned long now = micros();
  unsigned long nowMs = now / 1000;

  // Read stalk input (non-blocking, with debounce)
  uint8_t btn = read_stalk();
  if (btn != BTN_NONE) {
    pendingEvent = btn;  // Latch event for next CAN frame
  }

  // --- 20 Hz: send stalk CAN message ---
  if (now - lastMicros_20hz >= INTERVAL_20HZ) {
    lastMicros_20hz = now;
    send_stalk_msg(pendingEvent);
    pendingEvent = BTN_NONE;  // Clear after sending (pulse behavior)
  }

  // --- Heartbeat ---
  if (nowMs - lastMillis_hb >= HEARTBEAT_MS) {
    lastMillis_hb = nowMs;
    Serial.print(F("MAIN: "));
    Serial.print(mainOn ? "ON" : "OFF");
    Serial.print(F("  ADC: "));
    Serial.print(analogRead(STALK_PIN));
    Serial.print(F("  TX ok: "));
    Serial.print(txOk);
    Serial.print(F("  err: "));
    Serial.println(txError);
  }
}
