// ============================================================================
// Corolla CAN Emulator for FrogPilot Retrofit
// Spoofs ECU messages so openpilot (FrogPilot 0.9.7) recognizes a Toyota Corolla
// and allows steering control via the 2016 Corolla EPS.
//
// Hardware: Arduino + Seeed Studio CAN Bus Shield v2 (MCP2515, 16MHz crystal, CS=D9)
// Bus: 500kbps CAN, shared with EPS + SAS + Comma 3
// ============================================================================

#include <SPI.h>
#include <mcp_can.h>  // Seeed Studio CAN_BUS_Shield library

MCP_CAN CAN(9);  // Seeed CAN Bus Shield v2: CS on D9 (not D10)

// ============================================================================
// FLAG-BASED CONFIGURATION
// Toggle these to control which messages the emulator sends.
// ============================================================================

// Set to true when real hardware is connected and provides these messages.
// Set to false to have the emulator spoof them (bench testing without EPS/SAS).
#define REAL_EPS_CONNECTED    true  // EPS provides 0x260 (STEER_TORQUE_SENSOR), 0x262 (EPS_STATUS)
#define REAL_SAS_CONNECTED    true  // SAS provides 0x25 (STEER_ANGLE_SENSOR)
#define REAL_PEDAL_CONNECTED  true  // Comma Pedal provides 0x201 (GAS_SENSOR) at 50 Hz

// Messages the emulator can send. Defaults are what FrogPilot needs.
#define SEND_ACC_CONTROL      false  // 0x343 — WARNING: triggers relay_malfunction if true!
#define SEND_BRAKE_MODULE     true   // 0x224 — required by panda safety RX checks
#define SEND_PCM_CRUISE_2     true   // 0x1D3 — MAIN_ON flag (cruise availability)
#define SEND_GEAR_PACKET      true   // 0x3BC — gear position
#define SEND_BODY_CONTROL     true   // 0x620 — doors/seatbelt state
#define SEND_WHEEL_SPEEDS     true   // 0xAA  — required by panda safety RX checks (83 Hz)
#define SEND_SPEED            true   // 0xB4  — vehicle speed (also EPS init message)
#define SEND_PCM_CRUISE       true   // 0x1D2 — cruise state (required by panda safety)
#define SEND_ENGINE_RPM       true   // 0x1C4 — EPS init message (42 Hz required by carstate)
#define SEND_EPS_INIT_4CB     true   // 0x4CB — EPS init message
#define SEND_PRE_COLLISION    true   // 0x283 — required by carstate for non-TSS2 without DSU
#define SEND_VSC1S07          true   // 0x320 — vehicle stability control (20 Hz required)

// External VSS speed device auto-detection.
// When true, the emulator listens for 0xAA on the CAN bus. If it receives one
// within the timeout, it uses that speed and stops sending its own 0xAA/0xB4.
// If the device disappears (no 0xAA for SPEED_DEVICE_TIMEOUT_MS), the emulator
// falls back to sending 0xAA/0xB4 with speed=0.
// When false, the emulator always sends its own 0xAA/0xB4 from SPEED_SOURCE.
#define AUTO_DETECT_SPEED_DEVICE  true // you should only use this for debugging on the bench! should be set to false for use in car
#define SPEED_DEVICE_TIMEOUT_MS   1000  // 1 second without 0xAA = device gone

// Send the bulk "background" messages from the original emulator (0x367, 0x394, etc.)
// These fill out the CAN bus to look more like a real car. Required for OP to fingerprint the car, at least in its current state
#define SEND_BACKGROUND_MSGS  true

// ============================================================================
// PIN ASSIGNMENTS
// ============================================================================

#define CRUISE_BUTTON_PIN  3   // Toggle cruise engaged (0x1D2 CRUISE_ACTIVE)
#define MAIN_BUTTON_PIN    4   // Toggle cruise main on (0x1D3 MAIN_ON) — not used if SEND_PCM_CRUISE_2 is hardcoded on
#define GEAR_BUTTON_PIN    5   // Toggle gear: Drive <-> Park (0x3BC)
#define SPEED_POT_PIN      A0  // Potentiometer for wheel/vehicle speed (SPEED_SOURCE 1)

// Speed source: 0 = serial (type mph + enter), 1 = analog pot, 2 = VSS pulse counter
#define SPEED_SOURCE       0

// VSS (Vehicle Speed Sensor) configuration — only used when SPEED_SOURCE == 2
// Wire VSS signal to this pin (must support hardware interrupts).
// Pin 7 works on Pro Micro (ATmega32U4). On Uno, use pin 2 or 3.
#define VSS_PIN            7
// Calibration: pulses per kilometer. Measured empirically — do NOT calculate from gear ratios.
// Baseline: ~13.85 pulses/sec at 10 mph (16.09 km/h) → ~3098 pulses/km
// Refine by measuring at multiple speeds over longer distances.
#define VSS_PULSES_PER_KM  3098
// Sampling window in ms. Longer = smoother but more latency.
// 500ms gives ~3.5 pulses at 5 mph (acceptable) and ~7 at 10 mph (good).
#define VSS_SAMPLE_MS      500

// ============================================================================
// TIMING (micros-based, non-blocking)
// ============================================================================

// Interval in microseconds for each frequency tier
#define INTERVAL_83HZ   12048   // ~83 Hz (12.048 ms) — WHEEL_SPEEDS, STEER_ANGLE_SENSOR
#define INTERVAL_50HZ   20000   // 50 Hz (20 ms) — STEER_TORQUE_SENSOR, SPEED, ENGINE_RPM, VSC1S07
#define INTERVAL_40HZ   25000   // 40 Hz (25 ms) — BRAKE_MODULE
#define INTERVAL_33HZ   30303   // ~33 Hz (30.3 ms) — PCM_CRUISE, PCM_CRUISE_2, PRE_COLLISION
#define INTERVAL_25HZ   40000   // 25 Hz (40 ms) — EPS_STATUS
#define INTERVAL_10HZ   100000  // 10 Hz — background msgs
#define INTERVAL_3HZ    333333  // ~3 Hz — BODY_CONTROL_STATE
#define INTERVAL_1HZ    1000000 // 1 Hz — GEAR_PACKET, 0x4CB

unsigned long lastMicros_83hz = 0;
unsigned long lastMicros_50hz = 0;
unsigned long lastMicros_40hz = 0;
unsigned long lastMicros_33hz = 0;
unsigned long lastMicros_25hz = 0;
unsigned long lastMicros_10hz = 0;
unsigned long lastMicros_3hz  = 0;
unsigned long lastMicros_1hz  = 0;
unsigned long lastMillis_hb   = 0;  // heartbeat
#define HEARTBEAT_MS 2000

// ============================================================================
// STATE
// ============================================================================

bool cruiseEngaged = false;    // CRUISE_ACTIVE in 0x1D2
bool prevCruiseEngaged = false; // Previous state for edge detection
bool cruiseMain = true;        // MAIN_ON in 0x1D3 (default on)
bool gearInDrive = true;       // true = Drive, false = Park
uint8_t cruiseSetSpeed = 0;    // Captured speed at SET moment (km/h)
uint8_t msg_counter_1d2 = 0;  // Counter for 0x1D2 (used in checksum rotation)
uint8_t msg_counter_201 = 0;  // Counter for 0x201 (GAS_SENSOR, 4-bit rolling)

// Button debounce
unsigned long lastDebounce_cruise = 0;
unsigned long lastDebounce_main   = 0;
unsigned long lastDebounce_gear   = 0;
#define DEBOUNCE_MS 300

// TX error tracking
uint32_t txOk    = 0;
uint32_t txError = 0;

// Serial-controlled speed (SPEED_SOURCE 0)
// Type a number in mph into the serial monitor and hit enter
uint16_t serialSpeedKmh100 = 0;
char serialBuf[16];
uint8_t serialBufIdx = 0;

// VSS pulse counting (SPEED_SOURCE 2)
#if SPEED_SOURCE == 2
volatile unsigned long vssPulseCount = 0;
unsigned long vssLastSampleMs = 0;
unsigned long vssLastCount = 0;
uint16_t vssSpeedKmh100 = 0;  // speed in km/h * 100

void vssISR() {
  vssPulseCount++;
}
#endif

// CAN-received speed from external VSS device (auto-detect)
#if AUTO_DETECT_SPEED_DEVICE
uint16_t canSpeedKmh100 = 0;       // speed in km/h * 100, read from 0xAA on bus
unsigned long lastCanSpeedMs = 0;   // millis() of last 0xAA received
bool speedDevicePresent = false;    // true when 0xAA seen within timeout

// Read incoming CAN messages. Non-blocking — only reads if a message is in the
// MCP2515 RX buffer. Extracts speed from 0xAA (WHEEL_SPEEDS) and drains other
// messages to prevent the RX buffer from filling up.
// Also tracks whether the external device is alive (timeout-based).
void check_can_speed() {
  unsigned long nowMs = millis();

  if (CAN.checkReceive() == CAN_MSGAVAIL) {
    unsigned long rxId;
    uint8_t rxLen;
    uint8_t rxBuf[8];
    CAN.readMsgBuf(&rxId, &rxLen, rxBuf);

    if (rxId == 0xAA && rxLen >= 2) {
      uint16_t raw = ((uint16_t)rxBuf[0] << 8) | rxBuf[1];
      canSpeedKmh100 = (raw >= 6767) ? (raw - 6767) : 0;
      lastCanSpeedMs = nowMs;

      if (!speedDevicePresent) {
        speedDevicePresent = true;
        Serial.println(F("VSS device DETECTED — using CAN speed"));
      }
    }
  }

  // Timeout: if no 0xAA received for SPEED_DEVICE_TIMEOUT_MS, device is gone
  if (speedDevicePresent && (nowMs - lastCanSpeedMs) >= SPEED_DEVICE_TIMEOUT_MS) {
    speedDevicePresent = false;
    canSpeedKmh100 = 0;
    Serial.println(F("VSS device LOST — falling back to local speed"));
  }
}
#endif

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
  } else {
    txError++;
    // Uncomment for verbose TX error logging (noisy at high rates):
    // Serial.print("TX ERR 0x"); Serial.print(id, HEX);
    // Serial.print(" code="); Serial.println(result);
  }
}

void print_heartbeat() {
  unsigned long now = millis();
  if (now - lastMillis_hb < HEARTBEAT_MS) return;
  lastMillis_hb = now;

  #if SPEED_SOURCE == 1
    uint16_t spd = analogRead(SPEED_POT_PIN) * 18;
  #elif SPEED_SOURCE == 2
    uint16_t spd = vssSpeedKmh100;
  #else
    uint16_t spd = serialSpeedKmh100;
  #endif

  Serial.println("--- Status ---");
  Serial.print("  Cruise: "); Serial.println(cruiseEngaged ? "ENGAGED" : "off");
  Serial.print("  Main:   "); Serial.println(cruiseMain    ? "ON"       : "off");
  Serial.print("  Gear:   "); Serial.println(gearInDrive   ? "Drive"    : "Park");
  Serial.print("  Speed:  "); Serial.println(spd);
  Serial.print("  SetSpd: "); Serial.print(cruiseSetSpeed); Serial.println(" km/h");
  #if AUTO_DETECT_SPEED_DEVICE
  Serial.print("  VSSdev: "); Serial.println(speedDevicePresent ? "PRESENT" : "absent");
  Serial.print("  CAN spd:"); Serial.println(canSpeedKmh100);
  #endif
  Serial.print("  TX ok:  "); Serial.println(txOk);
  Serial.print("  TX err: "); Serial.println(txError);
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
  // All zeros = brake not pressed
  sendMsg(0x224, zeros, 8);
}

// 0x3BC — GEAR_PACKET (8 bytes, no checksum in DBC)
// Byte 1: gear position. 0x00 = Drive, 0x20 = not-in-drive (Park/Neutral)
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

// Original arrays from wocsor's emulator (various data lengths)
const uint16_t bg_1byte[]  = {0x423};
const uint8_t  bg_1byte_count = 1;

const uint16_t bg_2byte[]  = {0x367, 0x394, 0x3D3};
const uint8_t  bg_2byte_count = 3;

const uint16_t bg_4byte[]  = {0x351, 0x3BB, 0xBA};
const uint8_t  bg_4byte_count = 3;

// 0x2E4 removed — that's STEERING_LKA (steering command), only openpilot should send it.
// Sending it from the emulator triggers relay_malfunction detection.
const uint16_t bg_5byte[]  = {0x3E6};
const uint8_t  bg_5byte_count = 1;

const uint16_t bg_6byte[]  = {0x1AA, 0x384, 0x386};
const uint8_t  bg_6byte_count = 3;

// 0x283 removed — now has dedicated sender with checksum at 33 Hz
const uint16_t bg_7byte[]  = {0x365, 0x366, 0x3E7};
const uint8_t  bg_7byte_count = 3;

// 8-byte background messages — filtered to remove IDs with dedicated senders
// Removed: 0x25 (SAS), 0x260 (EPS), 0x320 (VSC1S07), 0x343 (stock ECU detection)
// These are gated by their respective flags/senders
const uint16_t bg_8byte[] = {
  0x24, 0x1D0, 0x2C1, 0x344,
  0x380, 0x381, 0x389, 0x38F, 0x399, 0x3A5, 0x3B0, 0x3B1, 0x3B7,
  0x3E8, 0x3E9, 0x3F9, 0x411, 0x412, 0x413, 0x414, 0x420,
  0x45A, 0x489, 0x48A, 0x48B, 0x4AC, 0x4D3, 0x4FF,
  0x610, 0x611, 0x614, 0x615, 0x619, 0x61A,
  0x621, 0x622, 0x623, 0x624,
  0x638, 0x63C, 0x63D, 0x640, 0x680, 0x6F3,
  0x770, 0x778, 0x7C6, 0x7CE,
  0x7E0, 0x7E1, 0x7E2, 0x7E3, 0x7E4, 0x7E5, 0x7E6, 0x7E7, 0x7E8
};
const uint8_t bg_8byte_count = sizeof(bg_8byte) / sizeof(bg_8byte[0]);

void send_background_msgs() {
  uint8_t d1[1] = {0};
  uint8_t d2[2] = {0, 0};
  uint8_t d4[4] = {0, 0, 0, 0};
  uint8_t d5[5] = {0, 0, 0, 0, 0};
  uint8_t d6[6] = {0, 0, 0, 0, 0, 0};
  uint8_t d7[7] = {0, 0, 0, 0, 0, 0, 0};

  for (uint8_t i = 0; i < bg_1byte_count; i++) sendMsg(bg_1byte[i], d1, 1);
  for (uint8_t i = 0; i < bg_2byte_count; i++) sendMsg(bg_2byte[i], d2, 2);
  for (uint8_t i = 0; i < bg_4byte_count; i++) sendMsg(bg_4byte[i], d4, 4);
  for (uint8_t i = 0; i < bg_5byte_count; i++) sendMsg(bg_5byte[i], d5, 5);
  for (uint8_t i = 0; i < bg_6byte_count; i++) sendMsg(bg_6byte[i], d6, 6);
  for (uint8_t i = 0; i < bg_7byte_count; i++) sendMsg(bg_7byte[i], d7, 7);
  for (uint8_t i = 0; i < bg_8byte_count; i++) sendMsg(bg_8byte[i], zeros, 8);

  // Conditionally send messages that real hardware might provide
  #if !REAL_SAS_CONNECTED
    sendMsg(0x25, zeros, 8);
  #endif
  // 0x260 NOT sent here — dedicated send_steer_torque_sensor() at 50 Hz handles it with correct checksum
  #if SEND_ACC_CONTROL
    sendMsg(0x343, zeros, 8);
  #endif
}

// ============================================================================
// SERIAL SPEED INPUT
// Type a speed in mph and press enter. E.g., "25" → 25 mph → ~40 km/h
// ============================================================================

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

// ============================================================================
// BUTTON HANDLING
// ============================================================================

void check_buttons() {
  unsigned long now = millis();

  // Cruise engage/disengage toggle
  if (digitalRead(CRUISE_BUTTON_PIN) == HIGH && (now - lastDebounce_cruise) > DEBOUNCE_MS) {
    lastDebounce_cruise = now;
    prevCruiseEngaged = cruiseEngaged;
    cruiseEngaged = !cruiseEngaged;

    if (cruiseEngaged && !prevCruiseEngaged) {
      // SET moment: capture current speed
      #if AUTO_DETECT_SPEED_DEVICE
        if (speedDevicePresent) {
          cruiseSetSpeed = (uint8_t)(canSpeedKmh100 / 100);
        } else
      #endif
      {
        #if SPEED_SOURCE == 2
          cruiseSetSpeed = (uint8_t)(vssSpeedKmh100 / 100);
        #elif SPEED_SOURCE == 1
          cruiseSetSpeed = (uint8_t)((uint32_t)analogRead(SPEED_POT_PIN) * 120 / 1023);
        #else
          cruiseSetSpeed = (uint8_t)(serialSpeedKmh100 / 100);
        #endif
      }
      Serial.print("Cruise: ENGAGED at ");
      Serial.print(cruiseSetSpeed);
      Serial.println(" km/h");
    } else {
      cruiseSetSpeed = 0;
      Serial.println("Cruise: disengaged");
    }
  }

  // Main on/off toggle
  if (digitalRead(MAIN_BUTTON_PIN) == HIGH && (now - lastDebounce_main) > DEBOUNCE_MS) {
    lastDebounce_main = now;
    cruiseMain = !cruiseMain;
    Serial.print("Main: ");
    Serial.println(cruiseMain ? "ON" : "off");
  }

  // Gear toggle
  if (digitalRead(GEAR_BUTTON_PIN) == HIGH && (now - lastDebounce_gear) > DEBOUNCE_MS) {
    lastDebounce_gear = now;
    gearInDrive = !gearInDrive;
    Serial.print("Gear: ");
    Serial.println(gearInDrive ? "DRIVE" : "Park");
  }
}

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Corolla CAN Emulator starting...");

  // Button pins
  pinMode(CRUISE_BUTTON_PIN, INPUT);
  pinMode(MAIN_BUTTON_PIN, INPUT);
  pinMode(GEAR_BUTTON_PIN, INPUT);

  // VSS interrupt setup
  #if SPEED_SOURCE == 2
  pinMode(VSS_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(VSS_PIN), vssISR, FALLING);
  Serial.print(F("Speed source: VSS on pin "));
  Serial.println(VSS_PIN);
  #elif SPEED_SOURCE == 1
  Serial.println(F("Speed source: potentiometer"));
  #else
  Serial.println(F("Speed source: serial (type mph + enter)"));
  #endif

  // Init MCP2515
  while (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ) != CAN_OK) {
    Serial.println("CAN init failed, retrying...");
    delay(500);
  }
  CAN.setMode(MCP_NORMAL);
  Serial.println("CAN BUS init ok!");

  Serial.println("--- Config ---");
  Serial.print("REAL_EPS_CONNECTED:   "); Serial.println(REAL_EPS_CONNECTED ? "true" : "false");
  Serial.print("REAL_SAS_CONNECTED:   "); Serial.println(REAL_SAS_CONNECTED ? "true" : "false");
  Serial.print("REAL_PEDAL_CONNECTED: "); Serial.println(REAL_PEDAL_CONNECTED ? "true" : "false");
  Serial.print("AUTO_DETECT_SPEED:    "); Serial.println(AUTO_DETECT_SPEED_DEVICE ? "true (listen for 0xAA)" : "false");
  Serial.print("SEND_ACC_CONTROL:     "); Serial.println(SEND_ACC_CONTROL ? "true (WARNING!)" : "false");
  Serial.println("--------------");
}

// ============================================================================
// MAIN LOOP (non-blocking, micros-based timing)
// ============================================================================

void loop() {
  unsigned long now = micros();

  // Speed in km/h * 100.
  #if SPEED_SOURCE == 2
    // VSS: update speed from pulse count every VSS_SAMPLE_MS
    // now is in micros; vssLastSampleMs is in millis
    unsigned long nowMs = now / 1000;
    if (nowMs - vssLastSampleMs >= (unsigned long)VSS_SAMPLE_MS) {
      noInterrupts();
      unsigned long count = vssPulseCount;
      interrupts();
      unsigned long pulses = count - vssLastCount;
      vssLastCount = count;
      unsigned long elapsedMs = nowMs - vssLastSampleMs;
      vssLastSampleMs = nowMs;
      // speed_kmh * 100 = (pulses / elapsedMs * 1000) / VSS_PULSES_PER_KM * 3600 * 100
      //                  = pulses * 360000000 / (elapsedMs * VSS_PULSES_PER_KM)
      // Use 64-bit to avoid overflow (pulses * 360000000 can exceed uint32 at ~12 pulses)
      if (elapsedMs > 0 && pulses > 0) {
        vssSpeedKmh100 = (uint16_t)((uint64_t)pulses * 360000000ULL / ((uint64_t)elapsedMs * VSS_PULSES_PER_KM));
      } else if (pulses == 0) {
        vssSpeedKmh100 = 0;
      }
    }
    uint16_t spd = vssSpeedKmh100;
  #elif SPEED_SOURCE == 1
    uint16_t spd = (uint32_t)analogRead(SPEED_POT_PIN) * 12000 / 1023;
  #else
    uint16_t spd = serialSpeedKmh100;
  #endif

  // --- 83 Hz tier: WHEEL_SPEEDS, STEER_ANGLE_SENSOR ---
  // STEER_ANGLE_SENSOR needs 80 Hz per carstate parser
  if (now - lastMicros_83hz >= INTERVAL_83HZ) {
    lastMicros_83hz = now;

    #if SEND_WHEEL_SPEEDS
      #if AUTO_DETECT_SPEED_DEVICE
        if (!speedDevicePresent) send_wheel_speeds(spd);
      #else
        send_wheel_speeds(spd);
      #endif
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

    #if SEND_SPEED
      #if AUTO_DETECT_SPEED_DEVICE
        if (!speedDevicePresent) send_speed(spd);
      #else
        send_speed(spd);
      #endif
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

  // --- 10 Hz tier: background messages ---
  if (now - lastMicros_10hz >= INTERVAL_10HZ) {
    lastMicros_10hz = now;

    #if SEND_BACKGROUND_MSGS
      send_background_msgs();
    #endif
  }

  // --- 3 Hz tier: BODY_CONTROL_STATE ---
  if (now - lastMicros_3hz >= INTERVAL_3HZ) {
    lastMicros_3hz = now;

    #if SEND_BODY_CONTROL
      send_body_control();
    #endif
  }

  // --- 1 Hz tier: GEAR_PACKET ---
  if (now - lastMicros_1hz >= INTERVAL_1HZ) {
    lastMicros_1hz = now;

    #if SEND_GEAR_PACKET
      send_gear_packet(gearInDrive);
    #endif
  }

  // --- CAN receive: auto-detect external VSS device ---
  #if AUTO_DETECT_SPEED_DEVICE
    check_can_speed();
  #endif

  // --- Serial speed input + button handling ---
  check_serial();
  check_buttons();

  // --- Heartbeat: print status every 2 seconds ---
  print_heartbeat();
}
