// ============================================================================
// VSS Speed Device — Standalone CAN Speed Sender
// Reads Vehicle Speed Sensor (VSS) pulses and broadcasts speed on CAN bus.
//
// Hardware: Pro Micro (ATmega32U4) + MCP2515 breakout board (16MHz crystal)
// Wiring:  MCP2515 CS=D10, SI=D11, SO=D12, SCK=D13, INT=D2 (unused)
//          VSS signal → D7 (hardware interrupt capable on ATmega32U4)
//
// Sends:
//   0xAA WHEEL_SPEEDS  at 83 Hz — 4 identical 16-bit BE wheel speeds
//   0xB4 SPEED         at 50 Hz — 16-bit BE speed with toyota_checksum
//
// This device owns 0xAA and 0xB4 on the bus. The main emulator should have
// EXTERNAL_SPEED_DEVICE set to true so it doesn't also send these.
// ============================================================================

#include <SPI.h>
#include <mcp_can.h>

// ============================================================================
// CONFIGURATION
// ============================================================================

#define VSS_PIN            7     // VSS signal input (must support hardware interrupt)
#define VSS_PULSES_PER_KM  3098  // Calibration: pulses per kilometer (measure empirically)
#define VSS_SAMPLE_MS      500   // Sampling window in ms
#define CAN_CS_PIN         10    // MCP2515 chip select (breakout board default)

// ============================================================================
// TIMING
// ============================================================================

#define INTERVAL_83HZ  12048   // ~83 Hz (12.048 ms) — WHEEL_SPEEDS
#define INTERVAL_50HZ  20000   // 50 Hz (20 ms) — SPEED

unsigned long lastMicros_83hz = 0;
unsigned long lastMicros_50hz = 0;
unsigned long lastMillis_hb   = 0;
#define HEARTBEAT_MS 2000

// ============================================================================
// STATE
// ============================================================================

MCP_CAN CAN(CAN_CS_PIN);

volatile unsigned long vssPulseCount = 0;
unsigned long vssLastSampleMs = 0;
unsigned long vssLastCount = 0;
uint16_t speedKmh100 = 0;  // speed in km/h * 100

// TX stats
uint32_t txOk    = 0;
uint32_t txError = 0;

// ============================================================================
// VSS INTERRUPT
// ============================================================================

void vssISR() {
  vssPulseCount++;
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

// 0xAA — WHEEL_SPEEDS (8 bytes, no checksum)
// DBC: (0.01, -67.67) → raw = (km/h + 67.67) * 100 = km/h*100 + 6767
void send_wheel_speeds(uint16_t spd_kmh_x100) {
  uint16_t raw = spd_kmh_x100 + 6767;
  uint8_t w1 = (raw >> 8);
  uint8_t w2 = raw & 0xFF;
  uint8_t data[8] = {w1, w2, w1, w2, w1, w2, w1, w2};
  sendMsg(0xAA, data, 8);
}

// 0xB4 — SPEED (8 bytes, toyota_checksum)
// DBC: SPEED signal at (47|16@0+, 0.01, 0) → bytes 5-6 big endian
void send_speed(uint16_t spd_kmh_x100) {
  uint8_t data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  data[5] = (spd_kmh_x100 >> 8);
  data[6] = spd_kmh_x100 & 0xFF;
  data[7] = toyota_checksum(0xB4, data, 8);
  sendMsg(0xB4, data, 8);
}

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  Serial.begin(115200);
  while (!Serial);  // Pro Micro needs this for USB serial
  Serial.println(F("VSS Speed Device starting..."));

  // VSS input
  pinMode(VSS_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(VSS_PIN), vssISR, FALLING);
  Serial.print(F("VSS pin: "));
  Serial.println(VSS_PIN);
  Serial.print(F("Pulses/km: "));
  Serial.println(VSS_PULSES_PER_KM);

  // Init MCP2515
  while (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ) != CAN_OK) {
    Serial.println(F("CAN init failed, retrying..."));
    delay(500);
  }
  CAN.setMode(MCP_NORMAL);
  Serial.println(F("CAN BUS init ok — sending 0xAA + 0xB4"));
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  unsigned long now = micros();
  unsigned long nowMs = now / 1000;

  // --- VSS speed calculation (every VSS_SAMPLE_MS) ---
  if (nowMs - vssLastSampleMs >= (unsigned long)VSS_SAMPLE_MS) {
    noInterrupts();
    unsigned long count = vssPulseCount;
    interrupts();

    unsigned long pulses = count - vssLastCount;
    vssLastCount = count;
    unsigned long elapsedMs = nowMs - vssLastSampleMs;
    vssLastSampleMs = nowMs;

    // speed_kmh * 100 = pulses * 360000000 / (elapsedMs * VSS_PULSES_PER_KM)
    // Use 64-bit to avoid overflow
    if (elapsedMs > 0 && pulses > 0) {
      speedKmh100 = (uint16_t)((uint64_t)pulses * 360000000ULL / ((uint64_t)elapsedMs * VSS_PULSES_PER_KM));
    } else if (pulses == 0) {
      speedKmh100 = 0;
    }
  }

  // --- 83 Hz: WHEEL_SPEEDS ---
  if (now - lastMicros_83hz >= INTERVAL_83HZ) {
    lastMicros_83hz = now;
    send_wheel_speeds(speedKmh100);
  }

  // --- 50 Hz: SPEED ---
  if (now - lastMicros_50hz >= INTERVAL_50HZ) {
    lastMicros_50hz = now;
    send_speed(speedKmh100);
  }

  // --- Heartbeat ---
  if (nowMs - lastMillis_hb >= HEARTBEAT_MS) {
    lastMillis_hb = nowMs;
    Serial.print(F("Speed: "));
    Serial.print(speedKmh100 / 100);
    Serial.print(F(" km/h (raw "));
    Serial.print(speedKmh100);
    Serial.print(F(")  TX ok: "));
    Serial.print(txOk);
    Serial.print(F("  err: "));
    Serial.println(txError);
  }
}
