// ============================================================================
// VSS Speed Device — Standalone CAN Speed + Gear Sender
// Reads Vehicle Speed Sensor (VSS) pulses and gear selection switches,
// broadcasts speed and gear position on CAN bus.
//
// Hardware: Longan CANBed V1 (ATmega32U4, Leonardo bootloader, 16MHz)
//           Onboard MCP2515 CAN controller, CS=D17
//           VSS signal → D3 (hardware interrupt capable on ATmega32U4)
//           Park/Neutral switch → GEAR_PN_PIN (INPUT_PULLUP, LOW = P/N)
//           Reverse switch → GEAR_REV_PIN (INPUT_PULLUP, LOW = Reverse)
//
// Library:  mcp_can 1.5.1 (coryjfowler) — this is what `#include <mcp_can.h>`
//           resolves to for the Leonardo build (checked with arduino-cli -v).
//
// Sends:
//   0xAA WHEEL_SPEEDS  at 83 Hz — 4 identical 16-bit BE wheel speeds
//   0xB4 SPEED         at 50 Hz — 16-bit BE speed with toyota_checksum
//   0x3BC GEAR_PACKET  at 10 Hz — gear position (D/R/P)
//   0x502 RETROFIT_VSS_DIAG at 2 Hz — boot count, CAN health, VSS pulse count
//         (layout in the DIAGNOSTICS section; decoded by openpilot's DBC)
//
// This device owns 0xAA, 0xB4, and 0x3BC on the bus. The main emulator
// should have SEND_GEAR_PACKET=false and AUTO_DETECT_SPEED_DEVICE=true.
// ============================================================================

#include <SPI.h>
#include <EEPROM.h>
#include <mcp_can.h>  // coryjfowler mcp_can 1.5.1

// ============================================================================
// CONFIGURATION
// ============================================================================

#define VSS_PIN            3     // VSS signal input (must support hardware interrupt)
// Speed fit: speed_kph = 1.45*pps - 1.81 (pps >= 1), else speed_kph = pps
#define VSS_STALE_US       2000000UL  // No pulse for this long → speed = 0
#define VSS_RING_SIZE      8          // Period averaging ring buffer length
#define EMA_K              3          // Output smoothing: smooth += (raw - smooth) / K per pulse
                                      // K=2: fast (50%), K=3: moderate (33%), K=4: smooth (25%)
#define CAN_CS_PIN         17         // CANBed V1 onboard MCP2515 CS pin

// Set true to always report Drive regardless of switch state (testing override)
#define FORCE_DRIVE        true

// Gear selection switches — INPUT_PULLUP, LOW = active
// Wire through voltage dividers from 12V car signals to 5V logic.
// Assign actual pin numbers when wiring.
#define GEAR_PN_PIN        4     // Park/Neutral switch (LOW = Park or Neutral)
#define GEAR_REV_PIN       5     // Reverse switch (LOW = Reverse)

// ============================================================================
// TIMING
// ============================================================================

#define INTERVAL_83HZ  12048   // ~83 Hz (12.048 ms) — WHEEL_SPEEDS
#define INTERVAL_50HZ  20000   // 50 Hz (20 ms) — SPEED
#define INTERVAL_10HZ  100000  // 10 Hz (100 ms) — GEAR_PACKET

unsigned long lastMicros_83hz = 0;
unsigned long lastMicros_50hz = 0;
unsigned long lastMicros_10hz = 0;
unsigned long lastMillis_hb   = 0;
#define HEARTBEAT_MS 100

// ============================================================================
// DIAGNOSTICS + CAN HEALTH
//
// 0x502 RETROFIT_VSS_DIAG (8 bytes, 2 Hz + once right after boot):
//   byte 0      BOOT_COUNT       increments every boot (EEPROM), wraps at 255
//   byte 1 hi   ROLLING_COUNTER  4-bit (deliberately not named COUNTER — openpilot's
//                                parser would count a jump after a reboot as a CAN error)
//   byte 1 lo   RESET_KIND       see RESET_* (the bootloader clears MCUSR, so usually 0)
//   byte 2 7:6  RECOVERY_REASON  last CAN controller recovery (RECOVER_*)
//   byte 2 b5   FORCE_DRIVE      compile-time override is on
//   byte 2 3:0  CAN_RECOVERIES   controller re-inits since boot (saturates at 15)
//   byte 3      CAN_ERROR_FLAGS  MCP2515 EFLG, OR-latched since the last frame. RX
//                                overflow bits are masked: this device never reads RX.
//   byte 4      TX_FAILS         failed sends since the last frame (saturates)
//   byte 5      VSS_PULSES       VSS pulses since the last frame (saturates)
//   byte 6      LOOP_MAX_MS      slowest loop() since the last frame (saturates)
//   byte 7      CHECKSUM         toyota_checksum
// ============================================================================

#define DIAG_CAN_ID          0x502
#define DIAG_INTERVAL_MS     500
#define HEALTH_INTERVAL_MS   100
#define TX_STUCK_MS          500   // no successful send for this long → re-init controller
#define RECOVERY_RETRY_MS    500

#define EE_BOOT_COUNT   0

// RESET_KIND values (shared with the emulator and stalk devices)
#define RESET_UNKNOWN     0
#define RESET_POWER_ON    1
#define RESET_BROWN_OUT   2
#define RESET_EXTERNAL    3
#define RESET_WATCHDOG    4

// RECOVERY_REASON values
#define RECOVER_NONE      0
#define RECOVER_TX_STUCK  1
#define RECOVER_MODE      2
#define RECOVER_BUS_OFF   3

#define REG_CANSTAT      0x0E
#define REG_EFLG         0x2D
#define CANSTAT_OPMOD    0xE0  // bits 7:5, 000 = normal
#define EFLG_RX_OVERFLOW 0xC0
#define EFLG_TXBO        0x20

// ============================================================================
// STATE
// ============================================================================

MCP_CAN CAN(CAN_CS_PIN);

volatile unsigned long vssPeriodRing[VSS_RING_SIZE];
volatile uint8_t vssRingIdx = 0;
volatile unsigned long vssLastPulseMicros = 0;
volatile bool vssHasPulse = false;
uint16_t speedKmh100 = 0;     // smoothed speed in km/h * 100 (sent on CAN)
uint16_t rawSpeedKmh100 = 0;  // unfiltered speed computation
uint32_t smoothedX256 = 0;    // EMA fixed-point state (speed*100 << 8)
uint8_t lastRingIdx = 0;      // pulse arrival tracking for EMA
uint8_t gearState = 0;     // DBC GEAR value: 0=D, 16=R, 32=P

// TX stats
uint32_t txOk    = 0;
uint32_t txError = 0;

// Diagnostics state
volatile uint16_t vssPulseCount = 0;  // pulses since the last diag frame (ISR)
uint8_t bootCount = 0;
uint8_t resetKind = RESET_UNKNOWN;
uint8_t diagRolling = 0;
uint8_t eflgLatched = 0;
uint8_t txFailsSinceDiag = 0;
uint8_t canRecoveries = 0;
uint8_t lastRecoveryReason = RECOVER_NONE;
unsigned long loopMaxUs = 0;
unsigned long lastTxOkMs = 0;
unsigned long lastHealthMs = 0;
unsigned long lastRecoveryMs = 0;
unsigned long lastDiagMs = 0;
bool diagDue = true;

// ============================================================================
// VSS INTERRUPT
// ============================================================================

void vssISR() {
  unsigned long now = micros();
  if (vssHasPulse) {
    vssPeriodRing[vssRingIdx] = now - vssLastPulseMicros;
    vssRingIdx = (vssRingIdx + 1) % VSS_RING_SIZE;
  }
  vssLastPulseMicros = now;
  vssHasPulse = true;
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
    lastTxOkMs = millis();
  } else {
    txError++;
    if (txFailsSinceDiag < 255) txFailsSinceDiag++;
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

// 0x3BC — GEAR_PACKET (8 bytes, no checksum in DBC)
// DBC: GEAR signal at 13|6@0+ — byte 1, bits 5:0
// Values: 0=D, 16=R, 32=P (P/N combined as P)
void send_gear_packet(uint8_t gear) {
  uint8_t data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  data[1] = gear & 0x3F;
  sendMsg(0x3BC, data, 8);
}

// Read gear switches and return DBC GEAR value
uint8_t read_gear() {
  #if FORCE_DRIVE
    return 0;  // Always Drive (testing override)
  #endif
  // Priority: Reverse > Park/Neutral > Drive
  if (digitalRead(GEAR_REV_PIN) == LOW) return 16;  // R
  if (digitalRead(GEAR_PN_PIN) == LOW) return 32;    // P
  return 0;                                           // D
}

// ============================================================================
// DIAGNOSTICS + CAN CONTROLLER HEALTH
// ============================================================================

// Why we booted. The Leonardo bootloader clears MCUSR, so this is usually
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

// Bring the MCP2515 up in normal mode. Used at boot and for recovery.
bool can_init() {
  if (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ) != CAN_OK) {
    return false;
  }
  if (CAN.setMode(MCP_NORMAL) != MCP2515_OK) {
    return false;
  }
  lastTxOkMs = millis();
  return true;
}

// Re-initialize a stuck or reset controller. While this device is silent openpilot
// times out 0xAA/0xB4 and disengages; recovery just gets speed flowing again.
void can_health_check(unsigned long nowMs) {
  uint8_t eflg = mcp_read_register(REG_EFLG);
  eflgLatched |= (eflg & ~EFLG_RX_OVERFLOW);

  uint8_t reason = RECOVER_NONE;
  if ((mcp_read_register(REG_CANSTAT) & CANSTAT_OPMOD) != 0) {
    reason = RECOVER_MODE;
  } else if (nowMs - lastTxOkMs >= TX_STUCK_MS) {
    reason = (eflg & EFLG_TXBO) ? RECOVER_BUS_OFF : RECOVER_TX_STUCK;
  }
  if (reason != RECOVER_NONE && nowMs - lastRecoveryMs >= RECOVERY_RETRY_MS) {
    lastRecoveryMs = nowMs;
    lastRecoveryReason = reason;
    if (canRecoveries < 255) canRecoveries++;
    Serial.print(F("CAN recovery, reason "));
    Serial.println(reason);
    if (can_init()) diagDue = true;
  }
}

void send_diag() {
  noInterrupts();
  uint16_t pulses = vssPulseCount;
  vssPulseCount = 0;
  interrupts();

  unsigned long loopMaxMs = loopMaxUs / 1000;
  uint8_t data[8];
  data[0] = bootCount;
  data[1] = (uint8_t)((diagRolling & 0x0F) << 4) | (resetKind & 0x0F);
  data[2] = (uint8_t)((lastRecoveryReason & 0x03) << 6) | (FORCE_DRIVE ? 0x20 : 0x00) |
            min(canRecoveries, (uint8_t)15);
  data[3] = eflgLatched;
  data[4] = txFailsSinceDiag;
  data[5] = pulses > 255 ? 255 : (uint8_t)pulses;
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
  Serial.println(F("VSS Speed Device starting..."));
  Serial.print(F("Boot #"));
  Serial.println(bootCount);

  // Gear selection switches
  pinMode(GEAR_PN_PIN, INPUT_PULLUP);
  pinMode(GEAR_REV_PIN, INPUT_PULLUP);

  // VSS input
  pinMode(VSS_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(VSS_PIN), vssISR, FALLING);
  Serial.print(F("VSS pin: "));
  Serial.println(VSS_PIN);
  Serial.println(F("Speed: period measurement, ring="));
  Serial.println(VSS_RING_SIZE);

  // Init onboard MCP2515
  while (!can_init()) {
    Serial.println(F("CAN init failed, retrying..."));
    delay(500);
  }
  Serial.println(F("CAN BUS init ok — sending 0xAA + 0xB4 + 0x3BC"));
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  unsigned long now = micros();
  unsigned long nowMs = now / 1000;

  // --- VSS speed calculation (period measurement) ---
  noInterrupts();
  bool hasPulse = vssHasPulse;
  unsigned long lastPulse = vssLastPulseMicros;
  uint8_t currentIdx = vssRingIdx;
  unsigned long ring[VSS_RING_SIZE];
  for (uint8_t i = 0; i < VSS_RING_SIZE; i++) ring[i] = vssPeriodRing[i];
  interrupts();

  if (!hasPulse || (now - lastPulse) > VSS_STALE_US) {
    rawSpeedKmh100 = 0;
    smoothedX256 = 0;
  } else {
    unsigned long sum = 0;
    uint8_t count = 0;
    for (uint8_t i = 0; i < VSS_RING_SIZE; i++) {
      if (ring[i] > 0) { sum += ring[i]; count++; }
    }
    if (count > 0) {
      float avgPeriod = (float)sum / (float)count;
      float pps = 1000000.0f / avgPeriod;
      float speed_kph = (pps >= 1.0f) ? (1.45f * pps - 1.81f) : pps;
      rawSpeedKmh100 = (speed_kph > 0.0f) ? (uint16_t)(speed_kph * 100.0f + 0.5f) : 0;
    }
    if (currentIdx != lastRingIdx) {
      lastRingIdx = currentIdx;
      int32_t error = (int32_t)rawSpeedKmh100 * 256 - (int32_t)smoothedX256;
      smoothedX256 += error / EMA_K;
    }
  }
  speedKmh100 = (uint16_t)(smoothedX256 / 256);

  // --- Gear selection (every loop — digitalRead is ~4µs) ---
  gearState = read_gear();

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

  // --- 10 Hz: GEAR_PACKET ---
  if (now - lastMicros_10hz >= INTERVAL_10HZ) {
    lastMicros_10hz = now;
    send_gear_packet(gearState);
  }

  // --- CAN controller health + 2 Hz diagnostics frame ---
  if (nowMs - lastHealthMs >= HEALTH_INTERVAL_MS) {
    lastHealthMs = nowMs;
    can_health_check(nowMs);
  }
  if (diagDue || nowMs - lastDiagMs >= DIAG_INTERVAL_MS) {
    diagDue = false;
    lastDiagMs = nowMs;
    send_diag();
  }

  // --- Heartbeat ---
  if (nowMs - lastMillis_hb >= HEARTBEAT_MS) {
    lastMillis_hb = nowMs;
    Serial.print(F("Speed: "));
    Serial.print(speedKmh100 / 100);
    Serial.print(F(" km/h (raw "));
    Serial.print(rawSpeedKmh100 / 100);
    Serial.print(F(")  Gear: "));
    switch (gearState) {
      case 0:  Serial.print(F("D")); break;
      case 16: Serial.print(F("R")); break;
      case 32: Serial.print(F("P")); break;
      default: Serial.print(F("?")); break;
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
