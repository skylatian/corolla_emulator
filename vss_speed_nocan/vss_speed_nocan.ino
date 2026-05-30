// ============================================================================
// VSS Speed (no CAN) — Serial speed readout for calibration
// Reads Vehicle Speed Sensor (VSS) pulses and prints speed over USB serial.
//
// Hardware: ATmega32U4 board (e.g. CANBed V1, Leonardo) with interrupt-capable pin
//           VSS signal → VSS_PIN (default D3)
//
// Use this to tune VSS_PULSES_PER_MILE. For vss_speed (CAN), convert:
//   VSS_PULSES_PER_KM = VSS_PULSES_PER_MILE / 1.609344
// ============================================================================

// ============================================================================
// CONFIGURATION
// ============================================================================

#define VSS_PIN              3     // VSS signal input (must support hardware interrupt)
#define VSS_PULSES_PER_MILE  4986  // Calibration: pulses per mile (was ~3098/km)
#define VSS_SAMPLE_MS        500   // Sampling window in ms

#define HEARTBEAT_MS 2000

// ============================================================================
// STATE
// ============================================================================

volatile unsigned long vssPulseCount = 0;
unsigned long vssLastSampleMs = 0;
unsigned long vssLastCount = 0;
uint16_t speedMph100 = 0;  // speed in mph * 100

unsigned long lastMillis_hb = 0;

// ============================================================================
// VSS INTERRUPT
// ============================================================================

void vssISR() {
  vssPulseCount++;
}

// ============================================================================
// SETUP
// ============================================================================

void setup() {
  Serial.begin(115200);
  Serial.println(F("VSS Speed (no CAN) starting..."));

  pinMode(VSS_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(VSS_PIN), vssISR, FALLING);
  Serial.print(F("VSS pin: "));
  Serial.println(VSS_PIN);
  Serial.print(F("Pulses/mile: "));
  Serial.println(VSS_PULSES_PER_MILE);
}

// ============================================================================
// MAIN LOOP
// ============================================================================

void loop() {
  unsigned long nowMs = millis();

  if (nowMs - vssLastSampleMs >= (unsigned long)VSS_SAMPLE_MS) {
    noInterrupts();
    unsigned long count = vssPulseCount;
    interrupts();

    unsigned long pulses = count - vssLastCount;
    vssLastCount = count;
    unsigned long elapsedMs = nowMs - vssLastSampleMs;
    vssLastSampleMs = nowMs;

    // speed_mph * 100 = pulses * 360000000 / (elapsedMs * VSS_PULSES_PER_MILE)
    if (elapsedMs > 0 && pulses > 0) {
      speedMph100 = (uint16_t)((uint64_t)pulses * 360000000ULL / ((uint64_t)elapsedMs * VSS_PULSES_PER_MILE));
    } else if (pulses == 0) {
      speedMph100 = 0;
    }
  }

  if (nowMs - lastMillis_hb >= HEARTBEAT_MS) {
    lastMillis_hb = nowMs;
 //   Serial.print(F("Speed: "));
 //   Serial.print(speedMph100 / 100); // use this one to read out mph.
 //   Serial.print(F(" mph (raw "));
 //   Serial.print(speedMph100);
//    Serial.print(F(")  pulses: "));
    Serial.println(vssPulseCount);
  }
}
