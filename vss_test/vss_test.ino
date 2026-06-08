// VSS Pulse Counter — calibration sketch
// Wire VSS signal (V-W wire, 0-5V square wave) to pin 7
// Open serial monitor at 115200, drive at known GPS speed, note pulses/sec
// Press ENTER to snapshot the last 3 seconds of readings

#define VSS_PIN 4

volatile unsigned long pulseCount = 0;

void vssISR() {
  pulseCount++;
}

// Rolling buffer of last 3 readings (1/sec each)
#define HISTORY 3
unsigned long history[HISTORY];
byte histIdx = 0;

unsigned long lastPrintMs = 0;
unsigned long lastCount = 0;
int snapNum = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial);  // Pro Micro needs this
  Serial.println("VSS pulse counter — pin 7");
  Serial.println("Press ENTER to snapshot");
  Serial.println();

  memset(history, 0, sizeof(history));
  pinMode(VSS_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(VSS_PIN), vssISR, FALLING);
}

void loop() {
  unsigned long now = millis();

  // Update once per second
  if (now - lastPrintMs >= 1000) {
    noInterrupts();
    unsigned long count = pulseCount;
    interrupts();

    unsigned long delta = count - lastCount;
    lastCount = count;
    lastPrintMs = now;

    history[histIdx] = delta;
    histIdx = (histIdx + 1) % HISTORY;

    Serial.print("pulses/sec: ");
    Serial.println(delta);
  }

  // Any serial input = snapshot
  if (Serial.available()) {
    while (Serial.available()) Serial.read();  // drain buffer

    unsigned long sum = 0;
    for (byte i = 0; i < HISTORY; i++) sum += history[i];
    float avg = (float)sum / HISTORY;

    snapNum++;
    Serial.println("=========================");
    Serial.print(">>> SNAP #");
    Serial.print(snapNum);
    Serial.print("  avg pulses/sec: ");
    Serial.println(avg, 1);
    Serial.println("=========================");
  }
}
