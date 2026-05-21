// Interrupt test for pin 3
// Prints both digitalRead (polled) and interrupt count
// Tap pin to GND to trigger (FALLING edge, INPUT_PULLUP)

#define TEST_PIN 3

volatile unsigned long count = 0;

void testISR() {
  count++;
}

void setup() {
  Serial.begin(115200);
  pinMode(TEST_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(TEST_PIN), testISR, FALLING);
  Serial.print("Interrupt test on pin ");
  Serial.println(TEST_PIN);
  Serial.print("digitalPinToInterrupt(");
  Serial.print(TEST_PIN);
  Serial.print(") = ");
  Serial.println(digitalPinToInterrupt(TEST_PIN));
}

void loop() {
  noInterrupts();
  unsigned long c = count;
  interrupts();

  Serial.print("pin=");
  Serial.print(digitalRead(TEST_PIN));
  Serial.print("  interrupts=");
  Serial.println(c);
  delay(500);
}
