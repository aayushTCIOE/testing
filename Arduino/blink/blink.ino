// void setup() {
// pinMode (LED_BUILTIN, OUTPUT);
// }

// void loop() {
// digitalWrite(LED_BUILTIN, HIGH);
// delay(1000);
// digitalWrite(LED_BUILTIN, LOW);
// delay(1000);
// }
#define LED_BUILTIN 2  // Built-in LED is usually on GPIO 2

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH); // Turn LED on
  delay(1000); // Wait for a second
  digitalWrite(LED_BUILTIN, LOW); // Turn LED off
  delay(1000); // Wait for a second
}
