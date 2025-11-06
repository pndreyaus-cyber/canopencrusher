#include <Arduino.h>

void setup() {
  pinMode(PC13, OUTPUT); // Onboard LED
}

void loop() {
  digitalWrite(PC13, LOW);  // LED ON
  delay(500);
  digitalWrite(PC13, HIGH); // LED OFF
  delay(500);
}
