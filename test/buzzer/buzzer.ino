#include "Arduino.h"

void setup() {
  // put your setup code here, to run once:
}

void loop() {
  // put your main code here, to run repeatedly:
  tone(3, 440);
  delay(1000);
  noTone(3);
  delay(1000);
}
