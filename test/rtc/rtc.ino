#include <Wire.h>
#include <avr/io.h>
#include <avr/interrupt.h>

volatile int millisecond = 0;
volatile int second = 0;
int RTC = 0x68;
int secTemp;
int minTemp;
int rtcSec;
int rtcMin;

ISR(INT0_vect) {
  millisecond += 1;
  if (millisecond > 999) {
    millisecond = 0;
    second += 1;
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Wire.begin();
  Wire.beginTransmission(RTC);
  Wire.write(0x0E);
  Wire.write(0b00001000);
  Wire.endTransmission();
  cli();
  EICRA |= 0b00000011;
  EIMSK |= 0b00000001;
  sei();
}

void loop() {
  Wire.beginTransmission(RTC);
  Wire.write(0x00);
  Wire.endTransmission(false);
  Wire.requestFrom(RTC,2,true);
  secTemp = Wire.read();
  minTemp = Wire.read();
  rtcSec = (secTemp & 15) + 10 * ((secTemp & (15 << 4)) >> 4);
  rtcMin = (minTemp & 15) + 10 * ((minTemp & (15 << 4)) >> 4);
  Serial.print(millisecond);
  Serial.print("  ");
  Serial.print(second);
  Serial.print("  ");
  Serial.print(rtcSec);
  Serial.print("  ");
  Serial.println(rtcMin);
}
