#include <MPU9250_RegisterMap.h>
#include <SparkFunMPU9250-DMP.h>

MPU9250_DMP imu;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  imu.begin();
  imu.setSensors(INV_XYZ_COMPASS);
}

void loop() {
  // put your main code here, to run repeatedly:
  imu.update(UPDATE_COMPASS);
  float magX = imu.calcMag(imu.mx);
  float magY = imu.calcMag(imu.my);
  float magZ = imu.calcMag(imu.mz);
  Serial.print(magX);
  Serial.print("\t");
  Serial.print(magY);
  Serial.print("\t");
  Serial.println(magZ);
}
