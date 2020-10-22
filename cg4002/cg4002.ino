#include <Wire.h>
#include <Kalman.h>

const int IMUAddress = 0x68;
const int MAGAddress = 0x0C;
double gyroX, gyroY, gyroZ, gyroZError = 0;
double accelX, accelY, accelZ, accelXError = 0, accelYError = 0, accelZError = 0;
double angleX = 0, angleY = 0, angleZ = 0, angleXBaseline, angleYBaseline;
double magX = 0, magY = 0, magZ = 0;
float prevTime, deltaTime;
double roll, pitch, yaw;
double prevAccelX, prevAccelY, prevAccelZ, prevAngleX, prevAngleY, prevAngleZ;
Kalman kalmanX;
Kalman kalmanY;

boolean startmove(double currAccelX, double currAccelY, double currAccelZ, double currAngleX, double currAngleY, double currAngleZ) {
  if (abs(currAccelX - prevAccelX) > 0.15) {
    return true;
  }
  if (abs(currAccelY - prevAccelY) > 0.15) {
    return true;
  }
  if (abs(currAccelZ - prevAccelZ) > 0.15) {
    return true;
  }
  if (abs(currAngleX - prevAngleX) > 2) {
    return true;
  }
  if (abs(currAngleY - prevAngleY) > 2) {
    return true;
  }
  if (abs(currAngleZ - prevAngleZ) > 2) {
    return true;
  }
  return false;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Wire.begin();
  Wire.beginTransmission(IMUAddress);
  Wire.write(0x6B);                  
  Wire.write(0x00);
  //Wire.write(0x19);
  //Wire.write(199);                  
  Wire.endTransmission(true);
  Wire.beginTransmission(IMUAddress);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(IMUAddress, 6, true);
  accelX = (Wire.read() << 8 | Wire.read())/16384.0;
  accelY = (Wire.read() << 8 | Wire.read())/16384.0;
  accelZ = (Wire.read() << 8 | Wire.read())/16384.0;
  roll  = atan2(accelY, accelZ) * RAD_TO_DEG;
  pitch = atan(-accelX / sqrt(accelY * accelY + accelZ * accelZ)) * RAD_TO_DEG;
  kalmanX.setAngle(roll);
  kalmanY.setAngle(pitch);
  for (int i = 0; i < 200; i +=1) {
    Wire.beginTransmission(IMUAddress);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(IMUAddress, 12, true);
    accelX = (Wire.read() << 8 | Wire.read())/16384.0;
    accelY = (Wire.read() << 8 | Wire.read())/16384.0;
    accelZ = (Wire.read() << 8 | Wire.read())/16384.0;
    accelXError += accelX;
    accelYError += accelY;
    accelZError += accelZ;
    Wire.read();
    Wire.read();
    gyroX = (Wire.read() << 8 | Wire.read())/131.0;
    gyroY = (Wire.read() << 8 | Wire.read())/131.0;
    gyroZ = (Wire.read() << 8 | Wire.read())/131.0;
    gyroZError += gyroZ;
    roll  = atan2(accelY, accelZ) * RAD_TO_DEG;
    pitch = atan(-accelX / sqrt(accelY * accelY + accelZ * accelZ)) * RAD_TO_DEG;
    angleX = kalmanX.getAngle(roll, gyroX, deltaTime);
    angleY = kalmanY.getAngle(pitch, gyroY, deltaTime);
  }
  accelXError /= 200;
  accelYError /= 200;
  accelZError /= 200;
  gyroZError /= 200;
  angleXBaseline = angleX;
  angleYBaseline = angleY;
  prevTime = millis();
}


void loop() {
  deltaTime = (millis() - prevTime)/1000;
  prevTime = millis();
  Wire.beginTransmission(IMUAddress);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(IMUAddress, 14, true);
  accelX = (Wire.read() << 8 | Wire.read())/16384.0;
  accelY = (Wire.read() << 8 | Wire.read())/16384.0;
  accelZ = (Wire.read() << 8 | Wire.read())/16384.0;
  Wire.read();
  Wire.read();
  gyroX = (Wire.read() << 8 | Wire.read())/131.0;
  gyroY = (Wire.read() << 8 | Wire.read())/131.0;
  gyroZ = (Wire.read() << 8 | Wire.read())/131.0;
  roll  = atan2(accelY, accelZ) * RAD_TO_DEG;
  pitch = atan(-accelX / sqrt(accelY * accelY + accelZ * accelZ)) * RAD_TO_DEG;
  angleX = kalmanX.getAngle(roll, gyroX, deltaTime) - angleXBaseline;
  angleY = kalmanY.getAngle(pitch, gyroY, deltaTime) - angleYBaseline;
  angleZ += (gyroZ - gyroZError) * deltaTime;
  Serial.print("angleX : ");
  Serial.print(angleX);
  Serial.print("\t");
  Serial.print("angleY : ");
  Serial.print(angleY);
  Serial.print("\t");
  Serial.print("angleZ : ");
  Serial.print(angleZ);
  Serial.print("\t");
  Serial.print("accX : ");
  Serial.print(accelX - accelXError);
  Serial.print("\t");
  Serial.print("accY : ");
  Serial.print(accelY - accelYError);
  Serial.print("\t");
  Serial.print("accZ : ");
  Serial.println(accelZ - accelZError);
  if (startmove(accelX - accelXError, accelY - accelYError, accelZ - accelZError, angleX, angleY, angleZ)) {
    Serial.println("dance started");
  }
  prevAccelX = accelX - accelXError;
  prevAccelY = accelY - accelYError;
  prevAccelZ = accelZ - accelZError;
  prevAngleX = angleX;
  prevAngleY = angleY;
  prevAngleZ = angleZ;
  /**Serial.print("angleX : ");
  Serial.print(gyroX);
  Serial.print("\t");
  Serial.print("angleY : ");
  Serial.print(gyroY);
  Serial.print("\t");
  Serial.print("angleZ : ");
  Serial.println(gyroZ);**/
}
