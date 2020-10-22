#include "I2Cdev.h"
#include "Wire.h"
#include "MPU6050_6Axis_MotionApps20.h"

MPU6050 mpu;
const int IMUAddress = 0x68;
bool dmpReady = false;  // set true if DMP init was successful
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes charently in FIFO
uint8_t fifoBuffer[64];
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;     // [x, y, z]            gravity-free accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
byte buffer[6];
float ypr[3]; 
double angleX, angleY, angleZ, angleXBaseline = 0, angleYBaseline = 0, angleZBaseline = 0;
double accelX, accelY, accelZ, accelXError = 0, accelYError = 0, accelZError = 0;
int8_t charAccelX, charAccelY, charAccelZ, charAngleX, charAngleY, charAngleZ;
int8_t prevCharAccelX, prevCharAccelY, prevCharAccelZ, prevCharAngleX, prevCharAngleY, prevCharAngleZ;
byte state; //0 = rest 1 = change position 2 = dance

void mpuRead(double angleXBaseline, double angleYBaseline, double angleZBaseline, double accelXError, double accelYError, double accelZError) {
    mpu.resetFIFO();
    fifoCount = mpu.getFIFOCount();
    while (fifoCount < packetSize){
      fifoCount = mpu.getFIFOCount();
    }
    fifoCount -= packetSize;
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    angleX = ypr[2] * 180 / M_PI - angleXBaseline;
    angleY = ypr[1] * 180 / M_PI - angleYBaseline;
    angleZ = ypr[0] * 180 / M_PI - angleZBaseline;
    I2Cdev::readBytes(IMUAddress, 0x3B, 6, buffer);
    accelX = (buffer[0] << 8 | buffer[1]) / 16384.0 - accelXError;
    accelY = (buffer[2] << 8 | buffer[3]) / 16384.0 - accelYError;
    accelZ = (buffer[4] << 8 | buffer[5]) / 16384.0 - accelZError;  
  
}

void doubleToCharConversion() {
  charAngleX = angleX / 360 * 256;
  charAngleY = angleY / 360 * 256;
  charAngleZ = angleZ / 360 * 256;
  if (accelX > 3) {
    accelX = 3;
  }
  if (accelY > 3) {
    accelY = 3;
  }
  if (accelZ > 3) {
    accelZ = 3;
  }
  if (accelX < -3) {
    accelX = -3;
  }
  if (accelY < -3) {
    accelY = -3;
  }
  if (accelZ < -3) {
    accelZ = -3;
  }
  charAccelX = accelX * 42;
  charAccelY = accelY * 42;
  charAccelZ = accelZ * 42;
}

void calibration() {
  for (int i = 0; i < 200; i += 1) {
    mpuRead(0, 0, 0, 0, 0, 0);
    angleXBaseline += angleX;
    angleYBaseline += angleY;
    angleZBaseline += angleZ;
    accelXError += accelX;
    accelYError += accelY;
    accelZError += accelZ;
  }
  angleXBaseline /= 200;
  angleYBaseline /= 200;
  angleZBaseline /= 200;
  accelXError /= 200;
  accelYError /= 200;
  accelZError /= 200;
}

void stateChecker() {
  if (abs(charAngleX - prevCharAngleX) > 3) {
    state = 2;
    return 0;
  }
  if (abs(charAngleY - prevCharAngleY) > 3) {
    state = 2;
    return 0;
  }
  if (abs(charAngleZ - prevCharAngleZ) > 3) {
    state = 2;
    return 0;
  }
  if (abs(charAccelY - prevCharAccelY) < 60 && abs(charAccelZ - prevCharAccelZ) < 10 && abs(charAccelX - prevCharAccelX) > 1) {
    state = 1;
    return 0;
  }
  if (abs(charAccelZ - prevCharAccelZ) > 3) {
    state = 2;
    return 0;
  }
  if (abs(charAccelZ - prevCharAccelZ) > 3) {
    state = 2;
    return 0;
  }
  if (abs(charAccelX - prevCharAccelX) > 3) {
    state = 2;
    return 0;
  }
  state = 0;
  return 0;
}

void setup() {
  Serial.begin(115200);
  Serial.println("checkpointA");
  mpu.initialize();
  Serial.println("checkpointB");
  mpu.dmpInitialize();
  Serial.println("checkpointC");
  mpu.setDMPEnabled(true);
  Serial.println("checkpointD");
  packetSize = mpu.dmpGetFIFOPacketSize();
  I2Cdev::writeBits(IMUAddress, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, 0);
  Serial.println("checkpoint");
  calibration();
  state = 0;
  mpuRead(angleXBaseline, angleYBaseline, angleZBaseline, accelXError, accelYError, accelZError);
  doubleToCharConversion();
  prevCharAccelX = charAccelX;
  prevCharAccelY = charAccelY;
  prevCharAccelZ = charAccelZ;
  prevCharAngleX = charAngleX;
  prevCharAngleY = charAngleY;
  prevCharAngleZ = charAngleZ;
}

void loop() {
  mpuRead(angleXBaseline, angleYBaseline, angleZBaseline, accelXError, accelYError, accelZError);
  doubleToCharConversion();
  stateChecker();
  prevCharAccelX = charAccelX;
  prevCharAccelY = charAccelY;
  prevCharAccelZ = charAccelZ;
  prevCharAngleX = charAngleX;
  prevCharAngleY = charAngleY;
  prevCharAngleZ = charAngleZ;
  Serial.print("ypr\t");
  Serial.print(charAngleX);
  Serial.print("\t");
  Serial.print(charAngleY);
  Serial.print("\t");
  Serial.print(charAngleZ);
  Serial.print("\t");
  Serial.print(charAccelX);
  Serial.print("\t");
  Serial.print(charAccelY);
  Serial.print("\t");
  Serial.println(charAccelZ);
  Serial.println(state);
  
}
