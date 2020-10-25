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
float cx, sx, cy, sy, cz, sz;
float a, b, c, d, e, f, g, h, i;
float tX, tY, tZ;
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
    angleX = ypr[2];
    angleY = ypr[1];
    angleZ = abs(ypr[0]) - M_PI/2;
    I2Cdev::readBytes(IMUAddress, 0x3B, 6, buffer);
    accelX = (buffer[0] << 8 | buffer[1]) / 16384.0;
    accelY = (buffer[2] << 8 | buffer[3]) / 16384.0;
    accelZ = (buffer[4] << 8 | buffer[5]) / 16384.0;
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
  for (int i = 0; i < 500; i += 1) {
    mpuRead(0, 0, 0, 0, 0, 0);
    angleXBaseline += angleX;
    angleYBaseline += angleY;
    angleZBaseline += angleZ;
    accelXError += accelX;
    accelYError += accelY;
    accelZError += accelZ;
  }
  angleXBaseline /= 500;
  angleYBaseline /= 500;
  angleZBaseline /= 500;
  accelXError /= 500;
  accelYError /= 500;
  accelZError /= 500;
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
  //Serial.println("checkpointA");
  mpu.initialize();
  //Serial.println("checkpointB");
  mpu.dmpInitialize();
  //Serial.println("checkpointC");
  mpu.setDMPEnabled(true);
  //Serial.println("checkpointD");
  packetSize = mpu.dmpGetFIFOPacketSize();
  I2Cdev::writeBits(IMUAddress, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, 0);
  //Serial.println("checkpoint");
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

void rotX() {
  a = 1;
  b = 0;
  c = 0;
  d = 0;
  e = cx;
  f = -1 * sx;
  g = 0;
  h = sx;
  i = cx;
}

void rotY() {
  a = cy;
  b = 0;
  c = sy;
  d = 0;
  e = 1;
  f = 0;
  g = -1 * sy;
  h = 0;
  i = cy;
}

void rotZ() {
  a = cz;
  b = -1 * sz;
  c = 0;
  d = sz;
  e = cz;
  f = 0;
  g = 0;
  h = 0;
  i = 1;
}

void rotZYX() {
  a = cy*cz;
  b = -1*sz*cx + cz*sy*sx;
  c = sz*sx + cz*sy*cx;
  d = sz*cy;
  e = cz*cx + sz*sy*sx;
  f = -1*cz*sx + sz*sy*cx;
  g = -1*sy;
  h = cy*sx;
  i = cy*cx;
}

void rotXZY() {
  a = cz*cy;
  b = -1*sz;
  c = cz*sy;
  d = cx*sz*cy - sx*sy;
  e = cx*cz;
  f = cx*sz*sy-sx*cy;
  g = sx*sz*cy-cx*sy;
  h = sx*cz;
  i = sx*sz*sy+cx*cy;
}

void rotXYZ() {
  a = cz*cy;
  b = -1*cy*sz;
  c = sy;
  d = sx*sy*cz+cx*sz;
  e = cx*cz-sx*sy*sz;
  f = -1*sx*cy;
  g = sx*sz-cx*sy*cz;
  h = sx*cz+cx*sy*sz;
  i = cx*cy;
}

void rotZXY() {
  a = cz*cy-sz*sx*sy;
  b = -1*sz*cx;
  c = cz*sy+sz*sx*cy;
  d = sz*cy+cz*sx*sy;
  e = cz*cx;
  f = sz*sy-cz*sx*cy;
  g = -1*cx*sy;
  h = sx;
  i = cx*cy;
}


void rotYXZ() {
  a = cy*cz+sy*sx*sz;
  b = sy*sx*cz-cy*sz;
  c = sy*cx;
  d = cx*sz;
  e = cx*cz;
  f = -1*sx;
  g = cy*sx*sz-sy*cz;
  h = cy*sx*cz+sy*sz;
  i = cy*cx;
}


void rotYZX() {
  a = cy*cz;
  b = sy*sx-cy*cz*cx;
  c = cy*cz*sx+sy*cx;
  d = sz;
  e = cz*cx;
  f = -1*sx*cz;
  g = -1*sy*cz;
  h = sy*sz*cx+cy*sx;
  i = cy*cx-sy*sz*sx;
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
  cx = cos(angleX);
  sx = sin(angleX);
  cy = cos(angleY);
  sy = sin(angleY);
  cz = cos(angleZ);
  sz = sin(angleZ);
  rotXYZ();
  tX = a * accelX + b * accelY + c * accelZ;
  tY = d * accelX + e * accelY + f * accelZ;
  tZ = g * accelX + h * accelY + i * accelZ;
//  Serial.print(angleX);
//  Serial.print("\t");
//  Serial.print(angleY);
//  Serial.print("\t");
//  Serial.print(angleZ);
//  Serial.print("\t");
//  Serial.print(" | ");
  Serial.print(tX);
  Serial.print("\t");
  Serial.print(tY);
  Serial.print("\t");
  Serial.println(tZ);
/**  Serial.print("\t");
  Serial.print(" | ");
  Serial.print(accelX);
  Serial.print("\t");
  Serial.print(accelY);
  Serial.print("\t");
  Serial.println(accelZ);**/
  //Serial.println(state);
  /**Serial.print(a);
  Serial.print("\t");
  Serial.print(b);
  Serial.print("\t");
  Serial.print(c);
  Serial.print("\t");
  Serial.print(d);
  Serial.print("\t");
  Serial.print(e);
  Serial.print("\t");
  Serial.print(f);
  Serial.print("\t");
  Serial.print(g);
  Serial.print("\t");
  Serial.print(h);
  Serial.print("\t");
  Serial.println(i);**/
  
}
