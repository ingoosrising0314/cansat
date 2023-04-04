#include <Wire.h>
#include <MPU9250.h>

MPU9250 imu;

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  imu.begin();
}

void loop() {
  imu.readSensor();
  
  float ax = imu.getAccelX_mss();
  float ay = imu.getAccelY_mss();
  float az = imu.getAccelZ_mss();
  
  float gx = imu.getGyroX_rads();
  float gy = imu.getGyroY_rads();
  float gz = imu.getGyroZ_rads();
  
  float mx = imu.getMagX_uT();
  float my = imu.getMagY_uT();
  float mz = imu.getMagZ_uT();
  
  Serial.print(ax);
  Serial.print(",");
  Serial.print(ay);
  Serial.print(",");
  Serial.print(az);
  Serial.print(",");
  Serial.print(gx);
  Serial.print(",");
  Serial.print(gy);
  Serial.print(",");
  Serial.print(gz);
  Serial.print(",");
  Serial.print(mx);
  Serial.print(",");
  Serial.print(my);
  Serial.print(",");
  Serial.println(mz);
  
  delay(10);
}
