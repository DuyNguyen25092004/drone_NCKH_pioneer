#include <MPU6050_tockn.h>
#include <Wire.h>

MPU6050 mpu6050(Wire);

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
}

void loop() {
  mpu6050.update();
  Serial.print("angleX : ");
  Serial.print((int)max(min(mpu6050.getAngleX(),180),-180));
  Serial.print("\tangleY : ");
  Serial.print((int)max(min(mpu6050.getAngleY(),180),-180));
  Serial.print("\tangleZ : ");
  Serial.println((int)max(min(mpu6050.getAngleZ(),180),-180));
  delay(200);
}
