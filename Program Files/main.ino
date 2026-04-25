#include <Wire.h>
#include <MPU6050.h>
#include <ESP32Servo.h>

Servo servoYaw;
Servo servoPitch;
Servo servoRoll;
const int yawPin = 23;
const int pitchPin = 16;
const int rollPin = 17;

int servo_yawVal;
int servo_pitchVal;
int servo_rollVal;

MPU6050 mpu(0x68);
float yawOffset = 0;
float yaw = 0;

void setup() {
  Serial.begin(115200);
  servoYaw.attach(yawPin);
  servoPitch.attach(pitchPin);
  servoRoll.attach(rollPin);
  servoYaw.write(90);
  servoPitch.write(90);
  servoRoll.write(90);
  Wire.begin(21, 22);
  delay(100);

  mpu.initialize();
  Serial.println(mpu.testConnection() ? "MPU6050 connected!" : "Connection failed!");

  Serial.println("Calibrating... Keep still!");
  long sum = 0;
  int16_t ax, ay, az, gx, gy, gz;
  for (int i = 0; i < 200; i++) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    sum += gz;
    delay(5);
  }
  yawOffset = sum / 200.0;
  Serial.println("Done! Initial = 0");
}

void loop() {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float accelX = ax / 16384.0;
  float accelY = ay / 16384.0;
  float accelZ = az / 16384.0;

  float roll  = atan2(accelY, accelZ) * 180.0 / PI;
  float pitch = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180.0 / PI;

  static unsigned long lastTime = 0;
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  yaw += ((gz - yawOffset) / 131.0) * dt;

  yaw = constrain(yaw, -90, 90);
  pitch = constrain(pitch, -90, 90);
  roll = constrain(roll, -90, 90);

  if ((int)yaw <= 0) {
    servo_yawVal = map((int)yaw, 0, -90, 90, 180);
  } else {
    servo_yawVal = map((int)yaw, 0, 90, 90, 0);
  }

  if ((int)pitch <= 0) {
    servo_pitchVal = map((int)pitch, 0, -90, 90, 180);
  } else {
    servo_pitchVal = map((int)pitch, 0, 90, 90, 0);
  }

  if ((int)roll <= 0) {
    servo_rollVal = map((int)roll, 0, -90, 90, 180);
  } else {
    servo_rollVal = map((int)roll, 0, 90, 90, 0);
  }

  servoYaw.write(servo_yawVal);
  servoPitch.write(servo_pitchVal);
  servoRoll.write(servo_rollVal);

  Serial.print("Yaw: "); Serial.print(yaw);
  Serial.print(" | Pitch: "); Serial.print(pitch);
  Serial.print(" | Servo Yaw: "); Serial.print(servo_yawVal);
  Serial.print(" | Servo Pitch: "); Serial.println(servo_pitchVal);

  delay(400);
}
