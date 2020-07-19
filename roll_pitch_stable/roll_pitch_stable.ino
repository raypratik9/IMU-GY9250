#include <Wire.h>
#include <Servo.h>
#define A 0.962
#define dt 0.020
Servo right_prop;
Servo left_prop;
Servo right_prop2;
Servo left_prop2;
float elapsedTime, time, timePrev;
int i;
float rad_to_deg = 180 / 3.141592654;
void pidcal();
void pidcal2();
float PID, pwmLeft, pwmRight, error, previous_error;
float pid_p = 0;
float pid_i = 0;
float pid_d = 0;
float PID2, pwmLeft2, pwmRight2, error2, previous_error2;
float pid_p2 = 0;
float pid_i2 = 0;
float pid_d2 = 0;
/////////////////PID CONSTANTS/////////////////
double kp = 3.55; //3.55
double ki = 0.005; //0.003
double kd = 2.05; //2.05
double kp2 = 3.55; //3.55
double ki2 = 0.005; //0.003
double kd2 = 2.05; //2.05

///////////////////////////////////////////////

double throttle = 1500;
float desired_angle = -7.30;
float desired_angle2 = -1.38;

int calx, caly;

double accel_x_cal, accel_y_cal, accel_z_cal;
double accelX, accelY, accelZ;

float rollangle, pitchangle;
float roll, pitch, yaw;

double gyroX, gyroY, gyroZ;
double gyro_x_cal, gyro_y_cal, gyro_z_cal;

void recordAccelRegisters();
void recordGyroRegisters();
void setup() {
  Serial.begin(9600);
  Wire.begin();
  right_prop.attach(6);
  left_prop.attach(9);
  right_prop2.attach(10);
  left_prop2.attach(11);
  time = millis();
  Wire.beginTransmission(0b1101000);
  Wire.write(0x6B);
  Wire.write(0b00000000);
  Wire.endTransmission();
  Wire.beginTransmission(0b1101000);
  Wire.write(0x1B);
  Wire.write(0x00000000);
  Wire.endTransmission();
  Wire.beginTransmission(0b1101000);
  Wire.write(0x1C);
  Wire.write(0b00000000);
  Wire.endTransmission();

  Serial.println("Calibrating Accelerometer......");
  for (calx = 1; calx <= 2000; calx++)
  {
    recordAccelRegisters();
    accel_x_cal += accelX;
    accel_y_cal += accelY;
    accel_z_cal += accelZ;
  }
  Serial.println("Calibrating Gyroscope......");
  for (caly = 1; caly <= 2000; caly++)
  {
    recordGyroRegisters();
    gyro_x_cal += gyroX;
    gyro_y_cal += gyroY;
    gyro_z_cal += gyroZ;
  }
  Serial.println("Calibration Done..!!!");
  gyro_x_cal /= 2000;
  gyro_y_cal /= 2000;
  gyro_z_cal /= 2000;

  gyro_x_cal /= 2000;
  gyro_y_cal /= 2000;
  gyro_z_cal /= 2000;
  left_prop.writeMicroseconds(2000);
  delay(4000);
  left_prop.writeMicroseconds(1000);
  right_prop.writeMicroseconds(1000);
  left_prop2.writeMicroseconds(1000);
  right_prop2.writeMicroseconds(1000);
  delay(7000);
}

void loop() {
  timePrev = time;
  time = millis();
  elapsedTime = (time - timePrev) / 1000;
  accelX = (accelX / 16384.0);
  accelY = (accelY / 16384.0);
  accelZ = (accelZ / 16384.0);

  gyroX = gyroX / 131.0;
  gyroY = gyroY / 131.0;
  gyroZ = gyroZ / 131.0;

  rollangle = atan2(accelY, accelZ) * 180 / PI;
  pitchangle = atan2(accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180 / PI;


  roll = A * (roll + gyroX * dt) + (1 - A) * rollangle;
  pitch = A * (pitch + gyroY * dt) + (1 - A) * pitchangle;

  yaw = gyroZ;

  Serial.print(" ROLL=");
  Serial.print(roll);
  Serial.print(" angle");
  Serial.print("    PITCH=");
  Serial.print(pitch);
  Serial.print(" angle");
  Serial.print("    YAW=");
  Serial.print(yaw);
  Serial.println(" deg/s");
  recordAccelRegisters();
  recordGyroRegisters();
  pidcal();
  pidcal2();
}
void recordAccelRegisters() {
  Wire.beginTransmission(0b1101000);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0b1101000, 6);
  while (Wire.available() < 6);
  accelX = Wire.read() << 8 | Wire.read();
  if (calx == 2000)accelX -= accel_x_cal;
  accelY = Wire.read() << 8 | Wire.read();
  if (calx == 2000)accelY -= accel_y_cal;
  accelZ = Wire.read() << 8 | Wire.read();
  if (calx == 2000)accelZ -= accel_z_cal;

}

void recordGyroRegisters() {
  Wire.beginTransmission(0b1101000);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0b1101000, 6);
  while (Wire.available() < 6);
  gyroX = Wire.read() << 8 | Wire.read();
  if (caly == 2000)gyroX -= gyro_x_cal;
  gyroY = Wire.read() << 8 | Wire.read();
  if (caly == 2000)gyroY -= gyro_y_cal;
  gyroZ = Wire.read() << 8 | Wire.read();
  if (caly == 2000)gyroZ -= gyro_z_cal;

}
void pidcal()
{
  error = pitch - desired_angle;

  pid_p = kp * error;

  if (-3 < error < 3)
  {
    pid_i = pid_i + (ki * error);
  }

  pid_d = kd * ((error - previous_error) / elapsedTime);


  PID = pid_p + pid_i + pid_d;

  if (PID < -1000)
  {
    PID = -1000;
  }
  if (PID > 1000)
  {
    PID = 1000;
  }
  pwmLeft = throttle + PID;
  pwmRight = throttle - PID;

  //Right
  if (pwmRight < 1000)
  {
    pwmRight = 1000;
  }
  if (pwmRight > 2000)
  {
    pwmRight = 2000;
  }

  //Left
  if (pwmLeft < 1000)
  {
    pwmLeft = 1000;
  }
  if (pwmLeft > 2000)
  {
    pwmLeft = 2000;
  }


  left_prop.writeMicroseconds(pwmLeft);
  right_prop.writeMicroseconds(pwmRight);
  previous_error = error;
}
void pidcal2()
{
  error2 = roll - desired_angle2;

  pid_p2 = kp * error2;


  if (-3 < error2 < 3)
  {
    pid_i2 = pid_i2 + (ki * error2);
  }

  pid_d2 = kd * ((error2 - previous_error2) / elapsedTime);

  PID2 = pid_p2 + pid_i2 + pid_d2;


  if (PID2 < -1000)
  {
    PID2 = -1000;
  }
  if (PID2 > 1000)
  {
    PID2 = 1000;
  }
  pwmLeft2 = throttle + PID2;
  pwmRight2 = throttle - PID2;

  //Right
  if (pwmRight2 < 1000)
  {
    pwmRight2 = 1000;
  }
  if (pwmRight2 > 2000)
  {
    pwmRight2 = 2000;
  }
  //Left
  if (pwmLeft2 < 1000)
  {
    pwmLeft2 = 1000;
  }
  if (pwmLeft2 > 2000)
  {
    pwmLeft2 = 2000;
  }

  left_prop2.writeMicroseconds(pwmLeft2);
  right_prop2.writeMicroseconds(pwmRight2);
  previous_error2 = error2;
}
