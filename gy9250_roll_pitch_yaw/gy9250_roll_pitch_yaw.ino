 
#include "MPU9250.h"

// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire, 0x68);
int status;
float roll, pitch, yaw;
float accelX, accelY, accelZ, gyroX, gyroY, gyroZ, magX, magY, magZ;
void setup() {
  // serial to display data
  Serial.begin(115200);
  while (!Serial) {}

  // start communication with IMU
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while (1) {}
  }
  status=IMU.calibrateAccel();
  if (status < 0) {
    Serial.println("Accel calibration failed");
  }
  Serial.println("Accel calibration Done");
  status=IMU.calibrateGyro();
  if (status < 0) {
    Serial.println("Gyro calibration failed");
  }
  else
    Serial.println("Gyro calibration Done");
  status=IMU.calibrateMag();
  if (status < 0) {
    Serial.println("Mag calibration failed");
  }
  else
    Serial.println("Mag calibration Done");
}

void loop() {
  // read the sensor
  IMU.readSensor();

  // display the data
  accelX = IMU.getAccelX_mss();
  accelY = IMU.getAccelY_mss();
  accelZ = IMU.getAccelZ_mss();
  gyroX = IMU.getGyroX_rads();
  gyroY = IMU.getGyroY_rads();
  gyroZ = IMU.getGyroZ_rads();
  magX = IMU.getMagX_uT();
  magY = IMU.getMagY_uT();
  magZ = IMU.getMagZ_uT();

  pitch = atan2 (accelY , ( sqrt ((accelX * accelX) + (accelZ * accelZ))));
  roll = atan2(-accelX , ( sqrt((accelY * accelY) + (accelZ * accelZ))));

  float Yh = (magY * cos(roll)) - (magZ * sin(roll));
  float Xh = (magX * cos(pitch)) + (magY * sin(roll) * sin(pitch)) + (magZ * cos(roll) * sin(pitch));

  yaw =  atan2(Yh, Xh);


  roll = roll * 57.3;
  pitch = pitch * 57.3;
  yaw = yaw * 57.3;

  Serial.print("pitch=");
  Serial.print(pitch);
  Serial.print("roll=");
  Serial.print(roll);
  Serial.print("yaw=");
  Serial.print(yaw);
  Serial.println();
}
