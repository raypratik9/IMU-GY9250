#include <Wire.h>
void pidcalX();
   void pidcalY();
  void motor();
int calx,caly;
void prntdata();
double accel_x_cal,accel_y_cal,accel_z_cal;
double accelX, accelY, accelZ;
float rad_to_deg = 180/3.141592654;
float roll,pitch,yaw;

double gyroX, gyroY, gyroZ;
double gyro_x_cal,gyro_y_cal,gyro_z_cal;
#include <Servo.h>
Servo motor1;
Servo motor2;
Servo motor3;
Servo motor4;
float pid_px=0;
float pid_ix=0;
float pid_dx=0;
float pid_py=0;
float pid_iy=0;
float pid_dy=0;
int i;
float elapsedTime, Time, timePrev;
float PIDx,PIDy, Motor1, Motor2,Motor3, Motor4, errorx,errory,previous_errory, previous_errorx;
double kp=3.55;//3.55
double ki=0.005;//0.003
double kd=2.05;//2.05
double throttle=1300;
float desired_angle = 0;
void setup() {
  Serial.begin(9600);
  Wire.begin();
  motor1.attach(6);
  motor2.attach(9);
  motor3.attach(10);
  motor4.attach(11);
   Wire.begin(); //begin the wire comunication
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
float Time = millis();
motor1.writeMicroseconds(1000);
motor2.writeMicroseconds(1000);
motor3.writeMicroseconds(1000);
motor4.writeMicroseconds(1000);
delay(7000);
}
void loop() {
  timePrev = Time;  // the previous time is stored before the actual time read
    Time = millis();  // actual time read
    elapsedTime = (Time - timePrev) / 1000; 
Wire.beginTransmission(0x68);
     Wire.write(0x3B); //Ask for the 0x3B register- correspond to AcX
     Wire.endTransmission(false);
     Wire.requestFrom(0x68,6,true); 
    
     accelX=Wire.read()<<8|Wire.read(); //each value needs two registres
     accelY=Wire.read()<<8|Wire.read();
     accelZ=Wire.read()<<8|Wire.read();

 
     accel_x_cal= atan((accelY/16384.0)/sqrt(pow((accelX/16384.0),2) + pow((accelZ/16384.0),2)))*rad_to_deg;
     /*---Y---*/
     accel_y_cal= atan(-1*(accelX/16384.0)/sqrt(pow((accelY/16384.0),2) + pow((accelZ/16384.0),2)))*rad_to_deg;
 
   Wire.beginTransmission(0x68);
   Wire.write(0x43); //Gyro data first adress
   Wire.endTransmission(false);
   Wire.requestFrom(0x68,4,true); //Just 4 registers
   
   gyroX=Wire.read()<<8|Wire.read(); //Once again we shif and sum
   gyroY=Wire.read()<<8|Wire.read();
 

   gyro_x_cal= gyroX/131.0; 
   /*---Y---*/
   gyro_y_cal= gyroY/131.0;

   roll= 0.98 *(roll+gyro_x_cal*elapsedTime) + 0.02*accel_x_cal;
   /*---Y axis angle---*/
   pitch= 0.98 *(pitch+gyro_y_cal*elapsedTime) + 0.02*accel_y_cal;
   pidcalX();
   pidcalY();
   motor();
 previous_errorx = errorx;
 previous_errory = errory;
 prntdata();
}

void pidcalX()
{
  errorx =roll- desired_angle;
pid_px = kp*errorx;

if(-3 <errorx <3)
{
  pid_ix = pid_ix+(ki*errorx);  
}
pid_dx = kd*((errorx - previous_errorx)/elapsedTime);

PIDx = pid_px + pid_ix + pid_dx;
if(PIDx < -1000)
{
  PIDx=-1000;
}
if(PIDx > 1000)
{
  PIDx=1000;
}
}
void pidcalY()
{
  errory =pitch- desired_angle;
pid_py = kp*errory;

if(-3 <errory <3)
{
  pid_iy = pid_iy+(ki*errory);  
}
pid_dy = kd*((errory - previous_errory)/elapsedTime);

PIDy = pid_py + pid_iy + pid_dy;
if(PIDy< -1000)
{
  PIDy=-1000;
}
if(PIDy > 1000)
{
  PIDy=1000;
}
}
void motor()
{
  Motor1=throttle+PIDx;
  Motor3=throttle-PIDx;
  Motor2=throttle+PIDy;
  Motor4=throttle-PIDy;
motor1.writeMicroseconds(Motor1);
motor2.writeMicroseconds(Motor2);
motor3.writeMicroseconds(Motor3);
motor4.writeMicroseconds(Motor4);
}
void prntdata()
{
   Serial.print(" ROLL=");
  Serial.print(roll);
  Serial.print(" angle");
  Serial.print("    PITCH=");
  Serial.print(pitch);
  Serial.print(" angle");
  Serial.print("    YAW=");
  Serial.print(yaw);
  Serial.print("Motor1");
  Serial.println(Motor1);
  Serial.print(Motor2);
  Serial.print("Motor2");
  Serial.print("Motor3");
  Serial.print(Motor3);
  Serial.print("Motor4");
  Serial.print(Motor4);
  Serial.print("pidX");
  Serial.print(PIDx);
  Serial.print("pidY");
  Serial.print(PIDy);
  Serial.print("elapsedTime");
  Serial.println(elapsedTime);

}
