// Author: Bryson Hunsaker
#include <Wire.h>
#include <MPU6050.h>

// Multiple for tuning how long motors should actuate for a given gyro deflection
const double PROPORTIONAL_GAIN = 2;
// Multiple on integral of error - set to 0 to have this act as a simple P controller
const double INTEGRAL_GAIN = 0.001;
// Logic pins
const int MOTOR_L1 = 2;
const int MOTOR_L2 = 3;
const int MOTOR_R1 = 4;
const int MOTOR_R2 = 5;

MPU6050 mpu;
double integral = 0.0;
bool debug = 0;
double exec_time = 0;


void setup() {
  Serial.begin(115200);

  // Initialize MPU6050
  Serial.println("Initialize MPU6050");
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(200);
  }
  
  // Calibrate gyroscope
  mpu.calibrateGyro();

  // Set threshold sensitivity
  mpu.setThreshold(1);
    
  pinMode(MOTOR_L1,OUTPUT);
  pinMode(MOTOR_L2,OUTPUT);
  pinMode(MOTOR_R1,OUTPUT);
  pinMode(MOTOR_R2,OUTPUT);
}


void forward() {
  digitalWrite(MOTOR_R1, HIGH);
  digitalWrite(MOTOR_R2, LOW);
  digitalWrite(MOTOR_L1, HIGH);
  digitalWrite(MOTOR_L2, LOW);
}

void reverse() {
  digitalWrite(MOTOR_R1, LOW);
  digitalWrite(MOTOR_R2, HIGH);
  digitalWrite(MOTOR_L1, LOW);
  digitalWrite(MOTOR_L2, HIGH);
}

void stop () {
  digitalWrite(MOTOR_R1, LOW);
  digitalWrite(MOTOR_R2, LOW);
  digitalWrite(MOTOR_L1, LOW);
  digitalWrite(MOTOR_L2, LOW);
}

void loop() {
  // For measuring elapsed time
  long int t0 = 0;
  long int t1 = 0;
  t0 = micros();

  // Read Gyro
  Vector normGyro = mpu.readNormalizeGyro();
    
  //debug loop to prevent stop exec
  while(debug){}
  
  //Serial.println(normGyro.XAxis);
  if(normGyro.XAxis < 0) {
    forward();
  } else if (normGyro.XAxis > 0) {
    reverse();
  } else {
    stop();
  }
  
  // Approximation of integral of error wrt time
  integral += (abs(normGyro.XAxis)) * exec_time;
  // PI control
  double correction = ((abs(normGyro.XAxis)) * PROPORTIONAL_GAIN) + (integral * INTEGRAL_GAIN);
  delay(correction);
  
  t1 = micros();
  exec_time = (t1-t0) / 1000;
}
