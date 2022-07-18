#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

// X Value for accel on a stable surface
double OFFSET = -1;
// Multiple for tuning how long motors should actuate for a given accel
double DELAY_FACTOR = 3;
int MOTOR_L1 = 2;
int MOTOR_L2 = 3;
int MOTOR_R1 = 4;
int MOTOR_R2 = 5;
int ENABLE_L = 10;
int ENABLE_R = 11;

void setup() {
  Serial.begin(115200);

  // Initialize MPU6050
  Serial.println("Initialize MPU6050");
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }

  //mpu.setAccelOffsetX(-2.25);
    
  pinMode(MOTOR_L1,OUTPUT);
  pinMode(MOTOR_L2,OUTPUT);
  pinMode(MOTOR_R1,OUTPUT);
  pinMode(MOTOR_R2,OUTPUT);
  pinMode(ENABLE_L,OUTPUT);
  pinMode(ENABLE_R,OUTPUT);
}


void forward() {
  digitalWrite(MOTOR_R1, HIGH);
  digitalWrite(MOTOR_R2, LOW);
  digitalWrite(MOTOR_L1, HIGH);
  digitalWrite(MOTOR_L2, LOW);
  analogWrite(ENABLE_R, 250);
  analogWrite(ENABLE_L, 250);
}

void reverse() {
  digitalWrite(MOTOR_R1, LOW);
  digitalWrite(MOTOR_R2, HIGH);
  digitalWrite(MOTOR_L1, LOW);
  digitalWrite(MOTOR_L2, HIGH);
  analogWrite(ENABLE_R, 250);
  analogWrite(ENABLE_L, 250);
}

void stop () {
  analogWrite(ENABLE_R, 0);
  analogWrite(ENABLE_L, 0);
  digitalWrite(MOTOR_R1, LOW);
  digitalWrite(MOTOR_R2, LOW);
  digitalWrite(MOTOR_L1, LOW);
  digitalWrite(MOTOR_L2, LOW);
}

void loop() {
  Vector normAccel = mpu.readNormalizeAccel();
  Serial.println(normAccel.XAxis);
  if(normAccel.XAxis < OFFSET) {
    //forward();
  } else if (normAccel.XAxis > OFFSET) {
    //reverse();
  }
  // Proportionality of response will come from delay
  delay((abs(normAccel.XAxis - OFFSET)) * DELAY_FACTOR);
}
