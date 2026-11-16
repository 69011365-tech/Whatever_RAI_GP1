#include <Wire.h>
#include <MPU6050_light.h>

MPU6050 mpu(Wire);

const int ENA = 11;
const int ENB = 6;
const int IN1 = 10;
const int IN2 = 9;
const int IN3 = 8;
const int IN4 = 7;

const int frontTrig = 5;
const int frontEcho = 4;
const int leftTrig = 2;
const int leftEcho = 3;
const int rightTrig = A2;
const int rightEcho = A1;
const int ir = A0;

const int DRIVE_SPEED = 100;
const int TURN_SPEED = 115;
const float TURN_90_ANGLE = 65.0;
const float TURN_180_ANGLE = 150.0;

const int SIDE_OPEN_DIST = 14;
const int FRONT_STOP_DIST = 7;
const int PRE_TURN_STEP_MS = 200;
const int EXIT_IGNORE_MS = 650;

float targetHeading = 0.0;
bool justStarted = true;

void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(frontTrig, OUTPUT);
  pinMode(frontEcho, INPUT);
  pinMode(leftTrig, OUTPUT);
  pinMode(leftEcho, INPUT);
  pinMode(rightTrig, OUTPUT);
  pinMode(rightEcho, INPUT);
  pinMode(ir, INPUT);

  Wire.begin();
  Wire.setWireTimeout(3000, true);

  Wire.beginTransmission(0x68);
  byte error68 = Wire.endTransmission();

  Wire.beginTransmission(0x69);
  byte error69 = Wire.endTransmission();

  if (error69 == 0) {
    mpu.setAddress(0x69);
  }

  mpu.begin();
  delay(1000);
  mpu.calcOffsets();
  targetHeading = 0.0;
}

void loop() {
  mpu.update();

  float frontDist = getDistance(frontTrig, frontEcho);
  float leftDist  = getDistance(leftTrig, leftEcho);
  float rightDist = getDistance(rightTrig, rightEcho);

  bool leftIsOpen  = (leftDist > SIDE_OPEN_DIST);
  bool rightIsOpen = (rightDist > SIDE_OPEN_DIST);
  bool frontIsOpen = (frontDist > FRONT_STOP_DIST || frontDist == 0);

  if (leftIsOpen) {
    justStarted = false;
    alignIntoIntersection(PRE_TURN_STEP_MS);
    turnLeft90IMU();
    exitTurnDriveForward(EXIT_IGNORE_MS);
  } 
  else if (rightIsOpen) {
    justStarted = false;
    alignIntoIntersection(PRE_TURN_STEP_MS);
    turnRight90IMU();
    exitTurnDriveForward(EXIT_IGNORE_MS);
  } 
  else if (frontIsOpen || justStarted) {
    driveStraightAtHeading();
    if (frontDist > 15) {
      justStarted = false;
    }
  } 
  else {
    cleanStop();
    turnAround180IMU();
    exitTurnDriveForward(EXIT_IGNORE_MS);
  }
}

void cleanStop() {
  stopMotors();
  delay(80);
}

void alignIntoIntersection(int maxMs) {
  unsigned long start = millis();
  while (millis() - start < maxMs) {
    float d = getDistance(frontTrig, frontEcho);
    if (d > 0 && d <= FRONT_STOP_DIST) {
      break;
    }
    mpu.update();
    float error = targetHeading - mpu.getAngleZ();
    int correction = error * 3;
    analogWrite(ENA, constrain(85 + correction, 65, 120));
    analogWrite(ENB, constrain(80 - correction, 65, 120));
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }
  cleanStop();
}

void exitTurnDriveForward(int ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    float d = getDistance(frontTrig, frontEcho);
    if (d > 0 && d <= 5) {
      cleanStop();
      break;
    }
    driveStraightAtHeading();
  }
}

void turnLeft90IMU() {
  mpu.update();
  float stopAngle = mpu.getAngleZ() + TURN_90_ANGLE;

  while (mpu.getAngleZ() < stopAngle) {
    mpu.update();
    analogWrite(ENA, TURN_SPEED);
    analogWrite(ENB, TURN_SPEED + 5);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }

  cleanStop();
  mpu.update();
  targetHeading = mpu.getAngleZ();
}

void turnRight90IMU() {
  mpu.update();
  float stopAngle = mpu.getAngleZ() - TURN_90_ANGLE;

  while (mpu.getAngleZ() > stopAngle) {
    mpu.update();
    analogWrite(ENA, TURN_SPEED);
    analogWrite(ENB, TURN_SPEED + 5);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }

  cleanStop();
  mpu.update();
  targetHeading = mpu.getAngleZ();
}

void turnAround180IMU() {
  mpu.update();
  float stopAngle = mpu.getAngleZ() + TURN_180_ANGLE;

  while (mpu.getAngleZ() < stopAngle) {
    mpu.update();
    analogWrite(ENA, TURN_SPEED);
    analogWrite(ENB, TURN_SPEED + 5);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }

  cleanStop();
  mpu.update();
  targetHeading = mpu.getAngleZ();
}

void driveStraightAtHeading() {
  mpu.update();
  float error = targetHeading - mpu.getAngleZ();
  int correction = error * 3;

  int rightSpeed = constrain(DRIVE_SPEED + correction, 65, 130);
  int leftSpeed  = constrain((DRIVE_SPEED - 5) - correction, 65, 130);

  analogWrite(ENA, rightSpeed);
  analogWrite(ENB, leftSpeed);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

float getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 15000);
  delay(10);
  return duration * 0.0343 / 2.0;
}