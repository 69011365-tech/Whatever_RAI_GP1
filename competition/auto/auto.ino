#include <Wire.h>
#include <MPU6050_light.h>

MPU6050 mpu(Wire);

struct Motor {
  int en;   
  int inA;   
  int inB;   
};

struct Ultrasonic {
  int trig;
  int echo;
  float distance; 
};

enum MotorId  { MOTOR_RIGHT = 0, MOTOR_LEFT = 1, NUM_MOTORS = 2 };
enum SensorId { SENSOR_FRONT = 0, SENSOR_LEFT = 1, SENSOR_RIGHT = 2, NUM_SENSORS = 3 };

Motor motors[NUM_MOTORS] = {
  {11, 10, 9},   // Right motor: ENA, IN1, IN2
  { 6,  8, 7}    // Left motor : ENB, IN3, IN4
};

Ultrasonic sensors[NUM_SENSORS] = {
  { 5,  4, 0},   // Front
  { 2,  3, 0},   // Left
  {A2, A1, 0}    // Right
};

const byte MPU_ADDRESSES[2] = {0x68, 0x69};

const int ir = A0;

const int DIR_FORWARD  =  1;
const int DIR_BACKWARD = -1;
const int DIR_STOP     =  0;

const int DRIVE_SPEED = 100;
const int TURN_SPEED  = 115;
const float TURN_90_ANGLE  = 65.0;
const float TURN_180_ANGLE = 150.0;

const int SIDE_OPEN_DIST   = 14;
const int FRONT_STOP_DIST  = 7;
const int PRE_TURN_STEP_MS = 200;
const int EXIT_IGNORE_MS   = 650;

float targetHeading = 0.0;
bool justStarted = true;

// ========================= SETUP ==========================
void setup() {
  // LOOP over motor array
  for (int i = 0; i < NUM_MOTORS; i++) {
    pinMode(motors[i].en,  OUTPUT);
    pinMode(motors[i].inA, OUTPUT);
    pinMode(motors[i].inB, OUTPUT);
  }

  // LOOP over sensor array
  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(sensors[i].trig, OUTPUT);
    pinMode(sensors[i].echo, INPUT);
  }
  pinMode(ir, INPUT);

  Wire.begin();
  Wire.setWireTimeout(3000, true);

  byte foundAddress = 0x68;
  for (int i = 0; i < 2; i++) {
    Wire.beginTransmission(MPU_ADDRESSES[i]);
    if (Wire.endTransmission() == 0) {
      foundAddress = MPU_ADDRESSES[i];
    }
  }
  if (foundAddress == 0x69) {
    mpu.setAddress(0x69);
  }

  mpu.begin();
  delay(1000);
  mpu.calcOffsets();
  targetHeading = 0.0;
}


void loop() {
  mpu.update();

  readAllSensors(sensors, NUM_SENSORS);

  float frontDist = sensors[SENSOR_FRONT].distance;
  float leftDist  = sensors[SENSOR_LEFT].distance;
  float rightDist = sensors[SENSOR_RIGHT].distance;

  bool leftIsOpen  = (leftDist > SIDE_OPEN_DIST);
  bool rightIsOpen = (rightDist > SIDE_OPEN_DIST);
  bool frontIsOpen = (frontDist > FRONT_STOP_DIST || frontDist == 0);

  if (leftIsOpen) {
    justStarted = false;
    handleTurn(TURN_90_ANGLE);          // left = positive angle
  }
  else if (rightIsOpen) {
    justStarted = false;
    handleTurn(-TURN_90_ANGLE);         // right = negative angle
  }
  else if (frontIsOpen || justStarted) {
    driveStraightAtHeading();
    if (frontDist > 15) {
      justStarted = false;
    }
  }
  else {
    cleanStop();
    turnIMU(TURN_180_ANGLE);
    exitTurnDriveForward(EXIT_IGNORE_MS);
  }
}

// ====================== MOTOR FUNCTIONS

void setMotor(const Motor *m, int speed, int dir) {
  if (dir == DIR_FORWARD) {
    digitalWrite(m->inA, LOW);
    digitalWrite(m->inB, HIGH);
  } else if (dir == DIR_BACKWARD) {
    digitalWrite(m->inA, HIGH);
    digitalWrite(m->inB, LOW);
  } else {
    digitalWrite(m->inA, LOW);
    digitalWrite(m->inB, LOW);
    speed = 0;
  }
  analogWrite(m->en, speed);
}

void drive(int rightSpeed, int rightDir, int leftSpeed, int leftDir) {
  setMotor(&motors[MOTOR_RIGHT], rightSpeed, rightDir);
  setMotor(&motors[MOTOR_LEFT],  leftSpeed,  leftDir);
}

void stopMotors() {
  for (int i = 0; i < NUM_MOTORS; i++) {
    setMotor(&motors[i], 0, DIR_STOP);
  }
}

void cleanStop() {
  stopMotors();
  delay(80);
}

void driveWithHeading(int rightBase, int leftBase, int maxSpeed) {
  mpu.update();
  float error = targetHeading - mpu.getAngleZ();
  int correction = error * 3;

  int rightSpeed = constrain(rightBase + correction, 65, maxSpeed);
  int leftSpeed  = constrain(leftBase  - correction, 65, maxSpeed);

  drive(rightSpeed, DIR_FORWARD, leftSpeed, DIR_FORWARD);
}

void driveStraightAtHeading() {
  driveWithHeading(DRIVE_SPEED, DRIVE_SPEED - 5, 130);
}

// ====================== MOVEMENT FUNCTIONS ================

void alignIntoIntersection(int maxMs) {
  unsigned long start = millis();
  while (millis() - start < maxMs) {
    float d = readSensor(&sensors[SENSOR_FRONT]);
    if (d > 0 && d <= FRONT_STOP_DIST) {
      break;
    }
    driveWithHeading(85, 80, 120);
  }
  cleanStop();
}

void exitTurnDriveForward(int ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    float d = readSensor(&sensors[SENSOR_FRONT]);
    if (d > 0 && d <= 5) {
      cleanStop();
      break;
    }
    driveStraightAtHeading();
  }
}

void turnIMU(float degrees) {
  mpu.update();
  float stopAngle = mpu.getAngleZ() + degrees;
  bool turningLeft = (degrees > 0);

  int rightDir = turningLeft ? DIR_FORWARD  : DIR_BACKWARD;
  int leftDir  = turningLeft ? DIR_BACKWARD : DIR_FORWARD;

  while (turningLeft ? (mpu.getAngleZ() < stopAngle)
                     : (mpu.getAngleZ() > stopAngle)) {
    mpu.update();
    drive(TURN_SPEED, rightDir, TURN_SPEED + 5, leftDir);
  }

  cleanStop();
  mpu.update();
  targetHeading = mpu.getAngleZ();
}

void handleTurn(float degrees) {
  alignIntoIntersection(PRE_TURN_STEP_MS);
  turnIMU(degrees);
  exitTurnDriveForward(EXIT_IGNORE_MS);
}

// ====================== SENSOR FUNCTIONS 

// POINTER: writes the result directly into the structure
float readSensor(Ultrasonic *s) {
  digitalWrite(s->trig, LOW);
  delayMicroseconds(2);
  digitalWrite(s->trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(s->trig, LOW);

  long duration = pulseIn(s->echo, HIGH, 15000);
  delay(10);
  s->distance = duration * 0.0343 / 2.0;
  return s->distance;
}

// POINTER to array + LOOP
void readAllSensors(Ultrasonic *list, int count) {
  for (int i = 0; i < count; i++) {
    readSensor(list + i);  
  }
}
