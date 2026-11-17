#include <Wire.h>
#include <EEPROM.h>

struct Motor {
  byte en;
  byte pinA;
  byte pinB;
};

struct Sonar {
  byte trig;
  byte echo;
  byte hits;
};

struct LineSensor {
  byte pin;
  byte blackLevel;
  byte hits;
};

struct Gyro {
  byte addr;
  float bias[3];
  byte axis;
  float sign;
  float yaw;
  float rate;
  float coastGain;
  float savedGain;
  unsigned long lastUs;
  unsigned long lastGoodMs;
};

struct Navigator {
  float heading;
  byte step;
  unsigned long lastPingMs;
  unsigned long ignoreSidesUntil;
  unsigned long straightStartMs;
};

enum { FRONT = 0, LEFT = 1, RIGHT = 2 };

Motor motors[2] = { {11, 10, 9}, {6, 8, 7} };
Sonar sonars[3] = { {5, 4, 0}, {2, 3, 0}, {A2, A1, 0} };
const byte PING_ORDER[4] = { FRONT, LEFT, FRONT, RIGHT };
const byte GYRO_ADDRESSES[2] = { 0x68, 0x69 };
const char AXIS_NAMES[] = "XYZ";
const byte LED = 13;
LineSensor ir = { 12, HIGH, 0 };
const byte IR_CONFIRM = 3;

const bool SWAP_LEFT_RIGHT = false;
const float LEFT_SIGN = SWAP_LEFT_RIGHT ? -1 : 1;

const float FRONT_STOP_CM = 5.0;
const float SIDE_OPEN_CM = 14.0;
const int SLOW_SPEED = 100;
const unsigned long SLOW_AFTER_TURN_MS = 1200;
const unsigned long EXIT_IGNORE_MS = 1000;
const unsigned long SONAR_TIMEOUT_US = 6000;
const unsigned long PING_GAP_MS = 30;

const int DRIVE_SPEED = 125;
const int ENB_TRIM = -5;
const float KP = 3.0;
const float KD = 0.15;

const int TURN_SPEED = 140;
const int TURN_SLOW = 110;
const float SLOW_ZONE_DEG = 25.0;
const float FINE_TOLERANCE_DEG = 2.0;
const float GYRO_SCALE = 32.8;
const byte EEPROM_MAGIC = 0xA7;

Gyro gyro = { 0x68, {0, 0, 0}, 2, 1, 0, 0, 0.05, -1, 0, 0 };
Navigator nav = { 0, 0, 0, 0, 0 };

void runMotor(const Motor *m, int power) {
  digitalWrite(m->pinA, power >= 0 ? LOW : HIGH);
  digitalWrite(m->pinB, power >= 0 ? HIGH : LOW);
  analogWrite(m->en, constrain(abs(power), 0, 255));
}

void setMotors(int a, int b) {
  runMotor(&motors[0], a);
  runMotor(&motors[1], b);
}

void spin(float dir, int power) {
  setMotors(dir * power, -dir * power);
}

void brake() {
  for (byte i = 0; i < 2; i++) {
    digitalWrite(motors[i].pinA, HIGH);
    digitalWrite(motors[i].pinB, HIGH);
  }
  for (byte i = 0; i < 2; i++) {
    analogWrite(motors[i].en, 255);
  }
}

float readSonar(const Sonar *s) {
  digitalWrite(s->trig, LOW);
  delayMicroseconds(2);
  digitalWrite(s->trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(s->trig, LOW);
  unsigned long t = pulseIn(s->echo, HIGH, SONAR_TIMEOUT_US);
  if (t == 0) return 999;
  return t * 0.0343 / 2.0;
}

bool readRaw(Gyro *g, int16_t v[3]) {
  Wire.beginTransmission(g->addr);
  Wire.write(0x43);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(g->addr, (byte)6) != 6) return false;
  for (byte i = 0; i < 3; i++) {
    byte hi = Wire.read();
    byte lo = Wire.read();
    v[i] = (int16_t)((hi << 8) | lo);
  }
  return true;
}

bool updateGyro(Gyro *g) {
  unsigned long now = micros();
  float dt = (now - g->lastUs) * 1e-6;
  g->lastUs = now;
  int16_t v[3];
  if (!readRaw(g, v)) return false;
  g->rate = g->sign * ((v[g->axis] - g->bias[g->axis]) / GYRO_SCALE);
  g->yaw += g->rate * dt;
  g->lastGoodMs = millis();
  return true;
}

bool gyroAlive(const Gyro *g) {
  return millis() - g->lastGoodMs < 150;
}

void waitWithGyro(unsigned long ms) {
  unsigned long t0 = millis();
  while (millis() - t0 < ms) updateGyro(&gyro);
}

bool sideOpen(const Sonar *s) {
  byte opens = 0;
  for (byte i = 0; i < 3; i++) {
    if (readSonar(s) > SIDE_OPEN_CM) opens++;
    waitWithGyro(30);
  }
  return opens >= 2;
}

void loadGain(Gyro *g) {
  if (EEPROM.read(0) != EEPROM_MAGIC) return;
  float stored;
  EEPROM.get(1, stored);
  if (stored >= 0.0 && stored <= 0.4) {
    g->coastGain = stored;
    g->savedGain = stored;
  }
}

void saveGain(Gyro *g) {
  if (abs(g->coastGain - g->savedGain) < 0.004) return;
  EEPROM.write(0, EEPROM_MAGIC);
  EEPROM.put(1, g->coastGain);
  g->savedGain = g->coastGain;
}

bool devicePresent(byte addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

void gyroWrite(const Gyro *g, byte reg, byte value) {
  Wire.beginTransmission(g->addr);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

void findGyroAddress(Gyro *g) {
  while (true) {
    for (byte i = 0; i < 2; i++) {
      if (devicePresent(GYRO_ADDRESSES[i])) {
        g->addr = GYRO_ADDRESSES[i];
        return;
      }
    }
    Serial.println(F("Gyro not answering - check SDA/SCL"));
    digitalWrite(LED, !digitalRead(LED));
    delay(100);
  }
}

void startGyro(Gyro *g) {
  setMotors(0, 0);
  findGyroAddress(g);
  gyroWrite(g, 0x6B, 0x00);
  gyroWrite(g, 0x1B, 0x10);
  gyroWrite(g, 0x1A, 0x03);
  delay(100);

  digitalWrite(LED, HIGH);
  Serial.println(F("Calibrating - keep the robot still"));
  delay(1000);
  long sum[3] = {0, 0, 0};
  int count = 0;
  for (int i = 0; i < 500; i++) {
    int16_t v[3];
    if (readRaw(g, v)) {
      for (byte k = 0; k < 3; k++) sum[k] += v[k];
      count++;
    }
    delay(3);
  }
  if (count > 0) {
    for (byte k = 0; k < 3; k++) g->bias[k] = (float)sum[k] / count;
  }
  digitalWrite(LED, LOW);
  g->yaw = 0;
  g->rate = 0;
  g->lastUs = micros();
  g->lastGoodMs = millis();
}

void findAxis(Gyro *g) {
  float angle[3] = {0, 0, 0};
  unsigned long t0 = millis();
  unsigned long last = micros();
  spin(1, TURN_SPEED);
  while (millis() - t0 < 600) {
    if (millis() - t0 > 300) brake();
    unsigned long now = micros();
    float dt = (now - last) * 1e-6;
    last = now;
    int16_t v[3];
    if (readRaw(g, v)) {
      for (byte k = 0; k < 3; k++) angle[k] += ((v[k] - g->bias[k]) / GYRO_SCALE) * dt;
    }
  }
  setMotors(0, 0);

  g->axis = 0;
  for (byte k = 1; k < 3; k++) {
    if (abs(angle[k]) > abs(angle[g->axis])) g->axis = k;
  }
  g->sign = (angle[g->axis] >= 0) ? 1 : -1;
  g->yaw = abs(angle[g->axis]);
  g->rate = 0;
  g->lastUs = micros();
  g->lastGoodMs = millis();

  Serial.print(F("Using axis "));
  Serial.print(AXIS_NAMES[g->axis]);
  Serial.print(F(", nudge "));
  Serial.println(g->yaw, 1);
  if (g->yaw < 3) Serial.println(F("WARNING: robot barely turned - raise TURN_SPEED"));
}

bool turnTo(Gyro *g, float goal) {
  float dir = (goal > g->yaw) ? 1 : -1;
  float rateAtCut = 0;
  unsigned long t0 = millis();
  while (millis() - t0 < 5000) {
    if (!updateGyro(g)) {
      if (!gyroAlive(g)) {
        setMotors(0, 0);
        return false;
      }
      continue;
    }
    float remaining = (goal - g->yaw) * dir;
    float lead = max(2.0, g->coastGain * abs(g->rate));
    if (remaining <= lead) {
      rateAtCut = abs(g->rate);
      break;
    }
    spin(dir, remaining > SLOW_ZONE_DEG ? TURN_SPEED : TURN_SLOW);
  }
  brake();
  waitWithGyro(350);

  float firstStop = g->yaw;
  float overshoot = (g->yaw - goal) * dir;
  if (rateAtCut > 30) {
    g->coastGain = constrain(g->coastGain + 0.6 * overshoot / rateAtCut, 0.0, 0.4);
    saveGain(g);
  }

  for (byte i = 0; i < 8; i++) {
    float err = goal - g->yaw;
    if (abs(err) <= FINE_TOLERANCE_DEG) break;
    spin(err > 0 ? 1 : -1, TURN_SLOW);
    waitWithGyro(35);
    brake();
    waitWithGyro(120);
  }
  setMotors(0, 0);

  Serial.print(F("Wanted "));
  Serial.print(goal, 0);
  Serial.print(F("  first stop "));
  Serial.print(firstStop, 1);
  Serial.print(F("  after correction "));
  Serial.println(g->yaw, 1);
  return true;
}

void driveStraight(const Navigator *n, const Gyro *g) {
  float err = n->heading - g->yaw;
  float corr = constrain(KP * err - KD * g->rate, -40, 40);
  int speed = (millis() - n->straightStartMs < SLOW_AFTER_TURN_MS) ? SLOW_SPEED : DRIVE_SPEED;
  int a = constrain(speed + corr, 60, 180);
  int b = constrain(speed + ENB_TRIM - corr, 60, 180);
  setMotors(a, b);
}

void stopForever(const __FlashStringHelper *why) {
  setMotors(0, 0);
  brake();
  delay(200);
  setMotors(0, 0);
  Serial.println(why);
  while (true) {
    digitalWrite(LED, !digitalRead(LED));
    delay(300);
  }
}

void turnBy(Navigator *n, float delta) {
  n->heading += delta;
  if (!turnTo(&gyro, n->heading)) stopForever(F("Gyro lost during turn - stopped"));
  waitWithGyro(200);
  for (byte i = 0; i < 3; i++) sonars[i].hits = 0;
  n->step = 0;
  n->lastPingMs = 0;
  n->ignoreSidesUntil = millis() + EXIT_IGNORE_MS;
  n->straightStartMs = millis();
}

bool onBlackTape(LineSensor *l) {
  if (digitalRead(l->pin) == l->blackLevel) l->hits++;
  else l->hits = 0;
  return l->hits >= IR_CONFIRM;
}

void stopBeforeTurn() {
  brake();
  waitWithGyro(200);
}

void pingNext(Navigator *n) {
  if (millis() - n->lastPingMs < PING_GAP_MS) return;
  n->lastPingMs = millis();
  bool sidesOn = millis() >= n->ignoreSidesUntil;
  Sonar *s = &sonars[PING_ORDER[n->step]];

  if (s == &sonars[FRONT]) {
    if (readSonar(s) <= FRONT_STOP_CM) s->hits++;
    else s->hits = 0;
  } else {
    if (sidesOn && readSonar(s) > SIDE_OPEN_CM) s->hits++;
    else s->hits = 0;
  }
  n->step = (n->step + 1) % 4;
}

void setup() {
  for (byte i = 0; i < 2; i++) {
    pinMode(motors[i].en, OUTPUT);
    pinMode(motors[i].pinA, OUTPUT);
    pinMode(motors[i].pinB, OUTPUT);
  }
  for (byte i = 0; i < 3; i++) {
    pinMode(sonars[i].trig, OUTPUT);
    pinMode(sonars[i].echo, INPUT);
  }
  pinMode(LED, OUTPUT);
  pinMode(ir.pin, INPUT);
  setMotors(0, 0);

  Serial.begin(115200);
  Wire.begin();
  Wire.setWireTimeout(3000, true);
  delay(1000);

  loadGain(&gyro);
  startGyro(&gyro);
  findAxis(&gyro);
  nav.heading = 0;
  turnTo(&gyro, 0);
  waitWithGyro(500);
  nav.straightStartMs = millis();
}

void loop() {
  if (!updateGyro(&gyro) && !gyroAlive(&gyro)) stopForever(F("Gyro lost - stopped"));

  if (onBlackTape(&ir)) stopForever(F("Black tape - parked"));

  pingNext(&nav);

  if (sonars[LEFT].hits >= 2) {
    Serial.println(F("Left open -> turn left"));
    stopBeforeTurn();
    turnBy(&nav, 90 * LEFT_SIGN);
    return;
  }

  if (sonars[RIGHT].hits >= 2) {
    Serial.println(F("Right open -> turn right"));
    stopBeforeTurn();
    turnBy(&nav, -90 * LEFT_SIGN);
    return;
  }

  if (sonars[FRONT].hits < 2) {
    driveStraight(&nav, &gyro);
    return;
  }

  brake();
  waitWithGyro(300);
  sonars[FRONT].hits = 0;

  if (sideOpen(&sonars[LEFT])) turnBy(&nav, 90 * LEFT_SIGN);
  else if (sideOpen(&sonars[RIGHT])) turnBy(&nav, -90 * LEFT_SIGN);
  else {
    Serial.println(F("Dead end -> U-turn"));
    turnBy(&nav, 180 * LEFT_SIGN);
  }
}
