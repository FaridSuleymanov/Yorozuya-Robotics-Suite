#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu4(0x69);  // sensor on fourth stepper output
MPU6050 mpuR(0x68);  // sensor on second axis right output

// ---- Stepper pins ----
const int stepPin4 = 24;  // fourth stepper motor
const int dirPin4  = 25;
const int enable4  = 26;

const int stepPinR = 9;   // second axis second stepper right
const int dirPinR  = 8;
const int enableR  = 10;

// ---- Steps per degree (includes gearbox ratios) ----
const double stepsPerDegree4 = 3.99;  // fourth stepper (4:1 reduction)
const double stepsPerDegreeR = 4.99;  // second axis right (8:1 reduction)

// ---- PID constants ----
const double Kp = 2.0;
const double Ki = 0.2;
const double Kd = 0.05;

// ---- Control variables ----
double targetAngle4 = 0.0, targetAngleR = 0.0;
double integral4 = 0.0, integralR = 0.0;
double prevError4 = 0.0, prevErrorR = 0.0;

unsigned long lastUpdate = 0;
const unsigned long loopIntervalMs = 10;  // 100 Hz

void setup() {
  Serial.begin(115200);
  Wire.begin();

  mpu4.initialize();
  mpuR.initialize();

  pinMode(stepPin4, OUTPUT);
  pinMode(dirPin4, OUTPUT);
  pinMode(enable4, OUTPUT);
  pinMode(stepPinR, OUTPUT);
  pinMode(dirPinR, OUTPUT);
  pinMode(enableR, OUTPUT);

  digitalWrite(enable4, LOW);
  digitalWrite(enableR, LOW);

  delay(1000);  // let sensors stabilize
  targetAngle4 = getAngle(mpu4);
  targetAngleR = getAngle(mpuR);
  Serial.print("Initial angle4: "); Serial.println(targetAngle4);
  Serial.print("Initial angleR: "); Serial.println(targetAngleR);
}

void loop() {
  if (millis() - lastUpdate < loopIntervalMs) return;
  lastUpdate = millis();

  stabilizeMotor(mpu4, stepPin4, dirPin4, stepsPerDegree4,
                 targetAngle4, prevError4, integral4);
  stabilizeMotor(mpuR, stepPinR, dirPinR, stepsPerDegreeR,
                 targetAngleR, prevErrorR, integralR);
}

// ---- Helper functions ----
double getAngle(MPU6050 &mpu) {
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);
  double ay_g = ay / 16384.0;
  double az_g = az / 16384.0;
  return -atan2(ay_g, az_g) * 180.0 / PI;
}

void stabilizeMotor(MPU6050 &mpu, int stepPin, int dirPin, double stepsPerDeg,
                    double target, double &prevErr, double &integral) {
  double angle = getAngle(mpu);
  double error = target - angle;
  integral += error * (loopIntervalMs / 1000.0);
  double derivative = (error - prevErr) / (loopIntervalMs / 1000.0);
  prevErr = error;

  double control = Kp * error + Ki * integral + Kd * derivative;

  if (abs(control) < 0.05) return;  // small deadband

  digitalWrite(dirPin, control > 0 ? HIGH : LOW);
  int steps = abs(control) * stepsPerDeg;
  steps = constrain(steps, 0, 50);  // limit burst
  for (int i = 0; i < steps; ++i) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(400);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(400);
  }
}
