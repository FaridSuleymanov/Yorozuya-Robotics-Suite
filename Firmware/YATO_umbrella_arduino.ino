#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>
#include <Wire.h>
#include <MPU6050.h>

// ----- MPU6050 Initialization -----
MPU6050 mpu2(0x69); // MPU6050 for Fourth Stepper (TMC2208)
MPU6050 mpu1(0x68); // MPU6050 for Second Axis Stepper (DM542)

Servo servo1;
Servo servo2;

// Stepper Motor Driver DM542
const int stepPin0 = 5; // second axis first stepper left
const int dirPin0 = 4; // second axis first stepper left
const int enable0 = 6; // second axis first stepper left
const int stepPin1 = 9; // second axis second stepper right
const int dirPin1 = 8; // second axis second stepper right
const int enable1 = 10; // second axis second stepper right
const int stepPin2 = 16; // first axis stepper
const int dirPin2 = 15; // first axis stepper
const int enable2 = 14; // first axis stepper
const int stepPin3 = 24; // fourth stepper motor (TMC2208 Step pin)
const int dirPin3 = 25; // fourth stepper motor (TMC2208 Direction pin)
const int enable3 = 26; // fourth stepper motor (TMC2208 Enable pin)
const int ms1Pin = 27;  // MS1 pin for microstepping
const int ms2Pin = 28;  // MS2 pin for microstepping

// ----- Constants for Step Calculation -----
const double stepsPerDegree3 = 3.99;  
const double stepsPerDegree1 = 4.99;  

// ----- Stepper Drive Timing -----
const unsigned long stepDelayMicros = 4000; 

// ----- Deadband -----
const int deadband = 1; 

bool firstReading2 = true, firstReading1 = true;
double filteredAngle2 = 0.0, filteredAngle1 = 0.0;
const double alpha = 0.1; 
bool fourthStepperCalibrated = false;
bool secondAxisCalibrated = false;

RF24 radio(2, 3); // CE, CSN
const byte address[6] = "42424";

// Joystick deadzone
#define JOYSTICK_DEADZONE 50

// Motors switch
#define MOTOR_SWITCH_PIN 29 

// Data structure for receiving remote data
struct Data_Package {
  int mapX;
  int mapY;
  boolean megumin_button;
  int map2X;
  int map2Y;
  boolean lights_buttonstater;
  boolean alarm_buttonstater;
};
// Create a variable with the above structure
Data_Package data;

void setup() {
  Serial.begin(115200);
  Wire.begin();
    // Setup motor shutdown switch
    pinMode(MOTOR_SWITCH_PIN, INPUT_PULLUP);
    
    // Initialize MPU6050 Sensors
    mpu2.initialize();
    if (!mpu2.testConnection()) {
        Serial.println("MPU6050 #2 (Fourth Stepper) connection failed!");
        while (1);
    }
    Serial.println("MPU6050 #2 connected!");

    mpu1.initialize();
    if (!mpu1.testConnection()) {
        Serial.println("MPU6050 #1 (Second Axis Stepper) connection failed!");
        while (1);
    }
    Serial.println("MPU6050 #1 connected!");
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();

  // Sets the two pins as Outputs for stepper motor
  pinMode(stepPin0, OUTPUT);
  pinMode(dirPin0, OUTPUT);
  pinMode(enable0, OUTPUT);
  pinMode(stepPin1, OUTPUT);
  pinMode(dirPin1, OUTPUT);
  pinMode(enable1, OUTPUT);
  pinMode(stepPin2, OUTPUT);
  pinMode(dirPin2, OUTPUT);
  pinMode(enable2, OUTPUT);
  pinMode(stepPin3, OUTPUT);
  pinMode(dirPin3, OUTPUT);
  pinMode(enable3, OUTPUT);

  // Set microstepping pins as outputs
  pinMode(ms1Pin, OUTPUT);
  pinMode(ms2Pin, OUTPUT);

  // Enable all motors
  digitalWrite(enable0, LOW); // Enable second axis first stepper
  digitalWrite(enable1, LOW); // Enable second axis second stepper
  digitalWrite(enable2, LOW); // Enable first axis stepper
  digitalWrite(enable3, LOW); // Enable fourth stepper motor

  // Initialize servos
  servo1.attach(22); // first servo trigger
  servo2.attach(23); // second servo trigger

  // Set initial microstepping mode (e.g., eighth step)
  setMicrostepping(HIGH, LOW);

  // Calibrate the Fourth Stepper first, then the Second Axis Stepper
    calibrateFourthStepper();
    calibrateSecondAxisStepper();
}

void loop() {
  unsigned long currentTime = micros();
  if (digitalRead(MOTOR_SWITCH_PIN) == LOW) {
    digitalWrite(enable0, HIGH); // Disable second axis first stepper
    digitalWrite(enable1, HIGH); // Disable second axis second stepper
    digitalWrite(enable2, HIGH); // Disable first axis stepper
    digitalWrite(enable3, HIGH); // Disable fourth stepper motor
    Serial.println("All stepper motors DISABLED by switch!");
    delay(100); // Debounce delay
    return; // Skip the rest of the loop
  } else {
    // Ensure motors are enabled if switch is not pressed
    digitalWrite(enable0, LOW);
    digitalWrite(enable1, LOW);
    digitalWrite(enable2, LOW);
    digitalWrite(enable3, LOW);
  }

  // Read data from remote
  if (radio.available()) {
    radio.read(&data, sizeof(Data_Package));

    // Debugging received data
    Serial.print("Received mapX: "); Serial.println(data.mapX);
    Serial.print("Received mapY: "); Serial.println(data.mapY);

    int speedX = map(abs(data.mapX), 0, 512, 10000, 200); // Map speed based on joystick X input
    int speedY = map(abs(data.mapY), 0, 512, 5000, 200); // Map speed based on joystick Y input

    if (((data.mapX >= -JOYSTICK_DEADZONE) && (data.mapX <= JOYSTICK_DEADZONE)) && ((data.mapY >= -JOYSTICK_DEADZONE) && (data.mapY <= JOYSTICK_DEADZONE))) {
      if (data.megumin_button == HIGH) {
        servo1.write(0);
        servo2.write(180);
      } else {
        servo1.write(90);
        servo2.write(90);
      }
    } else {
      if (data.mapX > JOYSTICK_DEADZONE) {
        digitalWrite(dirPin2, LOW); // Rotate left
        stepMotor(stepPin2, speedX);
      } else if (data.mapX < -JOYSTICK_DEADZONE) {
        digitalWrite(dirPin2, HIGH); // Rotate right
        stepMotor(stepPin2, speedX);
      }

      if (data.mapY > JOYSTICK_DEADZONE) {
        digitalWrite(dirPin0, LOW); // Rotate up left motor
        digitalWrite(dirPin1, LOW); // Rotate up right motor
        digitalWrite(dirPin3, LOW); // Rotate fourth motor
        stepMotor(stepPin0, speedY);
        stepMotor(stepPin1, speedY);
        stepMotor(stepPin3, speedY);
      } else if (data.mapY < -JOYSTICK_DEADZONE) {
        digitalWrite(dirPin0, HIGH); // Rotate down left motor
        digitalWrite(dirPin1, HIGH); // Rotate down right motor
        digitalWrite(dirPin3, HIGH); // Rotate fourth motor
        stepMotor(stepPin0, speedY);
        stepMotor(stepPin1, speedY);
        stepMotor(stepPin3, speedY);
      }
    }
  } else {
    Serial.println("No radio data available");
  }
}

// ----- Calibration for Fourth Stepper (TMC2208) -----
void calibrateFourthStepper() {
    int angleInt2, stepsToMove2;
    getCorrectionSteps(mpu2, filteredAngle2, firstReading2, stepsPerDegree3, angleInt2, stepsToMove2);

    Serial.print("Fourth Stepper: Current Angle: ");
    Serial.print(angleInt2);
    Serial.print("°, Steps Needed: ");
    Serial.println(stepsToMove2);

    if (stepsToMove2 > 0) {
        digitalWrite(enable3, LOW); // Ensure stepper stays enabled
        digitalWrite(dirPin3, angleInt2 > 0 ? LOW : HIGH);

        for (int i = 0; i < stepsToMove2; i++) {
            digitalWrite(stepPin3, HIGH);
            delayMicroseconds(stepDelayMicros);
            digitalWrite(stepPin3, LOW);
            delayMicroseconds(stepDelayMicros);
        }
    }

    Serial.println("Fourth Stepper Calibrated");
    fourthStepperCalibrated = true;
}

// ----- Calibration for Second Axis Stepper (DM542) -----
void calibrateSecondAxisStepper() {
    int angleInt1, stepsToMove1;
    getCorrectionSteps(mpu1, filteredAngle1, firstReading1, stepsPerDegree1, angleInt1, stepsToMove1);

    Serial.print("Second Axis Stepper: Current Angle: ");
    Serial.print(angleInt1);
    Serial.print("°, Steps Needed: ");
    Serial.println(stepsToMove1);

    if (stepsToMove1 > 0) {
        digitalWrite(enable1, LOW); // Ensure stepper stays enabled
        digitalWrite(dirPin1, angleInt1 > 0 ? HIGH : LOW);

        for (int i = 0; i < stepsToMove1; i++) {
            digitalWrite(stepPin1, HIGH);
            delayMicroseconds(stepDelayMicros);
            digitalWrite(stepPin1, LOW);
            delayMicroseconds(stepDelayMicros);
        }
    }

    Serial.println("Second Axis Stepper Calibrated");
    secondAxisCalibrated = true;
}

// ----- Get Correction Steps Function -----
void getCorrectionSteps(MPU6050 &mpu, double &filteredAngle, bool &firstReading, double stepsPerDegree, int &angleInt, int &stepsToMove) {
    int16_t ax, ay, az;
    mpu.getAcceleration(&ax, &ay, &az);

    double ax_g = ax / 16384.0;
    double ay_g = ay / 16384.0;
    double az_g = az / 16384.0;

    double angle = -atan2(ay_g, az_g) * 180.0 / PI;

    if (firstReading) {
        filteredAngle = angle;
        firstReading = false;
    } else {
        filteredAngle = alpha * angle + (1 - alpha) * filteredAngle;
    }

    angleInt = round(filteredAngle);

    if (abs(angleInt) < deadband) {
        angleInt = 0;
    }

    int desiredAngle = 0;
    stepsToMove = round(abs(angleInt - desiredAngle) * stepsPerDegree);
}

// ----- Function to Read MPU6050 Angle -----
double getCurrentAngle(MPU6050 &mpu) {
    int16_t ax, ay, az;
    mpu.getAcceleration(&ax, &ay, &az);

    double ax_g = ax / 16384.0;
    double ay_g = ay / 16384.0;
    double az_g = az / 16384.0;

    return -atan2(ay_g, az_g) * 180.0 / PI;
}

void stepMotor(int stepPin, int speed) {
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(speed);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(speed);
}

void setMicrostepping(int ms1, int ms2) {
  digitalWrite(ms1Pin, ms1);
  digitalWrite(ms2Pin, ms2);
}
