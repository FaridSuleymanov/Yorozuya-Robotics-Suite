#include <Wire.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include "CytronMotorDriver.h"

// [ADDED FOR MPU6050]
#include "MPU6050_light.h"

// [ADDED FOR GPS]
#include <TinyGPSPlus.h>

// [ADDED FOR COMPASS]
// Use the mprograms QMC5883LCompass library (install via Library Manager or add ZIP)
#include <QMC5883LCompass.h>

// Create a global compass object.
QMC5883LCompass compass;

// Configure the motor driver pins.
CytronMD motor1(PWM_DIR, 3, 22);  // Left motor (PWM on Pin 3, DIR on Pin 22)
CytronMD motor2(PWM_DIR, 9, 23);  // Right motor (PWM on Pin 9, DIR on Pin 23)

#define lightsrelay_Pin 8
#define alarmrelay_Pin 11

// Dual Relay (Active LOW) for motor power lines
#define MOTOR_RELAY_IN1_PIN 24
#define MOTOR_RELAY_IN2_PIN 25

//------------------ Switch --------------------------
// Engine switch (to enable/disable motor relays)
// (Using internal pull-up: Connect one terminal of the switch to pin 26 and the other to GND)
#define MOTOR_SWITCH_PIN    26

// ------------------ Encoder Setup ------------------
#define ENC1_A_PIN  2
#define ENC1_B_PIN  4
#define ENC2_A_PIN  18
#define ENC2_B_PIN  19

volatile long encoder1Count = 0;
volatile long encoder2Count = 0;
const float ENCODER_PPR = 400.0f;                // Pulses per revolution
const float DEGREES_PER_PULSE = 360.0f / ENCODER_PPR; // Degrees per pulse

RF24 radio(49, 53); // CE, CSN on pins 49 and 53
const byte address[6] = "42424";

// Data structure for radio (joystick) data:
struct Data_Package {
  int mapX;
  int mapY;
  boolean megumin_button;
  int map2X;
  int map2Y;
  boolean lights_buttonstater;
  boolean alarm_buttonstater;
};
Data_Package data;

// [ADDED FOR MPU6050]
MPU6050 mpu(Wire);

// [ADDED FOR GPS]
TinyGPSPlus gps;

// Low-pass filter variables for gyro:
float filteredX = 0.0;
float filteredY = 0.0;
float filteredZ = 0.0;
const float alpha = 0.95;  // Filtering factor

// --- Calibration / Mapping Constants ---
const int DEADZONE    = 50;    // Joystick values between -50 and 50 are ignored.
const int MIN_SPEED   = 20;    // Minimum motor power when outside the deadzone.
const int MAX_SPEED   = 80;    // Maximum motor power output.
const int JOYSTICK_MAX_UP   = 250;   // Observed maximum upward value.
const int JOYSTICK_MAX_LEFT = 250;   // Observed maximum leftward value.

///////////////////////////////////////////////////////
//              Interrupt Service Routines (ISRs)
///////////////////////////////////////////////////////
void ISR_encoder1A() {
  static bool lastStateB1 = LOW;
  bool currentStateB1 = digitalRead(ENC1_B_PIN);
  bool currentStateA1 = digitalRead(ENC1_A_PIN);
  if (currentStateA1 != currentStateB1) {
    encoder1Count++;
  } else {
    encoder1Count--;
  }
}

void ISR_encoder2A() {
  static bool lastStateA2 = LOW;
  static bool lastStateB2 = LOW;
  bool currentStateA2 = digitalRead(ENC2_A_PIN);
  bool currentStateB2 = digitalRead(ENC2_B_PIN);
  if (currentStateA2 != lastStateA2) {
    if (currentStateA2 == currentStateB2) {
      encoder2Count++;
    } else {
      encoder2Count--;
    }
  }
  lastStateA2 = currentStateA2;
}

// ---- NEW helper  -----------------------------------------
void updateMotorRelaysFromSwitch() {
  /*
     INPUT_PULLUP means:
       • switch OPEN  ➜ pin HIGH
       • switch CLOSED ➜ pin LOW   (we’ll call this “ON”)
     The motor-power relays are ACTIVE-LOW (LOW = energised). ❶❷
     When the switch is ON we want to CUT power, so we drive the relay
     inputs HIGH; when the switch is OFF we allow power by driving LOW.
  */
  bool engineSwitchOn = (digitalRead(MOTOR_SWITCH_PIN) == HIGH); // closed → ON
  if (engineSwitchOn) {                 // cut power
    digitalWrite(MOTOR_RELAY_IN1_PIN, HIGH);
    digitalWrite(MOTOR_RELAY_IN2_PIN, HIGH);
  } else {                              // supply power
    digitalWrite(MOTOR_RELAY_IN1_PIN, LOW);
    digitalWrite(MOTOR_RELAY_IN2_PIN, LOW);
  }
}


///////////////////////////////////////////////////////
//                      Setup
///////////////////////////////////////////////////////
void setup() {
  Wire.begin();
  
  // Initialize compass (QMC5883L)
  compass.init();
  // (Optionally adjust calibration or sampling rate here)
  
  // Encoders
  pinMode(ENC1_A_PIN, INPUT_PULLUP);
  pinMode(ENC1_B_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC1_A_PIN), ISR_encoder1A, CHANGE);
  
  pinMode(ENC2_A_PIN, INPUT_PULLUP);
  pinMode(ENC2_B_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC2_A_PIN), ISR_encoder2A, CHANGE);

  Serial.begin(115200);
  Serial.println("Dual RT3806-AB-400N Encoder system initialized on Arduino Mega.");
  
  // Initialize relay pins as outputs
  pinMode(lightsrelay_Pin, OUTPUT);
  pinMode(alarmrelay_Pin, OUTPUT);
  pinMode(MOTOR_RELAY_IN1_PIN, OUTPUT);
  pinMode(MOTOR_RELAY_IN2_PIN, OUTPUT);
  
  // Motor switch setup (using internal pull-up)
  pinMode(MOTOR_SWITCH_PIN, INPUT_PULLUP);
  
  // Radio setup
  radio.begin();
  radio.setDataRate(RF24_1MBPS);
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
  
  digitalWrite(alarmrelay_Pin, HIGH);
  digitalWrite(lightsrelay_Pin, LOW);

  updateMotorRelaysFromSwitch();   // set correct power state at boot
  
  // MPU6050 initialization
  byte status = mpu.begin();
  if (status != 0) {
    Serial.print("MPU6050 init failed (code ");
    Serial.print(status);
    Serial.println(")");
  } else {
    Serial.println("MPU6050 initialized successfully!");
    mpu.calcOffsets(); 
    Serial.println("MPU6050 offsets calculated.");
  }
  
  // GPS initialization
  Serial2.begin(9600);
  Serial.println("GPS Serial2 started at 9600 baud");
}

///////////////////////////////////////////////////////
//                      Loop
///////////////////////////////////////////////////////
void loop() {
  updateMotorRelaysFromSwitch();
  // --- Update MPU6050 data and apply low-pass filter ---
  mpu.update();
  float rawX = mpu.getGyroX();
  float rawY = mpu.getGyroY();
  float rawZ = mpu.getGyroZ();
  filteredX = alpha * filteredX + (1 - alpha) * rawX;
  filteredY = alpha * filteredY + (1 - alpha) * rawY;
  filteredZ = alpha * filteredZ + (1 - alpha) * rawZ;
  
  // --- Process GPS data (if available) ---
  while (Serial2.available() > 0) {
    char c = Serial2.read();
    gps.encode(c);
  }
  
  // --- Read Compass Data ---
  // First update the sensor then get the heading.
  compass.read();
  float heading = compass.getAzimuth();
  if (heading < 0) {
    heading += 360;
  }
  
  // --- Process radio (joystick) data if available ---
  String joyDir = "left joystick middle"; // Default value
  if (radio.available()) {
    radio.read(&data, sizeof(Data_Package));
    
    if (((data.map2X >= -512) && (data.map2X < -DEADZONE)) &&
        ((data.map2Y >= -DEADZONE) && (data.map2Y <= DEADZONE))) {
      int motorSpeed = map(abs(data.map2X), DEADZONE, 512, MIN_SPEED, MAX_SPEED);
      motor1.setSpeed(motorSpeed);
      motor2.setSpeed(-motorSpeed);
      joyDir = "left joystick right";
    }
    else if (((data.map2X <= 512) && (data.map2X > DEADZONE)) &&
             ((data.map2Y >= -DEADZONE) && (data.map2Y <= DEADZONE))) {
      int motorSpeed = map(abs(data.map2X), DEADZONE, JOYSTICK_MAX_LEFT, MIN_SPEED, MAX_SPEED);
      motor1.setSpeed(-motorSpeed);
      motor2.setSpeed(motorSpeed);
      joyDir = "left joystick left";
    }
    else if (((data.map2Y <= 512) && (data.map2Y > DEADZONE)) &&
             ((data.map2X >= -DEADZONE) && (data.map2X <= DEADZONE))) {
      int motorSpeed = map(abs(data.map2Y), DEADZONE, JOYSTICK_MAX_UP, MIN_SPEED, MAX_SPEED);
      motor1.setSpeed(motorSpeed);
      motor2.setSpeed(motorSpeed);
      joyDir = "left joystick up";
    }
    else if (((data.map2Y >= -512) && (data.map2Y < -DEADZONE)) &&
             ((data.map2X >= -DEADZONE) && (data.map2X <= DEADZONE))) {
      int motorSpeed = map(abs(data.map2Y), DEADZONE, 512, MIN_SPEED, MAX_SPEED);
      motor1.setSpeed(-motorSpeed);
      motor2.setSpeed(-motorSpeed);
      joyDir = "left joystick down";
    }
    else if (((data.map2X >= -512) && (data.map2X < -DEADZONE)) &&
             ((data.map2Y <= 512) && (data.map2Y > DEADZONE))) {
      int offset   = map(abs(data.map2X), DEADZONE, 512, 0, 20);
      int throttle = map(abs(data.map2Y), DEADZONE, JOYSTICK_MAX_UP, MIN_SPEED, MAX_SPEED);
      motor1.setSpeed(throttle);
      motor2.setSpeed(throttle - offset);
      joyDir = "left joystick right up";
    }
    else if (((data.map2X >= -512) && (data.map2X < -DEADZONE)) &&
             ((data.map2Y >= -512) && (data.map2Y < -DEADZONE))) {
      int offset   = map(abs(data.map2X), DEADZONE, 512, 0, 20);
      int throttle = map(abs(data.map2Y), DEADZONE, 512, MIN_SPEED, MAX_SPEED);
      motor1.setSpeed(-throttle);
      motor2.setSpeed(-(throttle - offset));
      joyDir = "left joystick right down";
    }
    else if (((data.map2X <= 512) && (data.map2X > DEADZONE)) &&
             ((data.map2Y <= 512) && (data.map2Y > DEADZONE))) {
      int offset   = map(abs(data.map2X), DEADZONE, JOYSTICK_MAX_LEFT, 0, 20);
      int throttle = map(abs(data.map2Y), DEADZONE, JOYSTICK_MAX_UP, MIN_SPEED, MAX_SPEED);
      motor1.setSpeed(throttle - offset);
      motor2.setSpeed(throttle);
      joyDir = "left joystick left up";
    }
    else if (((data.map2X <= 512) && (data.map2X > DEADZONE)) &&
             ((data.map2Y >= -512) && (data.map2Y < -DEADZONE))) {
      int offset   = map(abs(data.map2X), DEADZONE, JOYSTICK_MAX_LEFT, 0, 20);
      int throttle = map(abs(data.map2Y), DEADZONE, 512, MIN_SPEED, MAX_SPEED);
      motor1.setSpeed(-(throttle - offset));
      motor2.setSpeed(-throttle);
      joyDir = "left joystick left down";
    }
    else if (((data.map2X >= -DEADZONE) && (data.map2X <= DEADZONE)) &&
             ((data.map2Y >= -DEADZONE) && (data.map2Y <= DEADZONE))) {
      motor1.setSpeed(0);
      motor2.setSpeed(0);
      joyDir = "left joystick middle";
    }
  }
  
  // --- Encoder readings and speed calculations ---
  static long lastEncoder1Count = 0;
  static long lastEncoder2Count = 0;
  static unsigned long lastTime = millis();
  
  long currentEncoder1Count = encoder1Count;
  long currentEncoder2Count = encoder2Count;
  long deltaCount1 = currentEncoder1Count - lastEncoder1Count;
  long deltaCount2 = currentEncoder2Count - lastEncoder2Count;
  lastEncoder1Count = currentEncoder1Count;
  lastEncoder2Count = currentEncoder2Count;
  
  // Compute elapsed time (in seconds)
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;  // seconds
  lastTime = currentTime;
  
  // Wheel properties:
  const float wheelDiameter = 0.35;           // in meters (35 cm)
  const float wheelCircumference = PI * wheelDiameter; // in meters
  
  // Distance traveled per encoder (in meters)
  float distance1 = ((float)deltaCount1 / ENCODER_PPR) * wheelCircumference;
  float distance2 = ((float)deltaCount2 / ENCODER_PPR) * wheelCircumference;
  
  // Speed in m/s (if dt > 0)
  float speed1 = (dt > 0) ? (distance1 / dt) : 0;
  float speed2 = (dt > 0) ? (distance2 / dt) : 0;
  
  // --- Print comprehensive status ---
  Serial.print("Gyro: X: "); Serial.print(filteredX, 2);
  Serial.print(" Y: "); Serial.print(filteredY, 2);
  Serial.print(" Z: "); Serial.print(filteredZ, 2);
  
  Serial.print(" | GPS: ");
  if (gps.location.isValid()) {
    Serial.print(gps.location.lat(), 6);
    Serial.print(", ");
    Serial.print(gps.location.lng(), 6);
  } else {
    Serial.print("Not Valid");
  }
  
  Serial.print(" | Compass: ");
  Serial.print(heading, 2);
  Serial.print(" deg");
  
  Serial.print(" | Joystick: "); Serial.print(joyDir);
  
  Serial.print(" | Fan Relay: ");
  Serial.print((digitalRead(alarmrelay_Pin) == LOW) ? "OFF" : "ON");
  Serial.print(" | Lights Relay: ");
  Serial.print((digitalRead(lightsrelay_Pin) == LOW) ? "OFF" : "ON");
  
  Serial.print(" | Engine Switch: ");
  Serial.print((digitalRead(MOTOR_SWITCH_PIN) == LOW) ? "ON" : "OFF");
  
  Serial.print(" | Encoder1: ");
  if (deltaCount1 > 0) Serial.print("Backward ");
  else if (deltaCount1 < 0) Serial.print("Forward ");
  Serial.print("Speed: ");
  Serial.print(abs(speed1), 2);
  Serial.print(" m/s");
  
  Serial.print(" | Encoder2: ");
  if (deltaCount2 > 0) Serial.print("Backward ");
  else if (deltaCount2 < 0) Serial.print("Forward ");
  Serial.print("Speed: ");
  Serial.print(abs(speed2), 2);
  Serial.println(" m/s");
}
