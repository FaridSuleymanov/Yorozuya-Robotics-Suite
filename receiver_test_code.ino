#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>

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

RF24 radio(2, 3); // CE, CSN
const byte address[6] = "42424";

// Joystick deadzone
#define JOYSTICK_DEADZONE 50

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
}

void loop() {
  unsigned long currentTime = micros();

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
        Serial.println("relay off");
      } else {
        servo1.write(90);
        servo2.write(90);
        Serial.println("relay on");
      }
    } else {
      if (data.mapX > JOYSTICK_DEADZONE) {
        Serial.println("Joystick Right");
        digitalWrite(dirPin2, LOW); // Rotate left
        stepMotor(stepPin2, speedX);
      } else if (data.mapX < -JOYSTICK_DEADZONE) {
        Serial.println("Joystick Left");
        digitalWrite(dirPin2, HIGH); // Rotate right
        stepMotor(stepPin2, speedX);
      }

      if (data.mapY > JOYSTICK_DEADZONE) {
        Serial.println("Joystick Down");
        digitalWrite(dirPin0, LOW); // Rotate up left motor
        digitalWrite(dirPin1, LOW); // Rotate up right motor
        digitalWrite(dirPin3, LOW); // Rotate fourth motor
        stepMotor(stepPin0, speedY);
        stepMotor(stepPin1, speedY);
        stepMotor(stepPin3, speedY);
      } else if (data.mapY < -JOYSTICK_DEADZONE) {
        Serial.println("Joystick Up");
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
