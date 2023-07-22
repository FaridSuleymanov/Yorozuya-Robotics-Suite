#include <Wire.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <MPU6050_tockn.h>
#include "CytronMotorDriver.h"
// Configure the motor driver pins.
CytronMD motor1(PWM_DIR, 3, 22);  // PWM 1 = Pin 3 (AN1), DIR 1 = Pin 22 (IN1), left motor
CytronMD motor2(PWM_DIR, 9, 23); // PWM 2 = Pin 9 (AN2), DIR 2 = Pin 23 (IN2), right motor

#define lightsrelay_Pin 8
#define alarmrelay_Pin 11

RF24 radio(49, 53); // CE, CSN was 1, 2

const byte address[6] = "42424";

// Max size of this struct is 32 bytes - NRF24L01 buffer limit
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

MPU6050 mpu6050(Wire);

// the setup function runs once when you press reset or power the board
void setup() {
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  //initialize digital relay pins as an output
  pinMode(lightsrelay_Pin, OUTPUT);
  pinMode(alarmrelay_Pin, OUTPUT);
  //start radio
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
  Serial.begin(9600);
}

void loop() {
  if (radio.available()) {
    radio.read(&data, sizeof(Data_Package));
    mpu6050.update();
    float angle = mpu6050.getAngleZ();
    // map the angle (-90 to 90) to a motor speed (-255 to 255)
    int motorSpeedAdjustment = map(angle, -90, 90, -200, 200);
    // Control lights
    if (data.lights_buttonstater == LOW) {
      digitalWrite(lightsrelay_Pin, HIGH);
      Serial.print("lights on");
      Serial.print("\n");
    }
    if (data.lights_buttonstater == HIGH) {
      digitalWrite(lightsrelay_Pin, LOW);
      Serial.print("lights off");
      Serial.print("\n");
    }
    // Control alarm
    if (data.alarm_buttonstater == LOW) {
      digitalWrite(alarmrelay_Pin, HIGH);
      Serial.print("alarm on");
      Serial.print("\n");
    }
    if (data.alarm_buttonstater == HIGH) {
      digitalWrite(alarmrelay_Pin, LOW);
      Serial.print("alarm off");
      Serial.print("\n");
    }
    //left joystick right    
    if (((data.map2X >= -512) && (data.map2X < -20)) && ((data.map2Y >= -20) && (data.map2Y <= 20)))
    {
      Serial.print("left joystick right");
      Serial.print("\n");
      motor1.setSpeed(128); //move forward left engine
      motor2.setSpeed(-128); //move backward right engine
      }
      //left joystick left
    else if (((data.map2X <= 512) && (data.map2X > 20)) && ((data.map2Y >= -20) && (data.map2Y <= 20)))
    {
      Serial.print("left joystick left");
      Serial.print("\n");
      motor1.setSpeed(-128); //move backward left engine
      motor2.setSpeed(128);  //move forward right engine
      }
      //left joystick up
    else if (((data.map2Y <= 512) && (data.map2Y > 20)) && ((data.map2X >= -20) && (data.map2X <= 20)))
    {
      Serial.print("left joystick up");
      Serial.print("\n");
      // Calculate the new speed for each motor
      int leftMotorSpeed = 80 + motorSpeedAdjustment; // Adjusted base speed to 100
      int rightMotorSpeed = 80 - motorSpeedAdjustment; // Adjusted base speed to 100
      // Constrain the speed to 0-128
      leftMotorSpeed = constrain(leftMotorSpeed, 0, 100);
      rightMotorSpeed = constrain(rightMotorSpeed, 0, 100);
      // Move forward with both engines
      motor1.setSpeed(leftMotorSpeed);   // Move forward with left engine
      motor2.setSpeed(rightMotorSpeed);  // Move forward with right engine
  }
  //left joystick down
    else if (((data.map2Y >= -512) && (data.map2Y < -20)) && ((data.map2X >= -20) && (data.map2X <= 20)))
    {
      Serial.print("left joystick down");
      Serial.print("\n");
      motor1.setSpeed(-100);   // Motor 1 runs backward at 50%
      motor2.setSpeed(-100);  // Motor 2 runs backward at 50%
      }
    //left joystick right up
    else if (((data.map2X >= -512) && (data.map2X < -20)) && ((data.map2Y <= 512) && (data.map2Y > 20)))
    {
      Serial.print("left joystick right up");
      Serial.print("\n");
      motor1.setSpeed(100); //move forward left engine
      motor2.setSpeed(80); //move forward right engine
      }
    //left joystick right down
    else if (((data.map2X >= -512) && (data.map2X < -20)) && ((data.map2Y >= -512) && (data.map2Y < -20)))
    {
      Serial.print("left joystick right down");
      Serial.print("\n");
      //move backward left engine
      //move backward right engine
      motor1.setSpeed(-100); //move backward left engine
      motor2.setSpeed(-80); //move backward right engine
      }
    //left joystick left up
    else if (((data.map2X <= 512) && (data.map2X > 20)) && ((data.map2Y <= 512) && (data.map2Y > 20)))
    {
      Serial.print("left joystick right up");
      Serial.print("\n");
      motor1.setSpeed(80); //move forward left engine
      motor2.setSpeed(100); //move forward right engine
      }
    //left joystick left down
    else if (((data.map2X <= 512) && (data.map2X > 20)) && ((data.map2Y >= -512) && (data.map2Y < -20)))
    {
      Serial.print("left joystick right down");
      Serial.print("\n");
      motor1.setSpeed(-80); //move backward left engine
      motor2.setSpeed(-100); //move backward right engine
      }
  //left joystick in middle
  else if (((data.map2X >= -20) && (data.map2X <= 20)) && ((data.map2Y >= -20) && (data.map2Y  <= 20))) {
    Serial.print("left joystick middle");
    Serial.print("\n");
    motor1.setSpeed(0); // left engine stops.
    motor2.setSpeed(0); // right engine stops.
  }
  }
 }
