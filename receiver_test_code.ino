#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>
#include <Wire.h>
#include <MPU6050.h>
#define camera_relay 42 //camera relay Pin
#define control 7

RF24 radio(2, 3); // CE, CSN

const byte address[6] = "42424";
// defines pins numbers for stepper motors
const int stepPin0 = 5; //second axis first stepper
const int dirPin0 = 4; //second axis first stepper
const int enable0 = 6; //second axis first stepper
const int stepPin1 = 9; //second axis second stepper
const int dirPin1 = 8; //second axis second stepper
const int enable1 = 10; //second axis second stepper
const int stepPin2 = 16; //first axis stepper
const int dirPin2 = 15; //first axis stepper
const int enable2 = 14; //first axis stepper
Servo servo1;
Servo servo2;

MPU6050 sensor;
// Initialize previous gyro rate, time and angular displacement
double prevGyroRateX = 0;
unsigned long prevTime = 0;
double prevAngularDisplacement = 0;

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

void setup() {
  servo1.attach(22); //first servo trigger
  servo2.attach(23); //secind servo trigger
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
  // Sets the two pins as Outputs for stepper motor
  pinMode(stepPin0,OUTPUT);
  pinMode(dirPin0,OUTPUT);
  pinMode(stepPin1,OUTPUT);
  pinMode(dirPin1,OUTPUT);
  pinMode(enable0,OUTPUT);
  pinMode(enable1,OUTPUT);
  pinMode(stepPin2,OUTPUT);
  pinMode(dirPin2,OUTPUT);
  digitalWrite(enable0, LOW);
  digitalWrite(enable1, LOW);
  digitalWrite(enable2, LOW);
  if (sensor.testConnection()) {
    Serial.println("MPU6050 connection successful");
  } else {
    Serial.println("MPU6050 connection failed");
  }
}

void loop() {
  if (radio.available()) {
    radio.read(&data, sizeof(Data_Package));
    digitalWrite(camera_relay, HIGH); //turn on camera relay
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    // Read raw accel/gyro measurements from device
    sensor.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
    // Convert gyroscope data to degrees/s
    double gyroRateX = gx / 131.0;
  
    // Calculate the time elapsed since the last measurement
    unsigned long currentTime = micros();
    double deltaTime = (currentTime - prevTime) / 1000000.0; // Convert to seconds
  
    // Calculate the angular acceleration
    double angularAccelerationX = (gyroRateX - prevGyroRateX) / deltaTime;
  
    // Store the current gyro rate and time for the next loop iteration
    prevGyroRateX = gyroRateX;
    prevTime = currentTime;
  
    // Calculate angular displacements
    double angularDisplacement = (0.5 * angularAccelerationX * (deltaTime * deltaTime)) * (180 / 3.14);
  
    // Calculate weighted average of current and previous angular displacements
    double smoothedAngularDisplacement = 0.7 * angularDisplacement + 0.3 * prevAngularDisplacement;
  
    // Store the current angular displacement for the next loop iteration
    prevAngularDisplacement = angularDisplacement;
  
    // Calculate number of steps to counter angular displacement
    double numberofsteps = (smoothedAngularDisplacement / 360.0) * 200;
    numberofsteps = max(numberofsteps, 1); // Ensure at least one step
  
    // Calculate delay between steps
    double delayTime = (deltaTime / abs(numberofsteps)) * 1000;
    delayTime = constrain(delayTime, 500, 2000); // Ensure delay time is between 500 and 2000 microseconds
  
    Serial.print("Gyro rate:");
    Serial.println(gyroRateX);
// One axis movement at a time
    if (((data.mapY <= 512) && (data.mapY > 50)) && ((data.mapX >= -50) && (data.mapX <= 50))) // Joystick down
  {
    servo1.write(90);
    servo2.write(90);
    Serial.print("\n");
    digitalWrite(dirPin0,LOW);//Rotate up
    for(int x = 0; x < 4; x++)
    {
      digitalWrite(stepPin0,HIGH);
      delayMicroseconds(1000);
      // Change delay for changing speed of motor
      //500 = max fast
      //2000 = max slow
      digitalWrite(stepPin0,LOW);
      delayMicroseconds(1000);
    }
    digitalWrite(dirPin1,HIGH);//Rotate up
    for(int x = 0; x < 4; x++)
    {
      digitalWrite(stepPin1,HIGH);
      delayMicroseconds(1000);
      // Change delay for changing speed of motor
      //500 = max fast
      //2000 = max slow
      digitalWrite(stepPin1,LOW);
      delayMicroseconds(1000);
    }
  }
  else if (((data.mapX <= 512) && (data.mapX > 50)) && ((data.mapY >= -50) && (data.mapY <= 50))) // Joystick right
  {
    servo1.write(90);
    servo2.write(90);
    Serial.print("\n");
    digitalWrite(dirPin2,HIGH);//Rotate right
    for(int x = 0; x < 4; x++)
    {
      digitalWrite(stepPin2,HIGH);
      delayMicroseconds(2000);
      // Change delay for changing speed of motor
      //500 = max fast
      //2000 = max slow
      digitalWrite(stepPin2,LOW);
      delayMicroseconds(2000);
    }
    if (gyroRateX > 5.0) {
    // Code to execute if a is positive
    digitalWrite(dirPin0,HIGH); //Rotate down
    double adjustedDelayTime = delayTime * 0.7; // Reduce delay time by 20%
    for(int x = 0; x < abs(numberofsteps); x++) // controls the number of steps
    {
      digitalWrite(stepPin0,HIGH);
      delayMicroseconds(adjustedDelayTime); // controls the delay between steps
      digitalWrite(stepPin0,LOW);
      delayMicroseconds(adjustedDelayTime); // controls the delay between steps
    }
    digitalWrite(dirPin1,LOW); //Rotate down
    for(int x = 0; x < abs(numberofsteps); x++) // controls the number of steps
    {
      digitalWrite(stepPin1,HIGH);
      delayMicroseconds(adjustedDelayTime); // controls the delay between steps
      digitalWrite(stepPin1,LOW);
      delayMicroseconds(adjustedDelayTime); // controls the delay between steps
    }
  }
  else if (gyroRateX < -10.0) {
    digitalWrite(dirPin0,LOW); //Rotate up
    double adjustedDelayTime = delayTime * 0.8; // Reduce delay time by 20%
    for(int x = 0; x < abs(numberofsteps); x++) // controls the number of steps
    {
      digitalWrite(stepPin0,HIGH);
      delayMicroseconds(adjustedDelayTime); // controls the delay between steps
      digitalWrite(stepPin0,LOW);
      delayMicroseconds(adjustedDelayTime); // controls the delay between steps
    }
    digitalWrite(dirPin1,HIGH); //Rotate up
    for(int x = 0; x < abs(numberofsteps); x++) // controls the number of steps
    {
      digitalWrite(stepPin1,HIGH);
      delayMicroseconds(adjustedDelayTime); // controls the delay between steps
      digitalWrite(stepPin1,LOW);
      delayMicroseconds(adjustedDelayTime); // controls the delay between steps
    }
  }
  }
  else if (((data.mapY >= -512) && (data.mapY < -50)) && ((data.mapX >= -50) && (data.mapX <= 50))) // Joystick up
  {
    servo1.write(90);
    servo2.write(90);
    Serial.print("\n");
    digitalWrite(dirPin0,HIGH);//Rotate down
    for(int x = 0; x < 4; x++)
    {
      digitalWrite(stepPin0,HIGH);
      delayMicroseconds(1000);
      // Change delay for changing speed of motor
      //500 = max fast
      //2000 = max slow
      digitalWrite(stepPin0,LOW);
      delayMicroseconds(1000);
    }
    digitalWrite(dirPin1,LOW);//Rotate down
    for(int x = 0; x < 4; x++)
    {
      digitalWrite(stepPin1,HIGH);
      delayMicroseconds(1000);
      // Change delay for changing speed of motor
      //500 = max fast
      //2000 = max slow
      digitalWrite(stepPin1,LOW);
      delayMicroseconds(1000);
    }
  }
  else if (((data.mapX >= -512) && (data.mapX < -50)) && ((data.mapY >= -50) && (data.mapY <= 50))) // Joystick left
  {
    servo1.write(90);
    servo2.write(90);
    Serial.print("\n");
    digitalWrite(dirPin2,LOW);//Rotate left
    for(int x = 0; x < 4; x++)
    {
      digitalWrite(stepPin2,HIGH);
      delayMicroseconds(2000);
      // Change delay for changing speed of motor
      //500 = max fast
      //2000 = max slow
      digitalWrite(stepPin2,LOW);
      delayMicroseconds(2000);
    }
    if (gyroRateX > 5.0) {
    // Code to execute if a is positive
    digitalWrite(dirPin0,HIGH); //Rotate down
    double adjustedDelayTime = delayTime * 0.7; // Reduce delay time by 20%
    for(int x = 0; x < abs(numberofsteps); x++) // controls the number of steps
    {
      digitalWrite(stepPin0,HIGH);
      delayMicroseconds(adjustedDelayTime); // controls the delay between steps
      digitalWrite(stepPin0,LOW);
      delayMicroseconds(adjustedDelayTime); // controls the delay between steps
    }
    digitalWrite(dirPin1,LOW); //Rotate down
    for(int x = 0; x < abs(numberofsteps); x++) // controls the number of steps
    {
      digitalWrite(stepPin1,HIGH);
      delayMicroseconds(adjustedDelayTime); // controls the delay between steps
      digitalWrite(stepPin1,LOW);
      delayMicroseconds(adjustedDelayTime); // controls the delay between steps
    }
  }
  else if (gyroRateX < -10.0) {
    digitalWrite(dirPin0,LOW); //Rotate up
    double adjustedDelayTime = delayTime * 0.8; // Reduce delay time by 20%
    for(int x = 0; x < abs(numberofsteps); x++) // controls the number of steps
    {
      digitalWrite(stepPin0,HIGH);
      delayMicroseconds(adjustedDelayTime); // controls the delay between steps
      digitalWrite(stepPin0,LOW);
      delayMicroseconds(adjustedDelayTime); // controls the delay between steps
    }
    digitalWrite(dirPin1,HIGH); //Rotate up
    for(int x = 0; x < abs(numberofsteps); x++) // controls the number of steps
    {
      digitalWrite(stepPin1,HIGH);
      delayMicroseconds(adjustedDelayTime); // controls the delay between steps
      digitalWrite(stepPin1,LOW);
      delayMicroseconds(adjustedDelayTime); // controls the delay between steps
    }
  }
  }
// Two axis movement at the same time
  else if (((data.mapY <= 512) && (data.mapY > 50)) && ((data.mapX <= 512) && (data.mapX > 50))) // Joystick right down
  {
    servo1.write(90);
    servo2.write(90);
    Serial.print("\n");
    digitalWrite(dirPin2,HIGH);//Rotate right
    for(int x = 0; x < 4; x++)
    {
      digitalWrite(stepPin2,HIGH);
      delayMicroseconds(2000);
      // Change delay for changing speed of motor
      //500 = max fast
      //2000 = max slow
      digitalWrite(stepPin2,LOW);
      delayMicroseconds(2000);
    }
    digitalWrite(dirPin0,LOW);//Rotate up
    for(int x = 0; x < 4; x++)
    {
      digitalWrite(stepPin0,HIGH);
      delayMicroseconds(1000);
      // Change delay for changing speed of motor
      //500 = max fast
      //2000 = max slow
      digitalWrite(stepPin0,LOW);
      delayMicroseconds(1000);
    }
    digitalWrite(dirPin1,HIGH);//Rotate up
    for(int x = 0; x < 4; x++)
    {
      digitalWrite(stepPin1,HIGH);
      delayMicroseconds(1000);
      // Change delay for changing speed of motor
      //500 = max fast
      //2000 = max slow
      digitalWrite(stepPin1,LOW);
      delayMicroseconds(1000);
    }
  }
  else if (((data.mapY <= 512) && (data.mapY > 50)) && ((data.mapX >= -512) && (data.mapX < -50))) // Joystick left down
  {
    servo1.write(90);
    servo2.write(90);
    Serial.print("\n");
    digitalWrite(dirPin2,LOW);//Rotate left
    for(int x = 0; x < 4; x++)
    {
      digitalWrite(stepPin2,HIGH);
      delayMicroseconds(2000);
      // Change delay for changing speed of motor
      //500 = max fast
      //2000 = max slow
      digitalWrite(stepPin2,LOW);
      delayMicroseconds(2000);
    }
    digitalWrite(dirPin0,LOW);//Rotate up
    for(int x = 0; x < 4; x++)
    {
      digitalWrite(stepPin0,HIGH);
      delayMicroseconds(1000);
      // Change delay for changing speed of motor
      //500 = max fast
      //2000 = max slow
      digitalWrite(stepPin0,LOW);
      delayMicroseconds(1000);
    }
    digitalWrite(dirPin1,HIGH);//Rotate up
    for(int x = 0; x < 4; x++)
    {
      digitalWrite(stepPin1,HIGH);
      delayMicroseconds(1000);
      // Change delay for changing speed of motor
      //500 = max fast
      //2000 = max slow
      digitalWrite(stepPin1,LOW);
      delayMicroseconds(1000);
    }
  }
  else if (((data.mapY >= -512) && (data.mapY < -50)) && ((data.mapX <= 512) && (data.mapX > 50))) // Joystick right up
  {
    servo1.write(90);
    servo2.write(90);
    Serial.print("\n");
    digitalWrite(dirPin2,HIGH);//Rotate right
    for(int x = 0; x < 4; x++)
    {
      digitalWrite(stepPin2,HIGH);
      delayMicroseconds(2000);
      // Change delay for changing speed of motor
      //500 = max fast
      //2000 = max slow
      digitalWrite(stepPin2,LOW);
      delayMicroseconds(2000);
    }
    digitalWrite(dirPin0,HIGH);//Rotate down
    for(int x = 0; x < 4; x++)
    {
      digitalWrite(stepPin0,HIGH);
      delayMicroseconds(1000);
      // Change delay for changing speed of motor
      //500 = max fast
      //2000 = max slow
      digitalWrite(stepPin0,LOW);
      delayMicroseconds(1000);
    }
    digitalWrite(dirPin1,LOW);//Rotate down
    for(int x = 0; x < 4; x++)
    {
      digitalWrite(stepPin1,HIGH);
      delayMicroseconds(1000);
      // Change delay for changing speed of motor
      //500 = max fast
      //2000 = max slow
      digitalWrite(stepPin1,LOW);
      delayMicroseconds(1000);
    }
  }
  else if (((data.mapY >= -512) && (data.mapY < -50)) && ((data.mapX >= -512) && (data.mapX < -50))) // Joystick left up
  {
    servo1.write(90);
    servo2.write(90);
    Serial.print("\n");
    digitalWrite(dirPin2,LOW);//Rotate left
    for(int x = 0; x < 4; x++)
    {
      digitalWrite(stepPin2,HIGH);
      delayMicroseconds(2000);
      // Change delay for changing speed of motor
      //500 = max fast
      //2000 = max slow
      digitalWrite(stepPin2,LOW);
      delayMicroseconds(2000);
    }
    digitalWrite(dirPin0,HIGH);//Rotate down
    for(int x = 0; x < 4; x++)
    {
      digitalWrite(stepPin0,HIGH);
      delayMicroseconds(1000);
      // Change delay for changing speed of motor
      //500 = max fast
      //2000 = max slow
      digitalWrite(stepPin0,LOW);
      delayMicroseconds(1000);
    }
    digitalWrite(dirPin1,LOW);//Rotate down
    for(int x = 0; x < 4; x++)
    {
      digitalWrite(stepPin1,HIGH);
      delayMicroseconds(1000);
      // Change delay for changing speed of motor
      //500 = max fast
      //2000 = max slow
      digitalWrite(stepPin1,LOW);
      delayMicroseconds(1000);
    }
  }
  else if (((data.mapX >= -50) && (data.mapX <= 50)) && ((data.mapY >= -50) && (data.mapY <= 50)) && (data.megumin_button == HIGH))
  {
    servo1.write(0);
    servo2.write(180);
    Serial.print("relay off");
    Serial.print("\n");
    digitalWrite(control,LOW); // turn the MOSFET Switch ON
    if (gyroRateX > 5.0) {
    // Code to execute if a is positive
    digitalWrite(dirPin0,HIGH); //Rotate down
    double adjustedDelayTime = delayTime * 0.7; // Reduce delay time by 20%
    for(int x = 0; x < abs(numberofsteps); x++) // controls the number of steps
    {
      digitalWrite(stepPin0,HIGH);
      delayMicroseconds(adjustedDelayTime); // controls the delay between steps
      digitalWrite(stepPin0,LOW);
      delayMicroseconds(adjustedDelayTime); // controls the delay between steps
    }
    digitalWrite(dirPin1,LOW); //Rotate down
    for(int x = 0; x < abs(numberofsteps); x++) // controls the number of steps
    {
      digitalWrite(stepPin1,HIGH);
      delayMicroseconds(adjustedDelayTime); // controls the delay between steps
      digitalWrite(stepPin1,LOW);
      delayMicroseconds(adjustedDelayTime); // controls the delay between steps
    }
  }
  else if (gyroRateX < -10.0) {
    digitalWrite(dirPin0,LOW); //Rotate up
    double adjustedDelayTime = delayTime * 0.8; // Reduce delay time by 20%
    for(int x = 0; x < abs(numberofsteps); x++) // controls the number of steps
    {
      digitalWrite(stepPin0,HIGH);
      delayMicroseconds(adjustedDelayTime); // controls the delay between steps
      digitalWrite(stepPin0,LOW);
      delayMicroseconds(adjustedDelayTime); // controls the delay between steps
    }
    digitalWrite(dirPin1,HIGH); //Rotate up
    for(int x = 0; x < abs(numberofsteps); x++) // controls the number of steps
    {
      digitalWrite(stepPin1,HIGH);
      delayMicroseconds(adjustedDelayTime); // controls the delay between steps
      digitalWrite(stepPin1,LOW);
      delayMicroseconds(adjustedDelayTime); // controls the delay between steps
    }
  }
  }
  else if (((data.mapX >= -50) && (data.mapX <= 50)) && ((data.mapY >= -50) && (data.mapY <= 50)) && (data.megumin_button == LOW))
  {
    servo1.write(90);
    servo2.write(90);
    Serial.print("\n");
    digitalWrite(control,LOW); // turn the MOSFET Switch ON
    if (gyroRateX > 5.0) {
    // Code to execute if a is positive
    digitalWrite(dirPin0,HIGH); //Rotate down
    double adjustedDelayTime = delayTime * 0.7; // Reduce delay time by 20%
    for(int x = 0; x < abs(numberofsteps); x++) // controls the number of steps
    {
      digitalWrite(stepPin0,HIGH);
      delayMicroseconds(adjustedDelayTime); // controls the delay between steps
      digitalWrite(stepPin0,LOW);
      delayMicroseconds(adjustedDelayTime); // controls the delay between steps
    }
    digitalWrite(dirPin1,LOW); //Rotate down
    for(int x = 0; x < abs(numberofsteps); x++) // controls the number of steps
    {
      digitalWrite(stepPin1,HIGH);
      delayMicroseconds(adjustedDelayTime); // controls the delay between steps
      digitalWrite(stepPin1,LOW);
      delayMicroseconds(adjustedDelayTime); // controls the delay between steps
    }
  }
  else if (gyroRateX < -10.0) {
    digitalWrite(dirPin0,LOW); //Rotate up
    double adjustedDelayTime = delayTime * 0.8; // Reduce delay time by 20%
    for(int x = 0; x < abs(numberofsteps); x++) // controls the number of steps
    {
      digitalWrite(stepPin0,HIGH);
      delayMicroseconds(adjustedDelayTime); // controls the delay between steps
      digitalWrite(stepPin0,LOW);
      delayMicroseconds(adjustedDelayTime); // controls the delay between steps
    }
    digitalWrite(dirPin1,HIGH); //Rotate up
    for(int x = 0; x < abs(numberofsteps); x++) // controls the number of steps
    {
      digitalWrite(stepPin1,HIGH);
      delayMicroseconds(adjustedDelayTime); // controls the delay between steps
      digitalWrite(stepPin1,LOW);
      delayMicroseconds(adjustedDelayTime); // controls the delay between steps
    }
  }
  }
  }
}
