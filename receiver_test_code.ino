#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>
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
}

void loop() {
  if (radio.available()) {
    radio.read(&data, sizeof(Data_Package));
    digitalWrite(camera_relay, HIGH); //turn on camera relay
//    if (data.megumin_button == LOW) {
//      //turn on alarm lights
//      digitalWrite(relay_Pin, HIGH);
//      Serial.print("relay on");
//      Serial.print("\n");
//    }
//    if (data.megumin_button == HIGH) {
//      //turn off alarm
//      digitalWrite(relay_Pin, LOW);
//      Serial.print("relay off");
//      Serial.print("\n");
//    }
// One axis movement at a time
    if (((data.mapY <= 512) && (data.mapY > 50)) && ((data.mapX >= -50) && (data.mapX <= 50)))
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
  else if (((data.mapX <= 512) && (data.mapX > 50)) && ((data.mapY >= -50) && (data.mapY <= 50)))
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
  }
  else if (((data.mapY >= -512) && (data.mapY < -50)) && ((data.mapX >= -50) && (data.mapX <= 50)))
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
  else if (((data.mapX >= -512) && (data.mapX < -50)) && ((data.mapY >= -50) && (data.mapY <= 50)))
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
  }
// Two axis movement at the same time
  else if (((data.mapY <= 512) && (data.mapY > 50)) && ((data.mapX <= 512) && (data.mapX > 50)))
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
  else if (((data.mapY <= 512) && (data.mapY > 50)) && ((data.mapX >= -512) && (data.mapX < -50)))
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
  else if (((data.mapY >= -512) && (data.mapY < -50)) && ((data.mapX <= 512) && (data.mapX > 50)))
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
  else if (((data.mapY >= -512) && (data.mapY < -50)) && ((data.mapX >= -512) && (data.mapX < -50)))
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
  }
  else if (((data.mapX >= -50) && (data.mapX <= 50)) && ((data.mapY >= -50) && (data.mapY <= 50)) && (data.megumin_button == LOW))
  {
    servo1.write(90);
    servo2.write(90);
    Serial.print("\n");
    digitalWrite(control,LOW); // turn the MOSFET Switch ON
  }
  }
}
