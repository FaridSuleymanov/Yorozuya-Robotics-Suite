/*
 * Kagura UGV – main firmware
 *  • nRF24L01 joystick control (low-latency)
 *  • LoRa E32-433T20D telemetry every 10 s
 *  • USB Serial telemetry every 100 ms
 *  • Lights relay (pin 8) follows lights_buttonstater
 *  • Cooling-fan relay renamed coolingfan_Pin (pin 11)
 *
 *  Hardware: Arduino Mega 2560
 */

#include <Wire.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include "CytronMotorDriver.h"
#include "MPU6050_light.h"
#include <TinyGPSPlus.h>
#include <QMC5883LCompass.h>
#include "LoRa_E32.h"

// ────────────────────────── MOTOR DRIVER
CytronMD motor1(PWM_DIR, 3, 22);   // left
CytronMD motor2(PWM_DIR, 9, 23);   // right

// ────────────────────────── RELAYS & IO
#define lightsrelay_Pin       8     // active-LOW (LOW = ON)
#define coolingfan_Pin        11    // active-LOW
#define MOTOR_RELAY_IN1_PIN   24    // active-LOW
#define MOTOR_RELAY_IN2_PIN   25
#define MOTOR_SWITCH_PIN      26    // INPUT_PULLUP (open = OFF)

// ────────────────────────── ENCODERS
#define ENC1_A_PIN 2
#define ENC1_B_PIN 4
#define ENC2_A_PIN 18
#define ENC2_B_PIN 19
const float ENCODER_PPR = 400.0f;             // pulses / rev

// ─────────── LoRa E32 on Serial3
#define LORA_M0_PIN 27
#define LORA_M1_PIN 28
#define LORA_AUX_PIN 29
LoRa_E32 e32(&Serial3, LORA_AUX_PIN, LORA_M0_PIN, LORA_M1_PIN);
bool      loraReady = false;

// ─────────── nRF24 joystick
RF24 radio(49, 53);
const byte address[6] = "42424";
struct Data_Package {
  int  mapX, mapY;
  bool megumin_button;
  int  map2X, map2Y;
  bool lights_buttonstater;
  bool alarm_buttonstater;
};
Data_Package data{};

// ─────────── Sensors
MPU6050 mpu(Wire);
QMC5883LCompass compass;
TinyGPSPlus gps;

// ─────────── Runtime vars
volatile long enc1Count = 0, enc2Count = 0;
float gX=0, gY=0, gZ=0;         // filtered gyro
const float alpha = 0.95;
const int DEADZONE=50, MIN_SPEED=20, MAX_SPEED=80;
const int JOY_MAX_UP=250, JOY_MAX_LEFT=250;

// ───────────────── ISR
void ISR_enc1() { bool b=digitalRead(ENC1_B_PIN);
                  (digitalRead(ENC1_A_PIN)!=b) ? enc1Count++ : enc1Count--; }
void ISR_enc2() { static bool last=LOW;
                  bool a=digitalRead(ENC2_A_PIN), b=digitalRead(ENC2_B_PIN);
                  if(a!=last){ (a==b)? enc2Count++ : enc2Count--; } last=a; }

// ───────────────── helpers
void updateMotorRelays() {
  bool off = digitalRead(MOTOR_SWITCH_PIN)==HIGH;
  digitalWrite(MOTOR_RELAY_IN1_PIN, off?HIGH:LOW);
  digitalWrite(MOTOR_RELAY_IN2_PIN, off?HIGH:LOW);
}

String computeJoyDir(int x,int y){
  if(abs(x)<=DEADZONE && abs(y)<=DEADZONE)      return "middle";
  if(x<-DEADZONE && abs(y)<=DEADZONE)           return "right";
  if(x> DEADZONE && abs(y)<=DEADZONE)           return "left";
  if(y> DEADZONE && abs(x)<=DEADZONE)           return "up";
  if(y<-DEADZONE && abs(x)<=DEADZONE)           return "down";
  return "diag";
}

void driveMotors(int x,int y){
  if((x<-DEADZONE)&&(abs(y)<=DEADZONE)){
      int s=map(-x,DEADZONE,512,MIN_SPEED,MAX_SPEED);
      motor1.setSpeed(s); motor2.setSpeed(-s);
  } else if((x>DEADZONE)&&(abs(y)<=DEADZONE)){
      int s=map(x,DEADZONE,JOY_MAX_LEFT,MIN_SPEED,MAX_SPEED);
      motor1.setSpeed(-s); motor2.setSpeed(s);
  } else if((y>DEADZONE)&&(abs(x)<=DEADZONE)){
      int s=map(y,DEADZONE,JOY_MAX_UP,MIN_SPEED,MAX_SPEED);
      motor1.setSpeed(s); motor2.setSpeed(s);
  } else if((y<-DEADZONE)&&(abs(x)<=DEADZONE)){
      int s=map(-y,DEADZONE,512,MIN_SPEED,MAX_SPEED);
      motor1.setSpeed(-s); motor2.setSpeed(-s);
  } else if(abs(x)<=DEADZONE && abs(y)<=DEADZONE){
      motor1.setSpeed(0); motor2.setSpeed(0);
  }
}

// ───────────────── setup
void setup(){
  Wire.begin(); Serial.begin(115200);

  pinMode(lightsrelay_Pin,OUTPUT);
  pinMode(coolingfan_Pin,OUTPUT);
  pinMode(MOTOR_RELAY_IN1_PIN,OUTPUT); pinMode(MOTOR_RELAY_IN2_PIN,OUTPUT);
  pinMode(MOTOR_SWITCH_PIN,INPUT_PULLUP);
  pinMode(ENC1_A_PIN,INPUT_PULLUP); pinMode(ENC1_B_PIN,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC1_A_PIN),ISR_enc1,CHANGE);
  pinMode(ENC2_A_PIN,INPUT_PULLUP); pinMode(ENC2_B_PIN,INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC2_A_PIN),ISR_enc2,CHANGE);

  digitalWrite(coolingfan_Pin,HIGH);   // relay off (active-LOW)
  updateMotorRelays();

  compass.init();
  if(mpu.begin()==0){ mpu.calcOffsets(); }
  Serial2.begin(9600);

  radio.begin();
  radio.setDataRate(RF24_1MBPS);
  radio.openReadingPipe(0,address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();

  Serial3.begin(9600);
  pinMode(LORA_M0_PIN,OUTPUT); pinMode(LORA_M1_PIN,OUTPUT);
  digitalWrite(LORA_M0_PIN,LOW); digitalWrite(LORA_M1_PIN,LOW);
  loraReady = e32.begin();
}

// ───────────────── loop
void loop(){
  // ── Fast: joystick & lights
  if(radio.available()){
      radio.read(&data,sizeof(data));
      digitalWrite(lightsrelay_Pin, data.lights_buttonstater ? LOW : HIGH); // LOW=ON
      driveMotors(data.map2X, data.map2Y);
  }

  // ── Slow: sensors + telemetry
  static unsigned long lastUsb=0, lastLoRa=0;
  unsigned long now=millis();
  if(now-lastUsb>=100){ lastUsb=now;

      // Sensors
      mpu.update();
      gX=alpha*gX+(1-alpha)*mpu.getGyroX();
      gY=alpha*gY+(1-alpha)*mpu.getGyroY();
      gZ=alpha*gZ+(1-alpha)*mpu.getGyroZ();
      while(Serial2.available()) gps.encode(Serial2.read());
      compass.read(); float heading=compass.getAzimuth(); if(heading<0) heading+=360;

      static long lastE1=0,lastE2=0; static unsigned long lastT=now;
      long d1=enc1Count-lastE1, d2=enc2Count-lastE2;
      float dt=(now-lastT)/1000.0; lastE1=enc1Count; lastE2=enc2Count; lastT=now;
      float speed1=(dt>0)?((d1/ENCODER_PPR)*PI*0.35/dt):0;
      float speed2=(dt>0)?((d2/ENCODER_PPR)*PI*0.35/dt):0;

      // Build telemetry
      String tl;
      tl.reserve(220);
      tl += "Gyro X:" + String(gX,1) + " Y:" + String(gY,1) + " Z:" + String(gZ,1);
      tl += " | GPS:"  + String(gps.location.isValid() ? "OK" : "NV");
      tl += " | Compass:" + String(heading,1) + "deg";
      tl += " | Joy:" + computeJoyDir(data.map2X,data.map2Y);
      tl += " | Fan:"   + String(digitalRead(coolingfan_Pin)==LOW ? "OFF":"ON");
      tl += " | Light:" + String(digitalRead(lightsrelay_Pin)==LOW ? "OFF":"ON");
      tl += " | EngSw:" + String(digitalRead(MOTOR_SWITCH_PIN)==LOW ? "ON":"OFF");
      tl += " | Enc1:"  + String(speed1,2) + "m/s";
      tl += " | Enc2:"  + String(speed2,2) + "m/s";

      Serial.println(tl);

      // LoRa every 10 s
      if(loraReady && now-lastLoRa>=10000UL){
          Serial3.println(tl);
          lastLoRa = now;
      }
  }

  updateMotorRelays();
}
