#include "LoRa_E32.h"

#define LORA_M0  27
#define LORA_M1  28
#define LORA_AUX 29          // use -1 if AUX not wired

LoRa_E32 e32(&Serial3, LORA_AUX, LORA_M0, LORA_M1);

void setup() {
  Serial.begin(115200);       // USB console
  Serial3.begin(9600);

  pinMode(LORA_M0, OUTPUT);
  pinMode(LORA_M1, OUTPUT);
  digitalWrite(LORA_M0, LOW);
  digitalWrite(LORA_M1, LOW);

  if (!e32.begin()) {
    Serial.println(F("LoRa init failed"));
    while (1);
  }
  Serial.println(F("LoRa RX ready"));
}

void loop() {
  // Read bytes coming from the module and echo them to the PC
  while (Serial3.available()) {
    char c = Serial3.read();
    Serial.write(c);          // prints “hello” lines as they arrive
  }
}