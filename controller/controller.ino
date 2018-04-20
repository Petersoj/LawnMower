#include <SPI.h>
#include <nRF24L01.h>
#include "RF24.h"

RF24 radio(7, 8);

// Address used by radio
const byte address[6] = "GRASS";
int throttlePercentage = 0;
int leftMotorTrim = 0; // Expressed in a value 
int controllerBatteryPercentage = 100;
int mowerBatteryPercentage = 100;
int currentDrawLeftMotor = 0;
int currentDrawRightMotor = 0;
int currentDrawBladeMotor = 0;

void setup() {
  // Start NRF24L01+
  radio.begin();
  // Configure Radio
  radio.openWritingPipe(address);
  radio.setChannel(120);
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
  radio.stopListening();
}

void loop() {
  //radio.write(&text, sizeof(text));
  
}
