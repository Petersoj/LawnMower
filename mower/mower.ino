/*
 * April 2018 - Jacob Peterson
 * arduino@jacobpeterson.net
 */
/*
 * This is the sketch that will run on the the lawn mower Arduino
 * to control: Left/Right Wheel Window Motor Controllers,
 * Blade CIM Motor Controller, wireless transceivers, and more.
 */
 
#include <SPI.h>
#include <nRF24L01.h>
#include "RF24.h"

RF24 radio(7, 8);

const byte address[6] = "GRASS";

void setup() {
  //Serial.begin(19200);

  radio.begin();
  radio.openWritingPipe(address);
  radio.openReadingPipe(0, address);
  radio.setChannel(122);
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
}

void loop() {
  if (radio.available()) {
    char text[32] = "";
    radio.read(&text, sizeof(text));
    
    //Serial.println(text);
  }
}

int getBatteryPercentage() {
  //return map(analogRead());
  return 0;
}



