/*
 * April 2018 - Jacob Peterson
 * arduino@jacobpeterson.net
 */
/*
 * This is the sketch that will run on the the controller Aduino
 * to control: wireless transceivers, OLED display, lawn mower
 * throttle and turning, and more.
 */

// Libraries
#include <SPI.h>
#include <Wire.h>
#include <nRF24L01.h>
#include "RF24.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Setup Display
//#define OLED_RESET 4
const int OLED_RESET = 4;
Adafruit_SSD1306 display(OLED_RESET);

// Setup Radio
RF24 radio(7, 8); // CE, CSN pins


// - Pinout - 
// NRF24 pins
// CE - 7
// CSN - 8
// SCK - 13
// MOSI - 11
// MISO - 12
// SSD1306 Display pins
// SDA - A4
// SCL - A5
const byte throttlePin = A0;
const byte turnPin = A1;
const byte batteryVoltagePin = A2;
const byte leftTrimPin = 4;
const byte rightTrimPin = 5;


// Address used by radio
const byte addresses[][6] = {"LAWNr", "LAWNw"};

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
  radio.openReadingPipe(0, addresses[0]);
  radio.openWritingPipe(addresses[1]);
  radio.setChannel(122);
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
  radio.stopListening();
}

void loop() {
  //radio.write(&text, sizeof(text));
  
}
