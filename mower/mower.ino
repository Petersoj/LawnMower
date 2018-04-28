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

// Construct RF24 radio
RF24 radio(7, 8); // CE, CSN pins

// addresses[0] is the writing pipe for the mower
// addresses[1] is the writing pipe for the controller
const byte addresses[2][6] = {"mower","cntrl"};

// struct for transmitting data from mower
typedef struct {
  byte batteryPercentage;
  byte currentDraw;
}
TransmitRadioData;

// struct for receiving data from controller
typedef struct {
  byte throttle;
  byte turn;
}
ReceiveRadioData;

// Construct RadioData
TransmitRadioData transmitRadioData;
ReceiveRadioData receiveRadioData;


// - Pinout - unused: 10, A4, A5
// NRF24 pins
// CE - 7
// CSN - 8
// SCK - 13
// MOSI - 11
// MISO - 12
const byte relayPin = 4;
const byte bladePWMPin = 9; // Blade Controller InA pin should connect to 5V
const byte leftPWMPin = 5;
const byte rightPWMPin = 6;
const byte leftDriveInAPin = 2;
const byte leftDriveInBPin = 3;
const byte rightDriveInAPin = 0;
const byte rightDriveInBPin = 1;
// Analog In pins
const byte batteryVoltagePin = A0;
const byte bladeCurrentPin = A1;
const byte leftDriveCurrentPin = A2;
const byte rightDriveCurrentPin = A3;

const byte loopDelay = 100; // Loop executes 10 times a second
byte batteryReadIndex = 0; // Tracks how many times loop has executed

// Remember to shut off when connection is lost to the radio
// Use CS pins to determine if kill switch was flipped

void setup() {
  // Pin modes for relay and Motor Controller In A/B pins
  pinMode(0, OUTPUT);
  pinMode(1, OUTPUT);
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);

  // Turn on relay pin for latching circuit (to keep power on)
  digitalWrite(relayPin, HIGH);
  
  // Configure transceiver
  radio.begin();
  radio.openWritingPipe(addresses[0]);
  radio.openReadingPipe(0, addresses[1]);
  radio.setChannel(122);
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
  radio.setPayloadSize(max(sizeof(transmitRadioData), sizeof(receiveRadioData)));
}

void loop() {
//  if (radio.available()) {
//    char text[radio.getPayloadSize()] = "";
//    radio.read(&text, sizeof(text));
//    
//    Serial.println(text);
//  }
//  digitalWrite(pin, HIGH);

  if (radio.available()) { // Buffer should be full from the delay time
    radio.read(&receiveRadioData, sizeof(receiveRadioData)); // Read into struct obj
    
  } else { // Controller radio did not send anything or lost connection so shut off
    // This is a safety feature as the controller can never go out of range
    // and have the lawn mower still operate. Someone must go and turn back
    // on the lawn mower as kind of a reminder to stay in range :)
    
    // Disconnect relay NO which creates an open circuit and
    // disconnects everything from power
    digitalWrite(relayPin, LOW); 
  }

  // Battery percentage update
  if (batteryReadIndex > 10) { // Only update every second
    transmitRadioData.batteryPercentage = getBatteryPercentage();
    batteryReadIndex = 0;
  } else {
    batteryReadIndex++;
  }
  
  radio.startListening(); // Allow radio data buffer to fill during delay

  delay(loopDelay);
}

const voltageDividerMax = 5; // Max voltage analog input of a charged battery.
int getBatteryPercentage() {
  return map(analogRead(batteryVoltagePin, 0, 1023, 0, 100));
}

void autoTurnClockwise() {
  // TODO
}

void autoTurnCounterClockwise() {
  // TODO
}
