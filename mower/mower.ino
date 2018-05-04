/*
 * April 2018 - Jacob Peterson
 * arduino@jacobpeterson.net
 */
/*
 * This is the sketch that will run on the the lawn mower Arduino to control: 
 * Left/Right Wheel Window Motor Controllers, Blade CIM Motor Controller,
 * wireless transceivers, and more.
 * 
 * Motor Controllers used: 13A continuous VNH2SP30 (from china ebay!)
 * Radio Transceivers used: NRF24L01+
 * 
 * 
 * Features to add to RC version of Lawn Mower:
 *  - Use CS pins to determine if kill switch was flipped
 *  - Sending error messages to controller for diagnosis info
 *  - Remeber path using a 'record' button
 */
 
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// Construct RF24 radio
RF24 radio(7, 8); // CE, CSN pins

// Addresses used by radio
const byte mowerWritingAddress[] = "mower";
const byte controllerWritingAddress[] = "cntrl";

// struct for transmitting data to controller
typedef struct {
  byte batteryPercentage;
  byte currentDraw; // In Amps
}
TransmitRadioData;

// struct for receiving data from controller
typedef struct {
  byte throttle; // 0-100
  byte turn; // 0-126 left turn, 127 straight, 128-255 right turn
  boolean backwards; // Uses right joystick moved all the way down.
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

const byte loopDelay = 40; // Loop executes 25 times a second
float voltageDividerMaxInput = 5; // Raw divided voltage input of a charged battery.
float voltageDividerMinInput = 3.4; // Raw divided voltage input of a dead battery
byte throttle = 0; // PWM value of throttle after low pass filter

// TODO
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

  // Set fromHigh var according to predefined value
  // for mapping battery voltage to percentage
  voltageDividerMaxInput = (voltageDividerMaxInput / 5) * 1023;

  // Set fromLow var according to predefined value
  // for mapping battery voltage to percentage
  voltageDividerMinInput = (voltageDividerMinInput / 5) * 1023;
  
  // Configure transceiver
  radio.begin();
  radio.setChannel(122);
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
  radio.setPayloadSize(max(sizeof(transmitRadioData), sizeof(receiveRadioData)));
  radio.openWritingPipe(mowerWritingAddress[0]);
  radio.openReadingPipe(0, controllerWritingAddress[1]);
}

void loop() {
  
//  checkBattery(); // Reads battery voltage, updates, and checks values
//  checkCurrent(); // Reads motor current draw, updates, and checks values

  executeRadioLogic(); // Loop radio logic (rx/tx data and check for rx/tx loss)
  filterThrottleValue(); // Loops low pass filter for gradual throttle change
//  driveLogic(); // Updates driving/blade motor values

  delay(loopDelay); // Allow time for rx buffer to fill and values to update/change
}

// This method powers off everything using the optocoupler relay.
// This method may not power everything off instantly as
// the relay has a 5ms release time.
void poweroff() {
  // Disconnect relay NO which creates an open circuit and
  // disconnects everything from power.
  digitalWrite(relayPin, LOW);
}

byte unsuccessfulRxTxIndex = 0; // Counts unsuccessful reads/writes
void executeRadioLogic() {
  // This radio logic is primarily acting as the transmitter of data
  // although we do receive data needed from controller.
  
  // Close radio buffer and prepare for writing data
  radio.stopListening();

  // Write transmit data
  if (!radio.write(&transmitRadioData, sizeof(transmitRadioData))) {
    // Not radio acknowledgement was sent back and the RX/TX may be out of range!
    unsuccessfulRxTxIndex++;
  } else {
    unsuccessfulRxTxIndex = 0;
  }

  // Open radio buffer and prepare for reading data
  radio.startListening();

  // Loop to allow time for tx radio to send data and fill rx buffer
  unsigned long startingListeningTime = millis();
  while (!radio.available()) { // Loop until we have data available in radio buffer
    if (millis() - startingListeningTime > 200) { // Listening timeout is 200ms
      unsuccessfulRxTxIndex++;
    }
  }
  if (radio.available()) { // Check if data in rx buffer
    radio.read(&receiveRadioData, sizeof(receiveRadioData)); // Read into struct obj
    unsuccessfulRxTxIndex = 0; // Successful read so reset unsuccessful index
  }

  if (unsuccessfulRxTxIndex > 5) { // Threshold met of max ~1 second of dropped rx/tx
    // Controller radio did not send anything or lost connection and unsuccessful
    // data tx/rx threshold was met so power off arduino & circuity completely.
    // This is a safety feature as the controller can never go out of range
    // and have the lawn mower still operate. Someone must go and turn back
    // on the lawn mower as kind of a reminder to stay in range :)
    poweroff();
  }
}

boolean rightDriveDirectionChange = true; // True to initialize InA/B
boolean leftDriveDirectionChange = true; // True to initialize InA/B
void driveLogic() {
  // Right Drive Motor
  if (rightDriveDirectionChange) {
    if (receiveRadioData.backwards) {
      digitalWrite(rightDriveInAPin, LOW); // Low at first to prevent both on briefly
      digitalWrite(rightDriveInBPin, HIGH);
    } else {
      digitalWrite(rightDriveInBPin, LOW); // Low at first to prevent both on briefly
      digitalWrite(rightDriveInAPin, HIGH);      
    }
  }
  
  // Left Drive Motor
  if (rightDriveDirectionChange) {
    if (receiveRadioData.backwards) {
      digitalWrite(rightDriveInAPin, LOW); // Low at first to prevent both on briefly
      digitalWrite(rightDriveInBPin, HIGH);
    } else {
      digitalWrite(rightDriveInBPin, LOW); // Low at first to prevent both on briefly
      digitalWrite(rightDriveInAPin, HIGH);      
    }
  }
}

byte previousValue = 0; // Previously passed value
short timeDelay = 2000; // 2 Seconds to fully change to actual value
void filterThrottleValue() { // A simple low pass filter to gradually change values.
  
}

void checkCurrent() {
  // Current Readings logic - used for checking if there is an
  // excessive current draw from one of the motors. If there is,
  // it's possible that the motors are drawing their stall current
  // and are blocked by something so power off everything (safety first!)
  int leftDriveCurrent = readCurrentSense(leftDriveCurrentPin);
  int rightDriveCurrent = readCurrentSense(rightDriveCurrentPin);
  int bladeCurrent = readCurrentSense(bladeCurrentPin);
  if (leftDriveCurrent > 5 || rightDriveCurrent > 5 || bladeCurrent > 6) {
    poweroff();
  }
}

byte batteryReadIndex = 0; // Tracks how many times loop has executed
byte oneSecondsTime = 1000 / loopDelay; // Calc one seconds time from loopDelay
void checkBattery() {
  // Battery percentage update
  if (batteryReadIndex > oneSecondsTime) { // Only update every second
    transmitRadioData.batteryPercentage = getBatteryPercentage();
    batteryReadIndex = 0;
  } else {
    batteryReadIndex++;
  }
  
  if (transmitRadioData.batteryPercentage < 3) { // Lower than 3% shut off
    poweroff();
  }
}

int getBatteryPercentage() {
  // Maps analog input from battery voltage divider to a percentage
  int rawVoltage = analogRead(batteryVoltagePin);
  return map(rawVoltage, voltageDividerMinInput, voltageDividerMaxInput, 0, 100);
}

// Returns the current (in Amps) using the analog input of the Current Sense
// pins on the Motor Controllers. (This may be inaccurate to some degree)
int readCurrentSense(int currentSenseAnalogPin) {
  // Read voltage from CS analog in pin and convert to amps
  // CS pin of Motor Controller - 0.13 Volts/Amp
  return analogRead(currentSenseAnalogPin) / 0.13;
}

void autoTurnClockwise() {
  // TODO
}

void autoTurnCounterClockwise() {
  // TODO
}
