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
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h>
//#include <Adafruit_GFX.h> use later for cool fx :)
#include <Adafruit_SSD1306.h>


// Setup Display
//#define OLED_RESET 4
const int OLED_RESET = 4;
Adafruit_SSD1306 display(OLED_RESET);

#if (SSD1306_LCDHEIGHT != 64)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif


// Setup Radio
RF24 radio(7, 8); // CE, CSN pins

// Addresses used by radio
const byte mowerWritingAddress[] = "mower";
const byte controllerWritingAddress[] = "cntrl";

// struct for receiving data from mower
typedef struct {
  byte batteryPercentage;
  byte currentDraw; // In Amps
}
ReceiveRadioData;

// struct for transmitting data to mower
typedef struct {
  byte throttle; // 0-100
  byte turnRight; // 0-255
  byte turnLeft; // 0-255
  boolean backwards; // Uses right joystick moved all the way down.
}
TransmitRadioData;

// Construct RadioData
TransmitRadioData transmitRadioData;
ReceiveRadioData receiveRadioData;

// - Pinout - 
// NRF24 pins
// CE - 7
// CSN - 8
// SCK - 13
// MOSI - 11
// MISO - 12
// SSD1306 Display pins (i2c)
// SDA - A4
// SCL - A5
//const byte leftTrimPin = 4;
//const byte rightTrimPin = 5;
//const byte autoTurnRightPin = 6;
//const byte autoTurnLeftPin = 9;
// Analog In pins
const byte throttlePin = A0;
const byte turnPin = A1;
//const byte backwardsPin = A2;
//const byte batteryVoltagePin = A3;

void setup() {
  // Configure pin modes

  
  // Configure OLED information screen
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3D
  // Show image buffer on the display hardware.
  // Since the buffer is intialized with an Adafruit splashscreen
  // internally, this will display the splashscreen.
  display.display();
  delay(500);
  // Clear the buffer.
  display.clearDisplay();

  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(2);
  display.print("Throttle: ");
  display.setCursor(10, 0);
  display.display();
  
  // Configure transceiver
  radio.begin();
  radio.setChannel(122);
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
//  radio.setPayloadSize(max(sizeof(transmitRadioData), sizeof(receiveRadioData)));
  radio.openWritingPipe(controllerWritingAddress);
//  radio.openReadingPipe(0, mowerWritingAddress);
  
  radio.stopListening(); // Temporary until transceiving is functional

  Serial.begin(9600);
}

void loop() {
  executeRadioLogic();

  updateAnalogValues();

  updateScreen();

  delay(40);

  Serial.println(transmitRadioData.turnLeft);
  Serial.println(transmitRadioData.turnRight);
}

byte unsuccessfulTxIndex = 0; // Counts unsuccessful writes
void executeRadioLogic() {
  // Temporarly only acting as a receiver.

  // Write transmit data
  radio.write(&transmitRadioData, sizeof(transmitRadioData));
  

  // TODO Fix transceiving logic
  
  // This radio logic is primarily acting as the receiver of data
  // although we do send necessary data back.
  
//  if (radio.available()) { // Check if data in rx buffer
//    while (radio.available()) { // Get last data if multiple packets in buffer
//      radio.read(&receiveRadioData, sizeof(receiveRadioData)); // Read into struct obj
//    }
//
//    // Close radio buffer and prepare for writing data
//    radio.stopListening();
//
//    if (!radio.write(&transmitRadioData, sizeof(transmitRadioData))) {
//      unsuccessfulTxIndex++;
//    } else {
//      unsuccessfulTxIndex = 0;
//    }
//    
//    // Open radio buffer and prepare for reading data
//    radio.startListening();
//
//    if (unsuccessfulTxIndex > 3) { // 3 times in a row data tx was unsuccessful
//      // TODO inform user of error
//    }
//    Serial.println(receiveRadioData.currentDraw);
//  }
}

void updateAnalogValues() {
  
  int analogThrottle = analogRead(throttlePin);
  if (analogThrottle < 50) { // Clamp to prevent floating max
    analogThrottle = 50;
  } else if (analogThrottle > 950) { // Clamp to prevent floating min
    analogThrottle = 950;
  }
  byte finalThrottle = map(analogThrottle, 950, 50, 0, 255);
  transmitRadioData.throttle = finalThrottle;

  int analogTurn = analogRead(turnPin);
  byte finalTurnRight = 0;
  byte finalTurnLeft = 0;
  if (analogTurn < 450) {
    finalTurnRight = map(analogTurn, 450, 0, 0, 255);
  }
  if (analogTurn > 650) {
    finalTurnLeft = map(analogTurn, 0, 650, 0, 255);
  }
  
  transmitRadioData.turnRight = finalTurnRight;
  transmitRadioData.turnLeft = finalTurnLeft;
  
  // 1023? basically no throttle
  // 0? full throttle

//  int analogThrottle = analogRead(A4);
//  Serial.println(analogThrottle);
  
  // Turning analog
  // Less resistance is moving right (high number)
  // 2.6 middle
  // 4.45 max
  // 2.6/4.45 = 0.5842696629

  // Throttle Low resistance = 0.59
  // Max 4.52
}


// Method will update the OLED info screen.
void updateScreen() {
  display.print(transmitRadioData.throttle);
  display.display();
}


