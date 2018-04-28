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
const byte leftTrimPin = 4;
const byte rightTrimPin = 5;
const byte throttlePin = A0;
const byte turnPin = A1;
const byte batteryVoltagePin = A2;


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

  // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3D (for the 128x64)
  // init done
  
  // Show image buffer on the display hardware.
  // Since the buffer is intialized with an Adafruit splashscreen
  // internally, this will display the splashscreen.
  display.display();
  delay(500);

  // Clear the buffer.
  display.clearDisplay();
  
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
