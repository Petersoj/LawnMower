#include <SPI.h>
#include "RF24.h"

RF24 radio(7, 8);

const byte address[6] = "GRASS";
const char receiveText[] = "Ping";
int led = 4;
boolean on = false;

void setup() {
  pinMode(led, OUTPUT);
  
  //Serial.begin(19200);

  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setChannel(120);
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
  radio.startListening();
}

void loop() {
  if (radio.available()) {
    char text[32] = "";
    radio.read(&text, sizeof(text));
    if (!strcmp(text, receiveText)) {
      if (on) {
        //Serial.print("Low ");
        digitalWrite(led, LOW);
        on = false;
      } else {
        //Serial.print("High ");
        digitalWrite(led, HIGH);
        on = true;
      }
    }
    //Serial.println(text);
  }
}
