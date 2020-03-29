#include <SPI.h>
#include <SD.h>

#define SD_CS PB0
#define SD_DET PB12

void SD_Init() {
  pinMode(SD_DET, INPUT);
  if(digitalRead(SD_DET)) { //High: NO SD CARD
    Serial.println("NO SD CARD DETECTED");
    MCU2_Init();
  } else {
    SD.begin(SD_CS);
  }
}
