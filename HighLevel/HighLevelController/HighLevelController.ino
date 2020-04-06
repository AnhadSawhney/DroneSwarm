#include <EEPROM.h>
#include <Filters.h>

#define LED PA1
#include "Barometer.h"
#include "MicroSD.h"

void setup() {
  Serial.begin(115200);  //Setting the baudrate
  delay(2000);
  Serial.println("Start");
  BMP_Init(); //also initializes I2C
  GPS_Init();
  SD_Init(); //also initializes SPI for MCU2
  pinMode(PA1, OUTPUT);
}

void loop() { //should run at 250hz
  unsigned long s = micros();
  BMP_Everyloop();
  GPS_Everyloop();
  
  
  s = micros()-s;
  
  //Serial.print("LOOP FREQENCY:"); Serial.println(1000000.0/s);
  if(s < 4000) { //max loop frequency = 250hz
    delayMicroseconds(s);
  }
}
