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
  SD_Init(); //also initializes SPI for MCU2
  pinMode(PA1, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print(BMP_GetTemperature());
  Serial.print(" ");
  Serial.println(BMP_GetAltitude());
  digitalWrite(PA1, 1);
  delay(100);
  digitalWrite(PA1, 0);
  delay(100);
}
