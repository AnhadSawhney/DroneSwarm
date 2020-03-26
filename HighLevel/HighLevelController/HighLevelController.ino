#include <EEPROM.h>
#include <Filters.h>
#include <Wire.h>
#include <SPI.h>

#include "Barometer.h"

HardwareSerial Serial1(PA10, PA9);
#define gpsPort Serial1
#define GPS_PORT_NAME "Serial1"
#define DEBUG_PORT Serial

void setup() {
  Wire.setSDA(PB9);
  Wire.setSCL(PB8);
  Wire.begin();
  Serial.begin(115200);  //Setting the baudrate
  delay(2000);
  Serial.println("Start");
  BMP_Init();
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
