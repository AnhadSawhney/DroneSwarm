#include <SPI.h>
#include <SD.h>

#define SD_CS PB0
#define SD_DET PB12

void MCU2_Init() { //initialize communication with ATmega32U4
  SPI.begin(); //Initialize the SPI_1 port.
  SPI.setBitOrder(MSBFIRST); // Set the SPI_1 bit order
  SPI.setDataMode(SPI_MODE0); //Set the  SPI_1 data mode 0
  //SPI.setClockDivider(SPI_CLOCK_DIV16);      // Slow speed (72 / 16 = 4.5 MHz SPI_1 speed)
  pinMode(SD_CS, OUTPUT);
}
