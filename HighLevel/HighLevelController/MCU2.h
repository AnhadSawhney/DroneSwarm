#include <SPI.h>
#include <SD.h>

#define MCU_CS PA4

void MCU2_Init() { //initialize communication with ATmega32U4 over SPI1
  SPI.begin(); //Initialize the SPI_1 port.
  SPI.setBitOrder(MSBFIRST); // Set the SPI_1 bit order
  SPI.setDataMode(SPI_MODE0); //Set the  SPI_1 data mode 0
  //SPI.setClockDivider(SPI_CLOCK_DIV16);      // Slow speed (72 / 16 = 4.5 MHz SPI_1 speed)
  pinMode(MCU_CS, OUTPUT);
}
