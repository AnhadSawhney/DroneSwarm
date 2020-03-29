SPIClass SPI2(2); //Create an instance of the SPI Class called SPI_2 that uses the 2nd SPI Port
#define USE_SPI2
#include "RF24.h"

#define nRF_CE PB11
#define nRF_CS PB10

RF24 radio(nRF_CE, nRF_CS);

void SD_Init() {
  radio.begin();
  radio.setPALevel(RF24_PA_MAX);
}
