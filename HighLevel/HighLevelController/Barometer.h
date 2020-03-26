#define BMP 0x76
#define write8(reg, data) Wire.beginTransmission(BMP);Wire.write(reg);Wire.write(data);Wire.endTransmission(true)
#define read16() (int16_t)(Wire.read()|Wire.read()<<8 )
#define readU16() (uint16_t)(Wire.read()|Wire.read()<<8 )

uint16_t dig_T1;
int16_t dig_T2;
int16_t dig_T3;
uint16_t dig_P1;
int16_t dig_P2;
int16_t dig_P3;
int16_t dig_P4;
int16_t dig_P5;
int16_t dig_P6;
int16_t dig_P7;
int16_t dig_P8;
int16_t dig_P9;

int32_t t_fine;

void BMP_waitforboot() { //wait for IMU boot, verify ID
  do {
    Wire.beginTransmission(BMP);
    Wire.write(0xD0);
    Wire.endTransmission(false);
    Wire.requestFrom(BMP, 1, true);
  } while(Wire.read() != 0x58);
}

// The BMP280 includes factory calibration data stored on the device.
// Each device has different numbers, these must be retrieved and
// used in the calculations when taking measurements.

void BMP_Init() {
  BMP_waitforboot();
  Wire.beginTransmission(BMP);
  Wire.write(0x88);
  Wire.endTransmission(false);
  Wire.requestFrom(BMP, 24, true);
  dig_T1 = readU16();
  dig_T2 = read16();
  dig_T3 = read16();
  dig_P1 = readU16();
  dig_P2 = read16();
  dig_P3 = read16();
  dig_P4 = read16();
  dig_P5 = read16();
  dig_P6 = read16();
  dig_P7 = read16();
  dig_P8 = read16();
  dig_P9 = read16();
  write8(0xF5, 0b00011100); //0.5ms standby, filter coefficient 16, use i2c
  write8(0xF4, 0b01111111); //Normal mode, P oversample x 16, T oversample x4
}

/*
** temperature calculation
** @param : T  = stores the temperature value after calculation.
** @param : uT = the uncalibrated temperature value.
*/
void BMP_UpdateFineTemperature() {
  int32_t uncomp_temp = ((((int32_t)Wire.read()) << 12) | (((int32_t)Wire.read()) << 4) | (((int32_t)Wire.read()) >> 4));

  int32_t var1 = ((((uncomp_temp >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;

  int32_t var2 = (((((uncomp_temp >> 4) - ((int32_t)dig_T1)) * ((uncomp_temp >> 4) - ((int32_t)dig_T1))) >>  12) * ((int32_t)dig_T3)) >> 14;

  t_fine = var1 + var2;
}

float BMP_GetTemperature() {
  Wire.beginTransmission(BMP);
  Wire.write(0xFA);
  Wire.endTransmission(false);
  Wire.requestFrom(BMP, 3, true);
  BMP_UpdateFineTemperature();
  return ((int32_t)(t_fine * 5 + 128) >> 8)/100.0;
}

float BMP_GetAltitude() {
  Wire.beginTransmission(BMP);
  Wire.write(0xF7);
  Wire.endTransmission(false);
  Wire.requestFrom(BMP, 6, true);
  int32_t uncomp_pres = ((((int32_t)Wire.read()) << 12) | (((int32_t)Wire.read()) << 4) | (((int32_t)Wire.read()) >> 4));
  BMP_UpdateFineTemperature(); //Contains wire.read() 3 times
  
  int64_t var1, var2;
  var1 = ((int64_t)t_fine) - 128000;
  var2 = var1 * var1 * (int64_t)dig_P6;
  var2 = var2 + ((var1 * (int64_t)dig_P5) << 17);
  var2 = var2 + (((int64_t)dig_P4) << 35);
  var1 = ((var1 * var1 * (int64_t)dig_P3) >> 8) + ((var1 * (int64_t)dig_P2) << 12);
  var1 =(((((int64_t)1) << 47) + var1)) * ((int64_t)dig_P1) >> 33;

  if (var1 == 0) {
    return 0; // avoid exception caused by division by zero
  }
  int64_t p = 1048576 - uncomp_pres;
  p = (((p << 31) - var2) * 3125) / var1;
  var1 = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
  var2 = (((int64_t)dig_P8) * p) >> 19;

  p = (((p + var1 + var2) >> 8) + (((int64_t)dig_P7) << 4)) >> 8;
  
  return(44330.0*(1.0-pow((float)p/101325,0.19029))); // Pressure at a baseline P0 (mb), return altitude (meters) above baseline.
}
