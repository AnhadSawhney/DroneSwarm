#define IMU 0x28
#define IMURST A0
#define write8(reg, data) Wire.beginTransmission(IMU);Wire.write(reg);Wire.write(data);Wire.endTransmission(true)
#define read16() (int16_t)(Wire.read()|Wire.read()<<8 )

void IMU_waitforboot() { //wait for IMU boot, verify ID
  do {
    Wire.beginTransmission(IMU);
    Wire.write(0x00);
    Wire.endTransmission(false);
    Wire.requestFrom(IMU, 1, true);
  } while(Wire.read() != 0xA0);
}

byte IMU_checkcalibration() { //returns 0 (not calibrated) - 3 (fully calibrated)
  Wire.beginTransmission(IMU);
  Wire.write(0x35);
  Wire.endTransmission(false);
  Wire.requestFrom(IMU, 1, true);
  byte a = Wire.read(), b = 0;
  Serial.print("CALIBRATION: ");
  Serial.print(a, BIN);
  Serial.print(" SYS=");
  Serial.print(a >> 6);
  b += a >> 6;
  a &= 0b00111111;
  Serial.print(" GYR=");
  Serial.print(a >> 4);
  b += a >> 4;
  a &= 0b00001111;
  Serial.print(" ACC=");
  Serial.print(a >> 2);
  b += a >> 2;
  a &= 0b00000011;
  Serial.print(" MAG=");
  Serial.println(a);
  b += a;
  return b;
}

byte IMU_checkcalibrationfast() {
  Wire.beginTransmission(IMU);
  Wire.write(0x35);
  Wire.endTransmission(false);
  Wire.requestFrom(IMU, 1, true);
  byte a = Wire.read();
  byte b = 0;
  b += a >> 6;
  a &= 0b00111111;
  b += a >> 4;
  a &= 0b00001111;
  b += a >> 2;
  a &= 0b00000011;
  b += a;
  return b;
}

void IMU_syscheck() {
  IMU_checkcalibration();
  byte a;
  Wire.beginTransmission(IMU);
  Wire.write(0x36);
  Wire.endTransmission(false);
  Wire.requestFrom(IMU, 5, true);
  Serial.print("ST RESULT: ");
  a = Wire.read();
  Serial.print(a, BIN);
  if (a & 0b1000) {
    Serial.print(" MCU OK");
  }
  if (a & 0b100) {
    Serial.print(" GYR OK");
  }
  if (a & 0b10) {
    Serial.print(" MAG OK");
  }
  if (a & 0b1) {
    Serial.print(" ACC OK");
  }
  Wire.read();
  Wire.read();
  Serial.println();
  Serial.print("SYSTEM STATUS: ");
  switch(Wire.read()) {
    case 0:
      Serial.println("0 - System idle");
      break;
    case 1:
      Serial.println("1 - System Error");
      break;
    case 2:
      Serial.println("2 - Initializing peripherals");
      break;
    case 3:
      Serial.println("3 - System Init ialization");
      break;
    case 4:
      Serial.println("4 - Executing selftest");
      break;
    case 5:
      Serial.println("5 - Sensor fusion algorithm running");
      break;
    case 6:
      Serial.println("6 - System running without fusion algorithm");
      break;
  }
  Serial.print("SYSTEM ERROR: ");
  switch(Wire.read()) {
    case 0:
      Serial.println("0 - No error");
      break;
    case 1:
      Serial.println("1 - Peripheral init ialization error");
      break;
    case 2:
      Serial.println("2 - System init ialization error");
      break;
    case 3:
      Serial.println("3 - Self test result failed");
      break;
    case 4:
      Serial.println("4 - Register map value out of range");
      break;
    case 5:
      Serial.println("5 - Register map address out of range");
      break;
    case 6:
      Serial.println("6 - Register map write error");
      break;
    case 7:
      Serial.println("7 - BNO low power mode not available for selected operation mode");
      break;
    case 8:
      Serial.println("8 - Accelerometer power mode not available");
      break;
    case 9:
      Serial.println("9 - Fusion algorithm configuration error");
      break;
    case 0xA:
      Serial.println("A - Sensor configuration error");
      break;
  }
  Wire.beginTransmission(IMU);
  Wire.write(0x3D);
  Wire.endTransmission(false);
  Wire.requestFrom(IMU, 2, true);
  Serial.print("OPERATION MODE: ");
  switch (Wire.read() & 0b00001111) {
    case 0x00:
      Serial.println("CONFIG");
      break;
    case 0x01:
      Serial.println("ACCONLY");
      break;
    case 0x02:
      Serial.println("MAGONLY");
      break;
    case 0x03:
      Serial.println("GYRONLY");
      break;
    case 0x04:
      Serial.println("ACCMAG");
      break;
    case 0x05:
      Serial.println("ACCGYRO");
      break;
    case 0x06:
      Serial.println("MAGGYRO");
      break;
    case 0x07:
      Serial.println("AMG");
      break;
    case 0x08:
      Serial.println("IMU");
      break;
    case 0x09:
      Serial.println("COMPASS");
      break;
    case 0x0A:
      Serial.println("M4G");
      break;
    case 0x0B:
      Serial.println("NDOF_FMC_OFF");
      break;
    case 0x0C:
      Serial.println("NDOF");
      break;
  }
  Serial.print("POWER MODE: ");
  switch (Wire.read()) {
    case 0:
      Serial.println("NORMAL");
      break;
    case 1:
      Serial.println("LOW POWER");
      break;
    case 2:
      Serial.println("SUSPEND");
      break;
  }
}

void IMU_rawdata() {
  Wire.beginTransmission(IMU);
  Wire.write(0x08);
  Wire.endTransmission(false);
  Wire.requestFrom(IMU, 18, true);
  // Accelerometer
  Serial.print("ACC X=");
  Serial.print(read16()/100.00, 2); // m/s^2
  Serial.print(" Y=");
  Serial.print(read16()/100.00, 2); // m/s^2
  Serial.print(" Z=");
  Serial.print(read16()/100.00, 2); // m/s^2
  // Magnetometer
  Serial.print(" MAG X=");
  Serial.print(read16()/16.00, 2); // mT
  Serial.print(" Y=");
  Serial.print(read16()/16.00, 2); // mT
  Serial.print(" Z=");
  Serial.print(read16()/16.00, 2); // mT
  // Gyroscope
  Serial.print(" GYR X=");
  Serial.print(read16()/16.00, 2); // deg/s
  Serial.print(" Y=");
  Serial.print(read16()/16.00, 2); // deg/s
  Serial.print(" Z=");
  Serial.print(read16()/16.00, 2); // deg/s
  Wire.beginTransmission(IMU);
  Wire.write(0x1A);  
  Wire.endTransmission(false);
  Wire.requestFrom(IMU, 6, true);
  Serial.print(" Yaw=");
  Serial.print(read16()/16.00);
  Serial.print(" Roll=");
  Serial.print(read16()/16.00);
  Serial.print(" Pitch=");
  Serial.println(read16()/16.00);
}

void IMU_EEPROMcalibrate() {
  int eeAddress = 0;
  long storedID, newID;
  EEPROM.get(eeAddress, storedID);
  byte calib;

  write8(0x07, 0x01); //move to page 1
  
  Wire.beginTransmission(IMU);
  Wire.write(0x50);
  Wire.endTransmission(false);
  Wire.requestFrom(IMU, 4, true);
  newID = (int64_t)(Wire.read()|Wire.read()<<8|Wire.read()<<16|Wire.read()<<24);

  Serial.print("Found ID: ");
  Serial.println(newID, HEX);
  
  write8(0x07, 0x00); //move to page 0

  if (FORCECALIBRATION | storedID != newID) {
    if(FORCECALIBRATION) {
      Serial.println("Stored data overridden");
    } else {
      Serial.println("No Calibration Data for this sensor exists in EEPROM");
    }
    write8(0x3D, 0x0C); // Operation Mode: NDOF
    IMU_waitforboot();
    unsigned long previousMillis = 0;
    while(1) {
      calib = IMU_checkcalibration();
      if(calib == 12 && !LEDSTATE) {
        LEDON;
        break;
      } else {
        unsigned long currentMillis = millis();
        if (currentMillis - previousMillis >= 1<<(14-calib)) {
          previousMillis = currentMillis;
          if(LEDSTATE){
            LEDOFF;
          } else {
            LEDON;
          }
        }
      }
      delay(100);
    }
    write8(0x3D, 0x00); //Set to CONFIG mode
    IMU_waitforboot();
    //store calibration in eeprom
    EEPROM.put(eeAddress, newID);
    eeAddress += sizeof(uint64_t);
    Serial.println("Writing Calibration data to EEPROM...");
    Wire.beginTransmission(IMU);
    Wire.write(0x55);
    Wire.endTransmission(false);
    Wire.requestFrom(IMU, 22, true);
    for(byte i = 0; i < 22; i++) { //read data
      calib = Wire.read();
      EEPROM.put(eeAddress, calib);
      Serial.println(calib, HEX);
      eeAddress += sizeof(byte);
    }
    Serial.println("Calibration data loaded into EEPROM");
  } else {
    write8(0x3D, 0x00); //Set to CONFIG mode
    IMU_waitforboot();
    Serial.println("Found Calibration for this sensor in EEPROM");
    eeAddress += sizeof(uint64_t);
    Serial.println("Restoring Calibration data to the BNO055...");
    Wire.beginTransmission(IMU);
    Wire.write(0x55);
    for(byte i = 0; i < 22; i++) { //read data
      EEPROM.get(eeAddress, calib);
      Wire.write(calib);
      Serial.println(calib, HEX);
      eeAddress += sizeof(byte);
    }
    Wire.endTransmission(true);
    Serial.println("Calibration data loaded into BNO055");
  }
}
