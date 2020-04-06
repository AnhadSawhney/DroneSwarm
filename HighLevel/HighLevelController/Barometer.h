#include <Wire.h>

#define COMP_ALT_GPS //compensate altitude using GPS

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

float baselinePressure; //pressure recording at takeoff

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
  Wire.setSDA(PB9);
  Wire.setSCL(PB8);
  Wire.begin();
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
  baselinePressure = BMP_GetPressure();
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

float BMP_GetPressure() { 
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
}

float BMP_GetAltitude() {
  
  return(44330.0*(1.0-pow((float)p/baselinePressure,0.19029))) + baselineAltitude; // Pressure at a baseline P0 (mb), return altitude (meters) above baseline.
}

void BMP_Everyloop() { //function to be called every loop
  barometer_counter ++;

  //Every time this function is called the barometer_counter variable is incremented. This way a specific action
  //is executed at the correct moment. This is needed because requesting data from the MS5611 takes around 9ms to complete.

  if (barometer_counter == 1) {                                                 //When the barometer_counter variable is 1. get temperature and pressure
  }
  if (barometer_counter == 2) {                                                 //If the barometer_counter variable equals 2.
    //Calculate pressure as explained in the datasheet of the MS-5611.
    actual_pressure = actual_pressure_slow;                                                                   //The actual_pressure is used in the program for altitude calculations.
  }

  if (barometer_counter == 3) {                                                                               //When the barometer counter is 3

    barometer_counter = 0;                                                                                    //Set the barometer counter to 0 for the next measurements.
    //In the following part a rotating buffer is used to calculate the long term change between the various pressure measurements.
    //This total value can be used to detect the direction (up/down) and speed of the quadcopter and functions as the D-controller of the total PID-controller.
    if (manual_altitude_change == 1)pressure_parachute_previous = actual_pressure * 10;                       //During manual altitude change the up/down detection is disabled.
    parachute_throttle -= parachute_buffer[parachute_rotating_mem_location];                                  //Subtract the current memory position to make room for the new value.
    parachute_buffer[parachute_rotating_mem_location] = actual_pressure * 10 - pressure_parachute_previous;   //Calculate the new change between the actual pressure and the previous measurement.
    parachute_throttle += parachute_buffer[parachute_rotating_mem_location];                                  //Add the new value to the long term avarage value.
    pressure_parachute_previous = actual_pressure * 10;                                                       //Store the current measurement for the next loop.
    parachute_rotating_mem_location++;                                                                        //Increase the rotating memory location.
    if (parachute_rotating_mem_location == 30)parachute_rotating_mem_location = 0;                            //Start at 0 when the memory location 20 is reached.

    if (flight_mode >= 2 && takeoff_detected == 1) {                                                          //If the quadcopter is in altitude mode and flying.
      if (pid_altitude_setpoint == 0)pid_altitude_setpoint = actual_pressure;                                 //If not yet set, set the PID altitude setpoint.
      //When the throttle stick position is increased or decreased the altitude hold function is partially disabled. The manual_altitude_change variable
      //will indicate if the altitude of the quadcopter is changed by the pilot.
      manual_altitude_change = 0;                                                    //Preset the manual_altitude_change variable to 0.
      manual_throttle = 0;                                                           //Set the manual_throttle variable to 0.
      if (channel_3 > 1600) {                                                        //If the throtttle is increased above 1600us (60%).
        manual_altitude_change = 1;                                                  //Set the manual_altitude_change variable to 1 to indicate that the altitude is adjusted.
        pid_altitude_setpoint = actual_pressure;                                     //Adjust the setpoint to the actual pressure value so the output of the P- and I-controller are 0.
        manual_throttle = (channel_3 - 1600) / 3;                                    //To prevent very fast changes in hight limit the function of the throttle.
      }
      if (channel_3 < 1400) {                                                        //If the throtttle is lowered below 1400us (40%).
        manual_altitude_change = 1;                                                  //Set the manual_altitude_change variable to 1 to indicate that the altitude is adjusted.
        pid_altitude_setpoint = actual_pressure;                                     //Adjust the setpoint to the actual pressure value so the output of the P- and I-controller are 0.
        manual_throttle = (channel_3 - 1400) / 5;                                    //To prevent very fast changes in hight limit the function of the throttle.
      }

      //Calculate the PID output of the altitude hold.
      pid_altitude_input = actual_pressure;                                          //Set the setpoint (pid_altitude_input) of the PID-controller.
      pid_error_temp = pid_altitude_input - pid_altitude_setpoint;                   //Calculate the error between the setpoint and the actual pressure value.

      //To get better results the P-gain is increased when the error between the setpoint and the actual pressure value increases.
      //The variable pid_error_gain_altitude will be used to adjust the P-gain of the PID-controller.
      pid_error_gain_altitude = 0;                                                   //Set the pid_error_gain_altitude to 0.
      if (pid_error_temp > 10 || pid_error_temp < -10) {                             //If the error between the setpoint and the actual pressure is larger than 10 or smaller then -10.
        pid_error_gain_altitude = (abs(pid_error_temp) - 10) / 20.0;                 //The positive pid_error_gain_altitude variable is calculated based based on the error.
        if (pid_error_gain_altitude > 3)pid_error_gain_altitude = 3;                 //To prevent extreme P-gains it must be limited to 3.
      }

      //In the following section the I-output is calculated. It's an accumulation of errors over time.
      //The time factor is removed as the program loop runs at 250Hz.
      pid_i_mem_altitude += (pid_i_gain_altitude / 100.0) * pid_error_temp;
      if (pid_i_mem_altitude > pid_max_altitude)pid_i_mem_altitude = pid_max_altitude;
      else if (pid_i_mem_altitude < pid_max_altitude * -1)pid_i_mem_altitude = pid_max_altitude * -1;
      //In the following line the PID-output is calculated.
      //P = (pid_p_gain_altitude + pid_error_gain_altitude) * pid_error_temp.
      //I = pid_i_mem_altitude += (pid_i_gain_altitude / 100.0) * pid_error_temp (see above).
      //D = pid_d_gain_altitude * parachute_throttle.
      pid_output_altitude = (pid_p_gain_altitude + pid_error_gain_altitude) * pid_error_temp + pid_i_mem_altitude + pid_d_gain_altitude * parachute_throttle;
      //To prevent extreme PID-output the output must be limited.
      if (pid_output_altitude > pid_max_altitude)pid_output_altitude = pid_max_altitude;
      else if (pid_output_altitude < pid_max_altitude * -1)pid_output_altitude = pid_max_altitude * -1;
    }

    //If the altitude hold function is disabled some variables need to be reset to ensure a bumpless start when the altitude hold function is activated again.
    else if (flight_mode < 2 && pid_altitude_setpoint != 0) {                        //If the altitude hold mode is not set and the PID altitude setpoint is still set.
      pid_altitude_setpoint = 0;                                                     //Reset the PID altitude setpoint.
      pid_output_altitude = 0;                                                       //Reset the output of the PID controller.
      pid_i_mem_altitude = 0;                                                        //Reset the I-controller.
      manual_throttle = 0;                                                           //Set the manual_throttle variable to 0 .
      manual_altitude_change = 1;                                                    //Set the manual_altitude_change to 1.
    }
  }
}
