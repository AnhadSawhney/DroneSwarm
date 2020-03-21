#include <EEPROM.h>
#include <Filters.h>
#include <Wire.h>

#define FORCECALIBRATION false

#define LEDPIN 7
#define LEDOFF PORTE |= 0b01000000
#define LEDON PORTE &= 0b10111111
#define LEDSTATE !(PORTE & 0b01000000)

#include "IMUfunctions.h"
#include "vectors.h"

#define YAWp 2.0
#define PITCHp 1.0
#define ROLLp 1.0
#define THROTTLEp 1.0

#define YAWi 0.4
#define PITCHi 0
#define ROLLi 0
#define THROTTLEi 0.0

#define YAWd -4.0
#define PITCHd -1 //small negative
#define ROLLd -1//-3.0
#define THROTTLEd 0.0

#define PULSE_MIN 1000
#define PULSE_MAX 2000

//#define PRINTMOTORS

// ------------- Global variables used for PID controller --------------------
Vector3D orientation, targetOrientation, orientationError, orientationErrorSum;
Vector3D velocity = {0, 0, 0}, targetVelocity = {0, 0, 0};

unsigned long timer1 = 0, timer2 = 0; 

uint16_t holdingThrottle, 
         pulse_length_esc1,
         pulse_length_esc2,
         pulse_length_esc3,
         pulse_length_esc4;

float throttleErrorSum = 0;

FilterOnePole filterx(HIGHPASS, 1);
FilterOnePole filtery(HIGHPASS, 1);
FilterOnePole filterz(HIGHPASS, 1);
FilterOnePole filteryaw(HIGHPASS, 1);
FilterOnePole filterpitch(HIGHPASS, 1);
FilterOnePole filterroll(HIGHPASS, 1);
//FilterOnePole filterthrottle(HIGHPASS, 1);

void setmotors(uint16_t pulse) {
  pulse_length_esc1 = pulse;
  pulse_length_esc2 = pulse;
  pulse_length_esc3 = pulse;
  pulse_length_esc4 = pulse;
}

void IMU_euler() { //INPUT: PITCH YAW ROLL ABSOLUTE IN DEGREES
  Wire.beginTransmission(IMU);
  Wire.write(0x1A);  
  Wire.endTransmission(false);
  Wire.requestFrom(IMU, 6, true);
  orientation.yaw = fixangle(read16/16.00, orientation.yaw);
  orientation.roll = fixangle(read16/16.00, orientation.roll);
  orientation.pitch = fixangle(read16/16.00, orientation.pitch);
  targetOrientation.fix(orientation);
  orientationError = targetOrientation - orientation;
  //Serial.println("Roll Pitch Yaw ORoll OPitch OYaw");
  //orientation.print();
  //Serial.print(" ");
  //targetOrientation.println();
}

/*  Undo rotation by quaternion
 *  1.0-2.0*(y*y+z*z) 2.0*(x*y-z*w)     2.0*(x*z+y*w)       accx
 *  2.0*(x*y+z*w)     1.0-2.0*(x*x+z*z) 2.0*(y*z+x*w)     * accy
 *  2.0*(x*z-y*w)     2.0*(y*z-x*w)     1.0-2.0*(x*x+y*y)   accz
 */

void IMU_linaccel() { //Absolute, based on north and gravity
  Wire.beginTransmission(IMU);
  Wire.write(0x20);
  Wire.endTransmission(false);
  Wire.requestFrom(IMU, 14, true);
  float w = read16/16384.0;
  float x = read16/16384.0;
  float y = read16/16384.0;
  float z = read16/16384.0;
  Vector3D acc;
  acc.x = read16/100.0;
  acc.y = read16/100.0;
  acc.z = read16/100.0; // m/s^2

  Vector3D absAcc;
  
  absAcc.x = 2.0*((0.5-(y*y+z*z))*acc.x + (x*y-z*w)*acc.y + (x*z+y*w)*acc.z);
  absAcc.y = 2.0*((x*y+z*w)*acc.x + (0.5-(x*x+z*z))*acc.y + (y*z-x*w)*acc.z);
  absAcc.z = 2.0*((x*z-y*w)*acc.x + (y*z+x*w)*acc.y + (0.5-(x*x+y*y))*acc.z);

  float dt = (micros()-timer1)/1000000.0;
  timer1 = micros();
  
  velocity += absAcc*dt;

  filterx.input(velocity.x);
  filtery.input(velocity.y);
  filterz.input(velocity.z);

  Vector3D filteredVelocity = {filterx.output(), filtery.output(), filterz.output()};

  //absAcc.print();
  //Serial.print(" ");
  filteredVelocity.println();

  //Serial.print(targetVelocity.scalarProject(velocity));
  //Serial.print(" ");
  //Serial.println(velocity.magnitude());

  float error = targetVelocity.scalarProject(velocity) - velocity.magnitude();
  
  //THROTTLE PID:

  // Calculate sum of errors : Integral coefficients
  throttleErrorSum += error*dt;

  // PID = e.Kp + ∫e.Ki + Δe.Kd
  holdingThrottle += error * THROTTLEp + throttleErrorSum * THROTTLEi + absAcc.magnitude()*THROTTLEd; //derivative of velocity is just acceleration
}

ISR(TIMER1_COMPA_vect, ISR_NOBLOCK){ //timer1 interrupt occurs with 50hz frequency, output motor pulses, must be nonblocking because micros is used
  unsigned long loop_timer = micros();
  unsigned long difference;
  // Motor pins high
  PORTB |= 0b11100000;
  PORTC |= 0b01000000;
  byte i = 0b00001111;

  // Wait until all pins #4 #5 #6 #7 are LOW
  while(i) {
    difference = micros() - loop_timer;
    
    if (difference >= pulse_length_esc4) {
      PORTC &= 0b10111111;
      i &= 0b00000111;
    } 
    if (difference >= pulse_length_esc3) {
      PORTB &= 0b10111111;
      i &= 0b00001011;
    } 
    if (difference >= pulse_length_esc2) {
      PORTB &= 0b11011111;
      i &= 0b00001101;
    } 
    if (difference >= pulse_length_esc1) {
      PORTB &= 0b01111111;
      i &= 0b00001110;
    }
    if(difference >= PULSE_MAX) {
       PORTB &= 0b00011111;
       PORTC &= 0b10111111;
       break;
    }
  }
}

/**
 * Calculate motor speed for each motor of an X quadcopter depending on received instructions and measures from sensor
 * by applying PID control.
 *
 * (4) (2)     x
 *   \ /     z ↑
 *    X       \|
 *   / \       +----→ y
 * (3) (1)
 *
 * Motors 4 & 1 run clockwise.
 * Motors 2 & 3 run counter-clockwise.
 *
 * Each motor output is considered as a servomotor. As a result, value range is about 1000µs to 2000µs
 *
 * @return void
 */
void pidController() {
  float a;
  float dt = (micros()-timer2)/1000000.0;
  timer2 = micros();
  
  // Calculate sum of errors : Integral coefficients
  orientationErrorSum += orientationError*dt;

  // Calculate error delta : Derivative coefficients
  filteryaw.input(orientation.yaw);//Highpass filter = differentiator + lowpass
  filterpitch.input(orientation.pitch);//Highpass filter = differentiator + lowpass
  filterroll.input(orientation.roll);//Highpass filter = differentiator + lowpass
  Vector3D delta = {filterroll.output(), filterpitch.output(), filteryaw.output()};

  // PID = e.Kp + ∫e.Ki + Δe.Kd
  Vector3D pid = orientationError.componentMultiply(ROLLp, PITCHp, YAWp) + 
                 orientationErrorSum.componentMultiply(ROLLi, PITCHi, YAWi) +
                 delta.componentMultiply(ROLLd, PITCHd, YAWd);

  // Calculate pulse duration for each ESC
  pulse_length_esc1 = constrain(holdingThrottle + pid.roll - pid.pitch - pid.yaw, PULSE_MIN, PULSE_MAX); 
  pulse_length_esc2 = constrain(holdingThrottle + pid.roll + pid.pitch + pid.yaw, PULSE_MIN, PULSE_MAX); 
  pulse_length_esc3 = constrain(holdingThrottle - pid.roll - pid.pitch + pid.yaw, PULSE_MIN, PULSE_MAX);
  pulse_length_esc4 = constrain(holdingThrottle - pid.roll + pid.pitch - pid.yaw, PULSE_MIN, PULSE_MAX);

#ifdef PRINTMOTORS

  Serial.println("M1 M2 M3 M4");
  Serial.print(pulse_length_esc1);
  Serial.print(" ");
  Serial.print(pulse_length_esc2);
  Serial.print(" ");
  Serial.print(pulse_length_esc3);
  Serial.print(" ");
  Serial.println(pulse_length_esc4);

#endif
}

void setup() {
  //Timer Setup
  TCCR1A = 0;             // normal counting mode
  TCCR1B = _BV(CS11);     // set prescaler of 8
  TCNT1 = 0;              // clear the timer count
  // set compare match register for 50hz increments
  OCR1A = 39999;// = (16*10^6) / (50*8) - 1 (must be <65536)
  TIFR1 |= _BV(OCF1A);      // clear any pending interrupts;
  TIMSK1 |=  _BV(OCIE1A) ;  // enable the output compare interrupt
  TCCR1B |= _BV(WGM12);   // Clear Timer on Compare Match

  //motors off
  setmotors(1000);
  PORTB &= 0b00011111;
  PORTC &= 0b10111111;
  
  pinMode(LEDPIN, OUTPUT);
  pinMode(IMURST, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);  
  digitalWrite(IMURST, 1);
  LEDON;
  Wire.begin();

  IMU_waitforboot();

  Serial.begin(115200);  //Setting the baudrate
  delay(2000);
  Serial.println("Start");

  write8(0x3F, 0b01000000); // System trigger: Internal oscillator, reset

  IMU_waitforboot();

  write8(0x3E, 0x00); // Power Mode: Normal: 0X00 (or B00), Low Power: 0X01 (or B01) , Suspend Mode: 0X02 (orB10)

  delay(50);

  IMU_EEPROMcalibrate(); //this puts sensor into NDOF mode

  write8(0x41, 0x21); // AXIS MAP CONFIG
  write8(0x42, 0x04); // AXIS MAP SIGN

  write8(0x3B, 0b0001000); //android orientation, fahrenheit, degrees, deg/s, m/s^2
  
  write8(0x3D, 0x0C); // Operation Mode: NDOF:0X0C (or B1100) , IMU:0x08 (or B1000) , NDOF_FMC_OFF: 0x0B (or B1011), GYROONLY 0b0011 / 0x03, AMG 0b0111 / 0x07
  
  delay(100);

  IMU_waitforboot();

  Serial.println("----------INIT DONE----------");
  IMU_syscheck();  
  delay(1000);
  Wire.beginTransmission(IMU);
  Wire.write(0x1A);  
  Wire.endTransmission(false);
  Wire.requestFrom(IMU, 6, true);
  targetOrientation.yaw = read16/16.00;
  targetOrientation.roll = read16/16.00;
  targetOrientation.pitch = read16/16.00;
  Serial.print("SETPOINTS (PITCH, ROLL, YAW): ");
  targetOrientation.println();
  delay(100);
  LEDOFF;
}

void loop() { //should run at 100hz
  unsigned long s = micros();
  IMU_linaccel(); //highest priority, must run at 100hz
  IMU_euler(); //necessary for angle pid
  pidController();
  
  s = micros()-s;
  
  //Serial.print("LOOP FREQENCY:"); Serial.println(1000000.0/s);
  if(s < 10000) { //max loop frequency  = 100hz
    delayMicroseconds(s);
  }
}
