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

#define YAW      0
#define ROLL     1
#define PITCH    2

#define YAWp 0.0 //3.0
#define PITCHp 2.0// 2.0
#define ROLLp 0.0

#define YAWi 0
#define PITCHi 0
#define ROLLi 0

#define YAWd 0.0 //-2.0
#define PITCHd 2.0//must be positive
#define ROLLd 0.0

#define holdingthrottle 1300 //should be 1400
#define THROTTLE_LIMIT 2000

// ------------- Global variables used for PID controller --------------------
Vector3D orientation, orientationSetpoint, orientationError, orientationErrorSum, velocity;

unsigned long pulse_length_esc1,
        pulse_length_esc2,
        pulse_length_esc3,
        pulse_length_esc4,
        timer1 = 0, 
        timer2 = 0; 
        //timer3 = 0;

FilterOnePole filterx(HIGHPASS, 1);
FilterOnePole filtery(HIGHPASS, 1);
FilterOnePole filterz(HIGHPASS, 1);
FilterOnePole filteryaw(HIGHPASS, 1);
FilterOnePole filterpitch(HIGHPASS, 1);
FilterOnePole filterroll(HIGHPASS, 1);

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
  orientationSetpoint.fix(orientation);
  orientationError = orientationSetpoint - orientation;
  //Serial.println("Y R P");
  //Serial.print(measures[PITCH]);
  //Serial.print(" ");
  //Serial.println(instruction[PITCH]);
  //Serial.print(" ");
  //Serial.println(errors[PITCH]);
}

/*  Undo rotation by quaternion
 *  1.0-2.0*(y*y+z*z) 2.0*(x*y-z*w)     2.0*(x*z+y*w)       accx
 *  2.0*(x*y+z*w)     1.0-2.0*(x*x+z*z) 2.0*(y*z+x*w)     * accy
 *  2.0*(x*z-y*w)     2.0*(y*z-x*w)     1.0-2.0*(x*x+y*y)   accz
 */

void IMU_linaccel() { //Absolute, based on north and gravity
  Wire.beginTransmission(IMU);
  Wire.write(0x28);
  Wire.endTransmission(false);
  Wire.requestFrom(IMU, 6, true);
  Vector3D acc = Vector3D(read16/100.0, read16/100.0, read16/100.0); // m/s^2

  Wire.beginTransmission(IMU);
  Wire.write(0x20);
  Wire.endTransmission(false);
  Wire.requestFrom(IMU, 8, true);
  acc = acc.quaternionRotate(read16/16384.0, read16/16384.0, read16/16384.0, read16/16384.0);

  unsigned long t = micros();
  float dt = (t-timer1)/1000000.0;
  
  if(timer1 = 0){
    velocity = Vector3D();
  } else {
    velocity += acc*dt;
  }
  
  filterx.input(velocity.x);
  filtery.input(velocity.y);
  filterz.input(velocity.z);

  //Serial.println(dt);
  
  timer1 = t;
  
  //Serial.print(", ");
  //Serial.print(filterx.output());
  //Serial.print(",");
  //Serial.print(filtery.output());
  //Serial.print(",");
  //Serial.println(filterz.output());
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
    if(difference >= 2000) {
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
  unsigned long t = micros();
  float dt = (t-timer2)/1000000.0;
  
  // Calculate sum of errors : Integral coefficients
  orientationErrorSum += orientationError*dt;

  // Calculate error delta : Derivative coefficients
  filteryaw.input(orientation.yaw);//Highpass filter = differentiator + lowpass
  filterpitch.input(orientation.pitch);//Highpass filter = differentiator + lowpass
  filterroll.input(orientation.roll);//Highpass filter = differentiator + lowpass
  Vector3D delta = Vector3D(filterpitch.output(), filterroll.output(), filteryaw.output());

  // PID = e.Kp + ∫e.Ki + Δe.Kd
  Vector3D pid = orientationError.componentMultiply(PITCHp, ROLLp, YAWp) + 
                 orientationErrorSum.componentMultiply(PITCHi, ROLLi, YAWi) +
                 delta.componentMultiply(PITCHd, ROLLd, YAWd);

  // Calculate pulse duration for each ESC
  pulse_length_esc1 = constrain(holdingthrottle + pid.roll + pid.pitch - pid.yaw, 1000, THROTTLE_LIMIT);
  pulse_length_esc2 = constrain(holdingthrottle + pid.roll - pid.pitch + pid.yaw, 1000, THROTTLE_LIMIT);
  pulse_length_esc3 = constrain(holdingthrottle - pid.roll + pid.pitch + pid.yaw, 1000, THROTTLE_LIMIT);
  pulse_length_esc4 = constrain(holdingthrottle - pid.roll - pid.pitch - pid.yaw, 1000, THROTTLE_LIMIT);

  Serial.println("M1 M2 M3 M4");
  Serial.print(pulse_length_esc1);
  Serial.print(" ");
  Serial.print(pulse_length_esc2);
  Serial.print(" ");
  Serial.print(pulse_length_esc3);
  Serial.print(" ");
  Serial.println(pulse_length_esc4);
  timer2 = t;
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
  orientationSetpoint.yaw = fixangle(read16/16.00, orientationSetpoint.yaw);
  orientationSetpoint.roll = fixangle(read16/16.00, orientationSetpoint.roll);
  orientationSetpoint.pitch = fixangle(read16/16.00, orientationSetpoint.pitch);
  Serial.print("SETPOINTS (PITCH, ROLL, YAW): ");
  orientationSetpoint.println();
  delay(100);
  LEDOFF;
}

unsigned long previousMillis = 0;
byte calib;

void loop() { //should run at 100hz
  unsigned long s = micros();
  IMU_linaccel(); //highest priority, must run at 100hz
  IMU_euler(); //necessary for angle pid
  pidController();
  
  s = micros()-s;
  
  //Serial.print("LOOP FREQENCY:"); Serial.println(1000000.0/s);
  if(s < 10000) {
    delayMicroseconds(s);
  }/* else {
    Serial.println("WARN: LOOP FREQENCY <100HZ");
  }*/
}
