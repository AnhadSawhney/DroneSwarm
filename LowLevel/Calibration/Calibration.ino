#include <EEPROM.h>
#include <Wire.h>

#define FORCECALIBRATION false

#define LEDPIN 7
#define LEDOFF PORTE |= 0b01000000
#define LEDON PORTE &= 0b10111111
#define LEDSTATE !(PORTE & 0b01000000)

#include "IMUfunctions.h"

uint16_t pulse_length_esc1,
        pulse_length_esc2,
        pulse_length_esc3,
        pulse_length_esc4;

void setmotors(uint16_t pulse) {
  pulse_length_esc1 = pulse;
  pulse_length_esc2 = pulse;
  pulse_length_esc3 = pulse;
  pulse_length_esc4 = pulse;
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

void setup() {
  setmotors(2000);
  //Timer Setup
  TCCR1A = 0;             // normal counting mode
  TCCR1B = _BV(CS11);     // set prescaler of 8
  TCNT1 = 0;              // clear the timer count
  // set compare match register for 50hz increments
  OCR1A = 39999;// = (16*10^6) / (50*8) - 1 (must be <65536)
  TIFR1 |= _BV(OCF1A);      // clear any pending interrupts;
  TIMSK1 |=  _BV(OCIE1A) ;  // enable the output compare interrupt
  TCCR1B |= _BV(WGM12);   // Clear Timer on Compare Match

  //esc calibration
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
  delay(3000);
  Serial.println("Start");
  setmotors(1000); //ESC CALIBRATION DONE

  write8(0x3F, 0b01000000); // System trigger: Internal oscillator, reset

  IMU_waitforboot();

  //write8(0x3F, 0b00000001); // System trigger: Internal oscillator, self test

  //IMU_waitforboot();

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
  delay(8000);
  LEDOFF;
}

void loop() { //done calibrating
  delay(50);
  LEDON;
  delay(50);
  LEDOFF;
}
