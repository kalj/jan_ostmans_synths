// (*) All in the spirit of open-source and open-hardware
// Janost 2017 Sweden
// Infinity37 – Poly DIY synth
// https://janostman.wordpress.com/infinity37-a-fully-polyphonic-diy-synth/
// Copyright 2017 DSP Synthesizers Sweden.
//
// Author: Jan Ostman
//
// This program is free software: you can redistribute it and/or modify
//it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

#define F_CPU 16000000L
#define USART_BAUDRATE 31250 
#define BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)

#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>

#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))


// Define various ADC prescaler
const unsigned char PS_16 = (1 << ADPS2);
const unsigned char PS_32 = (1 << ADPS2) | (1 << ADPS0);
const unsigned char PS_64 = (1 << ADPS2) | (1 << ADPS1);
const unsigned char PS_128 = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

uint16_t FREQ[37];         //DCO pitch
int16_t VCA[37];         //VCA level
uint8_t INTEGRATOR[37];

int16_t PWSTEP=1300;


// ------------- SYNTH PARAMETERS ---------------
volatile uint8_t ATTACK=30;           // ENV Attack rate 0-255
volatile uint8_t RELEASE=3;            // ENV Release rate 0-255
volatile uint16_t PW=0x8000;           // DCO Pulse width 1-32767
volatile uint8_t PWRATE=8;
volatile uint8_t PWMOD=120;
// --------------------------------------------

const uint8_t sinetable[256] PROGMEM = {
  127,130,133,136,139,143,146,149,152,155,158,161,164,167,170,173,176,178,181,184,187,190,192,195,198,200,203,205,208,210,212,215,217,219,221,223,225,227,229,231,233,234,236,238,239,240,
  242,243,244,245,247,248,249,249,250,251,252,252,253,253,253,254,254,254,254,254,254,254,253,253,253,252,252,251,250,249,249,248,247,245,244,243,242,240,239,238,236,234,233,231,229,227,225,223,
  221,219,217,215,212,210,208,205,203,200,198,195,192,190,187,184,181,178,176,173,170,167,164,161,158,155,152,149,146,143,139,136,133,130,127,124,121,118,115,111,108,105,102,99,96,93,90,87,84,81,78,
  76,73,70,67,64,62,59,56,54,51,49,46,44,42,39,37,35,33,31,29,27,25,23,21,20,18,16,15,14,12,11,10,9,7,6,5,5,4,3,2,2,1,1,1,0,0,0,0,0,0,0,1,1,1,2,2,3,4,5,5,6,7,9,10,11,12,14,15,16,18,20,21,23,25,27,29,31,
  33,35,37,39,42,44,46,49,51,54,56,59,62,64,67,70,73,76,78,81,84,87,90,93,96,99,102,105,108,111,115,118,121,124

};

volatile uint8_t lfocounter;
         uint8_t lfostat=1;
         
         uint8_t MIDISTATE=0;
         uint8_t MIDIRUNNINGSTATUS=0;
         uint8_t MIDINOTE;
         uint8_t MIDIVEL;

uint8_t MIDItable[128];
uint8_t k=3;
uint8_t envcnt=0;

void setup() {
  // Set PB1 as output.
  DDRB |= (1 << DDB1);
  pinMode(2,OUTPUT);
  pinMode(3,OUTPUT);
  pinMode(4,OUTPUT);
  pinMode(5,OUTPUT);
  pinMode(6,OUTPUT);
  pinMode(7,OUTPUT);
  
  pinMode(11,OUTPUT);
  pinMode(13,OUTPUT);
  digitalWrite(13,LOW);
  pinMode(12,INPUT_PULLUP);
  
 // Set up Timer 1 to send a sample every interrupt.

    cli();

    // Set CTC mode
    // Have to set OCR1A *after*, otherwise it gets reset to 0!
    TCCR1B = (TCCR1B & ~_BV(WGM13)) | _BV(WGM12);
    TCCR1A = TCCR1A & ~(_BV(WGM11) | _BV(WGM10));
    
    // No prescaler
    TCCR1B = (TCCR1B & ~(_BV(CS12) | _BV(CS11))) | _BV(CS10);

    // Set the compare register (OCR1A).
    // OCR1A is a 16-bit register, so we have to do this with
    // interrupts disabled to be safe.
    OCR1A = 955;//F_CPU / SAMPLE_RATE; 

    // Enable interrupt when TCNT1 == OCR1A
    TIMSK1 |= _BV(OCIE1A);

    //set timer0 interrupt at 61Hz
    TCCR0A = 0;// set entire TCCR0A register to 0
    TCCR0B = 0;// same for TCCR0B
    TCNT0  = 0;//initialize counter value to 0
    // set compare match register for 62hz increments
    OCR0A = 255;// = 61Hz
    // turn on CTC mode
    TCCR0A |= (1 << WGM01);
    // Set CS01 and CS00 bits for prescaler 1024
    TCCR0B |= (1 << CS02) | (0 << CS01) | (1 << CS00);  //1024 prescaler 
    // enable timer compare interrupt
    TIMSK0 |= (1 << OCIE0A); 
    sei();
    // Set up Timer 2 to do pulse width modulation on the speaker
    // pin.

    // Use internal clock (datasheet p.160)
    ASSR &= ~(_BV(EXCLK) | _BV(AS2));

    // Set fast PWM mode  (p.157)
    TCCR2A |= _BV(WGM21) | _BV(WGM20);
    TCCR2B &= ~_BV(WGM22);
        TCCR2A = (TCCR2A | _BV(COM2A1)) & ~_BV(COM2A0);
        TCCR2A &= ~(_BV(COM2B1) | _BV(COM2B0));
        // No prescaler (p.158)
        TCCR2B = (TCCR2B & ~(_BV(CS12) | _BV(CS11))) | _BV(CS10);



        // Set initial pulse width to the first sample.
        OCR2A = 128;

  
// Set baud rate to 31,250. Requires modification if clock speed is not 16MHz.
   UCSR0B = (1 << RXEN0);   // Turn on reception circuitry 
   UCSR0C = (1 << UCSZ00) | (1 << UCSZ01); // Use 8-bit character sizes 

   UBRR0H = (BAUD_PRESCALE >> 8); // Load upper 8-bits of the baud rate value into the high byte of the UBRR register 
   UBRR0L = BAUD_PRESCALE; // Load lower 8-bits of the baud rate value into the low byte of the UBRR register

    // USART RX interrupt enable bit on
    UCSR0B |= _BV(RXCIE0);
   
  // set up the ADC
     
  if (analogRead(0));  //Kick the analog reading to work  
  
  ADCSRA &= ~PS_128;  // remove bits set by Arduino library

  // you can choose a prescaler from above.
  // PS_64 or PS_128
  ADCSRA |= PS_16;    // set our own prescaler to 16 

}


ISR(TIMER1_COMPA_vect) { // Timer1 interrupt
  //---- PolyTron DCO's --------------------------- 
  uint16_t Total=0;
  uint16_t Total2=0;
  if (VCA[0]) {
    FREQ[0]+=8192>>5;
    if (FREQ[0]<PW) Total+=VCA[0];
  }
  if (VCA[1]) {
    FREQ[1]+=8679>>5;
    if (FREQ[1]<PW) Total+=VCA[1];
  }
    if (VCA[2]) {
    FREQ[2]+=9195>>5;
    if (FREQ[2]<PW) Total+=VCA[2];
  }
  if (VCA[3]) {
    FREQ[3]+=9742>>5;
    if (FREQ[3]<PW) Total+=VCA[3];
  }
  if (VCA[4]) {
    FREQ[4]+=10321>>5;
    if (FREQ[4]<PW) Total+=VCA[4];
  }
  if (VCA[5]) {
    FREQ[5]+=10935>>5;
    if (FREQ[5]<PW) Total+=VCA[5];
  }
  if (VCA[6]) {
    FREQ[6]+=11585>>5;
    if (FREQ[6]<PW) Total+=VCA[6];
  }
  if (VCA[7]) {
    FREQ[7]+=12274>>5;
    if (FREQ[7]<PW) Total+=VCA[7];
  }
  if (VCA[8]) {
    FREQ[8]+=13004>>5;
    if (FREQ[8]<PW) Total+=VCA[8];
  }
  if (VCA[9]) {
    FREQ[9]+=13777>>5;
    if (FREQ[9]<PW) Total+=VCA[9];
  }
  if (VCA[10]) {
    FREQ[10]+=14596>>5;
    if (FREQ[10]<PW) Total+=VCA[10];
  }
  if (VCA[11]) {
    FREQ[11]+=15464>>5;
    if (FREQ[11]<PW) Total+=VCA[11];
  }
  if (VCA[0+12]) {
    FREQ[0+12]+=8192>>4;
    if (FREQ[0+12]<PW) Total+=VCA[0+12];
  }
  if (VCA[1+12]) {
    FREQ[1+12]+=8679>>4;
    if (FREQ[1+12]<PW) Total+=VCA[1+12];
  }
    if (VCA[2+12]) {
    FREQ[2+12]+=9195>>4;
    if (FREQ[2+12]<PW) Total+=VCA[2+12];
  }
  if (VCA[3+12]) {
    FREQ[3+12]+=9742>>4;
    if (FREQ[3+12]<PW) Total+=VCA[3+12];
  }
  if (VCA[4+12]) {
    FREQ[4+12]+=10321>>4;
    if (FREQ[4+12]<PW) Total+=VCA[4+12];
  }
  if (VCA[5+12]) {
    FREQ[5+12]+=10935>>4;
    if (FREQ[5+12]<PW) Total+=VCA[5+12];
  }
  if (VCA[6+12]) {
    FREQ[6+12]+=11585>>4;
    if (FREQ[6+12]<PW) Total+=VCA[6+12];
  }
  if (VCA[7+12]) {
    FREQ[7+12]+=12274>>4;
    if (FREQ[7+12]<PW) Total+=VCA[7+12];
  }
  if (VCA[8+12]) {
    FREQ[8+12]+=13004>>4;
    if (FREQ[8+12]<PW) Total+=VCA[8+12];
  }
  if (VCA[9+12]) {
    FREQ[9+12]+=13777>>4;
    if (FREQ[9+12]<PW) Total+=VCA[9+12];
  }
  if (VCA[10+12]) {
    FREQ[10+12]+=14596>>4;
    if (FREQ[10+12]<PW) Total+=VCA[10+12];
  }
  if (VCA[11+12]) {
    FREQ[11+12]+=15464>>4;
    if (FREQ[11+12]<PW) Total+=VCA[11+12];
  }
  if (VCA[0+24]) {
    FREQ[0+24]+=8192>>3;
    if (FREQ[0+24]<PW) Total+=VCA[0+24];
  }
  if (VCA[1+24]) {
    FREQ[1+24]+=8679>>3;
    if (FREQ[1+24]<PW) Total+=VCA[1+24];
  }
    if (VCA[2+24]) {
    FREQ[2+24]+=9195>>3;
    if (FREQ[2+24]<PW) Total+=VCA[2+24];
  }
  if (VCA[3+24]) {
    FREQ[3+24]+=9742>>3;
    if (FREQ[3+24]<PW) Total+=VCA[3+24];
  }
  if (VCA[4+24]) {
    FREQ[4+24]+=10321>>3;
    if (FREQ[4+24]<PW) Total+=VCA[4+24];
  }
  if (VCA[5+24]) {
    FREQ[5+24]+=10935>>3;
    if (FREQ[5+24]<PW) Total+=VCA[5+24];
  }
  if (VCA[6+24]) {
    FREQ[6+24]+=11585>>3;
    if (FREQ[6+24]<PW) Total+=VCA[6+24];
  }
  if (VCA[7+24]) {
    FREQ[7+24]+=12274>>3;
    if (FREQ[7+24]<PW) Total+=VCA[7+24];
  }
  if (VCA[8+24]) {
    FREQ[8+24]+=13004>>3;
    if (FREQ[8+24]<PW) Total+=VCA[8+24];
  }
  if (VCA[9+24]) {
    FREQ[9+24]+=13777>>3;
    if (FREQ[9+24]<PW) Total+=VCA[9+24];
  }
  if (VCA[10+24]) {
    FREQ[10+24]+=14596>>3;
    if (FREQ[10+24]<PW) Total+=VCA[10+24];
  }
  if (VCA[11+24]) {
    FREQ[11+24]+=15464>>3;
    if (FREQ[11+24]<PW) Total+=VCA[11+24];
  }
  if (VCA[0+36]) {
    FREQ[0+36]+=8192>>2;
    if (FREQ[0+36]<PW) Total+=VCA[0+36];
  }
  //-----------------------------------------------------------------
  
  OCR2A = Total>>5;
}

ISR(TIMER0_COMPA_vect) { // Timer0 interrupt
  
  //--------------------- ENV block ---------------------------------
  for (envcnt=0;envcnt<37;envcnt++) { 
    if ((MIDItable[envcnt]>0)&&(VCA[envcnt]<255)) {
      VCA[envcnt]+=ATTACK;
      if (VCA[envcnt]>255) VCA[envcnt]=255;
    }
    if ((MIDItable[envcnt]==0)&&(VCA[envcnt]>0)) {
      VCA[envcnt]-=RELEASE;
      if (VCA[envcnt]<0) VCA[envcnt]=0;
    }
  }

  //-----------------------------------------------------------------

  //------------------------------ LFO Block -----------------------
   
    lfocounter+=PWRATE;
    PW=32767-((pgm_read_byte_near( sinetable + lfocounter ) * PWMOD));  //LFO for pitch
    
    
    //----------------------------------------------------------------- 

    
    
}


ISR(USART_RX_vect) {
      uint8_t MIDIRX;

	  MIDIRX = UDR0;

	  /*
	  Handling "Running status"
	  1.Buffer is cleared (ie, set to 0) at power up.
	  2.Buffer stores the status when a Voice Category Status (ie, 0x80 to 0xEF) is received.
	  3.Buffer is cleared when a System Common Category Status (ie, 0xF0 to 0xF7) is received.
	  4.Nothing is done to the buffer when a RealTime Category message is received.
	  5.Any data bytes are ignored when the buffer is 0.
	  */
          if (MIDIRX>0xF7) return;

	  if ((MIDIRX>0xEF)&&(MIDIRX<0xF8)) {
		  MIDIRUNNINGSTATUS=0;
		  MIDISTATE=0;
		  return;
	  }

	  if (MIDIRX & 0x80) {
		  MIDIRUNNINGSTATUS=MIDIRX;
		  MIDISTATE=1;
		  return;
	  }

	  if (MIDIRX < 0x80) {
	  	  if (!MIDIRUNNINGSTATUS) return;
	  	  if (MIDISTATE==1) {
	  		  MIDINOTE=MIDIRX;
	  		  MIDISTATE++;
	  		  return;
	  	  }
	  	  if (MIDISTATE==2) {
	  		  MIDIVEL=MIDIRX;
	  		  MIDISTATE=1;
	  		  if ((MIDIRUNNINGSTATUS==0x80)||(MIDIRUNNINGSTATUS==0x90)) {
                            if ((MIDINOTE>35)||(MIDINOTE<96)) {
                              if (MIDIRUNNINGSTATUS==0x80) MIDIVEL=0;
                              MIDItable[MIDINOTE]=MIDIVEL;
                            }
                          }
	  		  //if ((MIDIRUNNINGSTATUS==0xB0)&&(MIDINOTE==1)) MIDILFOMOD=MIDIVEL;
          //if (MIDIRUNNINGSTATUS==0xE0) PITCHBEND=127-MIDIVEL;
	  		  return;
	  	  }
	  	  }

	  return;
}

 
void loop() {
  
      PORTD = (k<<2);
      if (digitalRead(12)) {
      //if (PORTB&16) {
       MIDItable[k-3]=0;
       digitalWrite(13,LOW);
      }
      else {
       MIDItable[k-3]=255;
       digitalWrite(13,HIGH);
      }
      k++;
      if (k==40) {
        k=3;
      }
    PORTD |= B11111100;

}


