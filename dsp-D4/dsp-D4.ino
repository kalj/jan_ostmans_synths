
//dsp-D4 drumchip (c)2015 DSP Synthesizers Sweden
//
//This code is free for non commercial use.
//If used a referens to DSP Synthesizers must be included.

#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>

//--------- Ringbuf parameters ----------
uint8_t Ringbuffer[256];
uint8_t RingWrite=0;
uint8_t RingRead=0;
volatile uint8_t RingCount=0;
//-----------------------------------------


const uint8_t BD[2154] PROGMEM =
{
  128,128,128,128,128,128,128,127,127,127,127,127,127,127,127,127,127,127,127,127,126,126,126,126,126,126,126,126,126,125,125,125,125,124,124,124,124,123,123,121,119,84,61,59,53,54,53,55,55,58,58,60,61,62,63,64,65,67,68,69,71,72,73,75,76,78,79,81,82,83,85,87,88,89,91,92,94,96,99,105,115,129,145,162,178,191,203,214,222,227,231,233,233,232,230,229,228,226,225,224
};

const uint8_t CH[482] PROGMEM =
{
  128,128,128,128,132,123,145,128,96,150,66,110,222,81,135,100,99,144,77,189,109,92,196,75,153,215,81,54,186,149,154,92,131,173,146,72,136,109,47,133,200,85,107,232,179,87,115,120,163,223,81,208,149,163,157,130,177,170,139,148,150,175,151,126,180,130,88,135,49,42,212,160,42,103,96,114,213,135,101,141,33,94,140,30,23,85,188,95,136,112,109,139,67,79,172,79,47,138,203,82
};

const uint8_t RS[1160] PROGMEM =
{
  128,128,128,128,128,127,128,127,128,126,129,129,130,86,38,42,42,45,46,49,51,54,60,77,125,200,224,226,226,224,223,221,219,218,216,214,212,212,211,209,208,206,204,202,200,198,196,193,188,174,132,54,21,19,19,22,24,27,30,33,41,61,119,188,204,207,206,205,203,202,200,199,197,196,195,194,193,191,190,188,187,185,184,181,180,176,173,162,134,64,9,3,1,4,6,8,10,12,14,16
};

const uint8_t SN[2212] PROGMEM =
{
  127,129,134,129,130,135,130,141,128,119,138,136,125,118,133,125,150,131,138,135,144,117,170,120,119,150,120,156,111,101,145,133,102,121,135,133,82,37,47,47,28,39,35,41,40,44,42,47,46,50,49,55,54,59,57,65,62,85,98,130,147,166,215,200,228,248,242,237,246,232,230,243,236,237,233,233,225,231,225,228,219,223,201,175,209,159,134,160,98,130,96,62,69,75,94,32
};

void setup() {
  OSCCAL=255;
  // Enable 64 MHz PLL and use as source for Timer1
  PLLCSR = 1<<PCKE | 1<<PLLE;

  // Set up Timer/Counter1 for PWM output
  TIMSK = 0; // Timer interrupts OFF
  TCCR1 = 1<<PWM1A | 2<<COM1A0 | 1<<CS10; // PWM A, clear on match, 1:1 prescale
                                          //GTCCR = 1<<PWM1B | 2<<COM1B0; // PWM B, clear on match
  OCR1A = 128; //OCR1B = 128; // 50% duty at start

  pinMode(0, INPUT_PULLUP);
  pinMode(1, OUTPUT); // Enable PWM output pin
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);

  //Set up Timer/Counter0 for 40kHz interrupt to output samples.

  TCCR0A = 3<<WGM00; // Fast PWM
  TCCR0B = 1<<WGM02 | 2<<CS00; // 1/8 prescale
  TIMSK = 1<<OCIE0A; // Enable compare match, disable overflow
  OCR0A = 24; // Divide by 200

}

void loop() {
  uint16_t samplecntBD,samplecntSN,samplecntCH,samplecntRS;
  uint16_t samplepntBD,samplepntSN,samplepntCH,samplepntRS;
  while(1) {
    if (RingCount<255) { //if space in ringbuffer
      int16_t total=0;
      if (samplecntBD) {
        total+=(pgm_read_byte_near(BD + samplepntBD++)-128);
        samplecntBD--;
      }
      if (samplecntSN) {
        total+=(pgm_read_byte_near(SN + samplepntSN++)-128);
        samplecntSN--;
      }
      if (samplecntCH) {
        total+=(pgm_read_byte_near(CH + samplepntCH++)-128);
        samplecntCH--;
      }
      if (samplecntRS) {
        total+=(pgm_read_byte_near(RS + samplepntRS++)-128);
        samplecntRS--;
      }
      total>>=1;
      total+=128;
      if (total>255) total=255;
      cli();
      Ringbuffer[RingWrite]=total;
      RingWrite++;
      RingCount++;
      sei();
      if (digitalRead(3)) {
        samplepntBD=0;
        samplecntBD=2154;
      }
      if (digitalRead(0)) {
        samplepntCH=0;
        samplecntCH=482;
      }
      if (digitalRead(2)) {
        samplepntRS=0;
        samplecntRS=1160;
      }
      if (digitalRead(4)) {
        samplepntSN=0;
        samplecntSN=2212;
      }
    }
  }
}

ISR(TIMER0_COMPA_vect) {
  //------------------- Ringbuffer handler -------------------------

  if (RingCount) { //If entry in FIFO..
    OCR1A = Ringbuffer[(RingRead++)]; //Output to DAC
    RingCount--;
  }

  //-----------------------------------------------------------------
} 
