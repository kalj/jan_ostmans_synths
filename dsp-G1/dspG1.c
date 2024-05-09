/*
Copyright 2016 DSP Synthesizers Sweden.
Author: Jan Ostman 
This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
See the GNU General Public License for more details.
*/

#include "wiring.h"
#include "LPC8xx.h"
#include "sct_fsm.h"

/* Control register bit definition. */
#define MRT_INT_ENA          (0x1<<0)
#define MRT_REPEATED_MODE    (0x00<<1)
#define MRT_ONE_SHOT_INT     (0x01<<1)
#define MRT_ONE_SHOT_STALL   (0x02<<1)

/* Status register bit definition */
#define MRT_STAT_IRQ_FLAG    (0x1<<0)
#define MRT_STAT_RUN         (0x1<<1)

#define SPI_CFG_ENABLE (0x1)
#define SPI_CFG_MASTER (0x4)
#define SPI_STAT_RXRDY (0x1)
#define SPI_STAT_TXRDY (0x2)
#define SPI_STAT_SSD (0x20)
#define SPI_STAT_MSTIDLE (0x100)
#define SPI_TXDATCTL_SSEL_N(s) ((s) << 16)
#define SPI_TXDATCTL_EOT (1 << 20)
#define SPI_TXDATCTL_EOF (1 << 21)
#define SPI_TXDATCTL_RXIGNORE (1 << 22)
#define SPI_TXDATCTL_FLEN(l) ((l) << 24)

//-------- Synth parameters --------------
uint32_t FREQ[15]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; //OSC pitches
volatile int32_t CUTOFF=65535;     //freq 0-65535
volatile int32_t CFREQ=65535;      //freq 0-65535
volatile int32_t RESONANCE=0;      //reso=0-65535
volatile uint8_t TRIG=1;           //MIDItrig 1=note ON
volatile uint32_t ATTACK0=0xFFFF;  //Attackrate in 30mS steps
volatile uint32_t DECAY0=0xFFFF;       //Decayrate in 30mS steps
volatile int32_t SUSTAIN0=0xFFFF;  //Sustainlevel 0-65535
volatile uint32_t RELEASE0=0xFFFF;    //Releaserate in 30mS steps
volatile uint32_t ATTACK1=0xFFFF;	   //Attackrate in 30mS steps
volatile uint32_t DECAY1=0xFFFF;       //Decayrate in 30mS steps
volatile int32_t SUSTAIN1=0xFFFF;  //Sustainlevel 0-65535
volatile uint32_t RELEASE1=0xFFFF;      //Releaserate in 30mS steps
volatile uint32_t DETUNE=1;		   //Osc spread or detune
volatile uint32_t RANGE=64;		   //Osc range
volatile uint8_t ENVMOD=0;		   //Filter ENVMOD
volatile uint8_t LFOMOD=0;		   //Filter LFOMOD
volatile uint8_t WAVEFORM=255;	   //OSC Waveform
volatile uint8_t WRAP=0;		   //OSC Wrap
volatile uint8_t LFORATE=0;		   //LFO frequency
volatile uint8_t LFOWAVE=0;		   //LFO waveform
volatile int32_t MASTERVOL=65535;  //Master volume
volatile uint8_t CHORUSRATE=0;     //Chorus rate
volatile uint8_t CHORUSMOD=0;      //Chorus modlevel
volatile int32_t CHORUSMIX;        //Chorus mix level
volatile int32_t CHORUSFB;         //Chorus feedback level
//-----------------------------------------

volatile uint8_t GATED=0;

volatile uint16_t DACL=1;
volatile uint16_t DACR=1;
uint32_t DCOPH[15];

volatile uint8_t state0=0;    //needs to be reset on a new trig
volatile uint8_t state1=0;    //needs to be reset on a new trig
volatile int32_t volume0=0;
volatile int32_t volume1=0;
volatile uint16_t envstat=1;
volatile uint16_t lfostat=1;

int32_t t[9];

int32_t vi;
int32_t vo;

int16_t DCO;
int16_t DCF;
int32_t ENV;

int32_t ENVsmoothing;

uint8_t lfocounter=0;
uint32_t next = 1;
volatile uint8_t shrandom;
uint8_t chrcounter=0;
uint8_t pwmcounter=0;

int8_t waveform[256];
const uint32_t NOTES[12]={208065>>2,220472>>2,233516>>2,247514>>2,262149>>2,277738>>2,294281>>2,311779>>2,330390>>2,349956>>2,370794>>2,392746>>2};

volatile uint8_t MIDISTATE=0;
volatile uint8_t MIDIRUNNINGSTATUS=0;
volatile uint8_t MIDINOTE;
volatile uint8_t MIDIVEL;
uint8_t OSCNOTES[5];
const uint16_t ENVRATES[32] = {65535>>2,61540>>2,57544>>2,53548>>2,49552>>2,45556>>2,41560>>2,37564>>2,33568>>2,29572>>2,25576>>2,21580>>2,17584>>2,13588>>2,9592>>2,5596>>2,1600>>2,1503>>2,1407>>2,1311>>2,1215>>2,1119>>2,1022>>2,926>>2,830>>2,734>>2,638>>2,541>>2,445>>2,349>>2,253>>2,157>>2 };

/*int8_t delayline[2048];
volatile uint16_t writeptr = 0;
volatile uint16_t delayL = 0;
volatile uint16_t delayR = 1023;
volatile int16_t leftCH;
volatile int16_t rightCH;
volatile int32_t chmixL;
volatile int32_t chmixR;
*/


//---------------- Get the base frequency for the MIDI note ---------------
uint32_t MIDI2FREQ(uint8_t note) {
  uint8_t key=note%12;
  if (note<36) return (NOTES[key]>>(1+(35-note)/12));
  if (note>47) return (NOTES[key]<<((note-36)/12));
  return NOTES[key];
}
//-------------------------------------------------------------------------

void MRT_IRQHandler(void) {
  if ( LPC_MRT->Channel[0].STAT & MRT_STAT_IRQ_FLAG ) {
    LPC_MRT->Channel[0].STAT = MRT_STAT_IRQ_FLAG;      /* clear interrupt flag */

    uint8_t i;

    //-------------------- 15 DCO block ------------------------------------------
    DCO=0;
    for (i=0;i<15;i++) {
    	DCOPH[i] += FREQ[i];              //Add freq to phaseacc's
    		DCO += waveform[(DCOPH[i]>>15)&255];  //Add DCO's to output
    }
    DCO = DCO<<4;
    //---------------------------------------------------------------------------

    //---------------- DCF block ---------------------------------------
    //C implementation for a 4pole lowpass DCF with resonance:

    vi=DCO;
    for (i=0;i<8;i++) {
    t[i] = ((t[i] * (65536 - CFREQ)) + (t[i+1] * CFREQ))>>16;  //+3dB per iteration
    }
    t[8] = vi-((t[0]*RESONANCE)>>16);  //resonance feedback
    DCF=t[0];
    //-----------------------------------------------------------------

    //--------------------- ENV block ---------------------------------
    if (!envstat--) {
    envstat=256;
    if (TRIG) {

    	if (state0==1) {
    		volume0 -= DECAY0;
    		if (SUSTAIN0>volume0) {
    			volume0=SUSTAIN0;
    			state0++;
    		}
    	}
    	if (state0==0) {
    		volume0 += ATTACK0;
    		if (volume0>0xFFFF) {
    			volume0=0xFFFF;
    			state0++;
    		}
    	}
    	if (state1==1) {
    		volume1 -= DECAY1;
    	    if (SUSTAIN1>volume1) {
    	        volume1=SUSTAIN1;
    	        state1++;
    	    }
    	}

    	if (state1==0) {
    		volume1 += ATTACK1;
    	    if (volume1>0xFFFF) {
    	    	volume1=0xFFFF;
    	    	state1++;
    	    }
    	}
    }
    if (!TRIG) {
    		volume0 -= RELEASE0;
    		if (volume0<0) {
    			volume0=0;
    		}

    		volume1 -= RELEASE1;
    		if (volume1<0) {
    		    volume1=0;
    		}

    }
    }

    //ENV=(volume0*DCF)>>16;
    ENVsmoothing += (volume0-ENVsmoothing)>>8;
    ENV=(ENVsmoothing*DCF)>>16;

    if (!lfostat--) {
        lfostat=1024;

    //-------------------- LFO block ----------------------------------
    int8_t TRILFO;
    lfocounter+=LFORATE;
    TRILFO=(lfocounter&127);
    if (lfocounter&128) TRILFO=127-(lfocounter&127);
    TRILFO=127-(TRILFO<<1);
    if (LFOWAVE<64) {
    	CFREQ=(int32_t)((TRILFO*LFOMOD)+((volume1>>8)*ENVMOD));
    	CFREQ=CFREQ+CUTOFF;
    	if (CFREQ>65535) CFREQ=65535;
    	if (CFREQ<0) CFREQ=0;
    }
    else {
    	if (lfocounter&128) {
    		CFREQ=(int32_t)((shrandom*LFOMOD)+((volume1>>8)*ENVMOD));
    		CFREQ=CFREQ+CUTOFF;
    		if (CFREQ>65535) CFREQ=65535;
    		if (CFREQ<0) CFREQ=0;
    		lfocounter=0;
    	}
    }


    /*
    chrcounter+=CHORUSRATE;
    uint8_t chrlfo=(chrcounter&127);
    if (chrcounter&128) chrlfo=127-(chrcounter&127);
    chrlfo=chrlfo<<1;
    //Set right & left delayline taps
    delayR=(CHORUSMOD*chrlfo)>>6;
    delayL=(CHORUSMOD*(255-chrlfo))>>6;
    */
	}
    //-----------------------------------------------------------------



    //-----------------------------------------------------------------

    //---------------- Stereo Chorus/Flanger block ---------------------
    /*
    //Right & left delay line taps
    leftCH=delayline[(delayL+writeptr)&0x7FF]<<8;
    rightCH=delayline[(delayR+writeptr)&0x7FF]<<8;
    //Dry & wet mix level
    chmixL=((leftCH*CHORUSMIX)>>16)+((ENV*(65535-CHORUSMIX))>>16);
    chmixR=((rightCH*CHORUSMIX)>>16)+((ENV*(65535-CHORUSMIX))>>16);
    //Delay line feedback
    delayline[writeptr++]=(ENV-(leftCH*CHORUSFB)-(rightCH*CHORUSFB))>>8;
    writeptr&=0x7FF;
    */
    //-------------------------------------------------------------------

    ENV=(MASTERVOL*ENV)>>16;
    //chmixR=(MASTERVOL*ENV)>>16;

    DACL=513+(ENV>>6);
    //DACR=513+(chmixR>>6);
	LPC_SCT->MATCHREL[1].L  = DACL; //Left Audio Output
	//LPC_SCT->MATCHREL[1].H  = DACR; //Right Audio Output

  }

  return;
}

void handleMIDICC(uint8_t CC,uint8_t value) {
	uint8_t i;


	/*
	 MIDI CCs:

	 CUTOFF       = CC#74
	 RESONANCE    = CC#71
	 FILTER A     = CC#82
	 FILTER D     = CC#83
	 FILTER S     = CC#28
	 FILTER R     = CC#29
	 FILTER ENV M = CC#81
	 FILTER LFO M = CC#01

	 WAVEFORM     = CC#76
	 WRAP		  = CC#04
	 RANGE        = CC#21
	 DETUNE       = CC#93
	 OSCENV A     = CC#73
	 OSCENV D     = CC#75
	 OSCENV S     = CC#31
	 OSCENV R     = CC#72

	 MASTERVOL    = CC#07
	 LFORATE      = CC#16
	 LFOWAVE      = CC#20

	 CHORUS		  = CC#12

	 My keyboard:
	 Filter Resonance (Timbre/Harmonic Intensity) (cc#71)
     Release Time (cc#72)
	 Attack time (cc#73)
	 Brightness/Cutoff Frequency (cc#74)
	 Decay Time (cc#75)
	 Vibrato Rate (cc#76)
	 Vibrato Depth (cc#77)
     Vibrato Delay (cc#78)

	 */
	switch(CC) {
	case 74:
		CUTOFF=value<<9;
	break;
	case 71:
		RESONANCE=value<<9;
	break;
	case 82:
		ATTACK1=ENVRATES[value>>2];
	break;
	case 83:
		DECAY1=ENVRATES[value>>2];
	break;
	case 28:
		SUSTAIN1=value<<9;
	break;
	case 29:
		RELEASE1=ENVRATES[value>>2];
	break;
	case 73:
		ATTACK0=ENVRATES[value>>2];
	break;
	case 75:
		DECAY0=ENVRATES[value>>2];
	break;
	case 31:
		SUSTAIN0=value<<9;
	break;
	case 72:
		RELEASE0=ENVRATES[value>>2];
		GATED=value;
	break;
	case 93:
		DETUNE=value;
		for (i=0;i<5;i++) {
		  if (FREQ[i*3]) {
			  FREQ[i*3+1]=FREQ[i*3]+((FREQ[i*3]/50)*DETUNE/127)+((FREQ[i*3]/2)*RANGE/32);
			  FREQ[i*3+2]=FREQ[i*3]-((FREQ[i*3]/50)*DETUNE/127)+((FREQ[i*3]/2)*RANGE/32);
		  }
		}
	break;
	case 21:
		RANGE=value;
		for (i=0;i<5;i++) {
		  if (FREQ[i*3]) {
			  FREQ[i*3+1]=FREQ[i*3]+((FREQ[i*3]/50)*DETUNE/127)+((FREQ[i*3]/2)*RANGE/32);
			  FREQ[i*3+2]=FREQ[i*3]-((FREQ[i*3]/50)*DETUNE/127)+((FREQ[i*3]/2)*RANGE/32);
		  }
		}
	break;
	case 81:
		ENVMOD=value<<1;
	break;
	case 01:
		LFOMOD=value<<1;
	break;
	case 16:
		LFORATE=(value>>1)+1;
	break;
	case 76:
		WAVEFORM=value<<1;
	break;
	case 04:
		WRAP=value<<1;
	break;
	case 20:
		LFOWAVE=value;
	break;
	case 07:
		MASTERVOL=value<<9;
	break;
	}

}

void handleMIDINOTE(uint8_t status,uint8_t note,uint8_t vel) {
	uint8_t i;
	//uint8_t trigflag=0;
	uint32_t freq;
	if ((!vel)&&(status==0x90)) status=0x80;
	if (status==0x80) {
	      for (i=0;i<5;i++) {
	    	  if (OSCNOTES[i]==note) {
	    		  if (!GATED) {
	    		  FREQ[i*3]=0;
	    		  FREQ[i*3+1]=0;
	    		  FREQ[i*3+2]=0;
	    		  }
	    		  OSCNOTES[i]=0;
	    	  }
	    	  //trigflag+=OSCNOTES[i];
	      }
	      if (!(OSCNOTES[0]|OSCNOTES[1]|OSCNOTES[2]|OSCNOTES[3]|OSCNOTES[4])) TRIG=0;
	      return;
	}

	if (status==0x90) {
		if ((!TRIG)&&(GATED)) {
			for (i=0;i<14;i++) {
				FREQ[i]=0;
			}
		}
		i=0;
		while (i<5) {
	      if (!OSCNOTES[i]) {
    		  freq=MIDI2FREQ(note);
    		  FREQ[i*3]=freq;
			  FREQ[i*3+1]=FREQ[i*3]+((FREQ[i*3]/50)*DETUNE/127)+((FREQ[i*3]/2)*RANGE/32);
			  FREQ[i*3+2]=FREQ[i*3]-((FREQ[i*3]/50)*DETUNE/127)+((FREQ[i*3]/2)*RANGE/32);
	    	  OSCNOTES[i]=note;
	    	  if (!TRIG) {
	    		  TRIG=1;
	    		  state0=0;
	    		  state1=0;
	    	  }
	    	  return;
	      }
	      i++;
		}
	}

}

void UART0_IRQHandler(void) {
      uint8_t MIDIRX;
	  while (!(LPC_USART0->STAT & UART_STATUS_TXRDY));
	  MIDIRX = LPC_USART0->RXDATA;

	  /*
	  Handling "Running status"
	  1.Buffer is cleared (ie, set to 0) at power up.
	  2.Buffer stores the status when a Voice Category Status (ie, 0x80 to 0xEF) is received.
	  3.Buffer is cleared when a System Common Category Status (ie, 0xF0 to 0xF7) is received.
	  4.Nothing is done to the buffer when a RealTime Category message is received.
	  5.Any data bytes are ignored when the buffer is 0.
	  */

	  if ((MIDIRX>0xBF)&&(MIDIRX<0xF8)) {
		  MIDIRUNNINGSTATUS=0;
		  MIDISTATE=0;
		  return;
	  }

	  if (MIDIRX>0xF7) return;

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
	  		  if ((MIDIRUNNINGSTATUS==0x80)||(MIDIRUNNINGSTATUS==0x90)) handleMIDINOTE(MIDIRUNNINGSTATUS,MIDINOTE,MIDIVEL);
	  		  if (MIDIRUNNINGSTATUS==0xB0) handleMIDICC(MIDINOTE,MIDIVEL);

	  		  return;
	  	  }
	  	  }

	  return;
}


void mrtInit(uint32_t delay)
{
  /* Enable clock to MRT and reset the MRT peripheral */
  LPC_SYSCON->SYSAHBCLKCTRL |= (0x1<<10);
  LPC_SYSCON->PRESETCTRL &= ~(0x1<<7);
  LPC_SYSCON->PRESETCTRL |= (0x1<<7);

  LPC_MRT->Channel[0].INTVAL = delay;
  LPC_MRT->Channel[0].INTVAL |= 0x1UL<<31;

  LPC_MRT->Channel[0].CTRL = MRT_REPEATED_MODE|MRT_INT_ENA;

  /* Enable the MRT Interrupt */
#if NMI_ENABLED
  NVIC_DisableIRQ( MRT_IRQn );
  NMI_Init( MRT_IRQn );
#else
  NVIC_EnableIRQ(MRT_IRQn);
#endif
  return;
}

/*
for(uint8_t bit = 0x80; bit; bit >>= 1) {
      if(c & bit) *dataport |=  datapinmask;
      else        *dataport &= ~datapinmask;
      *clkport |=  clkpinmask;
      *clkport &= ~clkpinmask;
    }
*/

int main(void)
{
  /* Initialise the GPIO block */
	  /* Enable AHB clock to the GPIO domain. */
	  LPC_SYSCON->SYSAHBCLKCTRL |=  (1 << 6);
	  LPC_SYSCON->SYSAHBCLKCTRL |=  (1 << 11); //Enable SPI0 clk
	  LPC_SYSCON->PRESETCTRL    &= ~(1 << 10);
	  LPC_SYSCON->PRESETCTRL    |=  (1 << 10);
	  LPC_SYSCON->PRESETCTRL    &= ~(1); //Reset SPI0
	  LPC_SYSCON->PRESETCTRL    |=  (1); //Reset SPI0
      LPC_SWM->PINENABLE0 = 0xffffffffUL; //All 6 GPIO enabled

      LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 8); // enable the SCT clock
      LPC_SCT->CTRL_L |= ((0 << 5) | (1<<3)); // set pre-scalar, SCT clock, clear counter
      LPC_SCT->CTRL_H |= ((0 << 5) | (1<<3)); // set pre-scalar, SCT clock, clear counter

      sct_fsm_init();
      LPC_SCT->CTRL_L &= ~(1<<2); // unhalt the SCT by clearing bit 2 in CTRL
      LPC_SCT->CTRL_H &= ~(1<<2); // unhalt the SCT by clearing bit 2 in CTRL
      LPC_SWM->PINASSIGN6 = 0x03ffffffUL; /* CTOUT_0 = P0_3*/
      LPC_SWM->PINASSIGN7 = 0x0fffff02UL; /* CTOUT_1 = P0_2*/

      /* Set Switch matrix for SPI0 here
      LPC_SWM->PINASSIGN3 = 0x**ffffffUL; //CLK
      LPC_SWM->PINASSIGN4 = 0xffffff**UL; //MOSI
      LPC_SWM->PINASSIGN4 = 0xffff**ffUL; //MISO
      LPC_SWM->PINASSIGN4 = 0xff**ffffUL; //SSEL
      */

      LPC_SPI0->DIV=1; //24MHz SPI CLK

      /* Pin Assign 8 bit Configuration */
      /* U0_TXD */
      /* U0_RXD */
      LPC_SWM->PINASSIGN0 = 0xffff0004UL;

      /* Initialise the UART0 block for printf output */
      uart0Init(31250);

      //SystemCoreClockUpdate();

      /* Configure the multi-rate timer for 44.1K ticks */
      //mrtInit(__SYSTEM_CLOCK/42188);
      mrtInit(48000000/42188);

      uint16_t i;
      for (i=0;i<256;i++) {
    	waveform[i]=(i-127);
      }
      for (i=0;i<5;i++) {
        FREQ[i] = 0;
      }
      i=0;
	  int8_t TRI;
	  int8_t SQR;
	  int8_t SAW;
	  int16_t SUM;
      while(1) {
    	  for (i=0;i<256;i++) {
    		  shrandom=i&255;
    	    SUM=0;
    	    TRI=(i&127);
    	    if (i&128) TRI=127-(i&127);
    	    if (i&128) {
    	      SQR=127;
    	    }
    	    else {
    	      SQR=-127;
    	    }
    	    SAW=i-127;
    	    if (WAVEFORM<128) {
    	    	SUM+=TRI*((127-WAVEFORM)*2);
    	    	SUM+=SQR*(WAVEFORM*2);
    	    }
    	    if (WAVEFORM>127) {
    	    	SUM+=SQR*((255-WAVEFORM)*2);
    	    	SUM+=SAW*((WAVEFORM-128)*2);
    	    }
    	    if (i<=WRAP) {
    	      waveform[i]=(waveform[i]+(-SUM>>8))>>1;
    	    }
    	    else {
    	      waveform[i]=(waveform[i]+(SUM>>8))>>1;
    	    }
    	    //shrandom=LPC_MRT->Channel[0].TIMER & 255;
    	  }
      }
}
