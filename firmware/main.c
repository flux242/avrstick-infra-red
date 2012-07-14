/* Name: main.c
 * Project: Datalogger based on AVR USB driver
 * Original Author: Christian Starkjohann
 * Edited by Ryan Owens (SparkFun Electronics)
 * Creation Date: 2006-04-23
 * Edited 2009-06-30
 * Tabsize: 4
 * Copyright: (c) 2006 by OBJECTIVE DEVELOPMENT Software GmbH
 * License: Proprietary, free under certain conditions. See Documentation.
 * This Revision: $Id: main.c 537 2008-02-28 21:13:01Z cs $
 */

#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>

#include "usbdrv.h"
#include "hidkeycodes.h"
#include "lookup.h"
//#include "oddebug.h"

/*
Pin assignment:

PB0, PB2 = USB data lines
PB1 = Yellow LED output
PB3 = White LED output
PB4 = Digital input (ADC2)
*/

#ifndef NODECODE
#define DECODE       // comment this out to log instead
#endif
#define START_LEN 29 // start bit length. Recording is started after start bit
#define MIN_LEN   9  // min length of a pulse. If less then recording is stopped 

#define WHITE_LED 3
#define YELLOW_LED 1

#define sbi(var, mask)   ((var) |= (uint8_t)(1 << mask))
#define cbi(var, mask)   ((var) &= (uint8_t)~(1 << mask))

#ifndef NULL
#define NULL    ((void *)0)
#endif

/* ------------------------------------------------------------------------- */

static uchar    reportBuffer[2];    /* buffer for HID reports */
static uchar    idleRate;           /* in 4 ms units */

static uchar    valueBuffer[16];
static uchar    *nextDigit;


#define RBSIZE   100
static  uchar    rbuff[RBSIZE];              // record buffer
volatile static  uchar    rbidx;
volatile static  uchar    rbcur;
volatile static  uchar    isRecBufBusy = 0;
volatile static  uchar    isRecording = 0;
static  uchar    cmdCounter = 0;
static  uchar    cmdPrev = 0;

/* ------------------------------------------------------------------------- */

PROGMEM char usbHidReportDescriptor[USB_CFG_HID_REPORT_DESCRIPTOR_LENGTH] = { /* USB report descriptor */
    0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
    0x09, 0x06,                    // USAGE (Keyboard)
    0xa1, 0x01,                    // COLLECTION (Application)
    0x05, 0x07,                    //   USAGE_PAGE (Keyboard)
    0x19, 0xe0,                    //   USAGE_MINIMUM (Keyboard LeftControl)
    0x29, 0xe7,                    //   USAGE_MAXIMUM (Keyboard Right GUI)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x25, 0x01,                    //   LOGICAL_MAXIMUM (1)
    0x75, 0x01,                    //   REPORT_SIZE (1)
    0x95, 0x08,                    //   REPORT_COUNT (8)
    0x81, 0x02,                    //   INPUT (Data,Var,Abs)
    0x95, 0x01,                    //   REPORT_COUNT (1)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x25, 0x65,                    //   LOGICAL_MAXIMUM (101)
    0x19, 0x00,                    //   USAGE_MINIMUM (Reserved (no event indicated))
    0x29, 0x65,                    //   USAGE_MAXIMUM (Keyboard Application)
    0x81, 0x00,                    //   INPUT (Data,Ary,Abs)
    0xc0                           // END_COLLECTION
};
/* We use a simplifed keyboard report descriptor which does not support the
 * boot protocol. We don't allow setting status LEDs and we only allow one
 * simultaneous key press (except modifiers). We can therefore use short
 * 2 byte input reports.
 * The report descriptor has been created with usb.org's "HID Descriptor Tool"
 * which can be downloaded from http://www.usb.org/developers/hidpage/.
 * Redundant entries (such as LOGICAL_MINIMUM and USAGE_PAGE) have been omitted
 * for the second INPUT item.
 */

/* ------------------------------------------------------------------------- */
#define diff(c1, c2) (c1>c2?(c1-c2):(c2-c1))
#define shiftbit(arg, bit) arg<<= 1; arg|= bit;
#if 0
#define shiftbit(arg, bit)        \
    __asm__ __volatile__ (        \
        "mov __tmp_reg__, %1\n\t" \
        "ror __tmp_reg__\n\t"     \
        "rol %0"                  \
        : "=r" (arg)              \
        : "r" (bit)               \
    )
#endif

#ifdef DECODE
#define SS  10
#define SF  17
#define LS  23
#define LF  30
#define EPS 2

static uchar decodeBuffer()
{
  uchar result = 0;
  uchar idx = 1;  // skip first 0 
  uchar cbit = 0; // current bit;
  uchar time;

  while (++idx<=rbidx) // start bit is skipped too 
  {
    time = rbuff[idx];
    if (time>127)
      time-=128;

    if (diff(time,SS)<=EPS) {
      shiftbit(result,cbit);
      continue;
    }
    else if (diff(time,SF)<=EPS)
      continue;
    else if (diff(time,LS)<=EPS) {
      cbit = 1;
      shiftbit(result,cbit);
      continue;
    }
    else if (diff(time,LF)<=EPS) {
      cbit = 0;
      shiftbit(result,cbit);
      continue;
    }
    else
      break;
  }
  
  if (result<128) // MSB shall be 1 if properly decoded
    return 0;
  else
    return result;
}
#endif

static void buildReport(void)
{
  uchar   key = 0;

  if(nextDigit != NULL)
  {
    key = *nextDigit;
  }
  reportBuffer[0] = 0;    /* no modifiers */
  reportBuffer[1] = key;
}

/* ------------------------------------------------------------------------- */
static void reportChar(uchar value, uchar decimate)
{
  uchar   digit;
 
  //The Buffer is constructed 'backwards'
  nextDigit = &valueBuffer[sizeof(valueBuffer)];  
  *--nextDigit = 0xff;/* terminate with 0xff */
  if (!decimate)
  {
    *--nextDigit = 0;
    *--nextDigit = value;
  }
  else
  {
    *--nextDigit = 0;
    *--nextDigit = KEY_RETURN;  

    // Convert char to ASCII.
    do
    {
      digit = value % 10;
      value /= 10;
      *--nextDigit = 0;
      if(digit == 0)
          *--nextDigit = KEY_0;
      else
          *--nextDigit = KEY_1 - 1 + digit;
    } while(value != 0);
  }

  *--nextDigit = 0; // bugfix for the first char been eaten
  *--nextDigit = 0;
}



/* ------------------------------------------------------------------------- */
/* ------------------------ interface to USB driver ------------------------ */
/* ------------------------------------------------------------------------- */

uchar usbFunctionSetup(uchar data[8])
{
  usbRequest_t    *rq = (void *)data;

  usbMsgPtr = reportBuffer;
  if((rq->bmRequestType & USBRQ_TYPE_MASK) == USBRQ_TYPE_CLASS)
  {    /* class request type */
      if(rq->bRequest == USBRQ_HID_GET_REPORT){  /* wValue: ReportType (highbyte), ReportID (lowbyte) */
          /* we only have one report type, so don't look at wValue */
          buildReport();
          return sizeof(reportBuffer);
      }else if(rq->bRequest == USBRQ_HID_GET_IDLE){
          usbMsgPtr = &idleRate;
          return 1;
      }else if(rq->bRequest == USBRQ_HID_SET_IDLE){
          idleRate = rq->wValue.bytes[1];
      }
  }
  else
  {
      /* no vendor specific requests implemented */
  }
 
  return 0;
}

/* ------------------------------------------------------------------------- */
/* ------------------------ Oscillator Calibration ------------------------- */
/* ------------------------------------------------------------------------- */

/* Calibrate the RC oscillator to 8.25 MHz. The core clock of 16.5 MHz is
 * derived from the 66 MHz peripheral clock by dividing. Our timing reference
 * is the Start Of Frame signal (a single SE0 bit) available immediately after
 * a USB RESET. We first do a binary search for the OSCCAL value and then
 * optimize this value with a neighboorhod search.
 * This algorithm may also be used to calibrate the RC oscillator directly to
 * 12 MHz (no PLL involved, can therefore be used on almost ALL AVRs), but this
 * is wide outside the spec for the OSCCAL value and the required precision for
 * the 12 MHz clock! Use the RC oscillator calibrated to 12 MHz for
 * experimental purposes only!
 */
static void calibrateOscillator(void)
{
  uchar       step = 128;
  uchar       trialValue = 0, optimumValue;
  int         x, optimumDev, targetValue = (unsigned)(1499 * (double)F_CPU / 10.5e6 + 0.5);

  /* do a binary search: */
  do {
      OSCCAL = trialValue + step;
      x = usbMeasureFrameLength();    /* proportional to current real frequency */
      if(x < targetValue)             /* frequency still too low */
          trialValue += step;
      step >>= 1;
  } while(step > 0);
  /* We have a precision of +/- 1 for optimum OSCCAL here */
  /* now do a neighborhood search for optimum value */
  optimumValue = trialValue;
  optimumDev = x; /* this is certainly far away from optimum */
  for(OSCCAL = trialValue - 1; OSCCAL <= trialValue + 1; OSCCAL++) {
      x = usbMeasureFrameLength() - targetValue;
      if(x < 0)
          x = -x;
      if(x < optimumDev){
          optimumDev = x;
          optimumValue = OSCCAL;
      }
  }
  OSCCAL = optimumValue;
}
/*
Note: This calibration algorithm may try OSCCAL values of up to 192 even if
the optimum value is far below 192. It may therefore exceed the allowed clock
frequency of the CPU in low voltage designs!
You may replace this search algorithm with any other algorithm you like if
you have additional constraints such as a maximum CPU clock.
For version 5.x RC oscillators (those with a split range of 2x128 steps, e.g.
ATTiny25, ATTiny45, ATTiny85), it may be useful to search for the optimum in
both regions.
*/

void  usbEventResetReady(void)
{
  calibrateOscillator();
  eeprom_write_byte(0, OSCCAL);   /* store the calibrated value in EEPROM */
}

/* ------------------------------------------------------------------------- */
static inline void timerInit(void)
{
  TCNT1 = 0x00; // clear timer
  TCCR1 = 0x0;  /* stopped */
  TIMSK |= _BV(TOIE1);
}

static inline void timerStart(void)
{
  TCNT1 = 0x00; // clear timer
  TCCR1 = 0x0b;  /* select clock: 16.5M/1024 -> 62uS*/
  //TCCR1 = 0x0a;  /* select clock: 16.5M/512 -> 31uS */
  //TIFR |= _BV(TOV1); // clear timer overflow if set
}

static inline void timerStop(void)
{
  TCCR1 = 0x0;  /* stopped */
  TCNT1 = 0x00; // clear timer
  //TIFR |= _BV(TOV1); // clear timer overflow if set
}

static inline void pinsInit()
{
  //DDRB = 0x00;
  DDRB |= _BV(PB1) | _BV(PB3);   /* output for LED */
  DDRB &= ~_BV(PB4); // pb4 as input
  PORTB |= _BV(PB4); // activate pull-up resistor
}

static inline void interruptsInit()
{
  PCMSK |= _BV(PCINT4);  
  GIMSK |= _BV(PCIE);
} 

ISR(TIM1_OVF_vect)
{
  timerStop();
  if (isRecording)  
  {
    isRecording = 0;
    isRecBufBusy = 1;
    rbcur = 0;
  }
}

ISR(PCINT0_vect)
{
  uchar time = TCNT1;

  if (isRecBufBusy)
    return;

  timerStart();

  if (isRecording)
  {
    if (time<MIN_LEN)
    { // filter out wrong sequences
      isRecording = 0;
      return;
    }

    if ( PINB & _BV(PB4) )
      cbi(time, 7);
    else
      sbi(time, 7);

    if (rbidx<RBSIZE)
      rbuff[++rbidx] = time;
  }
  else
  {
    if ( PINB & _BV(PB4) )
    { // high edge 
      if (time>START_LEN) // 62*REC_THRESHOLD uS
      {
        rbidx = 0;
        cbi(time, 7); // first low level period 
        rbuff[rbidx] = 0;  // 0 to denote new sequence
        rbuff[++rbidx] = time;
        sbi(PORTB, PB3); 
        isRecording = 1;
      }
      cbi(PORTB, PB1); 
    }
    else
    { // low edge 
      sbi(PORTB, PB1);
      cbi(PORTB, PB3);
    }
  }
}



/* ------------------------------------------------------------------------- */
/* --------------------------------- main ---------------------------------- */
/* ------------------------------------------------------------------------- */
int main(void)
{
  unsigned int i;
  uchar   calibrationValue;

  calibrationValue = eeprom_read_byte(0); /* calibration value from last time */
  if(calibrationValue != 0xff) {
      OSCCAL = calibrationValue;
  }
  //odDebugInit();

  //Production Test Routine - Turn on white LED.
  DDRB |= _BV(PB1);   /* output for LED */

  sbi(PORTB, PB1);
    for(i=0;i<20;i++) {  /* 300 ms disconnect */
        _delay_ms(15);
    }
  cbi(PORTB, PB1);

  //Initialize the USB Connection with the host computer.
  usbDeviceDisconnect();
  for(i=0;i<20;i++){  /* 300 ms disconnect */
      _delay_ms(15);
  }
  usbDeviceConnect();
  
  wdt_enable(WDTO_1S);
 
  pinsInit();
  interruptsInit(); 
  timerInit();
  usbInit();  //Initialize USB comm.
  sei();
  for(;;)
  {    /* main event loop */
    wdt_reset();
    usbPoll();  //Check to see if it's time to send a USB packet
    if (isRecBufBusy && nextDigit==NULL)
    {
#ifndef DECODE
      if (rbcur>rbidx)
        isRecBufBusy = 0; 
      else 
        reportChar(rbuff[rbcur++], 1);
#else
      uchar command;
      command = decodeBuffer();
      if (0!=command)
      {
        if (command!=cmdPrev)
        {
          reportChar(pgm_read_byte(&lookupTable[command & 0b00111111]), 0); // 64 values maximum
          cmdPrev = command;
          cmdCounter = 1;
        }
        else if (++cmdCounter==3)
        {
          cmdCounter = 0;
          cmdPrev = 0;
        }
      }
      isRecBufBusy = 0;
#endif
    }
    if(usbInterruptIsReady() && nextDigit!=NULL)
    { /* we can send another key */
      buildReport();  //Get the next 'key press' to send to the host. 
      usbSetInterrupt(reportBuffer, sizeof(reportBuffer));
      if(*++nextDigit == 0xff)    /* this was terminator character */
        nextDigit = NULL;
    }
  }
 
  return 0;
}

