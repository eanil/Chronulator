/**********************************************
 Chronutator clock project using MSP430G2211
 Copyright: Doug Paradis - 2010
 All rights reserved
 This code may be used for private use, as long
 as, copyright notice retained.

 ACLK  (pin 2) --> crystal osc output (pin labeled P1.0)
                   Use when troubleshooting on Launchpad.
                   Commented out in this listing.
 TA0.0 (pin 3) --> PWM for minute meter (pin labeled P1.1)
 TA0.1 (pin 4) --> PWM for hour meter   (pin labeled P1.2)
 P1.4  (pin 6) --> minute setting button (inc one min per push)
 P1.5  (pin 7) --> hour setting button (inc one hr per push)
 RST   (pin 10) --> reset
 Xin, Xout (pins 13,12) --> 32.768 KHz crystal

 P1.3, P1.6, P1.7 (pins 5,8,9) --> not used

 compiled using IAR Embedded Workbench
**********************************************

modified by Engin Anil to work on MSP430G2452 and compile using mspgcc
12.24.2015

*/


//#include  "msp430.h"

#include <msp430g2452.h>
#include <signal.h>

unsigned char cntr = 0;                              // sec counter
unsigned char cntr_trip_pt = 60;                     // change every 1 sec
unsigned char debounceFlg = 0;                       // flag for debouncing

// min
unsigned char mflg = 0;                             // minute flag
unsigned int m_period = 10000;                      // minute meter PWM period (number of counts of SMCLK)
unsigned int m_on = 5;                              // minute meter PWM setting 
unsigned char m_cntr = 0;                           // min counter
unsigned int m_zero = 5;                            // minute meter PWM zero setting  (can't actually be zero)
//  calibration array for minute meter (set every 5 min mark) -- values should be divisable by 5
//  min meter             5   10  15  20  25  30  35  40  45  50  55  60 
unsigned int m_cal[12] ={640,690,740,715,655,620,635,625,600,610,620,790}; // divisible by 5 
unsigned char m_index = 0;
unsigned char m_tick = 0;

// hour                                             -- see comments above for items below --
unsigned char hflg = 0;
unsigned int h_period = 10000;
unsigned int h_on = 5;                              // hour meter PWM setting  
unsigned char h_cntr = 0;    
unsigned int h_zero = 5;                            // (can't actually be zero)
//  calibration array for hour meter (set for each hour) -- values should be divisable by 12
//   hr mtr               1   2   3   4   5   6   7   8   9   10  11  12
unsigned int h_cal[12] ={744,796,708,708,760,720,708,768,684,684,780,732};   // divisible by 12 
unsigned char h_tick = 0;
unsigned char h_index = 0;

void one_min(void);
void debounce(char button);

int 
main(void)
{
  WDTCTL = WDTPW + WDTHOLD;                        // Stop watchdog timer
  BCSCTL1 = RSEL0 + RSEL3;                         // Select DCOCLK freq range of ~2.3 MHz (get rid of meter jitter)
  BCSCTL2 = DIVS_2 ;                               // MCLK = DCOCLK, SMCLK = DCOCLK/8
  // crystal Cload capacitor adjustment - only one line of next four should be uncommented
   BCSCTL3 = LFXT1S_0 + XCAP_3;                     // 32768KHz crystal, 12.5 pF
  // BCSCTL3 = LFXT1S_0 + XCAP_2;                     // 32768KHz crystal, 10 pF
  // BCSCTL3 = LFXT1S_0 + XCAP_1;                     // 32768KHz crystal, 6 pF
  // BCSCTL3 = LFXT1S_0 + XCAP_0;                     // 32768KHz crystal, 1 pF
  WDTCTL = WDTPW + WDTTMSEL + WDTCNTCL + WDTSSEL;  // WDT inv mode, ACK clk,
                                                   // 1000mS interval (assume 32KHz crystal)
                                                   // equivalent to "WDT_ADLY_1000" defined in h file
  IE1 |= WDTIE;                                    // Enable WDT interrupt
  TACTL = TASSEL_2 + MC_2;                         // TA clock = SMCLK, mode control = continuous
  TACCTL0 = OUTMOD_4 + CCIE;                       // output mode = toggle, interrupt enabled
  TACCTL1 = OUTMOD_1 + CCIE;                       // output mode = set, interrupt enabled
  P1SEL =  BIT1 + BIT2;                            //  P1.1 to TA0.0, P1.2 to TA0.1
  P1DIR =  BIT1 + BIT2;                            // P1.1 (min) and P1.2 (hr) to output direction
 
  //###### don't forget to comment out
  //P1DIR =  BIT0 + BIT1 + BIT2;                  // P1.0 (flash led when using Launchpad), 
                                                   // P1.5 (min) and P1.6 (hr) to output direction
   
  P1IE = BIT4 + BIT5;                              // Enable Port 1 interrupt for P1.4 and P1.5
  //P1IES = BIT4 + BIT5;                            // Interupt edge select, high-low (falling edge)
  P1OUT = BIT4 + BIT5;                             // Time chg buttons set to high
  P1REN = BIT4 + BIT5;                             // pull up resistors enable on time chg button pis

  TACCR0 = m_on;
  TACCR1 = h_on;

  _BIS_SR(LPM0_bits + GIE);                        // Enter LPM0 w/interrupt
  return 0;
}

static void
__attribute__((__interrupt__(TIMER0_A0_VECTOR)))
timer_A0(void)
{
  
  if (mflg == 0)
  {
    TACCR0 = TAR + m_on;
    mflg = 1;
  }
  else
  {
    TACCR0 = TAR + (m_period - m_on);
    mflg = 0;
  }
}

static void
__attribute__((__interrupt__(TIMER0_A1_VECTOR)))
timer_A1(void)
{

  if (hflg == 0)
  {
    TACCTL1 = OUTMOD_5 + CCIE;                     // output mode = reset, interrupt enabled
    TACCR1 = TAR + h_on;
    hflg = 1;
  }
  else
  {
    TACCTL1 = OUTMOD_1 + CCIE;                     // output mode = set, interrupt enabled
    TACCR1 = TAR + (h_period - h_on);
    hflg = 0;
  }
}

static void
__attribute__((__interrupt__(WDT_VECTOR)))
watchdow_timer(void)
{

  if (debounceFlg == 0)
  {
      //#######comment out
     //P1OUT ^= BIT0;                     // Toggle P1.0 using exclusive-OR (Use when monitoring ACLK)
    
    if (cntr == cntr_trip_pt)          // change pwm duty cycle
    {
       one_min();  
    }
    cntr++;
  } 
  else                                  // part of debounce routine 
  {
    IFG1 &= ~WDTIFG;                    // clear WDT timer+ interrupt pending (do when changing WDT settings)
    WDTCTL = WDT_ADLY_1000;             // Set Watchdog Timer delay to 1000 mS
    // IE1 |= WDTIE;                      // Enable WDT interrupt
    P1IFG = 0;                          // clear all p1 interrupts
    P1IE |= BIT4 + BIT5;                // Enable Port 1 interrupt for P1.4 and P1.5
    debounceFlg = 0;
  }
}

static void
__attribute__((__interrupt__(PORT1_VECTOR)))
port_1(void)
{

   P1IFG &= ~BIT0;                      // clear any interrupt on P1.0 (only needed if using Launchpad led)
  switch (P1IFG) {
  case BIT4:                           // P1.4 (increment mins)
    cntr = 0;                          // set seconds to 0
//#####comment out
    /*
    int j = 1;
    for (j = 1; j <= 4; j++)      // inc 4 mins  Uncomment these 4 lines when tuning m_cal            
    {
      one_min();
    } 
    */
    one_min();                         // increment 1 min
    debounce(BIT4);
    break;

  case BIT5:                           // P1.5 (increment hrs)
    // cntr = 0;                       // set seconds to 0
    j = 1;
    for (j = 1; j <= 60; j++)      // inc 1 hr                
    {
      one_min();
    }  
    debounce(BIT5);                    
    break;
  default:
    P1IFG = 0;                         // clear all p1 interrupts (note: pushing both buttons at same
                                       // time will end up here)
  }
}

void one_min (void)
{
      m_tick++;                                      // subdivsion between 2 - 5 min marks on min meter
      m_cntr++;                                      // number of minutes in this hour
      m_on += m_cal[m_index] / 5;                    // increase 1 min
      if (m_tick == 5)                               // is min meter at a 5 min mark on min meter?
      {  
        m_index++;                                   // increase index in m_cal array
        m_tick = 0;                                  // at a 5 min mark on min meter reset m_tick
        h_tick++;
        h_on += h_cal[h_index] / 12;                 // move hr meter one subdivision (12 per hr) between 2 hr marks
      }  
      if (h_tick == 12)                              // has hr meter reached an hr mark?
      {
        h_index++;                                   // increment the number of hours
        h_tick = 0;                                  // reset subdivisions of min meter
      }  
      cntr = 0;                                      // set second count to 0

         

    if (m_cntr >= 60)                                // reset on the hour (every 60 mins)
    {
      m_on = m_zero;                                 // move to zero on min meter
      m_index = 0;                                   // set number of mins this hr to zero
      m_tick = 0;                                    // set subdivisions of min meter to 0
      m_cntr = 0;                                    // set number of mins this hour to 0
      h_tick = 0;                                    // set subdivisions of hour meter to 0
      h_cntr++;                                      // increment hour count
    }
    if (h_cntr >= 12)                                // reset every 12 hours
    {
      h_on = h_zero;                                 // set hour meter to 0
      // m_index = 0;                                   // reset m_cal array index
      h_index = 0;                                   // reset h_cal array index
      // m_tick = 0;                                    // reset subdivisions of min meter 
      // h_tick = 0;                                    // reset subdivisions of hr meter
      h_cntr = 0;                                    // reset hour count
    } 
}  

// debounce routine assmes the use of
// 47k res to Vcc and .1uF cap to gnd, connected to button
void debounce(char button)
{
    debounceFlg = 1;
    P1IFG &= ~(button);                // clear interrupt flag for button pushed (p1.2 or p1.3)
    P1IE &= ~(button);                 // temporarily disable the interrupt
    IFG1 &= ~WDTIFG;                   // clear WDT timer+ interrupt pending (do when changing WDT settings)
    IE1 &= ~WDTIE;                     // disable WDT interrupt
    WDTCTL = WDTPW + WDTHOLD;          // temporarily stop Watchdog Timer
    WDTCTL = WDT_ADLY_250;             // Set Watchdog Timer delay to 250 mS
    IE1 |= WDTIE;                      // Enable WDT interrupt
}    
