#include "p33fxxxx.h"
#include "controlMCUSlugsMKIINewNav.h"
#include "controlMCUSlugsMKIINewNav_private.h"

volatile uint16_T ic1up;
volatile uint16_T ic2up;
volatile uint16_T ic3up;
volatile uint16_T ic4up;
volatile uint16_T ic5up;
volatile uint16_T ic7up;
volatile uint16_T ic8up;
void __attribute__((__interrupt__,__auto_psv__)) _IC1Interrupt(void)
{
  static uint16_T IC1TMR;
  uint16_T tmp;
  uint16_T calcul;                     /* intermediate value */
  while (IC1CONbits.ICBNE == 1)
    tmp = IC1BUF;                      /* take the last value */
  if ((IC1CON & 1)==1)                 /* rising edge */
  {
    IC1CON = (IC1CON & 0xfffe);        /* detect next falling edge */
  } else                               /* falling edge */
  {
    IC1CON = (IC1CON | 0x0001);        /* detect next rising edge */
    calcul = tmp - IC1TMR;
    ic1up = calcul;
  }

  IC1TMR = tmp;
  _IC1IF = 0;
}

void __attribute__((__interrupt__,__auto_psv__)) _IC2Interrupt(void)
{
  static uint16_T IC2TMR;
  uint16_T tmp;
  uint16_T calcul;                     /* intermediate value */
  while (IC2CONbits.ICBNE == 1)
    tmp = IC2BUF;                      /* take the last value */
  if ((IC2CON & 1)==1)                 /* rising edge */
  {
    IC2CON = (IC2CON & 0xfffe);        /* detect next falling edge */
  } else                               /* falling edge */
  {
    IC2CON = (IC2CON | 0x0001);        /* detect next rising edge */
    calcul = tmp - IC2TMR;
    ic2up = calcul;
  }

  IC2TMR = tmp;
  _IC2IF = 0;
}

void __attribute__((__interrupt__,__auto_psv__)) _IC3Interrupt(void)
{
  static uint16_T IC3TMR;
  uint16_T tmp;
  uint16_T calcul;                     /* intermediate value */
  while (IC3CONbits.ICBNE == 1)
    tmp = IC3BUF;                      /* take the last value */
  if ((IC3CON & 1)==1)                 /* rising edge */
  {
    IC3CON = (IC3CON & 0xfffe);        /* detect next falling edge */
  } else                               /* falling edge */
  {
    IC3CON = (IC3CON | 0x0001);        /* detect next rising edge */
    calcul = tmp - IC3TMR;
    ic3up = calcul;
  }

  IC3TMR = tmp;
  _IC3IF = 0;
}

void __attribute__((__interrupt__,__auto_psv__)) _IC4Interrupt(void)
{
  static uint16_T IC4TMR;
  uint16_T tmp;
  uint16_T calcul;                     /* intermediate value */
  while (IC4CONbits.ICBNE == 1)
    tmp = IC4BUF;                      /* take the last value */
  if ((IC4CON & 1)==1)                 /* rising edge */
  {
    IC4CON = (IC4CON & 0xfffe);        /* detect next falling edge */
  } else                               /* falling edge */
  {
    IC4CON = (IC4CON | 0x0001);        /* detect next rising edge */
    calcul = tmp - IC4TMR;
    ic4up = calcul;
  }

  IC4TMR = tmp;
  _IC4IF = 0;
}

void __attribute__((__interrupt__,__auto_psv__)) _IC5Interrupt(void)
{
  static uint16_T IC5TMR;
  uint16_T tmp;
  uint16_T calcul;                     /* intermediate value */
  while (IC5CONbits.ICBNE == 1)
    tmp = IC5BUF;                      /* take the last value */
  if ((IC5CON & 1)==1)                 /* rising edge */
  {
    IC5CON = (IC5CON & 0xfffe);        /* detect next falling edge */
  } else                               /* falling edge */
  {
    IC5CON = (IC5CON | 0x0001);        /* detect next rising edge */
    calcul = tmp - IC5TMR;
    ic5up = calcul;
  }

  IC5TMR = tmp;
  _IC5IF = 0;
}

void __attribute__((__interrupt__,__auto_psv__)) _IC7Interrupt(void)
{
  static uint16_T IC7TMR;
  uint16_T tmp;
  uint16_T calcul;                     /* intermediate value */
  while (IC7CONbits.ICBNE == 1)
    tmp = IC7BUF;                      /* take the last value */
  if ((IC7CON & 1)==1)                 /* rising edge */
  {
    IC7CON = (IC7CON & 0xfffe);        /* detect next falling edge */
  } else                               /* falling edge */
  {
    IC7CON = (IC7CON | 0x0001);        /* detect next rising edge */
    calcul = tmp - IC7TMR;
    ic7up = calcul;
  }

  IC7TMR = tmp;
  _IC7IF = 0;
}

void __attribute__((__interrupt__,__auto_psv__)) _IC8Interrupt(void)
{
  static uint16_T IC8TMR;
  uint16_T tmp;
  uint16_T calcul;                     /* intermediate value */
  while (IC8CONbits.ICBNE == 1)
    tmp = IC8BUF;                      /* take the last value */
  if ((IC8CON & 1)==1)                 /* rising edge */
  {
    IC8CON = (IC8CON & 0xfffe);        /* detect next falling edge */
  } else                               /* falling edge */
  {
    IC8CON = (IC8CON | 0x0001);        /* detect next rising edge */
    calcul = tmp - IC8TMR;
    ic8up = calcul;
  }

  IC8TMR = tmp;
  _IC8IF = 0;
}
