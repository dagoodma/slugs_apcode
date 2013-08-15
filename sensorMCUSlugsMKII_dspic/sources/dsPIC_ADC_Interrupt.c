#include "sensorMCUSlugsMKII.h"
#include "sensorMCUSlugsMKII_private.h"

volatile uint16_T ADCBuffChannel0[5] ;
volatile uint16_T ADCBuffChannel1[5] ;
volatile uint16_T ADCBuffChannel2[5] ;
volatile uint16_T ADCBuffChannel3[5] ;
volatile uint8_T ADCFlag ;

#include "p33fxxxx.h"

void __attribute__((__interrupt__,__auto_psv__)) _DMA1Interrupt(void)
{
  _DMA1IF = 0;
  static uint16_T cnt = 0;
  ADCBuffChannel0[cnt] = ADCBuffChannelDMA[0];
  ADCBuffChannel1[cnt] = ADCBuffChannelDMA[1];
  ADCBuffChannel2[cnt] = ADCBuffChannelDMA[2];
  ADCBuffChannel3[cnt] = ADCBuffChannelDMA[3];
  cnt++;
  if (cnt >= 5) {
    cnt = 0;
    if (!(++ADCFlag))
      ADCFlag = 255;
  }
}                                      /* end ADC or DMA Interrupt */
