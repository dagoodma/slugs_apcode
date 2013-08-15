/*
 * File: controlMCUSlugsMKIINewNav_main.c
 *
 * Real-Time Workshop code generated for Simulink model controlMCUSlugsMKIINewNav.
 *
 * Model version                        : 1.280
 * Real-Time Workshop file version      : 8.1 (R2011b) 08-Jul-2011
 * Real-Time Workshop file generated on : Thu Jun 20 11:03:39 2013
 * TLC version                          : 8.1 (Jul  9 2011)
 * C source code generated on           : Thu Jun 20 11:03:40 2013
 *--------------------------------------------------------------
 *   Embedded Coder for Microchip dsPIC family.                 |
 *   Generate .c and .h files from your Matlab/simulink model   |
 *   and compile the diagram to .hex and .coff file that can be |
 *   downloaded directly into the microcontroller               |
 *                                                              |
 * Licence Accorded to Prof. Gabriel Hugh Elkaim                |
 *                     Autonomous Systems Lab, UC Santa Cruz    |
 *                                                              |
 *   Written by Lubin KERHUEL -  http://www.kerhuel.eu          |
 *  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - |
 *   Version 3.7e                              14-Sep-2012      |
 *   For Matlab 7.13            R2011b                          |
 *--------------------------------------------------------------
 */

#include "controlMCUSlugsMKIINewNav.h"
#include "controlMCUSlugsMKIINewNav_private.h"
#ifdef _FSS                            /* for chip with memory protection options */

_FSS( RSS_NO_RAM & SSS_NO_FLASH & SWRP_WRPROTECT_OFF )
#endif
  _FGS( GCP_OFF & GSS_OFF & GWRP_OFF )
  _FOSCSEL( FNOSC_PRIPLL & IESO_OFF & 0xFFFF )
  _FOSC( FCKSM_CSDCMD & OSCIOFNC_OFF & POSCMD_EC)
  _FPOR( FPWRT_PWR1 )
  _FWDT( FWDTEN_OFF )
  _FICD( JTAGEN_OFF & ICS_PGD2 )
  uint16_T CalculusTimeStep;
void rt_OneStep()
{
  /* Disable interrupts here */
  /* Save FPU context here (if necessary) */
  /* Re-enable timer or interrupt here */
  controlMCUSlugsMKIINewNav_step(0);

  /* Get model outputs here */

  /* Disable interrupts here */
  /* Restore FPU context here (if necessary) */
  /* Enable interrupts here */
}

int main(void)
{
  PLLFBD = 38;                         /* configure clock speed */
  CLKDIV = 1;

  /* Initialize model */
  controlMCUSlugsMKIINewNav_initialize(1);
  for (;;) {
    /* Associate rt_OneStep() with a timer that executes at the base rate of the model */
    while (!_T1IF) ;
    _T1IF = 0;
    rt_OneStep();
    if (_T1IF)
      CalculusTimeStep = PR1;          /* Overload */
    else
      CalculusTimeStep = TMR1;
  }
}

/* File trailer for Real-Time Workshop generated code.
 *
 * [EOF]
 */
