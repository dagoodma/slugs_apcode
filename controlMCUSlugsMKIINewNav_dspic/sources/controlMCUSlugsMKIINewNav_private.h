/*
 * File: controlMCUSlugsMKIINewNav_private.h
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

#ifndef RTW_HEADER_controlMCUSlugsMKIINewNav_private_h_
#define RTW_HEADER_controlMCUSlugsMKIINewNav_private_h_
#include "rtwtypes.h"

/* dsPIC library */
#include "p33fxxxx.h"
#include "timer.h"
#define fcy                            40000000

/* initialization Block: <S416>/Input Capture */
#include <incap.h>

/*initialization Block: <S9>/PWM Servo Output [dR dE dA dT]*/
#include "pwm.h"
#define CALL_EVENT                     (-1)

/* Private macros used by the generated code to access rtModel */
#ifndef rtmSetFirstInitCond
# define rtmSetFirstInitCond(rtm, val) ((rtm)->Timing.firstInitCondFlag = (val))
#endif

#ifndef rtmIsFirstInitCond
# define rtmIsFirstInitCond(rtm)       ((rtm)->Timing.firstInitCondFlag)
#endif

#ifndef UCHAR_MAX
#include <limits.h>
#endif

#if ( UCHAR_MAX != (0xFFU) ) || ( SCHAR_MAX != (0x7F) )
#error "Code was generated for compiler with different sized uchar/char. Consider adjusting Emulation Hardware word size settings on the Hardware Implementation pane to match your compiler word sizes as defined in the compiler's limits.h header file. Alternatively, you can select 'None' for Emulation Hardware and select the 'Enable portable word sizes' option for ERT based targets, which will disable the preprocessor word size checks."
#endif

#if ( USHRT_MAX != (0xFFFFU) ) || ( SHRT_MAX != (0x7FFF) )
#error "Code was generated for compiler with different sized ushort/short. Consider adjusting Emulation Hardware word size settings on the Hardware Implementation pane to match your compiler word sizes as defined in the compilers limits.h header file. Alternatively, you can select 'None' for Emulation Hardware and select the 'Enable portable word sizes' option for ERT based targets, this will disable the preprocessor word size checks."
#endif

#if ( UINT_MAX != (0xFFFFU) ) || ( INT_MAX != (0x7FFF) )
#error "Code was generated for compiler with different sized uint/int. Consider adjusting Emulation Hardware word size settings on the Hardware Implementation pane to match your compiler word sizes as defined in the compilers limits.h header file. Alternatively, you can select 'None' for Emulation Hardware and select the 'Enable portable word sizes' option for ERT based targets, this will disable the preprocessor word size checks."
#endif

#if ( ULONG_MAX != (0xFFFFFFFFUL) ) || ( LONG_MAX != (0x7FFFFFFFL) )
#error "Code was generated for compiler with different sized ulong/long. Consider adjusting Emulation Hardware word size settings on the Hardware Implementation pane to match your compiler word sizes as defined in the compilers limits.h header file. Alternatively, you can select 'None' for Emulation Hardware and select the 'Enable portable word sizes' option for ERT based targets, this will disable the preprocessor word size checks."
#endif

#ifndef __RTWTYPES_H__
#error This file requires rtwtypes.h to be included
#else
#ifdef TMWTYPES_PREVIOUSLY_INCLUDED
#error This file requires rtwtypes.h to be included before tmwtypes.h
#else

/* Check for inclusion of an incorrect version of rtwtypes.h */
#ifndef RTWTYPES_ID_C08S16I16L32N16F1
#error This code was generated with a different "rtwtypes.h" than the file included
#endif                                 /* RTWTYPES_ID_C08S16I16L32N16F1 */
#endif                                 /* TMWTYPES_PREVIOUSLY_INCLUDED */
#endif                                 /* __RTWTYPES_H__ */

#define _SPI1EN                        (SPI1STATbits.SPIEN)
#define _SPI2EN                        (SPI2STATbits.SPIEN)

extern uint16_T CalculusTimeStep;      /* Global Variable Used in <S13>/Calculus Time Step*/
extern real32_T rt_atan2f_snf(real32_T u0, real32_T u1);

/* Exported functions */
extern volatile uint16_T ic1up;
extern volatile uint16_T ic2up;
extern volatile uint16_T ic3up;
extern volatile uint16_T ic4up;
extern volatile uint16_T ic5up;
extern volatile uint16_T ic7up;
extern volatile uint16_T ic8up;
extern uint8_T getNavMode();
extern uint16_T meanFilter5(uint16_T* u1);
extern uint8_T isApManual(uint16_T u1);
extern void gsRead(uint8_T* y1);
extern void readIpc(uint8_T* y1);
extern void protDecodeMavlink(uint8_T* u1);
extern void protDecodeMavlink(uint8_T* u1);
extern void updateLoad(uint8_T u1);
extern void getMidLevelCommands(real32_T* y1);
extern real32_T getDynamic();
extern void getRangeOfParams(uint8_T u1, uint8_T u2,real32_T* y1);
extern uint8_T justEnabled(uint8_T u1, uint8_T u2);
extern uint16_T meanFilter5(uint16_T* u1);
extern void updatePWMTrim(uint16_T u1, uint8_T u2);
extern uint16_T meanFilter5(uint16_T* u1);
extern void updatePWMTrim(uint16_T u1, uint8_T u2);
extern uint16_T meanFilter5(uint16_T* u1);
extern void updatePWMTrim(uint16_T u1, uint8_T u2);
extern uint16_T meanFilter5(uint16_T* u1);
extern void updatePWMTrim(uint16_T u1, uint8_T u2);
extern uint8_T isPassthrough();
extern void getXYZ(real32_T* y1);
extern uint8_T getMaxWp();
extern void getVned(real32_T* y1);
extern void getRangeOfParams(uint8_T u1, uint8_T u2,real32_T* y1);
extern uint8_T isWpFly();
extern void getRTB(uint8_T* y1);
extern void getMobileLocation(real32_T* y1);
extern void getRangeOfParams(uint8_T u1, uint8_T u2,real32_T* y1);
extern void getISRLocation(real32_T* y1);
extern uint8_T getNavMode();
extern uint8_T justEnabled(uint8_T u1, uint8_T u2);
extern void getGSLocation(real32_T* y1);
extern real32_T mySqrt(real32_T u1);
extern real32_T mySqrt(real32_T u1);
extern void getWP(uint8_T u1,real32_T* y1);
extern void getWP(uint8_T u1,real32_T* y1);
extern real32_T mySqrt(real32_T u1);
extern real32_T mySqrt(real32_T u1);
extern real32_T mySqrt(real32_T u1);
extern uint8_T justEnabled(uint8_T u1, uint8_T u2);
extern void getWP(uint8_T u1,real32_T* y1);
extern void setLogFloat2(real32_T* u1);
extern void getWP(uint8_T u1,real32_T* y1);
extern real32_T mySqrt(real32_T u1);
extern real32_T mySqrt(real32_T u1);
extern void setLogFloat1(real32_T* u1);
extern real32_T myAbs(real32_T u1);
extern real32_T mySqrt(real32_T u1);
extern real32_T mySqrt(real32_T u1);
extern real32_T mySqrt(real32_T u1);
extern real32_T myAbs(real32_T u1);
extern real32_T mySqrt(real32_T u1);
extern real32_T mySqrt(real32_T u1);
extern real32_T myAcos(real32_T u1);
extern real32_T mySqrt(real32_T u1);
extern real32_T myAcos(real32_T u1);
extern real32_T myAbs(real32_T u1);
extern real32_T myAbs(real32_T u1);
extern void setDiagnosticFloat(real32_T* u1);
extern void setDiagnosticShort(int16_T* u1);
extern void setNavNav(real32_T* u1);
extern real32_T mySqrt(real32_T u1);
extern real32_T myAtan2(real32_T u1, real32_T u2);
extern real32_T myAbs(real32_T u1);
extern real32_T myAtan(real32_T u1);
extern void getRangeOfParams(uint8_T u1, uint8_T u2,real32_T* y1);
extern real32_T getParamIdx(uint8_T u1);
extern void getRangeOfParams(uint8_T u1, uint8_T u2,real32_T* y1);
extern void getAttitude(real32_T* y1);
extern void getRangeOfParams(uint8_T u1, uint8_T u2,real32_T* y1);
extern real32_T getParamIdx(uint8_T u1);
extern real32_T myPow(real32_T u1, real32_T u2);
extern real32_T mySqrt(real32_T u1);
extern void getGSLocation(real32_T* y1);
extern void setNavLong(real32_T* u1);
extern real32_T myCos(real32_T u1);
extern void getRangeOfParams(uint8_T u1, uint8_T u2,real32_T* y1);
extern void getRangeOfParams(uint8_T u1, uint8_T u2,real32_T* y1);
extern real32_T myCos(real32_T u1);
extern real32_T mySin(real32_T u1);
extern void getRangeOfParams(uint8_T u1, uint8_T u2,real32_T* y1);
extern real32_T myExp(real32_T u1);
extern real32_T myExp(real32_T u1);
extern void getAccels(real32_T* y1);
extern void getAccBias(real32_T* y1);
extern real32_T myTan(real32_T u1);
extern real32_T myTan(real32_T u1);
extern real32_T myAtan(real32_T u1);
extern real32_T myAtan(real32_T u1);
extern void setNavLat(real32_T* u1);
extern void getPassValues(uint8_T* y1);
extern void updatePWM(uint16_T* u1);
extern void updateEuler(real32_T* u1);
extern void updatePQR(real32_T* u1);
extern void updateVISensor(uint16_T* u1);
extern void prepareTelemetryMavlink(uint8_T* y1);
extern void send2GS(uint8_T* u1);
extern void getRangeOfParams(uint8_T u1, uint8_T u2,real32_T* y1);
extern uint8_T getISRCameraOption1();
extern real32_T mySqrt(real32_T u1);
extern real32_T myAsin(real32_T u1);
extern real32_T myAtan2(real32_T u1, real32_T u2);
extern uint8_T getGoHome();
extern int16_T getPanISRAide(uint8_T u1);
extern int16_T getTiltISRAide(uint8_T u1);
extern uint8_T getGoHome();
extern uint16_T getTiltValue(uint8_T u1);
extern uint16_T getPanValue(uint8_T u1);
extern real32_T myAbs(real32_T u1);
extern void updatePan(uint16_T u1);
extern void updateRoll(uint16_T u1);
extern void updateTilt(uint16_T u1);
extern uint8_T getCurrentZoom();
extern void changeZoom(uint8_T u1);
extern void setCameraConfig();
extern void updatePTZ(int16_T* u1);
extern uint8_T getLightsOnOff();
extern uint8_T getLightsDayNight();
extern uint8_T getHilOnOff();
extern uint8_T isPassthrough();
inline extern void controlMCUInit();
extern void con_EmbeddedMATLABFunction_Init(rtDW_EmbeddedMATLABFunction_con
  *localDW);
extern void controlM_EmbeddedMATLABFunction(real32_T rtu_u, real_T rtu_T, real_T
  rtu_f, rtB_EmbeddedMATLABFunction_cont *localB,
  rtDW_EmbeddedMATLABFunction_con *localDW);
extern void controlMCUSlugsMKIINe_myMuxFun1(real32_T rtu_u1, real32_T rtu_u2,
  real32_T rtu_u3, rtB_myMuxFun1_controlMCUSlugsMK *localB);
extern void controlMCUSlugsMKIIN_negprotect(real32_T rtu_val,
  rtB_negprotect_controlMCUSlugsM *localB);
extern void contro_EmbeddedMATLABFunction_o(const real32_T rtu_x[3],
  rtB_EmbeddedMATLABFunction_co_h *localB);
extern void controlMCUSlugsMKIINe_ZerooutZ1(const real32_T rtu_Pin[3],
  rtB_ZerooutZ1_controlMCUSlugsMK *localB);
extern void contro_EmbeddedMATLABFunction_d(const real32_T rtu_x[3], const
  real32_T rtu_y[3], rtB_EmbeddedMATLABFunction_co_k *localB);
extern void controlMCUSlugsMKI_SelectNTerms(const real32_T rtu_T[3],
  rtB_SelectNTerms_controlMCUSlug *localB);
extern void controlMCUSlugsMKII_negprotect3(real32_T rtu_val,
  rtB_negprotect3_controlMCUSlugs *localB);
extern void controlMCU_BufferICChannel_Init(rtDW_BufferICChannel_controlMCU
  *localDW);
extern void controlMCUSlugs_BufferICChannel(uint16_T rtu_latest,
  rtB_BufferICChannel_controlMCUS *localB, rtDW_BufferICChannel_controlMCU
  *localDW);
extern void controlMCUSlugsMKII_myMuxFun1_e(uint16_T rtu_u1, uint16_T rtu_u2,
  uint16_T rtu_u3, uint16_T rtu_u4, uint16_T rty_y[4]);

#endif                                 /* RTW_HEADER_controlMCUSlugsMKIINewNav_private_h_ */

/* File trailer for Real-Time Workshop generated code.
 *
 * [EOF]
 */
