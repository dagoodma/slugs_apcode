/*
 * File: sensorMCUSlugsMKII_private.h
 *
 * Real-Time Workshop code generated for Simulink model sensorMCUSlugsMKII.
 *
 * Model version                        : 1.213
 * Real-Time Workshop file version      : 8.1 (R2011b) 08-Jul-2011
 * Real-Time Workshop file generated on : Wed May 13 21:47:48 2015
 * TLC version                          : 8.1 (Jul  9 2011)
 * C source code generated on           : Wed May 13 21:47:48 2015
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
 *   Version 3.7c                              09-Jan-2012      |
 *   For Matlab 7.10            R2010a                          |
 *--------------------------------------------------------------
 */

#ifndef RTW_HEADER_sensorMCUSlugsMKII_private_h_
#define RTW_HEADER_sensorMCUSlugsMKII_private_h_
#include "rtwtypes.h"

/* dsPIC library */
#include "p33fxxxx.h"
#include "timer.h"
#define fcy                            40000000

/* initialization Block: <S1>/Input Capture */
#include <incap.h>
#define CALL_EVENT                     (-1)
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

extern uint16_T CalculusTimeStep;      /* Global Variable Used in <S118>/Calculus Time Step*/
extern volatile uint16_T ADCBuffChannel0[5] ;
extern volatile uint16_T ADCBuffChannel1[5] ;
extern volatile uint16_T ADCBuffChannel2[5] ;
extern volatile uint16_T ADCBuffChannel3[5] ;
extern volatile uint8_T ADCFlag ;
extern volatile uint16_T ADCBuffChannelDMA[(4)] __attribute__((space(dma),
  aligned(256)));
extern real32_T rt_atan2f_snf(real32_T u0, real32_T u1);

/* Exported functions */
extern volatile uint16_T ic2up;
extern volatile uint16_T ic3up;
extern volatile uint16_T ic4up;
extern volatile uint16_T ic5up;
extern volatile uint16_T ic8up;
extern real32_T mySqrt(real32_T u1);
extern void getGSLocation(real32_T* y1);
extern void getPressure(int16_T* y1);
extern void updateRawADCData(int16_T* u1);
extern void getCubeData(int16_T* y1);
extern uint8_T isGPSNovatel();
extern void getGPSRawData(uint8_T* y1);
extern void gpsParse(uint8_T* u1);
extern void getGpsMainData(real32_T* y1);
extern void getGpsUbloxMainData(real32_T* y1);
extern uint8_T isFixValid();
extern real32_T myCos(real32_T u1);
extern real32_T mySin(real32_T u1);
extern void getGSLocation(real32_T* y1);
extern void hilRead(uint8_T* y1);
extern void protDecodeHil(uint8_T* u1);
extern void hil_getRawRead(int16_T* y1);
extern real32_T mySqrt(real32_T u1);
extern void hil_getXYZ(real32_T* y1);
extern void hil_getVned(real32_T* y1);
extern int32_T hil_getTs();
extern void hil_getEuler(real32_T* y1);
extern void hil_getRates(real32_T* y1);
extern void updateAttitude(real32_T* u1);
extern void updateTimeStamp(uint32_T u1);
extern void updateSensorData(real32_T* u1);
extern void updatePosition(real32_T* u1);
extern void updatePilotConsole(uint16_T* u1);
extern void updateBias(real32_T* u1);
extern void scheduleData(uint8_T u1,uint8_T* y1);
extern void spiSend(uint8_T* u1);
extern void updateAirData(real32_T* u1);
extern void updateSensorDiag(real32_T* u1);
extern void updateLoadData(uint8_T u1, uint16_T u2);
inline extern void sensorMCUInit();
extern void sensorMCUSlugsMKII_myMuxFun1(real_T rtu_u1, real_T rtu_u2, real_T
  rtu_u3, rtB_myMuxFun1_sensorMCUSlugsMKI *localB);
extern void sensorMCUSlugsMKII_negprotect(real32_T rtu_val,
  rtB_negprotect_sensorMCUSlugsMK *localB);
extern void sensorMC_EmbeddedMATLABFunction(const real32_T rtu_x[3],
  rtB_EmbeddedMATLABFunction_sens *localB);
extern void se_EmbeddedMATLABFunction1_Init(rtDW_EmbeddedMATLABFunction1_se
  *localDW);
extern void sensorM_EmbeddedMATLABFunction1(real32_T rtu_u, uint8_T rtu_NewGPS,
  rtB_EmbeddedMATLABFunction1_sen *localB, rtDW_EmbeddedMATLABFunction1_se
  *localDW, rtP_EmbeddedMATLABFunction1_sen *localP);
extern void sensorMCUSlugsMKII_myMuxFun1_l(real32_T rtu_u1, real32_T rtu_u2,
  real32_T rtu_u3, rtB_myMuxFun1_sensorMCUSlugsM_n *localB);
extern void s_EmbeddedMATLABFunction_p_Init(rtDW_EmbeddedMATLABFunction_s_i
  *localDW);
extern void sensor_EmbeddedMATLABFunction_c(real_T rtu_u, real_T rtu_T, real_T
  rtu_f, rtB_EmbeddedMATLABFunction_se_k *localB,
  rtDW_EmbeddedMATLABFunction_s_i *localDW);
extern void sensorMCUSlugsMKII_myMuxFun(real_T rtu_u1, real_T rtu_u2, real_T
  rtu_u3, rtB_myMuxFun_sensorMCUSlugsMKII *localB);
extern void sensor_EmbeddedMATLABFunction_g(const uint16_T rtu_u[5],
  rtB_EmbeddedMATLABFunction_se_l *localB);
extern void sensorMCUSlugsMKII_myMuxFun2(const real32_T rtu_u1[3], const
  real32_T rtu_u2[3], rtB_myMuxFun2_sensorMCUSlugsMKI *localB);

#endif                                 /* RTW_HEADER_sensorMCUSlugsMKII_private_h_ */

/* File trailer for Real-Time Workshop generated code.
 *
 * [EOF]
 */
