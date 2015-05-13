/*
 * File: sensorMCUSlugsMKII.h
 *
 * Real-Time Workshop code generated for Simulink model sensorMCUSlugsMKII.
 *
 * Model version                        : 1.211
 * Real-Time Workshop file version      : 8.1 (R2011b) 08-Jul-2011
 * Real-Time Workshop file generated on : Tue May 12 21:45:37 2015
 * TLC version                          : 8.1 (Jul  9 2011)
 * C source code generated on           : Tue May 12 21:45:38 2015
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

#ifndef RTW_HEADER_sensorMCUSlugsMKII_h_
#define RTW_HEADER_sensorMCUSlugsMKII_h_
#ifndef sensorMCUSlugsMKII_COMMON_INCLUDES_
# define sensorMCUSlugsMKII_COMMON_INCLUDES_
#include <math.h>
#include <string.h>
#include "rtwtypes.h"
#include "rt_nonfinite.h"
#include "rtGetInf.h"
#include "rtGetNaN.h"
#include "rt_defines.h"
#endif                                 /* sensorMCUSlugsMKII_COMMON_INCLUDES_ */

#include "sensorMCUSlugsMKII_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((void*) 0)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((void) 0)
#endif

#ifndef rtmGetStopRequested
# define rtmGetStopRequested(rtm)      ((void*) 0)
#endif

/* Block signals for system '<S19>/myMux Fun1' */
typedef struct {
  real_T y[3];                         /* '<S19>/myMux Fun1' */
} rtB_myMuxFun1_sensorMCUSlugsMKI;

/* Block signals for system '<S46>/negprotect' */
typedef struct {
  real32_T zpVal;                      /* '<S46>/negprotect' */
} rtB_negprotect_sensorMCUSlugsMK;

/* Block signals for system '<S47>/Embedded MATLAB Function' */
typedef struct {
  real32_T xDoty;                      /* '<S47>/Embedded MATLAB Function' */
} rtB_EmbeddedMATLABFunction_sens;

/* Block signals for system '<S94>/Embedded MATLAB Function1' */
typedef struct {
  real32_T y;                          /* '<S94>/Embedded MATLAB Function1' */
} rtB_EmbeddedMATLABFunction1_sen;

/* Block states (auto storage) for system '<S94>/Embedded MATLAB Function1' */
typedef struct {
  real32_T LastU;                      /* '<S94>/Embedded MATLAB Function1' */
  real32_T TimeSinceLast;              /* '<S94>/Embedded MATLAB Function1' */
  real32_T rate;                       /* '<S94>/Embedded MATLAB Function1' */
  real32_T OldRate;                    /* '<S94>/Embedded MATLAB Function1' */
  boolean_T LastU_not_empty;           /* '<S94>/Embedded MATLAB Function1' */
} rtDW_EmbeddedMATLABFunction1_se;

/* Block signals for system '<S86>/myMux Fun1' */
typedef struct {
  real32_T y[3];                       /* '<S86>/myMux Fun1' */
} rtB_myMuxFun1_sensorMCUSlugsM_n;

/* Block signals for system '<S124>/Embedded MATLAB Function' */
typedef struct {
  real_T y;                            /* '<S124>/Embedded MATLAB Function' */
} rtB_EmbeddedMATLABFunction_se_k;

/* Block states (auto storage) for system '<S124>/Embedded MATLAB Function' */
typedef struct {
  real_T a;                            /* '<S124>/Embedded MATLAB Function' */
  real_T b;                            /* '<S124>/Embedded MATLAB Function' */
  real_T y_km1;                        /* '<S124>/Embedded MATLAB Function' */
  real_T u_km1;                        /* '<S124>/Embedded MATLAB Function' */
  boolean_T a_not_empty;               /* '<S124>/Embedded MATLAB Function' */
} rtDW_EmbeddedMATLABFunction_s_i;

/* Block signals for system '<S124>/myMux Fun' */
typedef struct {
  real_T y[3];                         /* '<S124>/myMux Fun' */
} rtB_myMuxFun_sensorMCUSlugsMKII;

/* Block signals for system '<S119>/Embedded MATLAB Function' */
typedef struct {
  uint16_T y;                          /* '<S119>/Embedded MATLAB Function' */
} rtB_EmbeddedMATLABFunction_se_l;

/* Block signals for system '<Root>/myMux Fun2' */
typedef struct {
  real32_T y[6];                       /* '<Root>/myMux Fun2' */
} rtB_myMuxFun2_sensorMCUSlugsMKI;

/* Block signals (auto storage) */
typedef struct {
  real32_T VectorConcatenate[9];       /* '<S66>/Vector Concatenate' */
  real32_T Submatrix1[3];              /* '<S12>/Submatrix1' */
  real32_T CFunctionCall;              /* '<S51>/C Function Call' */
  real32_T GettheGSLocationupdateSensorMCU[3];/* '<S2>/Get the GS Location [updateSensorMCUState.c]' */
  real32_T UnitConversion;             /* '<S70>/Unit Conversion' */
  real32_T CFunctionCall1;             /* '<S71>/C Function Call1' */
  real32_T CFunctionCall_k;            /* '<S71>/C Function Call' */
  real32_T GettheGSLocationupdateSensorM_m[3];/* '<S166>/Get the GS Location [updateSensorMCUState.c]' */
  real32_T Merge;                      /* '<S156>/Merge' */
  real32_T CFunctionCall_kv;           /* '<S46>/C Function Call' */
  real32_T g_hat[3];                   /* '<S12>/Submatrix' */
  real32_T ReadXYZfromHILhilc[3];      /* '<S3>/Read XYZ from HIL [hil.c]' */
  real32_T ReadtheVnedfromHILhilc[3];  /* '<S3>/Read the Vned  from HIL [hil.c]' */
  real32_T ReadEulerfromHILhilc[3];    /* '<S3>/Read Euler  from HIL [hil.c]' */
  real32_T ReadPQRfromHILhilc[3];      /* '<S3>/Read PQR from HIL [hil.c]' */
  real32_T y[9];                       /* '<Root>/myMux Fun1' */
  real32_T y_n[4];                     /* '<S116>/myMux Fun1' */
  real32_T Sum;                        /* '<S168>/Sum' */
  real32_T u0k120k;                    /* '<S167>/[80k - 120k]' */
  real32_T In1;                        /* '<S164>/In1' */
  real32_T AirData[3];                 /* '<S120>/AirData' */
  real32_T Switch2[5];                 /* '<S119>/Switch2' */
  real32_T ProducetheGPSMainDataandupdatet[5];/* '<S147>/Produce the GPS Main Data and update the AP State (lat lon hei cog sog) [gpsUblox.c]' */
  real32_T ProducetheGPSMainDataandupdat_c[5];/* '<S146>/Produce the GPS Main Data and update the AP State (lat lon hei cog sog) [gps.c//novatel.c]' */
  real32_T In1_p[3];                   /* '<S15>/In1' */
  real32_T In2;                        /* '<S15>/In2' */
  real32_T In3;                        /* '<S15>/In3' */
  int32_T ReadtimestampfromHILhilc;    /* '<S3>/Read timestamp from HIL [hil.c]' */
  uint32_T Switch5;                    /* '<S3>/Switch5' */
  int16_T ReadthePressureDatapressurec[2];/* '<S119>/Read the Pressure Data [pressure.c]' */
  int16_T ReadtheCubeDataadisCube16405c[10];/* '<S119>/Read the Cube Data [adisCube16405.c]' */
  int16_T y_h[13];                     /* '<S119>/myMux Fun4' */
  int16_T y_nt[4];                     /* '<S119>/myMux Fun' */
  int16_T HILRawReadingshilc[13];      /* '<S115>/HIL Raw Readings [hil.c]' */
  uint16_T dT;                         /* '<S1>/Input Capture' */
  uint16_T dA;                         /* '<S1>/Input Capture' */
  uint16_T dE;                         /* '<S1>/Input Capture' */
  uint16_T dR;                         /* '<S1>/Input Capture' */
  uint16_T dFailsafe;                  /* '<S1>/Input Capture' */
  uint16_T DataTypeConversion[5];      /* '<S10>/Data Type Conversion' */
  uint16_T CalculusTimeStep_o1;        /* '<S118>/Calculus Time Step' */
  uint16_T CalculusTimeStep_o2;        /* '<S118>/Calculus Time Step' */
  uint16_T DataTypeConversion8;        /* '<S121>/Data Type Conversion8' */
  uint16_T Baro[5];                    /* '<S119>/ADC Input' */
  uint16_T Pitot[5];                   /* '<S119>/ADC Input' */
  uint16_T Power[5];                   /* '<S119>/ADC Input' */
  uint16_T Thermistor[5];              /* '<S119>/ADC Input' */
  uint8_T DataTypeConversion_e;        /* '<S4>/Data Type Conversion' */
  uint8_T ChecksifFixTypeis3updateSensorM;/* '<S2>/Checks if FixType is 3 [updateSensorMCUState.c]' */
  uint8_T SendDebugDatatoSerialPortandPre[110];/* '<S5>/Send Debug  Data to Serial Port and Prepare for SPI [IPCScheduler.c]' */
  uint8_T DataTypeConversion12;        /* '<S118>/Data Type Conversion12' */
  uint8_T IstheGPSNovatelorUbloxgpsPortc;/* '<S119>/Is the GPS Novatel or Ublox? [gpsPort.c]' */
  uint8_T ReadtheRawDatafromGPSgpsPortc[180];/* '<S146>/Read the Raw Data from GPS [gpsPort.c]' */
  uint8_T DatafromHILhilc[110];        /* '<S115>/Data from HIL [hil.c]' */
  boolean_T EnableHILfromControlMCU;   /* '<S4>/Enable HIL from  Control MCU' */
  boolean_T EnableHILAttitudefromControlMCU;/* '<S3>/Enable HIL Attitude from Control MCU' */
  rtB_myMuxFun2_sensorMCUSlugsMKI sf_myMuxFun4;/* '<Root>/myMux Fun4' */
  rtB_myMuxFun2_sensorMCUSlugsMKI sf_myMuxFun3;/* '<Root>/myMux Fun3' */
  rtB_myMuxFun2_sensorMCUSlugsMKI sf_myMuxFun2_l;/* '<Root>/myMux Fun2' */
  rtB_EmbeddedMATLABFunction_se_k sf_EmbeddedMATLABFunction_e;/* '<S162>/Embedded MATLAB Function' */
  rtB_EmbeddedMATLABFunction_se_k sf_EmbeddedMATLABFunction_f;/* '<S161>/Embedded MATLAB Function' */
  rtB_EmbeddedMATLABFunction_se_k sf_EmbeddedMATLABFunction_gc;/* '<S160>/Embedded MATLAB Function' */
  rtB_EmbeddedMATLABFunction_se_k sf_EmbeddedMATLABFunction_b;/* '<S159>/Embedded MATLAB Function' */
  rtB_EmbeddedMATLABFunction_se_l sf_EmbeddedMATLABFunction3_c;/* '<S119>/Embedded MATLAB Function3' */
  rtB_EmbeddedMATLABFunction_se_l sf_EmbeddedMATLABFunction2_dh;/* '<S119>/Embedded MATLAB Function2' */
  rtB_EmbeddedMATLABFunction_se_l sf_EmbeddedMATLABFunction1_b;/* '<S119>/Embedded MATLAB Function1' */
  rtB_EmbeddedMATLABFunction_se_l sf_EmbeddedMATLABFunction_gn;/* '<S119>/Embedded MATLAB Function' */
  rtB_myMuxFun_sensorMCUSlugsMKII sf_myMuxFun_f;/* '<S126>/myMux Fun' */
  rtB_EmbeddedMATLABFunction_se_k sf_EmbeddedMATLABFunction2_b;/* '<S126>/Embedded MATLAB Function2' */
  rtB_EmbeddedMATLABFunction_se_k sf_EmbeddedMATLABFunction1_o;/* '<S126>/Embedded MATLAB Function1' */
  rtB_EmbeddedMATLABFunction_se_k sf_EmbeddedMATLABFunction_d;/* '<S126>/Embedded MATLAB Function' */
  rtB_myMuxFun_sensorMCUSlugsMKII sf_myMuxFun;/* '<S125>/myMux Fun' */
  rtB_EmbeddedMATLABFunction_se_k sf_EmbeddedMATLABFunction2;/* '<S125>/Embedded MATLAB Function2' */
  rtB_EmbeddedMATLABFunction_se_k sf_EmbeddedMATLABFunction1;/* '<S125>/Embedded MATLAB Function1' */
  rtB_EmbeddedMATLABFunction_se_k sf_EmbeddedMATLABFunction_p;/* '<S125>/Embedded MATLAB Function' */
  rtB_myMuxFun_sensorMCUSlugsMKII sf_myMuxFun_c;/* '<S124>/myMux Fun' */
  rtB_EmbeddedMATLABFunction_se_k sf_EmbeddedMATLABFunction2_l;/* '<S124>/Embedded MATLAB Function2' */
  rtB_EmbeddedMATLABFunction_se_k sf_EmbeddedMATLABFunction1_f;/* '<S124>/Embedded MATLAB Function1' */
  rtB_EmbeddedMATLABFunction_se_k sf_EmbeddedMATLABFunction_c;/* '<S124>/Embedded MATLAB Function' */
  rtB_myMuxFun1_sensorMCUSlugsM_n sf_myMuxFun2_f;/* '<S86>/myMux Fun2' */
  rtB_myMuxFun1_sensorMCUSlugsM_n sf_myMuxFun1_l;/* '<S86>/myMux Fun1' */
  rtB_EmbeddedMATLABFunction1_sen sf_EmbeddedMATLABFunction2_d;/* '<S94>/Embedded MATLAB Function2' */
  rtB_EmbeddedMATLABFunction1_sen sf_EmbeddedMATLABFunction1_a;/* '<S94>/Embedded MATLAB Function1' */
  rtB_myMuxFun1_sensorMCUSlugsMKI sf_myMuxFun1;/* '<S88>/myMux Fun1' */
  rtB_EmbeddedMATLABFunction_sens sf_EmbeddedMATLABFunction;/* '<S52>/Embedded MATLAB Function' */
  rtB_negprotect_sensorMCUSlugsMK sf_negprotect;/* '<S51>/negprotect' */
  rtB_EmbeddedMATLABFunction_sens sf_EmbeddedMATLABFunction_g;/* '<S47>/Embedded MATLAB Function' */
  rtB_negprotect_sensorMCUSlugsMK sf_negprotect_m;/* '<S46>/negprotect' */
  rtB_myMuxFun1_sensorMCUSlugsMKI sf_myMuxFun1_i;/* '<S19>/myMux Fun1' */
} BlockIO_sensorMCUSlugsMKII;

/* Block states (auto storage) for system '<Root>' */
typedef struct {
  real_T DiscreteZeroPole_DSTATE;      /* '<S100>/Discrete Zero-Pole' */
  real_T DiscreteZeroPole_DSTATE_k;    /* '<S101>/Discrete Zero-Pole' */
  real_T DiscreteZeroPole_DSTATE_e;    /* '<S102>/Discrete Zero-Pole' */
  real_T DiscreteZeroPole_DSTATE_kq;   /* '<S32>/Discrete Zero-Pole' */
  real_T DiscreteZeroPole_DSTATE_g;    /* '<S33>/Discrete Zero-Pole' */
  real_T DiscreteZeroPole_DSTATE_g3;   /* '<S34>/Discrete Zero-Pole' */
  real_T DiscreteZeroPole_DSTATE_f;    /* '<S105>/Discrete Zero-Pole' */
  real_T DiscreteZeroPole_DSTATE_a;    /* '<S107>/Discrete Zero-Pole' */
  real_T DiscreteZeroPole_DSTATE_n;    /* '<S109>/Discrete Zero-Pole' */
  real_T DiscreteZeroPole_DSTATE_p;    /* '<S170>/Discrete Zero-Pole' */
  real_T aveCount;                     /* '<S150>/Enables//Disables the Computation of  initial Baro Bias' */
  real_T tIni;                         /* '<S150>/Enables//Disables the Computation of  initial Baro Bias' */
  real_T a;                            /* '<S98>/Embedded MATLAB Function' */
  real_T b;                            /* '<S98>/Embedded MATLAB Function' */
  real32_T DiscreteTimeIntegrator1_DSTATE[4];/* '<S12>/Discrete-Time Integrator1' */
  real32_T IntegerDelay_DSTATE[3];     /* '<S12>/Integer Delay' */
  real32_T UD_DSTATE;                  /* '<S78>/UD' */
  real32_T UD_DSTATE_e;                /* '<S79>/UD' */
  real32_T UD_DSTATE_k;                /* '<S80>/UD' */
  real32_T UnitDelay_DSTATE;           /* '<S91>/Unit Delay' */
  real32_T IntegerDelay1_DSTATE[15];   /* '<S23>/Integer Delay1' */
  real32_T IntegerDelay1_DSTATE_b[15]; /* '<S93>/Integer Delay1' */
  real32_T UnitDelay_DSTATE_p;         /* '<S89>/Unit Delay' */
  real32_T UnitDelay_DSTATE_f;         /* '<S90>/Unit Delay' */
  real32_T IntegerDelay_DSTATE_m;      /* '<S168>/Integer Delay' */
  uint32_T Output_DSTATE;              /* '<S112>/Output' */
  real32_T PrevY[3];                   /* '<S86>/Rate Limiter' */
  real32_T PrevY_e[3];                 /* '<S12>/Bias Rate Limiter' */
  real32_T y_km1;                      /* '<S98>/Embedded MATLAB Function' */
  real32_T u_km1;                      /* '<S98>/Embedded MATLAB Function' */
  real32_T lastGps_h;                  /* '<S87>/Embedded MATLAB Function3' */
  real32_T TimeSinceLast;              /* '<S87>/Embedded MATLAB Function3' */
  boolean_T a_not_empty;               /* '<S98>/Embedded MATLAB Function' */
  boolean_T lastGps_h_not_empty;       /* '<S87>/Embedded MATLAB Function3' */
  boolean_T Subsystem_MODE;            /* '<S86>/Subsystem' */
  rtDW_EmbeddedMATLABFunction_s_i sf_EmbeddedMATLABFunction_e;/* '<S162>/Embedded MATLAB Function' */
  rtDW_EmbeddedMATLABFunction_s_i sf_EmbeddedMATLABFunction_f;/* '<S161>/Embedded MATLAB Function' */
  rtDW_EmbeddedMATLABFunction_s_i sf_EmbeddedMATLABFunction_gc;/* '<S160>/Embedded MATLAB Function' */
  rtDW_EmbeddedMATLABFunction_s_i sf_EmbeddedMATLABFunction_b;/* '<S159>/Embedded MATLAB Function' */
  rtDW_EmbeddedMATLABFunction_s_i sf_EmbeddedMATLABFunction2_b;/* '<S126>/Embedded MATLAB Function2' */
  rtDW_EmbeddedMATLABFunction_s_i sf_EmbeddedMATLABFunction1_o;/* '<S126>/Embedded MATLAB Function1' */
  rtDW_EmbeddedMATLABFunction_s_i sf_EmbeddedMATLABFunction_d;/* '<S126>/Embedded MATLAB Function' */
  rtDW_EmbeddedMATLABFunction_s_i sf_EmbeddedMATLABFunction2;/* '<S125>/Embedded MATLAB Function2' */
  rtDW_EmbeddedMATLABFunction_s_i sf_EmbeddedMATLABFunction1;/* '<S125>/Embedded MATLAB Function1' */
  rtDW_EmbeddedMATLABFunction_s_i sf_EmbeddedMATLABFunction_p;/* '<S125>/Embedded MATLAB Function' */
  rtDW_EmbeddedMATLABFunction_s_i sf_EmbeddedMATLABFunction2_l;/* '<S124>/Embedded MATLAB Function2' */
  rtDW_EmbeddedMATLABFunction_s_i sf_EmbeddedMATLABFunction1_f;/* '<S124>/Embedded MATLAB Function1' */
  rtDW_EmbeddedMATLABFunction_s_i sf_EmbeddedMATLABFunction_c;/* '<S124>/Embedded MATLAB Function' */
  rtDW_EmbeddedMATLABFunction1_se sf_EmbeddedMATLABFunction2_d;/* '<S94>/Embedded MATLAB Function2' */
  rtDW_EmbeddedMATLABFunction1_se sf_EmbeddedMATLABFunction1_a;/* '<S94>/Embedded MATLAB Function1' */
} D_Work_sensorMCUSlugsMKII;

/* Parameters for system: '<S94>/Embedded MATLAB Function1' */
struct rtP_EmbeddedMATLABFunction1_sen_ {
  real_T SFunction_p1;                 /* Expression: apSampleTime
                                        * Referenced by: '<S94>/Embedded MATLAB Function1'
                                        */
};

/* Parameters (auto storage) */
struct Parameters_sensorMCUSlugsMKII_ {
  real_T SFunction_p1;                 /* Expression: apSampleTime
                                        * Referenced by: '<S87>/Embedded MATLAB Function3'
                                        */
  real_T DiscreteZeroPole_A;           /* Computed Parameter: DiscreteZeroPole_A
                                        * Referenced by: '<S170>/Discrete Zero-Pole'
                                        */
  real_T DiscreteZeroPole_C;           /* Computed Parameter: DiscreteZeroPole_C
                                        * Referenced by: '<S170>/Discrete Zero-Pole'
                                        */
  real_T DiscreteZeroPole_D;           /* Computed Parameter: DiscreteZeroPole_D
                                        * Referenced by: '<S170>/Discrete Zero-Pole'
                                        */
  real_T Constant_Value;               /* Expression: T
                                        * Referenced by: '<S159>/Constant'
                                        */
  real_T Constant1_Value;              /* Expression: f
                                        * Referenced by: '<S159>/Constant1'
                                        */
  real_T Constant_Value_j;             /* Expression: T
                                        * Referenced by: '<S162>/Constant'
                                        */
  real_T Constant1_Value_d;            /* Expression: f
                                        * Referenced by: '<S162>/Constant1'
                                        */
  real_T Constant_Value_n;             /* Expression: T
                                        * Referenced by: '<S98>/Constant'
                                        */
  real_T Constant1_Value_k;            /* Expression: f
                                        * Referenced by: '<S98>/Constant1'
                                        */
  real_T Constant_Value_i;             /* Expression: T
                                        * Referenced by: '<S125>/Constant'
                                        */
  real_T Constant1_Value_k2;           /* Expression: f
                                        * Referenced by: '<S125>/Constant1'
                                        */
  real_T Constant2_Value;              /* Expression: T
                                        * Referenced by: '<S125>/Constant2'
                                        */
  real_T Constant3_Value;              /* Expression: f
                                        * Referenced by: '<S125>/Constant3'
                                        */
  real_T Constant4_Value;              /* Expression: T
                                        * Referenced by: '<S125>/Constant4'
                                        */
  real_T Constant5_Value;              /* Expression: f
                                        * Referenced by: '<S125>/Constant5'
                                        */
  real_T DiscreteZeroPole_A_j;         /* Computed Parameter: DiscreteZeroPole_A_j
                                        * Referenced by: '<S100>/Discrete Zero-Pole'
                                        */
  real_T DiscreteZeroPole_C_e;         /* Computed Parameter: DiscreteZeroPole_C_e
                                        * Referenced by: '<S100>/Discrete Zero-Pole'
                                        */
  real_T DiscreteZeroPole_D_j;         /* Computed Parameter: DiscreteZeroPole_D_j
                                        * Referenced by: '<S100>/Discrete Zero-Pole'
                                        */
  real_T DiscreteZeroPole_A_l;         /* Computed Parameter: DiscreteZeroPole_A_l
                                        * Referenced by: '<S101>/Discrete Zero-Pole'
                                        */
  real_T DiscreteZeroPole_C_o;         /* Computed Parameter: DiscreteZeroPole_C_o
                                        * Referenced by: '<S101>/Discrete Zero-Pole'
                                        */
  real_T DiscreteZeroPole_D_ju;        /* Computed Parameter: DiscreteZeroPole_D_ju
                                        * Referenced by: '<S101>/Discrete Zero-Pole'
                                        */
  real_T DiscreteZeroPole_A_i;         /* Computed Parameter: DiscreteZeroPole_A_i
                                        * Referenced by: '<S102>/Discrete Zero-Pole'
                                        */
  real_T DiscreteZeroPole_C_k;         /* Computed Parameter: DiscreteZeroPole_C_k
                                        * Referenced by: '<S102>/Discrete Zero-Pole'
                                        */
  real_T DiscreteZeroPole_D_p;         /* Computed Parameter: DiscreteZeroPole_D_p
                                        * Referenced by: '<S102>/Discrete Zero-Pole'
                                        */
  real_T DiscreteZeroPole_A_c;         /* Computed Parameter: DiscreteZeroPole_A_c
                                        * Referenced by: '<S32>/Discrete Zero-Pole'
                                        */
  real_T DiscreteZeroPole_C_c;         /* Computed Parameter: DiscreteZeroPole_C_c
                                        * Referenced by: '<S32>/Discrete Zero-Pole'
                                        */
  real_T DiscreteZeroPole_D_jb;        /* Computed Parameter: DiscreteZeroPole_D_jb
                                        * Referenced by: '<S32>/Discrete Zero-Pole'
                                        */
  real_T DiscreteZeroPole_A_i3;        /* Computed Parameter: DiscreteZeroPole_A_i3
                                        * Referenced by: '<S33>/Discrete Zero-Pole'
                                        */
  real_T DiscreteZeroPole_C_i;         /* Computed Parameter: DiscreteZeroPole_C_i
                                        * Referenced by: '<S33>/Discrete Zero-Pole'
                                        */
  real_T DiscreteZeroPole_D_o;         /* Computed Parameter: DiscreteZeroPole_D_o
                                        * Referenced by: '<S33>/Discrete Zero-Pole'
                                        */
  real_T DiscreteZeroPole_A_e;         /* Computed Parameter: DiscreteZeroPole_A_e
                                        * Referenced by: '<S34>/Discrete Zero-Pole'
                                        */
  real_T DiscreteZeroPole_C_p;         /* Computed Parameter: DiscreteZeroPole_C_p
                                        * Referenced by: '<S34>/Discrete Zero-Pole'
                                        */
  real_T DiscreteZeroPole_D_l;         /* Computed Parameter: DiscreteZeroPole_D_l
                                        * Referenced by: '<S34>/Discrete Zero-Pole'
                                        */
  real_T DiscreteZeroPole_A_a;         /* Computed Parameter: DiscreteZeroPole_A_a
                                        * Referenced by: '<S105>/Discrete Zero-Pole'
                                        */
  real_T DiscreteZeroPole_C_cr;        /* Computed Parameter: DiscreteZeroPole_C_cr
                                        * Referenced by: '<S105>/Discrete Zero-Pole'
                                        */
  real_T DiscreteZeroPole_D_c;         /* Computed Parameter: DiscreteZeroPole_D_c
                                        * Referenced by: '<S105>/Discrete Zero-Pole'
                                        */
  real_T DiscreteZeroPole_A_aw;        /* Computed Parameter: DiscreteZeroPole_A_aw
                                        * Referenced by: '<S107>/Discrete Zero-Pole'
                                        */
  real_T DiscreteZeroPole_C_b;         /* Computed Parameter: DiscreteZeroPole_C_b
                                        * Referenced by: '<S107>/Discrete Zero-Pole'
                                        */
  real_T DiscreteZeroPole_D_co;        /* Computed Parameter: DiscreteZeroPole_D_co
                                        * Referenced by: '<S107>/Discrete Zero-Pole'
                                        */
  real_T Constant_Value_g;             /* Expression: T
                                        * Referenced by: '<S126>/Constant'
                                        */
  real_T Constant1_Value_l;            /* Expression: f
                                        * Referenced by: '<S126>/Constant1'
                                        */
  real_T Constant2_Value_c;            /* Expression: T
                                        * Referenced by: '<S126>/Constant2'
                                        */
  real_T Constant3_Value_d;            /* Expression: f
                                        * Referenced by: '<S126>/Constant3'
                                        */
  real_T Constant4_Value_h;            /* Expression: T
                                        * Referenced by: '<S126>/Constant4'
                                        */
  real_T Constant5_Value_f;            /* Expression: f
                                        * Referenced by: '<S126>/Constant5'
                                        */
  real_T Constant_Value_d;             /* Expression: T
                                        * Referenced by: '<S124>/Constant'
                                        */
  real_T Constant1_Value_h;            /* Expression: f
                                        * Referenced by: '<S124>/Constant1'
                                        */
  real_T Constant2_Value_j;            /* Expression: T
                                        * Referenced by: '<S124>/Constant2'
                                        */
  real_T Constant3_Value_j;            /* Expression: f
                                        * Referenced by: '<S124>/Constant3'
                                        */
  real_T Constant4_Value_l;            /* Expression: T
                                        * Referenced by: '<S124>/Constant4'
                                        */
  real_T Constant5_Value_d;            /* Expression: f
                                        * Referenced by: '<S124>/Constant5'
                                        */
  real_T DiscreteZeroPole_A_f;         /* Computed Parameter: DiscreteZeroPole_A_f
                                        * Referenced by: '<S109>/Discrete Zero-Pole'
                                        */
  real_T DiscreteZeroPole_C_m;         /* Computed Parameter: DiscreteZeroPole_C_m
                                        * Referenced by: '<S109>/Discrete Zero-Pole'
                                        */
  real_T DiscreteZeroPole_D_d;         /* Computed Parameter: DiscreteZeroPole_D_d
                                        * Referenced by: '<S109>/Discrete Zero-Pole'
                                        */
  real_T Constant_Value_dj;            /* Expression: T
                                        * Referenced by: '<S161>/Constant'
                                        */
  real_T Constant1_Value_j;            /* Expression: f
                                        * Referenced by: '<S161>/Constant1'
                                        */
  real_T Bias_Value;                   /* Expression: pitotOffset
                                        * Referenced by: '<S152>/Bias'
                                        */
  real_T u001maxDynPress_UpperSat;     /* Expression: maxDynPressure
                                        * Referenced by: '<S121>/[0.001  maxDynPress]'
                                        */
  real_T u001maxDynPress_LowerSat;     /* Expression: 0.001
                                        * Referenced by: '<S121>/[0.001  maxDynPress]'
                                        */
  real_T Gain_Gain;                    /* Expression: 100
                                        * Referenced by: '<S118>/Gain'
                                        */
  real_T Constant_Value_a;             /* Expression: T
                                        * Referenced by: '<S160>/Constant'
                                        */
  real_T Constant1_Value_k5;           /* Expression: f
                                        * Referenced by: '<S160>/Constant1'
                                        */
  real32_T Vn_fil_Y0;                  /* Computed Parameter: Vn_fil_Y0
                                        * Referenced by: '<S94>/Vn_fil'
                                        */
  real32_T Ve_fil_Y0;                  /* Computed Parameter: Ve_fil_Y0
                                        * Referenced by: '<S94>/Ve_fil'
                                        */
  real32_T Constant1_Value_a[3];       /* Computed Parameter: Constant1_Value_a
                                        * Referenced by: '<S3>/Constant1'
                                        */
  real32_T Constant_Value_ij[3];       /* Computed Parameter: Constant_Value_ij
                                        * Referenced by: '<S3>/Constant'
                                        */
  real32_T Out1_Y0;                    /* Computed Parameter: Out1_Y0
                                        * Referenced by: '<S164>/Out1'
                                        */
  real32_T u0k120k_UpperSat;           /* Computed Parameter: u0k120k_UpperSat
                                        * Referenced by: '<S167>/[80k - 120k]'
                                        */
  real32_T u0k120k_LowerSat;           /* Computed Parameter: u0k120k_LowerSat
                                        * Referenced by: '<S167>/[80k - 120k]'
                                        */
  real32_T IntegerDelay_InitialCondition;/* Computed Parameter: IntegerDelay_InitialCondition
                                          * Referenced by: '<S168>/Integer Delay'
                                          */
  real32_T Constant_Value_h;           /* Computed Parameter: Constant_Value_h
                                        * Referenced by: '<S172>/Constant'
                                        */
  real32_T gains_Value;                /* Computed Parameter: gains_Value
                                        * Referenced by: '<S172>/gains'
                                        */
  real32_T MeanTemperatureforCalibration_V;/* Computed Parameter: MeanTemperatureforCalibration_V
                                            * Referenced by: '<S172>/Mean Temperature for Calibration'
                                            */
  real32_T gains_Value_c;              /* Computed Parameter: gains_Value_c
                                        * Referenced by: '<S173>/gains'
                                        */
  real32_T MeanTemperatureforCalibration_c;/* Computed Parameter: MeanTemperatureforCalibration_c
                                            * Referenced by: '<S173>/Mean Temperature for Calibration'
                                            */
  real32_T DiscreteTimeIntegrator1_gainval;/* Computed Parameter: DiscreteTimeIntegrator1_gainval
                                            * Referenced by: '<S12>/Discrete-Time Integrator1'
                                            */
  real32_T DiscreteTimeIntegrator1_IC[4];/* Computed Parameter: DiscreteTimeIntegrator1_IC
                                          * Referenced by: '<S12>/Discrete-Time Integrator1'
                                          */
  real32_T DiscreteTimeIntegrator1_UpperSa[4];/* Computed Parameter: DiscreteTimeIntegrator1_UpperSa
                                               * Referenced by: '<S12>/Discrete-Time Integrator1'
                                               */
  real32_T DiscreteTimeIntegrator1_LowerSa[4];/* Computed Parameter: DiscreteTimeIntegrator1_LowerSa
                                               * Referenced by: '<S12>/Discrete-Time Integrator1'
                                               */
  real32_T Gain_Gain_e;                /* Computed Parameter: Gain_Gain_e
                                        * Referenced by: '<S60>/Gain'
                                        */
  real32_T Gain_Gain_d;                /* Computed Parameter: Gain_Gain_d
                                        * Referenced by: '<S63>/Gain'
                                        */
  real32_T Gain_Gain_j;                /* Computed Parameter: Gain_Gain_j
                                        * Referenced by: '<S58>/Gain'
                                        */
  real32_T Gain_Gain_l;                /* Computed Parameter: Gain_Gain_l
                                        * Referenced by: '<S64>/Gain'
                                        */
  real32_T Gain_Gain_p;                /* Computed Parameter: Gain_Gain_p
                                        * Referenced by: '<S59>/Gain'
                                        */
  real32_T Gain_Gain_k;                /* Computed Parameter: Gain_Gain_k
                                        * Referenced by: '<S62>/Gain'
                                        */
  real32_T Constant1_Value_f;          /* Computed Parameter: Constant1_Value_f
                                        * Referenced by: '<S12>/Constant1'
                                        */
  real32_T ZeroBound_UpperSat;         /* Computed Parameter: ZeroBound_UpperSat
                                        * Referenced by: '<S27>/Zero Bound'
                                        */
  real32_T ZeroBound_LowerSat;         /* Computed Parameter: ZeroBound_LowerSat
                                        * Referenced by: '<S27>/Zero Bound'
                                        */
  real32_T Deg2R_Gain;                 /* Computed Parameter: Deg2R_Gain
                                        * Referenced by: '<S81>/Deg2R'
                                        */
  real32_T Reequatorialradius_Value;   /* Computed Parameter: Reequatorialradius_Value
                                        * Referenced by: '<S82>/Re=equatorial radius'
                                        */
  real32_T const_Value;                /* Computed Parameter: const_Value
                                        * Referenced by: '<S82>/const'
                                        */
  real32_T const2_Value;               /* Computed Parameter: const2_Value
                                        * Referenced by: '<S85>/const2'
                                        */
  real32_T Rppolarradius_Value;        /* Computed Parameter: Rppolarradius_Value
                                        * Referenced by: '<S85>/Rp=polar radius'
                                        */
  real32_T Reequatorialradius1_Value;  /* Computed Parameter: Reequatorialradius1_Value
                                        * Referenced by: '<S85>/Re=equatorial radius1'
                                        */
  real32_T Deg2R_Gain_c;               /* Computed Parameter: Deg2R_Gain_c
                                        * Referenced by: '<S82>/Deg2R'
                                        */
  real32_T Deg2R1_Gain;                /* Computed Parameter: Deg2R1_Gain
                                        * Referenced by: '<S82>/Deg2R1'
                                        */
  real32_T const1_Value;               /* Computed Parameter: const1_Value
                                        * Referenced by: '<S82>/const1'
                                        */
  real32_T Reequatorialradius_Value_b; /* Computed Parameter: Reequatorialradius_Value_b
                                        * Referenced by: '<S73>/Re=equatorial radius'
                                        */
  real32_T const_Value_a;              /* Computed Parameter: const_Value_a
                                        * Referenced by: '<S73>/const'
                                        */
  real32_T const2_Value_b;             /* Computed Parameter: const2_Value_b
                                        * Referenced by: '<S74>/const2'
                                        */
  real32_T Rppolarradius_Value_f;      /* Computed Parameter: Rppolarradius_Value_f
                                        * Referenced by: '<S74>/Rp=polar radius'
                                        */
  real32_T Reequatorialradius1_Value_j;/* Computed Parameter: Reequatorialradius1_Value_j
                                        * Referenced by: '<S74>/Re=equatorial radius1'
                                        */
  real32_T Deg2R_Gain_g;               /* Computed Parameter: Deg2R_Gain_g
                                        * Referenced by: '<S73>/Deg2R'
                                        */
  real32_T Deg2R1_Gain_k;              /* Computed Parameter: Deg2R1_Gain_k
                                        * Referenced by: '<S73>/Deg2R1'
                                        */
  real32_T const1_Value_i;             /* Computed Parameter: const1_Value_i
                                        * Referenced by: '<S73>/const1'
                                        */
  real32_T UEN2NEU_Gain[9];            /* Computed Parameter: UEN2NEU_Gain
                                        * Referenced by: '<S17>/UEN 2 NEU'
                                        */
  real32_T UnitConversion_Gain;        /* Computed Parameter: UnitConversion_Gain
                                        * Referenced by: '<S70>/Unit Conversion'
                                        */
  real32_T Constant_Value_hh;          /* Computed Parameter: Constant_Value_hh
                                        * Referenced by: '<S13>/Constant'
                                        */
  real32_T Gain1_Gain;                 /* Computed Parameter: Gain1_Gain
                                        * Referenced by: '<S12>/Gain1'
                                        */
  real32_T IntegerDelay_InitialCondition_c;/* Computed Parameter: IntegerDelay_InitialCondition_c
                                            * Referenced by: '<S12>/Integer Delay'
                                            */
  real32_T UD_X0;                      /* Computed Parameter: UD_X0
                                        * Referenced by: '<S78>/UD'
                                        */
  real32_T Saturation1_UpperSat;       /* Computed Parameter: Saturation1_UpperSat
                                        * Referenced by: '<S75>/Saturation1'
                                        */
  real32_T Saturation1_LowerSat;       /* Computed Parameter: Saturation1_LowerSat
                                        * Referenced by: '<S75>/Saturation1'
                                        */
  real32_T UD_X0_g;                    /* Computed Parameter: UD_X0_g
                                        * Referenced by: '<S79>/UD'
                                        */
  real32_T Saturation1_UpperSat_d;     /* Computed Parameter: Saturation1_UpperSat_d
                                        * Referenced by: '<S76>/Saturation1'
                                        */
  real32_T Saturation1_LowerSat_m;     /* Computed Parameter: Saturation1_LowerSat_m
                                        * Referenced by: '<S76>/Saturation1'
                                        */
  real32_T UD_X0_f;                    /* Computed Parameter: UD_X0_f
                                        * Referenced by: '<S80>/UD'
                                        */
  real32_T Saturation1_UpperSat_m;     /* Computed Parameter: Saturation1_UpperSat_m
                                        * Referenced by: '<S77>/Saturation1'
                                        */
  real32_T Saturation1_LowerSat_j;     /* Computed Parameter: Saturation1_LowerSat_j
                                        * Referenced by: '<S77>/Saturation1'
                                        */
  real32_T Saturation1_UpperSat_e;     /* Computed Parameter: Saturation1_UpperSat_e
                                        * Referenced by: '<S16>/Saturation1'
                                        */
  real32_T Saturation1_LowerSat_a;     /* Computed Parameter: Saturation1_LowerSat_a
                                        * Referenced by: '<S16>/Saturation1'
                                        */
  real32_T Constant_Value_jv;          /* Computed Parameter: Constant_Value_jv
                                        * Referenced by: '<S92>/Constant'
                                        */
  real32_T u11_Gain[3];                /* Computed Parameter: u11_Gain
                                        * Referenced by: '<S2>/[1 1 -1]'
                                        */
  real32_T Gain_Gain_ks;               /* Computed Parameter: Gain_Gain_ks
                                        * Referenced by: '<S87>/Gain'
                                        */
  real32_T Gains_Value;                /* Computed Parameter: Gains_Value
                                        * Referenced by: '<S151>/Gains'
                                        */
  real32_T Gains_Value_m;              /* Computed Parameter: Gains_Value_m
                                        * Referenced by: '<S157>/Gains'
                                        */
  real32_T Gains_Value_c;              /* Computed Parameter: Gains_Value_c
                                        * Referenced by: '<S158>/Gains'
                                        */
  real32_T Bias_Value_i;               /* Computed Parameter: Bias_Value_i
                                        * Referenced by: '<S158>/Bias'
                                        */
  real32_T Bias_Value_b;               /* Computed Parameter: Bias_Value_b
                                        * Referenced by: '<S157>/Bias'
                                        */
  real32_T Constant_Value_dn;          /* Computed Parameter: Constant_Value_dn
                                        * Referenced by: '<S171>/Constant'
                                        */
  real32_T Bias_Value_l;               /* Computed Parameter: Bias_Value_l
                                        * Referenced by: '<S151>/Bias'
                                        */
  real32_T Constant3_Value_n;          /* Computed Parameter: Constant3_Value_n
                                        * Referenced by: '<S163>/Constant3'
                                        */
  real32_T Constant4_Value_d;          /* Computed Parameter: Constant4_Value_d
                                        * Referenced by: '<S163>/Constant4'
                                        */
  real32_T Constant5_Value_j;          /* Computed Parameter: Constant5_Value_j
                                        * Referenced by: '<S163>/Constant5'
                                        */
  real32_T Constant2_Value_cf;         /* Computed Parameter: Constant2_Value_cf
                                        * Referenced by: '<S163>/Constant2'
                                        */
  real32_T UnitConversion_Gain_c;      /* Computed Parameter: UnitConversion_Gain_c
                                        * Referenced by: '<S169>/Unit Conversion'
                                        */
  real32_T Gain_Gain_a;                /* Computed Parameter: Gain_Gain_a
                                        * Referenced by: '<S86>/Gain'
                                        */
  real32_T UnitDelay_X0;               /* Computed Parameter: UnitDelay_X0
                                        * Referenced by: '<S91>/Unit Delay'
                                        */
  real32_T Gain_Gain_g;                /* Computed Parameter: Gain_Gain_g
                                        * Referenced by: '<S91>/Gain'
                                        */
  real32_T IntegerDelay1_InitialCondition;/* Computed Parameter: IntegerDelay1_InitialCondition
                                           * Referenced by: '<S23>/Integer Delay1'
                                           */
  real32_T Gain2_Gain;                 /* Computed Parameter: Gain2_Gain
                                        * Referenced by: '<S23>/Gain2'
                                        */
  real32_T GyroGains1_Value;           /* Computed Parameter: GyroGains1_Value
                                        * Referenced by: '<S117>/Gyro Gains1'
                                        */
  real32_T IntegerDelay1_InitialConditio_b;/* Computed Parameter: IntegerDelay1_InitialConditio_b
                                            * Referenced by: '<S93>/Integer Delay1'
                                            */
  real32_T Gain2_Gain_l;               /* Computed Parameter: Gain2_Gain_l
                                        * Referenced by: '<S93>/Gain2'
                                        */
  real32_T RateLimiter_RisingLim;      /* Computed Parameter: RateLimiter_RisingLim
                                        * Referenced by: '<S86>/Rate Limiter'
                                        */
  real32_T RateLimiter_FallingLim;     /* Computed Parameter: RateLimiter_FallingLim
                                        * Referenced by: '<S86>/Rate Limiter'
                                        */
  real32_T RateLimiter_IC;             /* Computed Parameter: RateLimiter_IC
                                        * Referenced by: '<S86>/Rate Limiter'
                                        */
  real32_T ZeroBound_UpperSat_i;       /* Computed Parameter: ZeroBound_UpperSat_i
                                        * Referenced by: '<S26>/Zero Bound'
                                        */
  real32_T ZeroBound_LowerSat_n;       /* Computed Parameter: ZeroBound_LowerSat_n
                                        * Referenced by: '<S26>/Zero Bound'
                                        */
  real32_T BiasRateLimiter_RisingLim;  /* Computed Parameter: BiasRateLimiter_RisingLim
                                        * Referenced by: '<S12>/Bias Rate Limiter'
                                        */
  real32_T BiasRateLimiter_FallingLim; /* Computed Parameter: BiasRateLimiter_FallingLim
                                        * Referenced by: '<S12>/Bias Rate Limiter'
                                        */
  real32_T BiasRateLimiter_IC;         /* Computed Parameter: BiasRateLimiter_IC
                                        * Referenced by: '<S12>/Bias Rate Limiter'
                                        */
  real32_T UnitDelay_X0_k;             /* Computed Parameter: UnitDelay_X0_k
                                        * Referenced by: '<S89>/Unit Delay'
                                        */
  real32_T Gain_Gain_b;                /* Computed Parameter: Gain_Gain_b
                                        * Referenced by: '<S89>/Gain'
                                        */
  real32_T UnitDelay_X0_l;             /* Computed Parameter: UnitDelay_X0_l
                                        * Referenced by: '<S90>/Unit Delay'
                                        */
  real32_T Gain_Gain_gt;               /* Computed Parameter: Gain_Gain_gt
                                        * Referenced by: '<S90>/Gain'
                                        */
  real32_T Gain1_Gain_a;               /* Computed Parameter: Gain1_Gain_a
                                        * Referenced by: '<S86>/Gain1'
                                        */
  real32_T GyroGains2_Value;           /* Computed Parameter: GyroGains2_Value
                                        * Referenced by: '<S117>/Gyro Gains2'
                                        */
  real32_T u11_Gain_o[3];              /* Computed Parameter: u11_Gain_o
                                        * Referenced by: '<S129>/[ -1 -1 -1]'
                                        */
  real32_T GyroGains_Value;            /* Computed Parameter: GyroGains_Value
                                        * Referenced by: '<S117>/Gyro Gains'
                                        */
  real32_T u11_Gain_j[3];              /* Computed Parameter: u11_Gain_j
                                        * Referenced by: '<S127>/[ -1 -1 -1]'
                                        */
  real32_T Gain1_Gain_b[3];            /* Computed Parameter: Gain1_Gain_b
                                        * Referenced by: '<S44>/Gain1'
                                        */
  real32_T Gain2_Gain_o[2];            /* Computed Parameter: Gain2_Gain_o
                                        * Referenced by: '<S44>/Gain2'
                                        */
  real32_T Gain3_Gain[2];              /* Computed Parameter: Gain3_Gain
                                        * Referenced by: '<S44>/Gain3'
                                        */
  real32_T Gain_Gain_o;                /* Computed Parameter: Gain_Gain_o
                                        * Referenced by: '<S12>/Gain'
                                        */
  real32_T Gains_Value_g;              /* Computed Parameter: Gains_Value_g
                                        * Referenced by: '<S152>/Gains'
                                        */
  real32_T Gains_Value_b;              /* Computed Parameter: Gains_Value_b
                                        * Referenced by: '<S153>/Gains'
                                        */
  real32_T Gains_Value_a;              /* Computed Parameter: Gains_Value_a
                                        * Referenced by: '<S154>/Gains'
                                        */
  real32_T Bias_Value_c;               /* Computed Parameter: Bias_Value_c
                                        * Referenced by: '<S154>/Bias'
                                        */
  real32_T Gains_Value_k;              /* Computed Parameter: Gains_Value_k
                                        * Referenced by: '<S155>/Gains'
                                        */
  real32_T Bias_Value_la;              /* Computed Parameter: Bias_Value_la
                                        * Referenced by: '<S155>/Bias'
                                        */
  uint32_T Constant_Value_e;           /* Computed Parameter: Constant_Value_e
                                        * Referenced by: '<S114>/Constant'
                                        */
  uint32_T Output_X0;                  /* Computed Parameter: Output_X0
                                        * Referenced by: '<S112>/Output'
                                        */
  uint32_T FixPtConstant_Value;        /* Computed Parameter: FixPtConstant_Value
                                        * Referenced by: '<S113>/FixPt Constant'
                                        */
  uint32_T FixPtSwitch_Threshold;      /* Computed Parameter: FixPtSwitch_Threshold
                                        * Referenced by: '<S114>/FixPt Switch'
                                        */
  uint16_T IntegerDelay_DelayLength;   /* Computed Parameter: IntegerDelay_DelayLength
                                        * Referenced by: '<S168>/Integer Delay'
                                        */
  uint16_T IntegerDelay_DelayLength_k; /* Computed Parameter: IntegerDelay_DelayLength_k
                                        * Referenced by: '<S12>/Integer Delay'
                                        */
  uint16_T IntegerDelay1_DelayLength;  /* Computed Parameter: IntegerDelay1_DelayLength
                                        * Referenced by: '<S23>/Integer Delay1'
                                        */
  uint16_T IntegerDelay1_DelayLength_j;/* Computed Parameter: IntegerDelay1_DelayLength_j
                                        * Referenced by: '<S93>/Integer Delay1'
                                        */
  uint16_T ConverttoMicroseconds_Gain; /* Computed Parameter: ConverttoMicroseconds_Gain
                                        * Referenced by: '<S10>/Convert to  Microseconds'
                                        */
  uint8_T Switch2_Threshold;           /* Computed Parameter: Switch2_Threshold
                                        * Referenced by: '<S119>/Switch2'
                                        */
  uint8_T Switch_Threshold;            /* Computed Parameter: Switch_Threshold
                                        * Referenced by: '<S3>/Switch'
                                        */
  uint8_T Switch6_Threshold;           /* Computed Parameter: Switch6_Threshold
                                        * Referenced by: '<S3>/Switch6'
                                        */
  uint8_T Switch4_Threshold;           /* Computed Parameter: Switch4_Threshold
                                        * Referenced by: '<S3>/Switch4'
                                        */
  uint8_T Switch1_Threshold;           /* Computed Parameter: Switch1_Threshold
                                        * Referenced by: '<S3>/Switch1'
                                        */
  uint8_T Switch5_Threshold;           /* Computed Parameter: Switch5_Threshold
                                        * Referenced by: '<S3>/Switch5'
                                        */
  uint8_T Switch2_Threshold_g;         /* Computed Parameter: Switch2_Threshold_g
                                        * Referenced by: '<S3>/Switch2'
                                        */
  uint8_T Switch3_Threshold;           /* Computed Parameter: Switch3_Threshold
                                        * Referenced by: '<S3>/Switch3'
                                        */
  boolean_T Constant_Value_n3;         /* Computed Parameter: Constant_Value_n3
                                        * Referenced by: '<Root>/Constant'
                                        */
  boolean_T Constant2_Value_e;         /* Computed Parameter: Constant2_Value_e
                                        * Referenced by: '<S178>/Constant2'
                                        */
  boolean_T Constant1_Value_kt;        /* Computed Parameter: Constant1_Value_kt
                                        * Referenced by: '<S178>/Constant1'
                                        */
  rtP_EmbeddedMATLABFunction1_sen sf_EmbeddedMATLABFunction2_d;/* '<S94>/Embedded MATLAB Function2' */
  rtP_EmbeddedMATLABFunction1_sen sf_EmbeddedMATLABFunction1_a;/* '<S94>/Embedded MATLAB Function1' */
};

/* Real-time Model Data Structure */
struct RT_MODEL_sensorMCUSlugsMKII {
  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    uint32_T clockTick0;
  } Timing;
};

/* Block parameters (auto storage) */
extern Parameters_sensorMCUSlugsMKII sensorMCUSlugsMKII_P;

/* Block signals (auto storage) */
extern BlockIO_sensorMCUSlugsMKII sensorMCUSlugsMKII_B;

/* Block states (auto storage) */
extern D_Work_sensorMCUSlugsMKII sensorMCUSlugsMKII_DWork;

/* Model entry point functions */
extern void sensorMCUSlugsMKII_initialize(boolean_T firstTime);
extern void sensorMCUSlugsMKII_step(void);

/* Real-time Model object */
extern struct RT_MODEL_sensorMCUSlugsMKII *const sensorMCUSlugsMKII_M;

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'sensorMCUSlugsMKII'
 * '<S1>'   : 'sensorMCUSlugsMKII/Control Surface Input'
 * '<S2>'   : 'sensorMCUSlugsMKII/Position and Attitude Filter'
 * '<S3>'   : 'sensorMCUSlugsMKII/Select HIL or  Live'
 * '<S4>'   : 'sensorMCUSlugsMKII/Sensor Data'
 * '<S5>'   : 'sensorMCUSlugsMKII/To Control  MCU'
 * '<S6>'   : 'sensorMCUSlugsMKII/myMux Fun1'
 * '<S7>'   : 'sensorMCUSlugsMKII/myMux Fun2'
 * '<S8>'   : 'sensorMCUSlugsMKII/myMux Fun3'
 * '<S9>'   : 'sensorMCUSlugsMKII/myMux Fun4'
 * '<S10>'  : 'sensorMCUSlugsMKII/Control Surface Input/Convert to Microseconds '
 * '<S11>'  : 'sensorMCUSlugsMKII/Control Surface Input/myMux Fun5'
 * '<S12>'  : 'sensorMCUSlugsMKII/Position and Attitude Filter/Attitude Complimentary Filter COG'
 * '<S13>'  : 'sensorMCUSlugsMKII/Position and Attitude Filter/COG.SOG2V'
 * '<S14>'  : 'sensorMCUSlugsMKII/Position and Attitude Filter/Compute GS Location'
 * '<S15>'  : 'sensorMCUSlugsMKII/Position and Attitude Filter/Enabled Subsystem'
 * '<S16>'  : 'sensorMCUSlugsMKII/Position and Attitude Filter/GPS Update'
 * '<S17>'  : 'sensorMCUSlugsMKII/Position and Attitude Filter/Geod2LTP1'
 * '<S18>'  : 'sensorMCUSlugsMKII/Position and Attitude Filter/Position Filter'
 * '<S19>'  : 'sensorMCUSlugsMKII/Position and Attitude Filter/Attitude Complimentary Filter COG/3D Filter'
 * '<S20>'  : 'sensorMCUSlugsMKII/Position and Attitude Filter/Attitude Complimentary Filter COG/3x3 Cross Product'
 * '<S21>'  : 'sensorMCUSlugsMKII/Position and Attitude Filter/Attitude Complimentary Filter COG/3x3 Cross Product1'
 * '<S22>'  : 'sensorMCUSlugsMKII/Position and Attitude Filter/Attitude Complimentary Filter COG/3x3 Cross Product2'
 * '<S23>'  : 'sensorMCUSlugsMKII/Position and Attitude Filter/Attitude Complimentary Filter COG/Deriv '
 * '<S24>'  : 'sensorMCUSlugsMKII/Position and Attitude Filter/Attitude Complimentary Filter COG/Direction Cosine Matrix to Rotation Angles'
 * '<S25>'  : 'sensorMCUSlugsMKII/Position and Attitude Filter/Attitude Complimentary Filter COG/Embedded MATLAB Function'
 * '<S26>'  : 'sensorMCUSlugsMKII/Position and Attitude Filter/Attitude Complimentary Filter COG/Normalize Vector1'
 * '<S27>'  : 'sensorMCUSlugsMKII/Position and Attitude Filter/Attitude Complimentary Filter COG/Normalize Vector2'
 * '<S28>'  : 'sensorMCUSlugsMKII/Position and Attitude Filter/Attitude Complimentary Filter COG/Quaternion Normalize'
 * '<S29>'  : 'sensorMCUSlugsMKII/Position and Attitude Filter/Attitude Complimentary Filter COG/Quaternions to  Direction Cosine Matrix'
 * '<S30>'  : 'sensorMCUSlugsMKII/Position and Attitude Filter/Attitude Complimentary Filter COG/myMux Fun2'
 * '<S31>'  : 'sensorMCUSlugsMKII/Position and Attitude Filter/Attitude Complimentary Filter COG/q dot calc'
 * '<S32>'  : 'sensorMCUSlugsMKII/Position and Attitude Filter/Attitude Complimentary Filter COG/3D Filter/Filter1'
 * '<S33>'  : 'sensorMCUSlugsMKII/Position and Attitude Filter/Attitude Complimentary Filter COG/3D Filter/Filter2'
 * '<S34>'  : 'sensorMCUSlugsMKII/Position and Attitude Filter/Attitude Complimentary Filter COG/3D Filter/Filter3'
 * '<S35>'  : 'sensorMCUSlugsMKII/Position and Attitude Filter/Attitude Complimentary Filter COG/3D Filter/myMux Fun1'
 * '<S36>'  : 'sensorMCUSlugsMKII/Position and Attitude Filter/Attitude Complimentary Filter COG/3x3 Cross Product/Subsystem'
 * '<S37>'  : 'sensorMCUSlugsMKII/Position and Attitude Filter/Attitude Complimentary Filter COG/3x3 Cross Product/Subsystem1'
 * '<S38>'  : 'sensorMCUSlugsMKII/Position and Attitude Filter/Attitude Complimentary Filter COG/3x3 Cross Product1/Subsystem'
 * '<S39>'  : 'sensorMCUSlugsMKII/Position and Attitude Filter/Attitude Complimentary Filter COG/3x3 Cross Product1/Subsystem1'
 * '<S40>'  : 'sensorMCUSlugsMKII/Position and Attitude Filter/Attitude Complimentary Filter COG/3x3 Cross Product2/Subsystem'
 * '<S41>'  : 'sensorMCUSlugsMKII/Position and Attitude Filter/Attitude Complimentary Filter COG/3x3 Cross Product2/Subsystem1'
 * '<S42>'  : 'sensorMCUSlugsMKII/Position and Attitude Filter/Attitude Complimentary Filter COG/Direction Cosine Matrix to Rotation Angles/AxisRotDefault'
 * '<S43>'  : 'sensorMCUSlugsMKII/Position and Attitude Filter/Attitude Complimentary Filter COG/Direction Cosine Matrix to Rotation Angles/AxisRotZeroR3'
 * '<S44>'  : 'sensorMCUSlugsMKII/Position and Attitude Filter/Attitude Complimentary Filter COG/Direction Cosine Matrix to Rotation Angles/Get DCM Values'
 * '<S45>'  : 'sensorMCUSlugsMKII/Position and Attitude Filter/Attitude Complimentary Filter COG/Normalize Vector1/Norm'
 * '<S46>'  : 'sensorMCUSlugsMKII/Position and Attitude Filter/Attitude Complimentary Filter COG/Normalize Vector1/Norm/Subsystem2'
 * '<S47>'  : 'sensorMCUSlugsMKII/Position and Attitude Filter/Attitude Complimentary Filter COG/Normalize Vector1/Norm/Subsystem3'
 * '<S48>'  : 'sensorMCUSlugsMKII/Position and Attitude Filter/Attitude Complimentary Filter COG/Normalize Vector1/Norm/Subsystem2/negprotect'
 * '<S49>'  : 'sensorMCUSlugsMKII/Position and Attitude Filter/Attitude Complimentary Filter COG/Normalize Vector1/Norm/Subsystem3/Embedded MATLAB Function'
 * '<S50>'  : 'sensorMCUSlugsMKII/Position and Attitude Filter/Attitude Complimentary Filter COG/Normalize Vector2/Norm'
 * '<S51>'  : 'sensorMCUSlugsMKII/Position and Attitude Filter/Attitude Complimentary Filter COG/Normalize Vector2/Norm/Subsystem2'
 * '<S52>'  : 'sensorMCUSlugsMKII/Position and Attitude Filter/Attitude Complimentary Filter COG/Normalize Vector2/Norm/Subsystem3'
 * '<S53>'  : 'sensorMCUSlugsMKII/Position and Attitude Filter/Attitude Complimentary Filter COG/Normalize Vector2/Norm/Subsystem2/negprotect'
 * '<S54>'  : 'sensorMCUSlugsMKII/Position and Attitude Filter/Attitude Complimentary Filter COG/Normalize Vector2/Norm/Subsystem3/Embedded MATLAB Function'
 * '<S55>'  : 'sensorMCUSlugsMKII/Position and Attitude Filter/Attitude Complimentary Filter COG/Quaternion Normalize/Quaternion Modulus'
 * '<S56>'  : 'sensorMCUSlugsMKII/Position and Attitude Filter/Attitude Complimentary Filter COG/Quaternion Normalize/Quaternion Modulus/Quaternion Norm'
 * '<S57>'  : 'sensorMCUSlugsMKII/Position and Attitude Filter/Attitude Complimentary Filter COG/Quaternions to  Direction Cosine Matrix/A11'
 * '<S58>'  : 'sensorMCUSlugsMKII/Position and Attitude Filter/Attitude Complimentary Filter COG/Quaternions to  Direction Cosine Matrix/A12'
 * '<S59>'  : 'sensorMCUSlugsMKII/Position and Attitude Filter/Attitude Complimentary Filter COG/Quaternions to  Direction Cosine Matrix/A13'
 * '<S60>'  : 'sensorMCUSlugsMKII/Position and Attitude Filter/Attitude Complimentary Filter COG/Quaternions to  Direction Cosine Matrix/A21'
 * '<S61>'  : 'sensorMCUSlugsMKII/Position and Attitude Filter/Attitude Complimentary Filter COG/Quaternions to  Direction Cosine Matrix/A22'
 * '<S62>'  : 'sensorMCUSlugsMKII/Position and Attitude Filter/Attitude Complimentary Filter COG/Quaternions to  Direction Cosine Matrix/A23'
 * '<S63>'  : 'sensorMCUSlugsMKII/Position and Attitude Filter/Attitude Complimentary Filter COG/Quaternions to  Direction Cosine Matrix/A31'
 * '<S64>'  : 'sensorMCUSlugsMKII/Position and Attitude Filter/Attitude Complimentary Filter COG/Quaternions to  Direction Cosine Matrix/A32'
 * '<S65>'  : 'sensorMCUSlugsMKII/Position and Attitude Filter/Attitude Complimentary Filter COG/Quaternions to  Direction Cosine Matrix/A33'
 * '<S66>'  : 'sensorMCUSlugsMKII/Position and Attitude Filter/Attitude Complimentary Filter COG/Quaternions to  Direction Cosine Matrix/Create Transformation Matrix'
 * '<S67>'  : 'sensorMCUSlugsMKII/Position and Attitude Filter/Attitude Complimentary Filter COG/Quaternions to  Direction Cosine Matrix/Quaternion Normalize'
 * '<S68>'  : 'sensorMCUSlugsMKII/Position and Attitude Filter/Attitude Complimentary Filter COG/Quaternions to  Direction Cosine Matrix/Quaternion Normalize/Quaternion Modulus'
 * '<S69>'  : 'sensorMCUSlugsMKII/Position and Attitude Filter/Attitude Complimentary Filter COG/Quaternions to  Direction Cosine Matrix/Quaternion Normalize/Quaternion Modulus/Quaternion Norm'
 * '<S70>'  : 'sensorMCUSlugsMKII/Position and Attitude Filter/COG.SOG2V/Angle Conversion'
 * '<S71>'  : 'sensorMCUSlugsMKII/Position and Attitude Filter/COG.SOG2V/Subsystem5'
 * '<S72>'  : 'sensorMCUSlugsMKII/Position and Attitude Filter/COG.SOG2V/myMux Fun2'
 * '<S73>'  : 'sensorMCUSlugsMKII/Position and Attitude Filter/Compute GS Location/Geod2ECEF1'
 * '<S74>'  : 'sensorMCUSlugsMKII/Position and Attitude Filter/Compute GS Location/Geod2ECEF1/eccentricity^2'
 * '<S75>'  : 'sensorMCUSlugsMKII/Position and Attitude Filter/GPS Update/Detect Change'
 * '<S76>'  : 'sensorMCUSlugsMKII/Position and Attitude Filter/GPS Update/Detect Change1'
 * '<S77>'  : 'sensorMCUSlugsMKII/Position and Attitude Filter/GPS Update/Detect Change2'
 * '<S78>'  : 'sensorMCUSlugsMKII/Position and Attitude Filter/GPS Update/Detect Change/Difference1'
 * '<S79>'  : 'sensorMCUSlugsMKII/Position and Attitude Filter/GPS Update/Detect Change1/Difference1'
 * '<S80>'  : 'sensorMCUSlugsMKII/Position and Attitude Filter/GPS Update/Detect Change2/Difference1'
 * '<S81>'  : 'sensorMCUSlugsMKII/Position and Attitude Filter/Geod2LTP1/ECEF2LTP (UEN)'
 * '<S82>'  : 'sensorMCUSlugsMKII/Position and Attitude Filter/Geod2LTP1/Geod2ECEF1'
 * '<S83>'  : 'sensorMCUSlugsMKII/Position and Attitude Filter/Geod2LTP1/ECEF2LTP (UEN)/From Body to Universal1'
 * '<S84>'  : 'sensorMCUSlugsMKII/Position and Attitude Filter/Geod2LTP1/ECEF2LTP (UEN)/From Body to Universal1/Rotation Matrix from Long////Lat'
 * '<S85>'  : 'sensorMCUSlugsMKII/Position and Attitude Filter/Geod2LTP1/Geod2ECEF1/eccentricity^2'
 * '<S86>'  : 'sensorMCUSlugsMKII/Position and Attitude Filter/Position Filter/BlendPosVel'
 * '<S87>'  : 'sensorMCUSlugsMKII/Position and Attitude Filter/Position Filter/BlendPosVel/2 FIlter Blend'
 * '<S88>'  : 'sensorMCUSlugsMKII/Position and Attitude Filter/Position Filter/BlendPosVel/3D Filter'
 * '<S89>'  : 'sensorMCUSlugsMKII/Position and Attitude Filter/Position Filter/BlendPosVel/CompFiltRate'
 * '<S90>'  : 'sensorMCUSlugsMKII/Position and Attitude Filter/Position Filter/BlendPosVel/CompFiltRate1'
 * '<S91>'  : 'sensorMCUSlugsMKII/Position and Attitude Filter/Position Filter/BlendPosVel/CompRateOnly'
 * '<S92>'  : 'sensorMCUSlugsMKII/Position and Attitude Filter/Position Filter/BlendPosVel/Compare To Constant'
 * '<S93>'  : 'sensorMCUSlugsMKII/Position and Attitude Filter/Position Filter/BlendPosVel/Deriv '
 * '<S94>'  : 'sensorMCUSlugsMKII/Position and Attitude Filter/Position Filter/BlendPosVel/Subsystem'
 * '<S95>'  : 'sensorMCUSlugsMKII/Position and Attitude Filter/Position Filter/BlendPosVel/myMux Fun1'
 * '<S96>'  : 'sensorMCUSlugsMKII/Position and Attitude Filter/Position Filter/BlendPosVel/myMux Fun2'
 * '<S97>'  : 'sensorMCUSlugsMKII/Position and Attitude Filter/Position Filter/BlendPosVel/2 FIlter Blend/Embedded MATLAB Function3'
 * '<S98>'  : 'sensorMCUSlugsMKII/Position and Attitude Filter/Position Filter/BlendPosVel/2 FIlter Blend/Tustin Lowpass,  Auto Initial Condition4'
 * '<S99>'  : 'sensorMCUSlugsMKII/Position and Attitude Filter/Position Filter/BlendPosVel/2 FIlter Blend/Tustin Lowpass,  Auto Initial Condition4/Embedded MATLAB Function'
 * '<S100>' : 'sensorMCUSlugsMKII/Position and Attitude Filter/Position Filter/BlendPosVel/3D Filter/Filter1'
 * '<S101>' : 'sensorMCUSlugsMKII/Position and Attitude Filter/Position Filter/BlendPosVel/3D Filter/Filter2'
 * '<S102>' : 'sensorMCUSlugsMKII/Position and Attitude Filter/Position Filter/BlendPosVel/3D Filter/Filter3'
 * '<S103>' : 'sensorMCUSlugsMKII/Position and Attitude Filter/Position Filter/BlendPosVel/3D Filter/myMux Fun1'
 * '<S104>' : 'sensorMCUSlugsMKII/Position and Attitude Filter/Position Filter/BlendPosVel/CompFiltRate/1D Filter1'
 * '<S105>' : 'sensorMCUSlugsMKII/Position and Attitude Filter/Position Filter/BlendPosVel/CompFiltRate/1D Filter1/Filter2'
 * '<S106>' : 'sensorMCUSlugsMKII/Position and Attitude Filter/Position Filter/BlendPosVel/CompFiltRate1/1D Filter1'
 * '<S107>' : 'sensorMCUSlugsMKII/Position and Attitude Filter/Position Filter/BlendPosVel/CompFiltRate1/1D Filter1/Filter2'
 * '<S108>' : 'sensorMCUSlugsMKII/Position and Attitude Filter/Position Filter/BlendPosVel/CompRateOnly/1D Filter1'
 * '<S109>' : 'sensorMCUSlugsMKII/Position and Attitude Filter/Position Filter/BlendPosVel/CompRateOnly/1D Filter1/Filter2'
 * '<S110>' : 'sensorMCUSlugsMKII/Position and Attitude Filter/Position Filter/BlendPosVel/Subsystem/Embedded MATLAB Function1'
 * '<S111>' : 'sensorMCUSlugsMKII/Position and Attitude Filter/Position Filter/BlendPosVel/Subsystem/Embedded MATLAB Function2'
 * '<S112>' : 'sensorMCUSlugsMKII/Select HIL or  Live/TimeStamp'
 * '<S113>' : 'sensorMCUSlugsMKII/Select HIL or  Live/TimeStamp/Increment Real World'
 * '<S114>' : 'sensorMCUSlugsMKII/Select HIL or  Live/TimeStamp/Wrap To Zero'
 * '<S115>' : 'sensorMCUSlugsMKII/Sensor Data/Raw HIL  Readings'
 * '<S116>' : 'sensorMCUSlugsMKII/Sensor Data/Sensor Suite'
 * '<S117>' : 'sensorMCUSlugsMKII/Sensor Data/Sensor Suite/ADIS16405 Signal Cond '
 * '<S118>' : 'sensorMCUSlugsMKII/Sensor Data/Sensor Suite/Compute CPU Load'
 * '<S119>' : 'sensorMCUSlugsMKII/Sensor Data/Sensor Suite/If no HIL then Read all the Sensors'
 * '<S120>' : 'sensorMCUSlugsMKII/Sensor Data/Sensor Suite/If no HIL then update Air Data'
 * '<S121>' : 'sensorMCUSlugsMKII/Sensor Data/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration'
 * '<S122>' : 'sensorMCUSlugsMKII/Sensor Data/Sensor Suite/myMux Fun'
 * '<S123>' : 'sensorMCUSlugsMKII/Sensor Data/Sensor Suite/myMux Fun1'
 * '<S124>' : 'sensorMCUSlugsMKII/Sensor Data/Sensor Suite/ADIS16405 Signal Cond /3D Tustin Lowpass, Auto Initial Condition'
 * '<S125>' : 'sensorMCUSlugsMKII/Sensor Data/Sensor Suite/ADIS16405 Signal Cond /3D Tustin Lowpass, Auto Initial Condition1'
 * '<S126>' : 'sensorMCUSlugsMKII/Sensor Data/Sensor Suite/ADIS16405 Signal Cond /3D Tustin Lowpass, Auto Initial Condition2'
 * '<S127>' : 'sensorMCUSlugsMKII/Sensor Data/Sensor Suite/ADIS16405 Signal Cond /Rotate Cube Axis'
 * '<S128>' : 'sensorMCUSlugsMKII/Sensor Data/Sensor Suite/ADIS16405 Signal Cond /Rotate Cube Axis '
 * '<S129>' : 'sensorMCUSlugsMKII/Sensor Data/Sensor Suite/ADIS16405 Signal Cond /Rotate Cube Axis  '
 * '<S130>' : 'sensorMCUSlugsMKII/Sensor Data/Sensor Suite/ADIS16405 Signal Cond /3D Tustin Lowpass, Auto Initial Condition/Embedded MATLAB Function'
 * '<S131>' : 'sensorMCUSlugsMKII/Sensor Data/Sensor Suite/ADIS16405 Signal Cond /3D Tustin Lowpass, Auto Initial Condition/Embedded MATLAB Function1'
 * '<S132>' : 'sensorMCUSlugsMKII/Sensor Data/Sensor Suite/ADIS16405 Signal Cond /3D Tustin Lowpass, Auto Initial Condition/Embedded MATLAB Function2'
 * '<S133>' : 'sensorMCUSlugsMKII/Sensor Data/Sensor Suite/ADIS16405 Signal Cond /3D Tustin Lowpass, Auto Initial Condition/myMux Fun'
 * '<S134>' : 'sensorMCUSlugsMKII/Sensor Data/Sensor Suite/ADIS16405 Signal Cond /3D Tustin Lowpass, Auto Initial Condition1/Embedded MATLAB Function'
 * '<S135>' : 'sensorMCUSlugsMKII/Sensor Data/Sensor Suite/ADIS16405 Signal Cond /3D Tustin Lowpass, Auto Initial Condition1/Embedded MATLAB Function1'
 * '<S136>' : 'sensorMCUSlugsMKII/Sensor Data/Sensor Suite/ADIS16405 Signal Cond /3D Tustin Lowpass, Auto Initial Condition1/Embedded MATLAB Function2'
 * '<S137>' : 'sensorMCUSlugsMKII/Sensor Data/Sensor Suite/ADIS16405 Signal Cond /3D Tustin Lowpass, Auto Initial Condition1/myMux Fun'
 * '<S138>' : 'sensorMCUSlugsMKII/Sensor Data/Sensor Suite/ADIS16405 Signal Cond /3D Tustin Lowpass, Auto Initial Condition2/Embedded MATLAB Function'
 * '<S139>' : 'sensorMCUSlugsMKII/Sensor Data/Sensor Suite/ADIS16405 Signal Cond /3D Tustin Lowpass, Auto Initial Condition2/Embedded MATLAB Function1'
 * '<S140>' : 'sensorMCUSlugsMKII/Sensor Data/Sensor Suite/ADIS16405 Signal Cond /3D Tustin Lowpass, Auto Initial Condition2/Embedded MATLAB Function2'
 * '<S141>' : 'sensorMCUSlugsMKII/Sensor Data/Sensor Suite/ADIS16405 Signal Cond /3D Tustin Lowpass, Auto Initial Condition2/myMux Fun'
 * '<S142>' : 'sensorMCUSlugsMKII/Sensor Data/Sensor Suite/If no HIL then Read all the Sensors/Embedded MATLAB Function'
 * '<S143>' : 'sensorMCUSlugsMKII/Sensor Data/Sensor Suite/If no HIL then Read all the Sensors/Embedded MATLAB Function1'
 * '<S144>' : 'sensorMCUSlugsMKII/Sensor Data/Sensor Suite/If no HIL then Read all the Sensors/Embedded MATLAB Function2'
 * '<S145>' : 'sensorMCUSlugsMKII/Sensor Data/Sensor Suite/If no HIL then Read all the Sensors/Embedded MATLAB Function3'
 * '<S146>' : 'sensorMCUSlugsMKII/Sensor Data/Sensor Suite/If no HIL then Read all the Sensors/if GPS is Novatel'
 * '<S147>' : 'sensorMCUSlugsMKII/Sensor Data/Sensor Suite/If no HIL then Read all the Sensors/if GPS is Ublox'
 * '<S148>' : 'sensorMCUSlugsMKII/Sensor Data/Sensor Suite/If no HIL then Read all the Sensors/myMux Fun'
 * '<S149>' : 'sensorMCUSlugsMKII/Sensor Data/Sensor Suite/If no HIL then Read all the Sensors/myMux Fun4'
 * '<S150>' : 'sensorMCUSlugsMKII/Sensor Data/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Baro Altimeter'
 * '<S151>' : 'sensorMCUSlugsMKII/Sensor Data/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Barometer Unit Conversion'
 * '<S152>' : 'sensorMCUSlugsMKII/Sensor Data/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Pitot Apply Calibration'
 * '<S153>' : 'sensorMCUSlugsMKII/Sensor Data/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Pitot Unit Conversion (psi to pa)'
 * '<S154>' : 'sensorMCUSlugsMKII/Sensor Data/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Pitot Unit Conversion (psi)'
 * '<S155>' : 'sensorMCUSlugsMKII/Sensor Data/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Power Unit Conversion'
 * '<S156>' : 'sensorMCUSlugsMKII/Sensor Data/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Temp Compensate Baro'
 * '<S157>' : 'sensorMCUSlugsMKII/Sensor Data/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Temperature Apply Calibration'
 * '<S158>' : 'sensorMCUSlugsMKII/Sensor Data/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Temperature Unit Conversion'
 * '<S159>' : 'sensorMCUSlugsMKII/Sensor Data/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Tustin Lowpass, Auto Initial Condition1'
 * '<S160>' : 'sensorMCUSlugsMKII/Sensor Data/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Tustin Lowpass, Auto Initial Condition2'
 * '<S161>' : 'sensorMCUSlugsMKII/Sensor Data/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Tustin Lowpass, Auto Initial Condition3'
 * '<S162>' : 'sensorMCUSlugsMKII/Sensor Data/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Tustin Lowpass, Auto Initial Condition4'
 * '<S163>' : 'sensorMCUSlugsMKII/Sensor Data/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Baro Altimeter/Compute Barometric Height'
 * '<S164>' : 'sensorMCUSlugsMKII/Sensor Data/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Baro Altimeter/Enabled Subsystem'
 * '<S165>' : 'sensorMCUSlugsMKII/Sensor Data/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Baro Altimeter/Enables//Disables the Computation of  initial Baro Bias'
 * '<S166>' : 'sensorMCUSlugsMKII/Sensor Data/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Baro Altimeter/GS Height'
 * '<S167>' : 'sensorMCUSlugsMKII/Sensor Data/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Baro Altimeter/Initial Baro Bias'
 * '<S168>' : 'sensorMCUSlugsMKII/Sensor Data/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Baro Altimeter/Zero Out Height'
 * '<S169>' : 'sensorMCUSlugsMKII/Sensor Data/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Baro Altimeter/Compute Barometric Height/Length Conversion'
 * '<S170>' : 'sensorMCUSlugsMKII/Sensor Data/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Baro Altimeter/Initial Baro Bias/Low Pass8'
 * '<S171>' : 'sensorMCUSlugsMKII/Sensor Data/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Temp Compensate Baro/Compare To Constant'
 * '<S172>' : 'sensorMCUSlugsMKII/Sensor Data/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Temp Compensate Baro/Hi Temp Compensation'
 * '<S173>' : 'sensorMCUSlugsMKII/Sensor Data/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Temp Compensate Baro/Lo Temp Compensation'
 * '<S174>' : 'sensorMCUSlugsMKII/Sensor Data/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Tustin Lowpass, Auto Initial Condition1/Embedded MATLAB Function'
 * '<S175>' : 'sensorMCUSlugsMKII/Sensor Data/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Tustin Lowpass, Auto Initial Condition2/Embedded MATLAB Function'
 * '<S176>' : 'sensorMCUSlugsMKII/Sensor Data/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Tustin Lowpass, Auto Initial Condition3/Embedded MATLAB Function'
 * '<S177>' : 'sensorMCUSlugsMKII/Sensor Data/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Tustin Lowpass, Auto Initial Condition4/Embedded MATLAB Function'
 * '<S178>' : 'sensorMCUSlugsMKII/To Control  MCU/IPC Data'
 */
#endif                                 /* RTW_HEADER_sensorMCUSlugsMKII_h_ */

/* File trailer for Real-Time Workshop generated code.
 *
 * [EOF]
 */
