/*
 * File: controlMCUSlugsMKIINewNav.h
 *
 * Real-Time Workshop code generated for Simulink model controlMCUSlugsMKIINewNav.
 *
 * Model version                        : 1.315
 * Real-Time Workshop file version      : 8.1 (R2011b) 08-Jul-2011
 * Real-Time Workshop file generated on : Wed Jan 21 12:06:26 2015
 * TLC version                          : 8.1 (Jul  9 2011)
 * C source code generated on           : Wed Jan 21 12:06:28 2015
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

#ifndef RTW_HEADER_controlMCUSlugsMKIINewNav_h_
#define RTW_HEADER_controlMCUSlugsMKIINewNav_h_
#ifndef controlMCUSlugsMKIINewNav_COMMON_INCLUDES_
# define controlMCUSlugsMKIINewNav_COMMON_INCLUDES_
#include <math.h>
#include <string.h>
#include "rtwtypes.h"
#include "rt_nonfinite.h"
#include "rtGetInf.h"
#include "rtGetNaN.h"
#include "rt_defines.h"
#endif                                 /* controlMCUSlugsMKIINewNav_COMMON_INCLUDES_ */

#include "controlMCUSlugsMKIINewNav_types.h"

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

/* Block signals for system '<S51>/Embedded MATLAB Function' */
typedef struct {
  real32_T y;                          /* '<S51>/Embedded MATLAB Function' */
} rtB_EmbeddedMATLABFunction_cont;

/* Block states (auto storage) for system '<S51>/Embedded MATLAB Function' */
typedef struct {
  real_T a;                            /* '<S51>/Embedded MATLAB Function' */
  real_T b;                            /* '<S51>/Embedded MATLAB Function' */
  real32_T y_km1;                      /* '<S51>/Embedded MATLAB Function' */
  real32_T u_km1;                      /* '<S51>/Embedded MATLAB Function' */
  boolean_T a_not_empty;               /* '<S51>/Embedded MATLAB Function' */
} rtDW_EmbeddedMATLABFunction_con;

/* Block signals for system '<S16>/myMux Fun1' */
typedef struct {
  real32_T y[3];                       /* '<S16>/myMux Fun1' */
} rtB_myMuxFun1_controlMCUSlugsMK;

/* Block signals for system '<S122>/negprotect' */
typedef struct {
  real32_T zpVal;                      /* '<S122>/negprotect' */
} rtB_negprotect_controlMCUSlugsM;

/* Block signals for system '<S400>/Embedded MATLAB Function' */
typedef struct {
  real32_T xDoty;                      /* '<S400>/Embedded MATLAB Function' */
} rtB_EmbeddedMATLABFunction_co_h;

/* Block signals for system '<S136>/Zero out Z1' */
typedef struct {
  real32_T P[3];                       /* '<S136>/Zero out Z1' */
} rtB_ZerooutZ1_controlMCUSlugsMK;

/* Block signals for system '<S264>/Embedded MATLAB Function' */
typedef struct {
  real32_T xDoty;                      /* '<S264>/Embedded MATLAB Function' */
} rtB_EmbeddedMATLABFunction_co_k;

/* Block signals for system '<S272>/Select N  Terms' */
typedef struct {
  real32_T N[3];                       /* '<S272>/Select N  Terms' */
} rtB_SelectNTerms_controlMCUSlug;

/* Block signals for system '<S160>/negprotect3' */
typedef struct {
  real32_T zpVal;                      /* '<S160>/negprotect3' */
} rtB_negprotect3_controlMCUSlugs;

/* Block signals for system '<S415>/Buffer IC Channel' */
typedef struct {
  uint16_T history[7];                 /* '<S415>/Buffer IC Channel' */
} rtB_BufferICChannel_controlMCUS;

/* Block states (auto storage) for system '<S415>/Buffer IC Channel' */
typedef struct {
  uint16_T oldValues[7];               /* '<S415>/Buffer IC Channel' */
} rtDW_BufferICChannel_controlMCU;

/* Block signals (auto storage) */
typedef struct {
  real_T Add;                          /* '<S459>/Add' */
  real32_T GetMidLevelCommandsnavSupportc[3];/* '<Root>/Get MidLevel Commands [navSupport.c]' */
  real32_T GetDynamicPnavSupportc;     /* '<Root>/Get Dynamic P [navSupport.c]' */
  real32_T GetRangeofValuesnavSupportc[3];/* '<S4>/Get Range of Values [navSupport.c]' */
  real32_T GetXYZnavSupportc[3];       /* '<Root>/Get XYZ [navSupport.c]' */
  real32_T GetVnednavSupportc[3];      /* '<Root>/Get Vned [navSupport.c]' */
  real32_T GetRangeofValuesnavSupportc_k[3];/* '<Root>/Get Range of Values [navSupport.c]' */
  real32_T GetMobileLocationnavSupportc[2];/* '<Root>/Get Mobile Location [navSupport.c] ' */
  real32_T GetRangeofValuesnavSupportc_c[3];/* '<Root>/Get Range of Values [navSupport.c]   ' */
  real32_T GetRangeofValuesnavSupportc_g[2];/* '<S4>/Get Range of Values [navSupport.c] ' */
  real32_T GetasingleParamnavSupportc; /* '<S4>/Get a single Param [navSupport.c]' */
  real32_T GetRangeofValuesnavSupportc_m[3];/* '<S10>/Get Range of Values [navSupport.c]' */
  real32_T GetAttitudenavSupportc[6];  /* '<S10>/Get Attitude [navSupport.c]' */
  real32_T Product1[3];                /* '<S10>/Product1' */
  real32_T GetRangeofValuesnavSupportc_md[3];/* '<S4>/Get Range of Values [navSupport.c]   ' */
  real32_T GetasingleParamnavSupportc1;/* '<S4>/Get a single Param [navSupport.c]1' */
  real32_T GetRangeofValuesnavSupportc_d[3];/* '<S4>/Get Range of Values [navSupport.c]  ' */
  real32_T GetRangeofValuesnavSupportc_h[3];/* '<S4>/Get Range of Values [navSupport.c]     ' */
  real32_T apUtilsc;                   /* '<S21>/[apUtils.c]' */
  real32_T CFunctionCall;              /* '<S22>/C Function Call' */
  real32_T GetRangeofValuesnavSupportc_n[4];/* '<Root>/Get Range of Values [navSupport.c] ' */
  real32_T GettheAccelerometersnavSupportc[3];/* '<S10>/Get the Accelerometers [navSupport.c]' */
  real32_T GettheAccelBiasnavSupportc[3];/* '<S10>/Get the Accel Bias [navSupport.c]' */
  real32_T GetRangeofValuesnavSupportc_d3[2];/* '<S1>/Get Range of Values [navSupport.c]     ' */
  real32_T y[6];                       /* '<S10>/myMux Fun2' */
  real32_T y_i[3];                     /* '<S10>/myMux Fun1' */
  real32_T CFunctionCall_o;            /* '<S501>/C Function Call' */
  real32_T Product2[3];                /* '<S460>/Product2' */
  real32_T u1;                         /* '<S480>/[-1,1]' */
  real32_T CFunctionCall_oo;           /* '<S480>/C Function Call' */
  real32_T CFunctionCall_m;            /* '<S481>/C Function Call' */
  real32_T DataTypeConversion_j;       /* '<S436>/Data Type Conversion' */
  real32_T DataTypeConversion_je;      /* '<S437>/Data Type Conversion' */
  real32_T DataTypeConversion_k;       /* '<S438>/Data Type Conversion' */
  real32_T DataTypeConversion_h;       /* '<S439>/Data Type Conversion' */
  real32_T GetISRLocationnavSupportc[3];/* '<S18>/Get ISR Location [navSupport.c]' */
  real32_T IC3;                        /* '<S137>/IC3' */
  real32_T IC4[2];                     /* '<S137>/IC4' */
  real32_T CFunctionCall_a;            /* '<S212>/C Function Call' */
  real32_T Merge[3];                   /* '<S128>/Merge' */
  real32_T CFunctionCall_p;            /* '<S231>/C Function Call' */
  real32_T Switch;                     /* '<S217>/Switch' */
  real32_T Switch2;                    /* '<S217>/Switch2' */
  real32_T CFunctionCall_k;            /* '<S221>/C Function Call' */
  real32_T Merge2;                     /* '<S128>/Merge2' */
  real32_T CFunctionCall_i;            /* '<S220>/C Function Call' */
  real32_T Product;                    /* '<S219>/Product' */
  real32_T CFunctionCall_pf;           /* '<S237>/C Function Call' */
  real32_T Merge1;                     /* '<S128>/Merge1' */
  real32_T DataTypeConversion9[3];     /* '<S128>/Data Type Conversion9' */
  real32_T y_e[3];                     /* '<S128>/myMux Fun5' */
  real32_T y_k[4];                     /* '<S128>/myMux Fun1' */
  real32_T GettheGSLocationupdateControlMC[3];/* '<S406>/Get the GS Location [updateControlMCUState.c]' */
  real32_T NumericalUnity[3];          /* '<S409>/Numerical Unity' */
  real32_T DataTypeConversion_ks[2];   /* '<S406>/Data Type Conversion' */
  real32_T DataTypeConversion1[3];     /* '<S406>/Data Type Conversion1' */
  real32_T UEN2NEU[3];                 /* '<S152>/UEN 2 NEU' */
  real32_T CFunctionCall_mk;           /* '<S199>/C Function Call' */
  real32_T Sum2;                       /* '<S129>/Sum2' */
  real32_T CFunctionCall_ox;           /* '<S151>/C Function Call' */
  real32_T CFunctionCall_p5;           /* '<S165>/C Function Call' */
  real32_T CFunctionCall_op;           /* '<S173>/C Function Call' */
  real32_T u1_h;                       /* '<S160>/[-1 1]' */
  real32_T CFunctionCall_l;            /* '<S170>/C Function Call' */
  real32_T CFunctionCall_aw;           /* '<S186>/C Function Call' */
  real32_T u1_m;                       /* '<S161>/[-1 1]' */
  real32_T CFunctionCall_j;            /* '<S183>/C Function Call' */
  real32_T CFunctionCall_pg;           /* '<S164>/C Function Call' */
  real32_T CFunctionCall_n;            /* '<S163>/C Function Call' */
  real32_T CFunctionCall_b;            /* '<S244>/C Function Call' */
  real32_T CFunctionCall_f5;           /* '<S385>/C Function Call' */
  real32_T CFunctionCall_bg;           /* '<S258>/C Function Call' */
  real32_T CFunctionCall_b2;           /* '<S259>/C Function Call' */
  real32_T GetWaypointCoordinatesnavSuppor[3];/* '<S331>/Get Waypoint Coordinates [navSupport.c]' */
  real32_T GetWaypointCoordinatesnavSupp_j[3];/* '<S332>/Get Waypoint Coordinates [navSupport.c]' */
  real32_T CFunctionCall_kc;           /* '<S338>/C Function Call' */
  real32_T CFunctionCall_mu;           /* '<S345>/C Function Call' */
  real32_T WP0L2IPT1[3];               /* '<S252>/Subtract' */
  real32_T GetWaypointCoordinatesnavSupp_n[3];/* '<S276>/Get Waypoint Coordinates [navSupport.c]' */
  real32_T GetWaypointCoordinatesnavSupp_c[3];/* '<S277>/Get Waypoint Coordinates [navSupport.c]' */
  real32_T UEN2NEU_l[3];               /* '<S313>/UEN 2 NEU' */
  real32_T CFunctionCall_av;           /* '<S283>/C Function Call' */
  real32_T CFunctionCall_ml;           /* '<S290>/C Function Call' */
  real32_T Product1_e;                 /* '<S251>/Product1' */
  real32_T CFunctionCall_fw;           /* '<S401>/C Function Call' */
  real32_T Reshape1[3];                /* '<S394>/Reshape1' */
  real32_T Add_m;                      /* '<S114>/Add' */
  real32_T CFunctionCall_nt;           /* '<S120>/C Function Call' */
  real32_T CFunctionCall_lc;           /* '<S122>/C Function Call' */
  real32_T Switch3_o;                  /* '<S116>/Switch3' */
  real32_T Switch3_bm;                 /* '<S97>/Switch3' */
  real32_T u060;                       /* '<S87>/[-60 60]' */
  real32_T apUtilsc_b;                 /* '<S99>/[apUtils.c]' */
  real32_T bankLimit;                  /* '<S82>/bank Limit' */
  real32_T CFunctionCall_ao;           /* '<S85>/C Function Call' */
  real32_T Divide;                     /* '<S58>/Divide' */
  real32_T CFunctionCall_kw;           /* '<S80>/C Function Call' */
  real32_T NegFeedback;                /* '<S48>/Neg Feedback ' */
  real32_T bankLimit_o;                /* '<S64>/bank Limit' */
  real32_T CFunctionCall_mlv;          /* '<S68>/C Function Call' */
  real32_T Subtract;                   /* '<S65>/Subtract' */
  real32_T Divide_b;                   /* '<S67>/Divide' */
  real32_T CFunctionCall_h;            /* '<S74>/C Function Call' */
  real32_T c;                          /* '<S71>/Math Function' */
  real32_T c_d;                        /* '<S71>/1-c' */
  real32_T Subtract_g;                 /* '<S25>/Subtract' */
  real32_T Switch3_ob;                 /* '<S27>/Switch3' */
  real32_T Subtract_f;                 /* '<S29>/Subtract' */
  real32_T Merge_e;                    /* '<S26>/Merge' */
  real32_T PsiDotLimit;                /* '<S15>/Psi Dot  Limit' */
  real32_T Switch3_a3;                 /* '<S28>/Switch3' */
  real32_T Switch3_fi;                 /* '<S46>/Switch3' */
  real32_T Switch3_ik;                 /* '<S47>/Switch3' */
  real32_T T;                          /* '<S41>/-T' */
  real32_T CFunctionCall_d;            /* '<S43>/C Function Call' */
  real32_T NumericalUnity_f;           /* '<S44>/Numerical Unity' */
  real32_T c_i;                        /* '<S41>/1-c' */
  real32_T T_m;                        /* '<S31>/-T' */
  real32_T CFunctionCall_ai;           /* '<S33>/C Function Call' */
  real32_T NumericalUnity_p;           /* '<S34>/Numerical Unity' */
  real32_T c_k;                        /* '<S31>/1-c' */
  int16_T y_o[3];                      /* '<S443>/myMux Fun1' */
  int16_T GettheAidePanFCBEX1000c;     /* '<S460>/Get the Aide Pan [FCBEX1000c]' */
  int16_T GettheAideTiltFCBEX1000c;    /* '<S460>/Get the Aide Tilt [FCBEX1000.c]' */
  int16_T y_eh[3];                     /* '<S128>/myMux Fun4' */
  uint16_T InputCapture_o1;            /* '<S416>/Input Capture' */
  uint16_T InputCapture_o2;            /* '<S416>/Input Capture' */
  uint16_T InputCapture_o3;            /* '<S416>/Input Capture' */
  uint16_T InputCapture_o4;            /* '<S416>/Input Capture' */
  uint16_T InputCapture_o5;            /* '<S416>/Input Capture' */
  uint16_T InputCapture_o6;            /* '<S416>/Input Capture' */
  uint16_T InputCapture_o7;            /* '<S416>/Input Capture' */
  uint16_T ChoosetheMediannavSupportc; /* '<S420>/Choose the Median [navSupport.c]' */
  uint16_T CalculusTimeStep_o1;        /* '<S13>/Calculus Time Step' */
  uint16_T CalculusTimeStep_o2;        /* '<S13>/Calculus Time Step' */
  uint16_T Merge_f[4];                 /* '<S442>/Merge' */
  uint16_T DataTypeConversion_jet[4];  /* '<S441>/Data Type Conversion' */
  uint16_T Switch2_b;                  /* '<S443>/Switch2' */
  uint16_T Switch3_nv5;                /* '<S443>/Switch3' */
  uint16_T Switch1_l;                  /* '<S443>/Switch1' */
  uint16_T Sum;                        /* '<S460>/Sum' */
  uint16_T Sum1;                       /* '<S460>/Sum1' */
  uint16_T ZeroRoll;                   /* '<S460>/Zero Roll' */
  uint16_T GetthePanFCBEX1000c1;       /* '<S459>/Get the Pan [FCBEX1000.c]1' */
  uint16_T GetthePanFCBEX1000c;        /* '<S459>/Get the Pan [FCBEX1000.c]' */
  uint16_T Switch1_e;                  /* '<S459>/Switch1' */
  uint16_T Switch_n;                   /* '<S465>/Switch' */
  uint16_T ChoosetheMediannavSupportc_c;/* '<S419>/Choose the Median [navSupport.c]' */
  uint16_T DataTypeConversion_i;       /* '<S432>/Data Type Conversion' */
  uint16_T ChoosetheMediannavSupportc1;/* '<S419>/Choose the Median [navSupport.c]1' */
  uint16_T DataTypeConversion_f;       /* '<S433>/Data Type Conversion' */
  uint16_T ChoosetheMediannavSupportc2;/* '<S419>/Choose the Median [navSupport.c]2' */
  uint16_T DataTypeConversion_n;       /* '<S434>/Data Type Conversion' */
  uint16_T ChoosetheMediannavSupportc3;/* '<S419>/Choose the Median [navSupport.c]3' */
  uint16_T DataTypeConversion_iq;      /* '<S435>/Data Type Conversion' */
  uint16_T y_l[2];                     /* '<S416>/myMux Fun1' */
  uint8_T NavigationModenavSupportc;   /* '<S9>/Navigation Mode [navSupport.c]' */
  uint8_T ManualorAutonavSupportc;     /* '<S7>/Manual or Auto? [navSupport.c]' */
  uint8_T DataTypeConversion12;        /* '<S13>/Data Type Conversion12' */
  uint8_T DatafromGSmavlinkCommsControlMc[202];/* '<S2>/Data from GS [mavlinkCommsControlMcu.c]' */
  uint8_T DatafromIPCinterProcCommSlavec[202];/* '<S2>/Data from IPC [interProcCommSlave.c]' */
  uint8_T DataTypeConversion1_o;       /* '<S431>/Data Type Conversion1' */
  uint8_T DetectRisingTransitionnavSuppor;/* '<S431>/Detect Rising Transition [navSupport.c]' */
  uint8_T isitinMidLevelPtorSPTnavSupport;/* '<Root>/is it in Mid Level, Pt or SPT? [navSupport.c]1' */
  uint8_T GetMaxWPnavSupportc;         /* '<Root>/Get Max WP [navSupport.c]' */
  uint8_T wpFlynavSupportc;            /* '<Root>/wp Fly? [navSupport.c]' */
  uint8_T GetRTBOrdernavSupportc[2];   /* '<Root>/Get RTB Order [navSupport.c]' */
  uint8_T CreateTelemetrySentencemavlinkC[120];/* '<S9>/Create Telemetry  Sentence [mavlinkCommsControlMcu.c]' */
  uint8_T ISROption1forCameraTrackingnavS;/* '<S9>/ISR Option 1 for Camera Tracking [navSupport.c]' */
  uint8_T GettheZoomFCBEX1000c;        /* '<S443>/Get the Zoom [FCBEX1000.c]' */
  uint8_T TurnonLightsupdateControlMCUSta;/* '<S9>/Turn on Lights? [updateControlMCUState.c]' */
  uint8_T VisibleorIRupdateControlMCUStat;/* '<S9>/Visible or IR? [updateControlMCUState.c]' */
  uint8_T getifinHILornotupdateControlMcu;/* '<S6>/get if in HIL or not [updateControlMcuStatec]' */
  uint8_T isitinMidLevelPtorSPTnavSuppo_n;/* '<Root>/is it in Mid Level, Pt or SPT? [navSupport.c]' */
  uint8_T SetAidestoZeroFCBEX1000c;    /* '<S460>/Set Aides to Zero? [FCBEX1000.c]' */
  uint8_T SetCameratoHomePositionFCBEX100;/* '<S459>/Set Camera to Home Position? [FCBEX1000.c]1' */
  uint8_T GetPassSwitchesnavSupportc[4];/* '<S446>/Get Pass Switches [navSupport.c]' */
  uint8_T NavigationModenavSupportc_c; /* '<S18>/Navigation Mode [navSupport.c]' */
  uint8_T IC;                          /* '<S135>/IC' */
  uint8_T WP0;                         /* '<S135>/computeCurrentWP' */
  uint8_T WP1;                         /* '<S135>/computeCurrentWP' */
  boolean_T DataTypeConversion_l;      /* '<S9>/Data Type Conversion' */
  boolean_T DataTypeConversion1_f;     /* '<S9>/Data Type Conversion1' */
  boolean_T DataTypeConversion1_a;     /* '<S6>/Data Type Conversion1' */
  rtB_negprotect_controlMCUSlugsM sf_negprotect_j;/* '<S501>/negprotect' */
  rtB_EmbeddedMATLABFunction_co_h sf_EmbeddedMATLABFunction_lb;/* '<S500>/Embedded MATLAB Function' */
  rtB_myMuxFun1_controlMCUSlugsMK sf_myMuxFun5_c;/* '<S477>/myMux Fun5' */
  rtB_EmbeddedMATLABFunction_cont sf_EmbeddedMATLABFunction_ab;/* '<S485>/Embedded MATLAB Function' */
  rtB_EmbeddedMATLABFunction_cont sf_EmbeddedMATLABFunction_e4;/* '<S484>/Embedded MATLAB Function' */
  rtB_EmbeddedMATLABFunction_cont sf_EmbeddedMATLABFunction_px;/* '<S483>/Embedded MATLAB Function' */
  rtB_EmbeddedMATLABFunction_cont sf_EmbeddedMATLABFunction_dk;/* '<S466>/Embedded MATLAB Function' */
  rtB_EmbeddedMATLABFunction_cont sf_EmbeddedMATLABFunction_lo;/* '<S470>/Embedded MATLAB Function' */
  rtB_BufferICChannel_controlMCUS sf_BufferFailsafeChannel;/* '<S420>/Buffer Failsafe Channel' */
  rtB_EmbeddedMATLABFunction_cont sf_EmbeddedMATLABFunction_i;/* '<S426>/Embedded MATLAB Function' */
  rtB_EmbeddedMATLABFunction_cont sf_EmbeddedMATLABFunction;/* '<S425>/Embedded MATLAB Function' */
  rtB_BufferICChannel_controlMCUS sf_BufferICChannel3;/* '<S415>/Buffer IC Channel3' */
  rtB_BufferICChannel_controlMCUS sf_BufferICChannel2;/* '<S415>/Buffer IC Channel2' */
  rtB_BufferICChannel_controlMCUS sf_BufferICChannel1;/* '<S415>/Buffer IC Channel1' */
  rtB_BufferICChannel_controlMCUS sf_BufferICChannel;/* '<S415>/Buffer IC Channel' */
  rtB_myMuxFun1_controlMCUSlugsMK sf_myMuxFun2_c;/* '<S128>/myMux Fun2' */
  rtB_ZerooutZ1_controlMCUSlugsMK sf_ZerooutZ2;/* '<S128>/Zero out Z2' */
  rtB_ZerooutZ1_controlMCUSlugsMK sf_ZerooutZ1;/* '<S128>/Zero out Z1' */
  rtB_negprotect3_controlMCUSlugs sf_negprotect_n;/* '<S218>/negprotect' */
  rtB_negprotect3_controlMCUSlugs sf_negprotect_d;/* '<S217>/negprotect' */
  rtB_ZerooutZ1_controlMCUSlugsMK sf_ZerooutZ1_g;/* '<S225>/Zero out Z1' */
  rtB_negprotect_controlMCUSlugsM sf_negprotect_i;/* '<S231>/negprotect' */
  rtB_EmbeddedMATLABFunction_co_h sf_EmbeddedMATLABFunction_dv;/* '<S230>/Embedded MATLAB Function' */
  rtB_negprotect_controlMCUSlugsM sf_negprotect_p;/* '<S212>/negprotect' */
  rtB_EmbeddedMATLABFunction_co_h sf_EmbeddedMATLABFunction_e;/* '<S211>/Embedded MATLAB Function' */
  rtB_negprotect_controlMCUSlugsM sf_negprotect_dl;/* '<S199>/negprotect' */
  rtB_EmbeddedMATLABFunction_co_h sf_EmbeddedMATLABFunction_p;/* '<S198>/Embedded MATLAB Function' */
  rtB_negprotect3_controlMCUSlugs sf_negprotect2;/* '<S148>/negprotect2' */
  rtB_negprotect3_controlMCUSlugs sf_negprotect1;/* '<S148>/negprotect1' */
  rtB_negprotect_controlMCUSlugsM sf_negprotect_dz;/* '<S165>/negprotect' */
  rtB_negprotect3_controlMCUSlugs sf_negprotect3_b;/* '<S161>/negprotect3' */
  rtB_EmbeddedMATLABFunction_co_k sf_EmbeddedMATLABFunction_ip;/* '<S182>/Embedded MATLAB Function' */
  rtB_negprotect_controlMCUSlugsM sf_negprotect_l0;/* '<S186>/negprotect' */
  rtB_EmbeddedMATLABFunction_co_h sf_EmbeddedMATLABFunction_py;/* '<S185>/Embedded MATLAB Function' */
  rtB_negprotect3_controlMCUSlugs sf_negprotect3;/* '<S160>/negprotect3' */
  rtB_EmbeddedMATLABFunction_co_k sf_EmbeddedMATLABFunction_ls;/* '<S169>/Embedded MATLAB Function' */
  rtB_negprotect_controlMCUSlugsM sf_negprotect_ht;/* '<S173>/negprotect' */
  rtB_EmbeddedMATLABFunction_co_h sf_EmbeddedMATLABFunction_eq;/* '<S172>/Embedded MATLAB Function' */
  rtB_ZerooutZ1_controlMCUSlugsMK sf_ZerooutZ2_a;/* '<S146>/Zero out Z2' */
  rtB_negprotect_controlMCUSlugsM sf_negprotect_a;/* '<S244>/negprotect' */
  rtB_EmbeddedMATLABFunction_co_h sf_EmbeddedMATLABFunction_dr;/* '<S243>/Embedded MATLAB Function' */
  rtB_ZerooutZ1_controlMCUSlugsMK sf_ZerooutZ2_e;/* '<S135>/Zero out Z2' */
  rtB_negprotect_controlMCUSlugsM sf_negprotect_dg;/* '<S385>/negprotect' */
  rtB_EmbeddedMATLABFunction_co_h sf_EmbeddedMATLABFunction_n;/* '<S384>/Embedded MATLAB Function' */
  rtB_SelectNTerms_controlMCUSlug sf_SelectNTerms_a;/* '<S330>/Select N  Terms' */
  rtB_negprotect_controlMCUSlugsM sf_negprotect_h;/* '<S345>/negprotect' */
  rtB_EmbeddedMATLABFunction_co_h sf_EmbeddedMATLABFunction_l;/* '<S344>/Embedded MATLAB Function' */
  rtB_negprotect_controlMCUSlugsM sf_negprotect_l;/* '<S338>/negprotect' */
  rtB_EmbeddedMATLABFunction_co_h sf_EmbeddedMATLABFunction_f;/* '<S337>/Embedded MATLAB Function' */
  rtB_ZerooutZ1_controlMCUSlugsMK sf_ZerooutZ3;/* '<S251>/Zero out Z3' */
  rtB_ZerooutZ1_controlMCUSlugsMK sf_ZerooutZ2_k;/* '<S251>/Zero out Z2' */
  rtB_ZerooutZ1_controlMCUSlugsMK sf_ZerooutZ1_l;/* '<S251>/Zero out Z1' */
  rtB_SelectNTerms_controlMCUSlug sf_SelectNTerms;/* '<S272>/Select N  Terms' */
  rtB_negprotect_controlMCUSlugsM sf_negprotect_k;/* '<S290>/negprotect' */
  rtB_EmbeddedMATLABFunction_co_h sf_EmbeddedMATLABFunction_j5;/* '<S289>/Embedded MATLAB Function' */
  rtB_negprotect_controlMCUSlugsM sf_negprotect_nk;/* '<S283>/negprotect' */
  rtB_EmbeddedMATLABFunction_co_h sf_EmbeddedMATLABFunction_i4;/* '<S282>/Embedded MATLAB Function' */
  rtB_negprotect_controlMCUSlugsM sf_negprotect_d0;/* '<S259>/negprotect' */
  rtB_EmbeddedMATLABFunction_co_k sf_EmbeddedMATLABFunction_bh;/* '<S257>/Embedded MATLAB Function' */
  rtB_EmbeddedMATLABFunction_co_k sf_EmbeddedMATLABFunction_do;/* '<S264>/Embedded MATLAB Function' */
  rtB_ZerooutZ1_controlMCUSlugsMK sf_ZerooutZ1_o;/* '<S136>/Zero out Z1' */
  rtB_negprotect_controlMCUSlugsM sf_negprotect_nu;/* '<S401>/negprotect' */
  rtB_EmbeddedMATLABFunction_co_h sf_EmbeddedMATLABFunction_o;/* '<S400>/Embedded MATLAB Function' */
  rtB_myMuxFun1_controlMCUSlugsMK sf_myMuxFun1_i;/* '<S17>/myMux Fun1' */
  rtB_negprotect_controlMCUSlugsM sf_negprotect_e;/* '<S122>/negprotect' */
  rtB_EmbeddedMATLABFunction_cont sf_EmbeddedMATLABFunction_d;/* '<S113>/Embedded MATLAB Function' */
  rtB_EmbeddedMATLABFunction_cont sf_EmbeddedMATLABFunction_a;/* '<S89>/Embedded MATLAB Function' */
  rtB_myMuxFun1_controlMCUSlugsMK sf_myMuxFun1_f;/* '<S16>/myMux Fun1' */
  rtB_EmbeddedMATLABFunction_cont sf_EmbeddedMATLABFunction_b;/* '<S51>/Embedded MATLAB Function' */
} BlockIO_controlMCUSlugsMKIINewN;

/* Block states (auto storage) for system '<Root>' */
typedef struct {
  real_T Add3_DWORK1;                  /* '<S91>/Add3' */
  real32_T IntegerDelay3_DSTATE;       /* '<S19>/Integer Delay3' */
  real32_T IntegerDelay3_DSTATE_i;     /* '<S20>/Integer Delay3' */
  real32_T IntegerDelay1_DSTATE;       /* '<S146>/Integer Delay1' */
  real32_T IntegerDelay_DSTATE;        /* '<S129>/Integer Delay' */
  real32_T IntegerDelay3_DSTATE_l;     /* '<S263>/Integer Delay3' */
  real32_T IntegerDelay1_DSTATE_p;     /* '<S251>/Integer Delay1' */
  real32_T DiscreteTimeIntegrator_DSTATE[3];/* '<S391>/Discrete-Time Integrator' */
  real32_T IntegerDelay3_DSTATE_g;     /* '<S116>/Integer Delay3' */
  real32_T FixPtUnitDelay1_DSTATE;     /* '<S126>/FixPt Unit Delay1' */
  real32_T IntegerDelay3_DSTATE_c;     /* '<S95>/Integer Delay3' */
  real32_T IntegerDelay2_DSTATE;       /* '<S106>/Integer Delay2' */
  real32_T IntegerDelay3_DSTATE_o;     /* '<S108>/Integer Delay3' */
  real32_T IntegerDelay_DSTATE_c;      /* '<S106>/Integer Delay' */
  real32_T IntegerDelay1_DSTATE_j[2];  /* '<S106>/Integer Delay1' */
  real32_T IntegerDelay3_DSTATE_lo;    /* '<S107>/Integer Delay3' */
  real32_T IntegerDelay3_DSTATE_i4;    /* '<S97>/Integer Delay3' */
  real32_T IntegerDelay3_DSTATE_gy;    /* '<S94>/Integer Delay3' */
  real32_T NDelays_DSTATE[5];          /* '<S90>/NDelays' */
  real32_T IntegerDelay2_DSTATE_b;     /* '<S102>/Integer Delay2' */
  real32_T IntegerDelay3_DSTATE_oy;    /* '<S103>/Integer Delay3' */
  real32_T IntegerDelay_DSTATE_o;      /* '<S102>/Integer Delay' */
  real32_T IntegerDelay1_DSTATE_m[2];  /* '<S102>/Integer Delay1' */
  real32_T IntegerDelay3_DSTATE_d;     /* '<S104>/Integer Delay3' */
  real32_T IntegerDelay3_DSTATE_cv;    /* '<S96>/Integer Delay3' */
  real32_T NDelays_DSTATE_n[5];        /* '<S92>/NDelays' */
  real32_T IntegerDelay2_DSTATE_g;     /* '<S110>/Integer Delay2' */
  real32_T IntegerDelay3_DSTATE_f;     /* '<S111>/Integer Delay3' */
  real32_T IntegerDelay_DSTATE_i;      /* '<S110>/Integer Delay' */
  real32_T IntegerDelay1_DSTATE_g[2];  /* '<S110>/Integer Delay1' */
  real32_T IntegerDelay3_DSTATE_n;     /* '<S112>/Integer Delay3' */
  real32_T IntegerDelay3_DSTATE_m;     /* '<S83>/Integer Delay3' */
  real32_T IntegerDelay3_DSTATE_k;     /* '<S84>/Integer Delay3' */
  real32_T IntegerDelay3_DSTATE_h;     /* '<S54>/Integer Delay3' */
  real32_T NDelays_DSTATE_j[5];        /* '<S57>/NDelays' */
  real32_T IntegerDelay2_DSTATE_o;     /* '<S77>/Integer Delay2' */
  real32_T IntegerDelay3_DSTATE_mx;    /* '<S78>/Integer Delay3' */
  real32_T IntegerDelay_DSTATE_ie;     /* '<S77>/Integer Delay' */
  real32_T IntegerDelay1_DSTATE_jy[2]; /* '<S77>/Integer Delay1' */
  real32_T IntegerDelay3_DSTATE_gf;    /* '<S79>/Integer Delay3' */
  real32_T IntegerDelay3_DSTATE_p;     /* '<S53>/Integer Delay3' */
  real32_T NDelays_DSTATE_a[5];        /* '<S55>/NDelays' */
  real32_T IntegerDelay2_DSTATE_g4;    /* '<S61>/Integer Delay2' */
  real32_T IntegerDelay3_DSTATE_c5;    /* '<S62>/Integer Delay3' */
  real32_T IntegerDelay_DSTATE_k;      /* '<S61>/Integer Delay' */
  real32_T IntegerDelay1_DSTATE_b[2];  /* '<S61>/Integer Delay1' */
  real32_T IntegerDelay3_DSTATE_le;    /* '<S63>/Integer Delay3' */
  real32_T UD_DSTATE;                  /* '<S73>/UD' */
  real32_T IntegerDelay_DSTATE_h;      /* '<S65>/Integer Delay' */
  real32_T IntegerDelay1_DSTATE_e;     /* '<S65>/Integer Delay1' */
  real32_T IntegerDelay3_DSTATE_ij;    /* '<S66>/Integer Delay3' */
  real32_T UD_DSTATE_b;                /* '<S35>/UD' */
  real32_T IntegerDelay_DSTATE_l;      /* '<S25>/Integer Delay' */
  real32_T IntegerDelay1_DSTATE_k;     /* '<S25>/Integer Delay1' */
  real32_T IntegerDelay3_DSTATE_nd;    /* '<S27>/Integer Delay3' */
  real32_T UD_DSTATE_o;                /* '<S45>/UD' */
  real32_T IntegerDelay_DSTATE_f;      /* '<S29>/Integer Delay' */
  real32_T IntegerDelay1_DSTATE_i;     /* '<S29>/Integer Delay1' */
  real32_T IntegerDelay_DSTATE_cl;     /* '<S15>/Integer Delay' */
  real32_T IntegerDelay1_DSTATE_c;     /* '<S15>/Integer Delay1' */
  real32_T IntegerDelay3_DSTATE_e;     /* '<S28>/Integer Delay3' */
  real32_T IntegerDelay_DSTATE_fj;     /* '<S30>/Integer Delay' */
  real32_T IntegerDelay1_DSTATE_bo[2]; /* '<S30>/Integer Delay1' */
  real32_T IntegerDelay2_DSTATE_n;     /* '<S30>/Integer Delay2' */
  real32_T IntegerDelay3_DSTATE_o5;    /* '<S46>/Integer Delay3' */
  real32_T IntegerDelay3_DSTATE_ns;    /* '<S47>/Integer Delay3' */
  real32_T Memory1_PreviousInput;      /* '<S91>/Memory1' */
  real32_T Memory1_PreviousInput_c;    /* '<S90>/Memory1' */
  real32_T Memory1_PreviousInput_b;    /* '<S92>/Memory1' */
  real32_T Memory1_PreviousInput_n;    /* '<S57>/Memory1' */
  real32_T Memory1_PreviousInput_g;    /* '<S55>/Memory1' */
  uint8_T IntegerDelay_DSTATE_d;       /* '<S128>/Integer Delay' */
  uint8_T IntegerDelay1_DSTATE_j2;     /* '<S128>/Integer Delay1' */
  uint8_T IntegerDelay_DSTATE_p;       /* '<S135>/Integer Delay' */
  uint8_T FixPtUnitDelay2_DSTATE;      /* '<S126>/FixPt Unit Delay2' */
  boolean_T Delay_DSTATE;              /* '<S412>/Delay' */
  uint8_T fromWp;                      /* '<S135>/computeCurrentWP' */
  uint8_T toWp;                        /* '<S135>/computeCurrentWP' */
  uint8_T persistentDidReachIP;        /* '<S135>/Embedded MATLAB Function' */
  uint8_T DiscreteTimeIntegrator_IC_LOADI;/* '<S391>/Discrete-Time Integrator' */
  boolean_T IC1_FirstOutputTime;       /* '<S418>/IC1' */
  boolean_T IC1_FirstOutputTime_p;     /* '<S137>/IC1' */
  boolean_T IC2_FirstOutputTime;       /* '<S137>/IC2' */
  boolean_T IC3_FirstOutputTime;       /* '<S137>/IC3' */
  boolean_T IC4_FirstOutputTime;       /* '<S137>/IC4' */
  boolean_T IC_FirstOutputTime;        /* '<S134>/IC' */
  boolean_T IC_FirstOutputTime_l;      /* '<S135>/IC' */
  boolean_T IC_FirstOutputTime_h;      /* '<S65>/IC' */
  boolean_T IC_FirstOutputTime_a;      /* '<S25>/IC' */
  boolean_T IC_FirstOutputTime_m;      /* '<S29>/IC' */
  boolean_T L1OutputFeedbackControllerWithP;/* '<S3>/L1 Output Feedback Controller With  Projection Operator' */
  boolean_T SideslipCompensation_MODE; /* '<S48>/Sideslip Compensation' */
  rtDW_EmbeddedMATLABFunction_con sf_EmbeddedMATLABFunction_ab;/* '<S485>/Embedded MATLAB Function' */
  rtDW_EmbeddedMATLABFunction_con sf_EmbeddedMATLABFunction_e4;/* '<S484>/Embedded MATLAB Function' */
  rtDW_EmbeddedMATLABFunction_con sf_EmbeddedMATLABFunction_px;/* '<S483>/Embedded MATLAB Function' */
  rtDW_EmbeddedMATLABFunction_con sf_EmbeddedMATLABFunction_dk;/* '<S466>/Embedded MATLAB Function' */
  rtDW_EmbeddedMATLABFunction_con sf_EmbeddedMATLABFunction_lo;/* '<S470>/Embedded MATLAB Function' */
  rtDW_BufferICChannel_controlMCU sf_BufferFailsafeChannel;/* '<S420>/Buffer Failsafe Channel' */
  rtDW_EmbeddedMATLABFunction_con sf_EmbeddedMATLABFunction_i;/* '<S426>/Embedded MATLAB Function' */
  rtDW_EmbeddedMATLABFunction_con sf_EmbeddedMATLABFunction;/* '<S425>/Embedded MATLAB Function' */
  rtDW_BufferICChannel_controlMCU sf_BufferICChannel3;/* '<S415>/Buffer IC Channel3' */
  rtDW_BufferICChannel_controlMCU sf_BufferICChannel2;/* '<S415>/Buffer IC Channel2' */
  rtDW_BufferICChannel_controlMCU sf_BufferICChannel1;/* '<S415>/Buffer IC Channel1' */
  rtDW_BufferICChannel_controlMCU sf_BufferICChannel;/* '<S415>/Buffer IC Channel' */
  rtDW_EmbeddedMATLABFunction_con sf_EmbeddedMATLABFunction_d;/* '<S113>/Embedded MATLAB Function' */
  rtDW_EmbeddedMATLABFunction_con sf_EmbeddedMATLABFunction_a;/* '<S89>/Embedded MATLAB Function' */
  rtDW_EmbeddedMATLABFunction_con sf_EmbeddedMATLABFunction_b;/* '<S51>/Embedded MATLAB Function' */
} D_Work_controlMCUSlugsMKIINewNa;

/* Invariant block signals (auto storage) */
typedef struct {
  const real32_T e1;                   /* '<S209>/e^1' */
  const real32_T u;                    /* '<S209>/ ' */
  const real32_T Sum5;                 /* '<S209>/Sum5' */
  const real32_T e2;                   /* '<S206>/Sum3' */
  const real32_T e1_d;                 /* '<S411>/e^1' */
  const real32_T u_g;                  /* '<S411>/ ' */
  const real32_T Sum5_i;               /* '<S411>/Sum5' */
  const real32_T e2_g;                 /* '<S410>/Sum3' */
  const real32_T e1_n;                 /* '<S158>/e^1' */
  const real32_T u_o;                  /* '<S158>/ ' */
  const real32_T Sum5_h;               /* '<S158>/Sum5' */
  const real32_T e2_n;                 /* '<S155>/Sum3' */
  const real32_T e1_m;                 /* '<S357>/e^1' */
  const real32_T u_m;                  /* '<S357>/ ' */
  const real32_T Sum5_c;               /* '<S357>/Sum5' */
  const real32_T e2_b;                 /* '<S354>/Sum3' */
  const real32_T e1_dl;                /* '<S374>/e^1' */
  const real32_T u_e;                  /* '<S374>/ ' */
  const real32_T Sum5_l;               /* '<S374>/Sum5' */
  const real32_T e2_h;                 /* '<S371>/Sum3' */
  const real32_T e1_i;                 /* '<S302>/e^1' */
  const real32_T u_h;                  /* '<S302>/ ' */
  const real32_T Sum5_d;               /* '<S302>/Sum5' */
  const real32_T e2_o;                 /* '<S299>/Sum3' */
  const real32_T e1_i5;                /* '<S319>/e^1' */
  const real32_T u_b;                  /* '<S319>/ ' */
  const real32_T Sum5_n;               /* '<S319>/Sum5' */
  const real32_T e2_ge;                /* '<S316>/Sum3' */
  const real32_T e1_e;                 /* '<S399>/e^1' */
  const real32_T u_i;                  /* '<S399>/ ' */
  const real32_T Sum5_ib;              /* '<S399>/Sum5' */
  const real32_T e2_e;                 /* '<S396>/Sum3' */
  const real32_T Add3;                 /* '<S91>/Add3' */
  const real32_T Add4;                 /* '<S91>/Add4' */
  const real32_T Divide1;              /* '<S40>/Divide1' */
  const real32_T Divide2;              /* '<S40>/Divide2' */
  const boolean_T InitializeSPISSLasInput;/* '<Root>/Initialize SPI SSL as  Input' */
} ConstBlockIO_controlMCUSlugsMKI;

/* Constant parameters (auto storage) */
typedef struct {
  /* Pooled Parameter (Expression: [0 0 1;0 1 0;1 0 0])
   * Referenced by:
   *   '<S130>/UEN 2 NEU'
   *   '<S152>/UEN 2 NEU'
   *   '<S394>/UEN 2 NEU'
   *   '<S296>/UEN 2 NEU'
   *   '<S313>/UEN 2 NEU'
   *   '<S351>/UEN 2 NEU'
   *   '<S368>/UEN 2 NEU'
   */
  real32_T pooled45[9];
} ConstParam_controlMCUSlugsMKIIN;

/* Real-time Model Data Structure */
struct RT_MODEL_controlMCUSlugsMKIINew {
  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    boolean_T firstInitCondFlag;
    struct {
      uint8_T TID[3];
    } TaskCounters;
  } Timing;
};

/* Block signals (auto storage) */
extern BlockIO_controlMCUSlugsMKIINewN controlMCUSlugsMKIINewNav_B;

/* Block states (auto storage) */
extern D_Work_controlMCUSlugsMKIINewNa controlMCUSlugsMKIINewNav_DWork;
extern const ConstBlockIO_controlMCUSlugsMKI controlMCUSlugsMKIINewNa_ConstB;/* constant block i/o */

/* Constant parameters (auto storage) */
extern const ConstParam_controlMCUSlugsMKIIN controlMCUSlugsMKIINewNa_ConstP;

/* Model entry point functions */
extern void controlMCUSlugsMKIINewNav_initialize(boolean_T firstTime);
extern void controlMCUSlugsMKIINewNav_step(int_T tid);

/* Real-time Model object */
extern struct RT_MODEL_controlMCUSlugsMKIINew *const controlMCUSlugsMKIINewNav_M;

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
 * '<Root>' : 'controlMCUSlugsMKIINewNav'
 * '<S1>'   : 'controlMCUSlugsMKIINewNav/Camera Location'
 * '<S2>'   : 'controlMCUSlugsMKIINewNav/Data  Sources'
 * '<S3>'   : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation'
 * '<S4>'   : 'controlMCUSlugsMKIINewNav/PID Gains'
 * '<S5>'   : 'controlMCUSlugsMKIINewNav/Pilot'
 * '<S6>'   : 'controlMCUSlugsMKIINewNav/Pilot LED and HIL Control Pins'
 * '<S7>'   : 'controlMCUSlugsMKIINewNav/Trim Vals'
 * '<S8>'   : 'controlMCUSlugsMKIINewNav/Update  States'
 * '<S9>'   : 'controlMCUSlugsMKIINewNav/Update PWM Commands and Send Telemetry'
 * '<S10>'  : 'controlMCUSlugsMKIINewNav/get Nav Vars'
 * '<S11>'  : 'controlMCUSlugsMKIINewNav/myMux Fun1'
 * '<S12>'  : 'controlMCUSlugsMKIINewNav/Camera Location/myMux Fun4'
 * '<S13>'  : 'controlMCUSlugsMKIINewNav/Data  Sources/Compute Load'
 * '<S14>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Compute psi_dot_m'
 * '<S15>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/L1 Output Feedback Controller With  Projection Operator'
 * '<S16>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Lateral Channel Encaps'
 * '<S17>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Longitudinal Channel Encaps'
 * '<S18>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps'
 * '<S19>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Protect NaNs'
 * '<S20>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Compute psi_dot_m/Protect NaNs'
 * '<S21>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Compute psi_dot_m/dsPIC_cos'
 * '<S22>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Compute psi_dot_m/dsPIC_sin'
 * '<S23>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Compute psi_dot_m/dsPIC_cos/Environment Controller1'
 * '<S24>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Compute psi_dot_m/dsPIC_sin/Environment Controller'
 * '<S25>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/L1 Output Feedback Controller With  Projection Operator/Control  Law'
 * '<S26>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/L1 Output Feedback Controller With  Projection Operator/Projection Operator'
 * '<S27>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/L1 Output Feedback Controller With  Projection Operator/Protect NaNs'
 * '<S28>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/L1 Output Feedback Controller With  Projection Operator/Protect NaNs2'
 * '<S29>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/L1 Output Feedback Controller With  Projection Operator/State Predictor'
 * '<S30>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/L1 Output Feedback Controller With  Projection Operator/Trapezoidal Integrator'
 * '<S31>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/L1 Output Feedback Controller With  Projection Operator/Control  Law/Compute Coef'
 * '<S32>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/L1 Output Feedback Controller With  Projection Operator/Control  Law/Detect Change'
 * '<S33>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/L1 Output Feedback Controller With  Projection Operator/Control  Law/Compute Coef/dsPIC_exp'
 * '<S34>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/L1 Output Feedback Controller With  Projection Operator/Control  Law/Compute Coef/dsPIC_exp/Environment Controller'
 * '<S35>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/L1 Output Feedback Controller With  Projection Operator/Control  Law/Detect Change/Difference1'
 * '<S36>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/L1 Output Feedback Controller With  Projection Operator/Projection Operator/Compare To Zero'
 * '<S37>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/L1 Output Feedback Controller With  Projection Operator/Projection Operator/Compare To Zero1'
 * '<S38>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/L1 Output Feedback Controller With  Projection Operator/Projection Operator/Enabled Subsystem'
 * '<S39>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/L1 Output Feedback Controller With  Projection Operator/Projection Operator/Enabled Subsystem1'
 * '<S40>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/L1 Output Feedback Controller With  Projection Operator/Projection Operator/F(theta)'
 * '<S41>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/L1 Output Feedback Controller With  Projection Operator/State Predictor/Compute Coef'
 * '<S42>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/L1 Output Feedback Controller With  Projection Operator/State Predictor/Detect Change'
 * '<S43>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/L1 Output Feedback Controller With  Projection Operator/State Predictor/Compute Coef/dsPIC_exp'
 * '<S44>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/L1 Output Feedback Controller With  Projection Operator/State Predictor/Compute Coef/dsPIC_exp/Environment Controller'
 * '<S45>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/L1 Output Feedback Controller With  Projection Operator/State Predictor/Detect Change/Difference1'
 * '<S46>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/L1 Output Feedback Controller With  Projection Operator/Trapezoidal Integrator/Protect NaNs'
 * '<S47>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/L1 Output Feedback Controller With  Projection Operator/Trapezoidal Integrator/Protect NaNs1'
 * '<S48>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Lateral Channel Encaps/Lateral Channel'
 * '<S49>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Lateral Channel Encaps/myMux Fun1'
 * '<S50>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Lateral Channel Encaps/psi_dot_c Select logic1'
 * '<S51>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Lateral Channel Encaps/Lateral Channel/0.016 sec  tau'
 * '<S52>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Lateral Channel Encaps/Lateral Channel/Angle Conversion1'
 * '<S53>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Lateral Channel Encaps/Lateral Channel/Protect NaNs'
 * '<S54>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Lateral Channel Encaps/Lateral Channel/Protect NaNs2'
 * '<S55>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Lateral Channel Encaps/Lateral Channel/Roll Control [PID]'
 * '<S56>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Lateral Channel Encaps/Lateral Channel/Sideslip Compensation'
 * '<S57>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Lateral Channel Encaps/Lateral Channel/Yaw Damper [PID]'
 * '<S58>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Lateral Channel Encaps/Lateral Channel/psi_dot to Bank'
 * '<S59>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Lateral Channel Encaps/Lateral Channel/0.016 sec  tau/Embedded MATLAB Function'
 * '<S60>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Lateral Channel Encaps/Lateral Channel/Roll Control [PID]/Saturation Dynamic'
 * '<S61>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Lateral Channel Encaps/Lateral Channel/Roll Control [PID]/Trapezoidal Integrator'
 * '<S62>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Lateral Channel Encaps/Lateral Channel/Roll Control [PID]/Trapezoidal Integrator/Protect NaNs '
 * '<S63>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Lateral Channel Encaps/Lateral Channel/Roll Control [PID]/Trapezoidal Integrator/Protect NaNs1'
 * '<S64>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Lateral Channel Encaps/Lateral Channel/Sideslip Compensation/Bank to Psi Dot'
 * '<S65>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Lateral Channel Encaps/Lateral Channel/Sideslip Compensation/Low Pass'
 * '<S66>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Lateral Channel Encaps/Lateral Channel/Sideslip Compensation/Protect NaNs2'
 * '<S67>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Lateral Channel Encaps/Lateral Channel/Sideslip Compensation/psi_dot to Bank'
 * '<S68>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Lateral Channel Encaps/Lateral Channel/Sideslip Compensation/Bank to Psi Dot/dsPIC_tan'
 * '<S69>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Lateral Channel Encaps/Lateral Channel/Sideslip Compensation/Bank to Psi Dot/negprotect'
 * '<S70>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Lateral Channel Encaps/Lateral Channel/Sideslip Compensation/Bank to Psi Dot/dsPIC_tan/Environment Controller'
 * '<S71>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Lateral Channel Encaps/Lateral Channel/Sideslip Compensation/Low Pass/Compute Coef'
 * '<S72>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Lateral Channel Encaps/Lateral Channel/Sideslip Compensation/Low Pass/Detect Change'
 * '<S73>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Lateral Channel Encaps/Lateral Channel/Sideslip Compensation/Low Pass/Detect Change/Difference1'
 * '<S74>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Lateral Channel Encaps/Lateral Channel/Sideslip Compensation/psi_dot to Bank/dsPIC_atan'
 * '<S75>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Lateral Channel Encaps/Lateral Channel/Sideslip Compensation/psi_dot to Bank/dsPIC_atan/Environment Controller'
 * '<S76>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Lateral Channel Encaps/Lateral Channel/Yaw Damper [PID]/Saturation Dynamic'
 * '<S77>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Lateral Channel Encaps/Lateral Channel/Yaw Damper [PID]/Trapezoidal Integrator'
 * '<S78>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Lateral Channel Encaps/Lateral Channel/Yaw Damper [PID]/Trapezoidal Integrator/Protect NaNs '
 * '<S79>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Lateral Channel Encaps/Lateral Channel/Yaw Damper [PID]/Trapezoidal Integrator/Protect NaNs1'
 * '<S80>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Lateral Channel Encaps/Lateral Channel/psi_dot to Bank/dsPIC_atan'
 * '<S81>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Lateral Channel Encaps/Lateral Channel/psi_dot to Bank/dsPIC_atan/Environment Controller'
 * '<S82>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Lateral Channel Encaps/psi_dot_c Select logic1/Bank to Psi Dot'
 * '<S83>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Lateral Channel Encaps/psi_dot_c Select logic1/Bank to Psi Dot/Protect NaNs'
 * '<S84>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Lateral Channel Encaps/psi_dot_c Select logic1/Bank to Psi Dot/Protect NaNs1'
 * '<S85>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Lateral Channel Encaps/psi_dot_c Select logic1/Bank to Psi Dot/dsPIC_tan'
 * '<S86>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Lateral Channel Encaps/psi_dot_c Select logic1/Bank to Psi Dot/dsPIC_tan/Environment Controller'
 * '<S87>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Longitudinal Channel Encaps/Longitudinal Channel'
 * '<S88>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Longitudinal Channel Encaps/myMux Fun1'
 * '<S89>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Longitudinal Channel Encaps/Longitudinal Channel/0.5 sec  tau for Turns'
 * '<S90>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Longitudinal Channel Encaps/Longitudinal Channel/Airspeed Hold [PID]'
 * '<S91>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Longitudinal Channel Encaps/Longitudinal Channel/Height Error to Pitch  Command [PI]'
 * '<S92>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Longitudinal Channel Encaps/Longitudinal Channel/Pitch To Elevator [PID]'
 * '<S93>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Longitudinal Channel Encaps/Longitudinal Channel/Pitot Control End'
 * '<S94>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Longitudinal Channel Encaps/Longitudinal Channel/Protect NaNs'
 * '<S95>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Longitudinal Channel Encaps/Longitudinal Channel/Protect NaNs1'
 * '<S96>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Longitudinal Channel Encaps/Longitudinal Channel/Protect NaNs2'
 * '<S97>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Longitudinal Channel Encaps/Longitudinal Channel/Protect NaNs4'
 * '<S98>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Longitudinal Channel Encaps/Longitudinal Channel/Rate Limiter Dynamic with External IC'
 * '<S99>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Longitudinal Channel Encaps/Longitudinal Channel/dsPIC_cos'
 * '<S100>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Longitudinal Channel Encaps/Longitudinal Channel/0.5 sec  tau for Turns/Embedded MATLAB Function'
 * '<S101>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Longitudinal Channel Encaps/Longitudinal Channel/Airspeed Hold [PID]/Saturation Dynamic'
 * '<S102>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Longitudinal Channel Encaps/Longitudinal Channel/Airspeed Hold [PID]/Trapezoidal Integrator'
 * '<S103>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Longitudinal Channel Encaps/Longitudinal Channel/Airspeed Hold [PID]/Trapezoidal Integrator/Protect NaNs '
 * '<S104>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Longitudinal Channel Encaps/Longitudinal Channel/Airspeed Hold [PID]/Trapezoidal Integrator/Protect NaNs1'
 * '<S105>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Longitudinal Channel Encaps/Longitudinal Channel/Height Error to Pitch  Command [PI]/Saturation Dynamic'
 * '<S106>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Longitudinal Channel Encaps/Longitudinal Channel/Height Error to Pitch  Command [PI]/Trapezoidal Integrator'
 * '<S107>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Longitudinal Channel Encaps/Longitudinal Channel/Height Error to Pitch  Command [PI]/Trapezoidal Integrator/Protect NaNs'
 * '<S108>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Longitudinal Channel Encaps/Longitudinal Channel/Height Error to Pitch  Command [PI]/Trapezoidal Integrator/Protect NaNs '
 * '<S109>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Longitudinal Channel Encaps/Longitudinal Channel/Pitch To Elevator [PID]/Saturation Dynamic'
 * '<S110>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Longitudinal Channel Encaps/Longitudinal Channel/Pitch To Elevator [PID]/Trapezoidal Integrator'
 * '<S111>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Longitudinal Channel Encaps/Longitudinal Channel/Pitch To Elevator [PID]/Trapezoidal Integrator/Protect NaNs '
 * '<S112>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Longitudinal Channel Encaps/Longitudinal Channel/Pitch To Elevator [PID]/Trapezoidal Integrator/Protect NaNs1'
 * '<S113>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Longitudinal Channel Encaps/Longitudinal Channel/Pitot Control End/ 0.3 sec tau for high speeds'
 * '<S114>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Longitudinal Channel Encaps/Longitudinal Channel/Pitot Control End/Air Density from Height'
 * '<S115>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Longitudinal Channel Encaps/Longitudinal Channel/Pitot Control End/Compute Airspeed'
 * '<S116>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Longitudinal Channel Encaps/Longitudinal Channel/Pitot Control End/Protect NaN'
 * '<S117>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Longitudinal Channel Encaps/Longitudinal Channel/Pitot Control End/lowpass limit'
 * '<S118>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Longitudinal Channel Encaps/Longitudinal Channel/Pitot Control End/ 0.3 sec tau for high speeds/Embedded MATLAB Function'
 * '<S119>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Longitudinal Channel Encaps/Longitudinal Channel/Pitot Control End/Air Density from Height/Length Conversion'
 * '<S120>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Longitudinal Channel Encaps/Longitudinal Channel/Pitot Control End/Air Density from Height/dsPIC Power'
 * '<S121>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Longitudinal Channel Encaps/Longitudinal Channel/Pitot Control End/Air Density from Height/dsPIC Power/Environment Controller'
 * '<S122>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Longitudinal Channel Encaps/Longitudinal Channel/Pitot Control End/Compute Airspeed/dsPIC_SQRT'
 * '<S123>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Longitudinal Channel Encaps/Longitudinal Channel/Pitot Control End/Compute Airspeed/dsPIC_SQRT/Environment Controller'
 * '<S124>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Longitudinal Channel Encaps/Longitudinal Channel/Pitot Control End/Compute Airspeed/dsPIC_SQRT/negprotect'
 * '<S125>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Longitudinal Channel Encaps/Longitudinal Channel/Rate Limiter Dynamic with External IC/Saturation Dynamic'
 * '<S126>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Longitudinal Channel Encaps/Longitudinal Channel/Rate Limiter Dynamic with External IC/Unit Delay External IC'
 * '<S127>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Longitudinal Channel Encaps/Longitudinal Channel/dsPIC_cos/Environment Controller1'
 * '<S128>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation'
 * '<S129>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation'
 * '<S130>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Geod2LTP'
 * '<S131>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Get 2D Groundspeed'
 * '<S132>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/L2+'
 * '<S133>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Line Segment'
 * '<S134>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Minimum Turn Radius'
 * '<S135>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation'
 * '<S136>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/RTB//Follow Mobile Navigation'
 * '<S137>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Recent Enable'
 * '<S138>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Zero out Z1'
 * '<S139>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Zero out Z2'
 * '<S140>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/myMux Fun1'
 * '<S141>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/myMux Fun2'
 * '<S142>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/myMux Fun3'
 * '<S143>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/myMux Fun4'
 * '<S144>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/myMux Fun5'
 * '<S145>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/myMux Fun6'
 * '<S146>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Get SP Location'
 * '<S147>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Inside the Circle,  Keep Straight until  intersection'
 * '<S148>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Intersection. Circular Navigation'
 * '<S149>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/No intersection,  Navigate to ISR'
 * '<S150>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Vector Norm'
 * '<S151>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/dsPIC_ABS'
 * '<S152>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Get SP Location/Geod2LTP'
 * '<S153>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Get SP Location/Zero out Z2'
 * '<S154>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Get SP Location/Geod2LTP/ECEF2LTP (UEN)'
 * '<S155>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Get SP Location/Geod2LTP/Geod2ECEF1'
 * '<S156>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Get SP Location/Geod2LTP/ECEF2LTP (UEN)/From Body to Universal1'
 * '<S157>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Get SP Location/Geod2LTP/ECEF2LTP (UEN)/From Body to Universal1/Rotation Matrix from Long////Lat'
 * '<S158>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Get SP Location/Geod2LTP/Geod2ECEF1/eccentricity^2'
 * '<S159>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Inside the Circle,  Keep Straight until  intersection/Compute Head of Circle'
 * '<S160>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Intersection. Circular Navigation/Compute Angle Between Vectors1'
 * '<S161>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Intersection. Circular Navigation/Compute Angle Between Vectors2'
 * '<S162>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Intersection. Circular Navigation/Embedded MATLAB Function'
 * '<S163>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Intersection. Circular Navigation/dsPIC_ABS'
 * '<S164>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Intersection. Circular Navigation/dsPIC_ABS1'
 * '<S165>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Intersection. Circular Navigation/dsPIC_SQRT'
 * '<S166>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Intersection. Circular Navigation/negprotect1'
 * '<S167>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Intersection. Circular Navigation/negprotect2'
 * '<S168>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Intersection. Circular Navigation/Compute Angle Between Vectors1/Vector Norm'
 * '<S169>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Intersection. Circular Navigation/Compute Angle Between Vectors1/dsPIC Dot Product'
 * '<S170>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Intersection. Circular Navigation/Compute Angle Between Vectors1/dsPIC_acos'
 * '<S171>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Intersection. Circular Navigation/Compute Angle Between Vectors1/negprotect3'
 * '<S172>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Intersection. Circular Navigation/Compute Angle Between Vectors1/Vector Norm/dsPIC Dot Product'
 * '<S173>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Intersection. Circular Navigation/Compute Angle Between Vectors1/Vector Norm/dsPIC_SQRT'
 * '<S174>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Intersection. Circular Navigation/Compute Angle Between Vectors1/Vector Norm/dsPIC Dot Product/Embedded MATLAB Function'
 * '<S175>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Intersection. Circular Navigation/Compute Angle Between Vectors1/Vector Norm/dsPIC Dot Product/Environment Controller'
 * '<S176>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Intersection. Circular Navigation/Compute Angle Between Vectors1/Vector Norm/dsPIC_SQRT/Environment Controller'
 * '<S177>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Intersection. Circular Navigation/Compute Angle Between Vectors1/Vector Norm/dsPIC_SQRT/negprotect'
 * '<S178>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Intersection. Circular Navigation/Compute Angle Between Vectors1/dsPIC Dot Product/Embedded MATLAB Function'
 * '<S179>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Intersection. Circular Navigation/Compute Angle Between Vectors1/dsPIC Dot Product/Environment Controller'
 * '<S180>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Intersection. Circular Navigation/Compute Angle Between Vectors1/dsPIC_acos/Environment Controller'
 * '<S181>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Intersection. Circular Navigation/Compute Angle Between Vectors2/Vector Norm'
 * '<S182>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Intersection. Circular Navigation/Compute Angle Between Vectors2/dsPIC Dot Product'
 * '<S183>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Intersection. Circular Navigation/Compute Angle Between Vectors2/dsPIC_acos'
 * '<S184>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Intersection. Circular Navigation/Compute Angle Between Vectors2/negprotect3'
 * '<S185>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Intersection. Circular Navigation/Compute Angle Between Vectors2/Vector Norm/dsPIC Dot Product'
 * '<S186>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Intersection. Circular Navigation/Compute Angle Between Vectors2/Vector Norm/dsPIC_SQRT'
 * '<S187>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Intersection. Circular Navigation/Compute Angle Between Vectors2/Vector Norm/dsPIC Dot Product/Embedded MATLAB Function'
 * '<S188>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Intersection. Circular Navigation/Compute Angle Between Vectors2/Vector Norm/dsPIC Dot Product/Environment Controller'
 * '<S189>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Intersection. Circular Navigation/Compute Angle Between Vectors2/Vector Norm/dsPIC_SQRT/Environment Controller'
 * '<S190>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Intersection. Circular Navigation/Compute Angle Between Vectors2/Vector Norm/dsPIC_SQRT/negprotect'
 * '<S191>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Intersection. Circular Navigation/Compute Angle Between Vectors2/dsPIC Dot Product/Embedded MATLAB Function'
 * '<S192>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Intersection. Circular Navigation/Compute Angle Between Vectors2/dsPIC Dot Product/Environment Controller'
 * '<S193>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Intersection. Circular Navigation/Compute Angle Between Vectors2/dsPIC_acos/Environment Controller'
 * '<S194>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Intersection. Circular Navigation/dsPIC_ABS/Environment Controller'
 * '<S195>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Intersection. Circular Navigation/dsPIC_ABS1/Environment Controller'
 * '<S196>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Intersection. Circular Navigation/dsPIC_SQRT/Environment Controller'
 * '<S197>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Intersection. Circular Navigation/dsPIC_SQRT/negprotect'
 * '<S198>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Vector Norm/dsPIC Dot Product'
 * '<S199>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Vector Norm/dsPIC_SQRT'
 * '<S200>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Vector Norm/dsPIC Dot Product/Embedded MATLAB Function'
 * '<S201>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Vector Norm/dsPIC Dot Product/Environment Controller'
 * '<S202>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Vector Norm/dsPIC_SQRT/Environment Controller'
 * '<S203>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Vector Norm/dsPIC_SQRT/negprotect'
 * '<S204>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/dsPIC_ABS/Environment Controller'
 * '<S205>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Geod2LTP/ECEF2LTP (UEN)'
 * '<S206>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Geod2LTP/Geod2ECEF1'
 * '<S207>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Geod2LTP/ECEF2LTP (UEN)/From Body to Universal1'
 * '<S208>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Geod2LTP/ECEF2LTP (UEN)/From Body to Universal1/Rotation Matrix from Long////Lat'
 * '<S209>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Geod2LTP/Geod2ECEF1/eccentricity^2'
 * '<S210>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Get 2D Groundspeed/Vector Norm'
 * '<S211>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Get 2D Groundspeed/Vector Norm/dsPIC Dot Product'
 * '<S212>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Get 2D Groundspeed/Vector Norm/dsPIC_SQRT'
 * '<S213>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Get 2D Groundspeed/Vector Norm/dsPIC Dot Product/Embedded MATLAB Function'
 * '<S214>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Get 2D Groundspeed/Vector Norm/dsPIC Dot Product/Environment Controller'
 * '<S215>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Get 2D Groundspeed/Vector Norm/dsPIC_SQRT/Environment Controller'
 * '<S216>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Get 2D Groundspeed/Vector Norm/dsPIC_SQRT/negprotect'
 * '<S217>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/L2+/Get Sin, Cos eta'
 * '<S218>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/L2+/L2+'
 * '<S219>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/L2+/Subsystem'
 * '<S220>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/L2+/dsPIC_ABS'
 * '<S221>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/L2+/dsPIC_atan2'
 * '<S222>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/L2+/if sin(eta) < 0 - maxAcc else                 maxAcc '
 * '<S223>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/L2+/Get Sin, Cos eta/2D Dot Product '
 * '<S224>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/L2+/Get Sin, Cos eta/Compare To Zero'
 * '<S225>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/L2+/Get Sin, Cos eta/Get  Groundspeed'
 * '<S226>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/L2+/Get Sin, Cos eta/Get ZProd'
 * '<S227>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/L2+/Get Sin, Cos eta/negprotect'
 * '<S228>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/L2+/Get Sin, Cos eta/Get  Groundspeed/Vector Norm'
 * '<S229>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/L2+/Get Sin, Cos eta/Get  Groundspeed/Zero out Z1'
 * '<S230>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/L2+/Get Sin, Cos eta/Get  Groundspeed/Vector Norm/dsPIC Dot Product'
 * '<S231>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/L2+/Get Sin, Cos eta/Get  Groundspeed/Vector Norm/dsPIC_SQRT'
 * '<S232>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/L2+/Get Sin, Cos eta/Get  Groundspeed/Vector Norm/dsPIC Dot Product/Embedded MATLAB Function'
 * '<S233>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/L2+/Get Sin, Cos eta/Get  Groundspeed/Vector Norm/dsPIC Dot Product/Environment Controller'
 * '<S234>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/L2+/Get Sin, Cos eta/Get  Groundspeed/Vector Norm/dsPIC_SQRT/Environment Controller'
 * '<S235>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/L2+/Get Sin, Cos eta/Get  Groundspeed/Vector Norm/dsPIC_SQRT/negprotect'
 * '<S236>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/L2+/L2+/negprotect'
 * '<S237>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/L2+/Subsystem/dsPIC_atan'
 * '<S238>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/L2+/Subsystem/dsPIC_atan/Environment Controller'
 * '<S239>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/L2+/dsPIC_ABS/Environment Controller'
 * '<S240>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/L2+/dsPIC_atan2/Environment Controller'
 * '<S241>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/L2+/if sin(eta) < 0 - maxAcc else                 maxAcc /Compare To Zero'
 * '<S242>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Line Segment/Vector Norm'
 * '<S243>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Line Segment/Vector Norm/dsPIC Dot Product'
 * '<S244>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Line Segment/Vector Norm/dsPIC_SQRT'
 * '<S245>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Line Segment/Vector Norm/dsPIC Dot Product/Embedded MATLAB Function'
 * '<S246>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Line Segment/Vector Norm/dsPIC Dot Product/Environment Controller'
 * '<S247>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Line Segment/Vector Norm/dsPIC_SQRT/Environment Controller'
 * '<S248>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Line Segment/Vector Norm/dsPIC_SQRT/negprotect'
 * '<S249>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Compute WP Switching Trigger'
 * '<S250>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Embedded MATLAB Function'
 * '<S251>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet'
 * '<S252>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable'
 * '<S253>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Vector Norm'
 * '<S254>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Zero out Z2'
 * '<S255>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/computeCurrentWP'
 * '<S256>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Compute WP Switching Trigger/Compute  Projection'
 * '<S257>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Compute WP Switching Trigger/dsPIC Dot Product'
 * '<S258>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Compute WP Switching Trigger/dsPIC_ABS'
 * '<S259>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Compute WP Switching Trigger/dsPIC_SQRT'
 * '<S260>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Compute WP Switching Trigger/max'
 * '<S261>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Compute WP Switching Trigger/max1'
 * '<S262>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Compute WP Switching Trigger/min'
 * '<S263>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Compute WP Switching Trigger/Compute  Projection/Protect NaNs'
 * '<S264>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Compute WP Switching Trigger/Compute  Projection/dsPIC Dot Product'
 * '<S265>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Compute WP Switching Trigger/Compute  Projection/dsPIC Dot Product/Embedded MATLAB Function'
 * '<S266>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Compute WP Switching Trigger/Compute  Projection/dsPIC Dot Product/Environment Controller'
 * '<S267>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Compute WP Switching Trigger/dsPIC Dot Product/Embedded MATLAB Function'
 * '<S268>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Compute WP Switching Trigger/dsPIC Dot Product/Environment Controller'
 * '<S269>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Compute WP Switching Trigger/dsPIC_ABS/Environment Controller'
 * '<S270>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Compute WP Switching Trigger/dsPIC_SQRT/Environment Controller'
 * '<S271>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Compute WP Switching Trigger/dsPIC_SQRT/negprotect'
 * '<S272>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/Compute Frenet'
 * '<S273>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/Zero out Z1'
 * '<S274>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/Zero out Z2'
 * '<S275>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/Zero out Z3'
 * '<S276>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index1'
 * '<S277>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index2'
 * '<S278>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/Compute Frenet/Normalize Vector'
 * '<S279>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/Compute Frenet/Normalize Vector '
 * '<S280>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/Compute Frenet/Select N  Terms'
 * '<S281>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/Compute Frenet/Normalize Vector/Vector Norm'
 * '<S282>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/Compute Frenet/Normalize Vector/Vector Norm/dsPIC Dot Product'
 * '<S283>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/Compute Frenet/Normalize Vector/Vector Norm/dsPIC_SQRT'
 * '<S284>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/Compute Frenet/Normalize Vector/Vector Norm/dsPIC Dot Product/Embedded MATLAB Function'
 * '<S285>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/Compute Frenet/Normalize Vector/Vector Norm/dsPIC Dot Product/Environment Controller'
 * '<S286>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/Compute Frenet/Normalize Vector/Vector Norm/dsPIC_SQRT/Environment Controller'
 * '<S287>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/Compute Frenet/Normalize Vector/Vector Norm/dsPIC_SQRT/negprotect'
 * '<S288>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/Compute Frenet/Normalize Vector /Vector Norm'
 * '<S289>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/Compute Frenet/Normalize Vector /Vector Norm/dsPIC Dot Product'
 * '<S290>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/Compute Frenet/Normalize Vector /Vector Norm/dsPIC_SQRT'
 * '<S291>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/Compute Frenet/Normalize Vector /Vector Norm/dsPIC Dot Product/Embedded MATLAB Function'
 * '<S292>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/Compute Frenet/Normalize Vector /Vector Norm/dsPIC Dot Product/Environment Controller'
 * '<S293>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/Compute Frenet/Normalize Vector /Vector Norm/dsPIC_SQRT/Environment Controller'
 * '<S294>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/Compute Frenet/Normalize Vector /Vector Norm/dsPIC_SQRT/negprotect'
 * '<S295>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index1/Environment Controller1'
 * '<S296>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index1/Geod2LTP2'
 * '<S297>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index1/Get WP from idx SIM'
 * '<S298>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index1/Geod2LTP2/ECEF2LTP (UEN)'
 * '<S299>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index1/Geod2LTP2/Geod2ECEF1'
 * '<S300>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index1/Geod2LTP2/ECEF2LTP (UEN)/From Body to Universal1'
 * '<S301>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index1/Geod2LTP2/ECEF2LTP (UEN)/From Body to Universal1/Rotation Matrix from Long////Lat'
 * '<S302>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index1/Geod2LTP2/Geod2ECEF1/eccentricity^2'
 * '<S303>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index1/Get WP from idx SIM/Geod2LTP1'
 * '<S304>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index1/Get WP from idx SIM/Zero out Z1'
 * '<S305>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index1/Get WP from idx SIM/Zero out Z2'
 * '<S306>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index1/Get WP from idx SIM/Zero out Z3'
 * '<S307>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index1/Get WP from idx SIM/Geod2LTP1/ECEF2LTP (UEN)'
 * '<S308>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index1/Get WP from idx SIM/Geod2LTP1/Geod2ECEF1'
 * '<S309>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index1/Get WP from idx SIM/Geod2LTP1/ECEF2LTP (UEN)/From Body to Universal1'
 * '<S310>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index1/Get WP from idx SIM/Geod2LTP1/ECEF2LTP (UEN)/From Body to Universal1/Rotation Matrix from Long////Lat'
 * '<S311>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index1/Get WP from idx SIM/Geod2LTP1/Geod2ECEF1/eccentricity^2'
 * '<S312>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index2/Environment Controller1'
 * '<S313>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index2/Geod2LTP2'
 * '<S314>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index2/Get WP from idx SIM'
 * '<S315>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index2/Geod2LTP2/ECEF2LTP (UEN)'
 * '<S316>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index2/Geod2LTP2/Geod2ECEF1'
 * '<S317>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index2/Geod2LTP2/ECEF2LTP (UEN)/From Body to Universal1'
 * '<S318>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index2/Geod2LTP2/ECEF2LTP (UEN)/From Body to Universal1/Rotation Matrix from Long////Lat'
 * '<S319>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index2/Geod2LTP2/Geod2ECEF1/eccentricity^2'
 * '<S320>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index2/Get WP from idx SIM/Geod2LTP1'
 * '<S321>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index2/Get WP from idx SIM/Zero out Z1'
 * '<S322>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index2/Get WP from idx SIM/Zero out Z2'
 * '<S323>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index2/Get WP from idx SIM/Zero out Z3'
 * '<S324>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index2/Get WP from idx SIM/Geod2LTP1/ECEF2LTP (UEN)'
 * '<S325>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index2/Get WP from idx SIM/Geod2LTP1/Geod2ECEF1'
 * '<S326>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index2/Get WP from idx SIM/Geod2LTP1/ECEF2LTP (UEN)/From Body to Universal1'
 * '<S327>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index2/Get WP from idx SIM/Geod2LTP1/ECEF2LTP (UEN)/From Body to Universal1/Rotation Matrix from Long////Lat'
 * '<S328>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index2/Get WP from idx SIM/Geod2LTP1/Geod2ECEF1/eccentricity^2'
 * '<S329>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem'
 * '<S330>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/Compute Frenet'
 * '<S331>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index'
 * '<S332>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index1'
 * '<S333>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/Compute Frenet/Normalize Vector'
 * '<S334>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/Compute Frenet/Normalize Vector '
 * '<S335>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/Compute Frenet/Select N  Terms'
 * '<S336>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/Compute Frenet/Normalize Vector/Vector Norm'
 * '<S337>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/Compute Frenet/Normalize Vector/Vector Norm/dsPIC Dot Product'
 * '<S338>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/Compute Frenet/Normalize Vector/Vector Norm/dsPIC_SQRT'
 * '<S339>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/Compute Frenet/Normalize Vector/Vector Norm/dsPIC Dot Product/Embedded MATLAB Function'
 * '<S340>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/Compute Frenet/Normalize Vector/Vector Norm/dsPIC Dot Product/Environment Controller'
 * '<S341>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/Compute Frenet/Normalize Vector/Vector Norm/dsPIC_SQRT/Environment Controller'
 * '<S342>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/Compute Frenet/Normalize Vector/Vector Norm/dsPIC_SQRT/negprotect'
 * '<S343>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/Compute Frenet/Normalize Vector /Vector Norm'
 * '<S344>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/Compute Frenet/Normalize Vector /Vector Norm/dsPIC Dot Product'
 * '<S345>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/Compute Frenet/Normalize Vector /Vector Norm/dsPIC_SQRT'
 * '<S346>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/Compute Frenet/Normalize Vector /Vector Norm/dsPIC Dot Product/Embedded MATLAB Function'
 * '<S347>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/Compute Frenet/Normalize Vector /Vector Norm/dsPIC Dot Product/Environment Controller'
 * '<S348>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/Compute Frenet/Normalize Vector /Vector Norm/dsPIC_SQRT/Environment Controller'
 * '<S349>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/Compute Frenet/Normalize Vector /Vector Norm/dsPIC_SQRT/negprotect'
 * '<S350>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index/Environment Controller1'
 * '<S351>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index/Geod2LTP2'
 * '<S352>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index/Get WP from idx SIM'
 * '<S353>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index/Geod2LTP2/ECEF2LTP (UEN)'
 * '<S354>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index/Geod2LTP2/Geod2ECEF1'
 * '<S355>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index/Geod2LTP2/ECEF2LTP (UEN)/From Body to Universal1'
 * '<S356>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index/Geod2LTP2/ECEF2LTP (UEN)/From Body to Universal1/Rotation Matrix from Long////Lat'
 * '<S357>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index/Geod2LTP2/Geod2ECEF1/eccentricity^2'
 * '<S358>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index/Get WP from idx SIM/Geod2LTP1'
 * '<S359>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index/Get WP from idx SIM/Zero out Z1'
 * '<S360>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index/Get WP from idx SIM/Zero out Z2'
 * '<S361>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index/Get WP from idx SIM/Zero out Z3'
 * '<S362>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index/Get WP from idx SIM/Geod2LTP1/ECEF2LTP (UEN)'
 * '<S363>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index/Get WP from idx SIM/Geod2LTP1/Geod2ECEF1'
 * '<S364>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index/Get WP from idx SIM/Geod2LTP1/ECEF2LTP (UEN)/From Body to Universal1'
 * '<S365>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index/Get WP from idx SIM/Geod2LTP1/ECEF2LTP (UEN)/From Body to Universal1/Rotation Matrix from Long////Lat'
 * '<S366>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index/Get WP from idx SIM/Geod2LTP1/Geod2ECEF1/eccentricity^2'
 * '<S367>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index1/Environment Controller1'
 * '<S368>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index1/Geod2LTP2'
 * '<S369>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index1/Get WP from idx SIM'
 * '<S370>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index1/Geod2LTP2/ECEF2LTP (UEN)'
 * '<S371>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index1/Geod2LTP2/Geod2ECEF1'
 * '<S372>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index1/Geod2LTP2/ECEF2LTP (UEN)/From Body to Universal1'
 * '<S373>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index1/Geod2LTP2/ECEF2LTP (UEN)/From Body to Universal1/Rotation Matrix from Long////Lat'
 * '<S374>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index1/Geod2LTP2/Geod2ECEF1/eccentricity^2'
 * '<S375>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index1/Get WP from idx SIM/Geod2LTP1'
 * '<S376>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index1/Get WP from idx SIM/Zero out Z1'
 * '<S377>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index1/Get WP from idx SIM/Zero out Z2'
 * '<S378>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index1/Get WP from idx SIM/Zero out Z3'
 * '<S379>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index1/Get WP from idx SIM/Geod2LTP1/ECEF2LTP (UEN)'
 * '<S380>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index1/Get WP from idx SIM/Geod2LTP1/Geod2ECEF1'
 * '<S381>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index1/Get WP from idx SIM/Geod2LTP1/ECEF2LTP (UEN)/From Body to Universal1'
 * '<S382>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index1/Get WP from idx SIM/Geod2LTP1/ECEF2LTP (UEN)/From Body to Universal1/Rotation Matrix from Long////Lat'
 * '<S383>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index1/Get WP from idx SIM/Geod2LTP1/Geod2ECEF1/eccentricity^2'
 * '<S384>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Vector Norm/dsPIC Dot Product'
 * '<S385>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Vector Norm/dsPIC_SQRT'
 * '<S386>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Vector Norm/dsPIC Dot Product/Embedded MATLAB Function'
 * '<S387>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Vector Norm/dsPIC Dot Product/Environment Controller'
 * '<S388>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Vector Norm/dsPIC_SQRT/Environment Controller'
 * '<S389>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Vector Norm/dsPIC_SQRT/negprotect'
 * '<S390>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/RTB//Follow Mobile Navigation/Compute Mobile Location'
 * '<S391>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/RTB//Follow Mobile Navigation/Simple Predictor'
 * '<S392>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/RTB//Follow Mobile Navigation/Vector Norm'
 * '<S393>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/RTB//Follow Mobile Navigation/Zero out Z1'
 * '<S394>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/RTB//Follow Mobile Navigation/Compute Mobile Location/Geod2LTP'
 * '<S395>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/RTB//Follow Mobile Navigation/Compute Mobile Location/Geod2LTP/ECEF2LTP (UEN)'
 * '<S396>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/RTB//Follow Mobile Navigation/Compute Mobile Location/Geod2LTP/Geod2ECEF1'
 * '<S397>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/RTB//Follow Mobile Navigation/Compute Mobile Location/Geod2LTP/ECEF2LTP (UEN)/From Body to Universal1'
 * '<S398>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/RTB//Follow Mobile Navigation/Compute Mobile Location/Geod2LTP/ECEF2LTP (UEN)/From Body to Universal1/Rotation Matrix from Long////Lat'
 * '<S399>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/RTB//Follow Mobile Navigation/Compute Mobile Location/Geod2LTP/Geod2ECEF1/eccentricity^2'
 * '<S400>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/RTB//Follow Mobile Navigation/Vector Norm/dsPIC Dot Product'
 * '<S401>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/RTB//Follow Mobile Navigation/Vector Norm/dsPIC_SQRT'
 * '<S402>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/RTB//Follow Mobile Navigation/Vector Norm/dsPIC Dot Product/Embedded MATLAB Function'
 * '<S403>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/RTB//Follow Mobile Navigation/Vector Norm/dsPIC Dot Product/Environment Controller'
 * '<S404>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/RTB//Follow Mobile Navigation/Vector Norm/dsPIC_SQRT/Environment Controller'
 * '<S405>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/RTB//Follow Mobile Navigation/Vector Norm/dsPIC_SQRT/negprotect'
 * '<S406>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Recent Enable/Grab Upon Enable'
 * '<S407>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Recent Enable/Subsystem'
 * '<S408>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Recent Enable/Grab Upon Enable/Compute GS Location'
 * '<S409>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Recent Enable/Grab Upon Enable/Environment Controller'
 * '<S410>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Recent Enable/Grab Upon Enable/Compute GS Location/Geod2ECEF1'
 * '<S411>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Recent Enable/Grab Upon Enable/Compute GS Location/Geod2ECEF1/eccentricity^2'
 * '<S412>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Recent Enable/Subsystem/Detect Rising Edge'
 * '<S413>' : 'controlMCUSlugsMKIINewNav/Pilot/Increment Real World'
 * '<S414>' : 'controlMCUSlugsMKIINewNav/Pilot/Wrap To Zero'
 * '<S415>' : 'controlMCUSlugsMKIINewNav/Trim Vals/Bufffer IC'
 * '<S416>' : 'controlMCUSlugsMKIINewNav/Trim Vals/Control Surface Input'
 * '<S417>' : 'controlMCUSlugsMKIINewNav/Trim Vals/Convert to Microseconds '
 * '<S418>' : 'controlMCUSlugsMKIINewNav/Trim Vals/Detect Transition High to Low'
 * '<S419>' : 'controlMCUSlugsMKIINewNav/Trim Vals/Grab I.C.'
 * '<S420>' : 'controlMCUSlugsMKIINewNav/Trim Vals/Mean Filter the Transition'
 * '<S421>' : 'controlMCUSlugsMKIINewNav/Trim Vals/Bufffer IC/Buffer IC Channel'
 * '<S422>' : 'controlMCUSlugsMKIINewNav/Trim Vals/Bufffer IC/Buffer IC Channel1'
 * '<S423>' : 'controlMCUSlugsMKIINewNav/Trim Vals/Bufffer IC/Buffer IC Channel2'
 * '<S424>' : 'controlMCUSlugsMKIINewNav/Trim Vals/Bufffer IC/Buffer IC Channel3'
 * '<S425>' : 'controlMCUSlugsMKIINewNav/Trim Vals/Control Surface Input/2 sec  tau'
 * '<S426>' : 'controlMCUSlugsMKIINewNav/Trim Vals/Control Surface Input/2 sec  tau1'
 * '<S427>' : 'controlMCUSlugsMKIINewNav/Trim Vals/Control Surface Input/myMux Fun1'
 * '<S428>' : 'controlMCUSlugsMKIINewNav/Trim Vals/Control Surface Input/myMux Fun5'
 * '<S429>' : 'controlMCUSlugsMKIINewNav/Trim Vals/Control Surface Input/2 sec  tau/Embedded MATLAB Function'
 * '<S430>' : 'controlMCUSlugsMKIINewNav/Trim Vals/Control Surface Input/2 sec  tau1/Embedded MATLAB Function'
 * '<S431>' : 'controlMCUSlugsMKIINewNav/Trim Vals/Detect Transition High to Low/Subsystem'
 * '<S432>' : 'controlMCUSlugsMKIINewNav/Trim Vals/Grab I.C./Convert to Microseconds '
 * '<S433>' : 'controlMCUSlugsMKIINewNav/Trim Vals/Grab I.C./Convert to Microseconds 1'
 * '<S434>' : 'controlMCUSlugsMKIINewNav/Trim Vals/Grab I.C./Convert to Microseconds 2'
 * '<S435>' : 'controlMCUSlugsMKIINewNav/Trim Vals/Grab I.C./Convert to Microseconds 3'
 * '<S436>' : 'controlMCUSlugsMKIINewNav/Trim Vals/Grab I.C./Convert to Rad'
 * '<S437>' : 'controlMCUSlugsMKIINewNav/Trim Vals/Grab I.C./Convert to Rad1'
 * '<S438>' : 'controlMCUSlugsMKIINewNav/Trim Vals/Grab I.C./Convert to Rad2'
 * '<S439>' : 'controlMCUSlugsMKIINewNav/Trim Vals/Grab I.C./Convert to Rad3'
 * '<S440>' : 'controlMCUSlugsMKIINewNav/Trim Vals/Mean Filter the Transition/Buffer Failsafe Channel'
 * '<S441>' : 'controlMCUSlugsMKIINewNav/Update PWM Commands and Send Telemetry/Convert to Microseconds'
 * '<S442>' : 'controlMCUSlugsMKIINewNav/Update PWM Commands and Send Telemetry/Generate PWM Signals Based on the Control Type'
 * '<S443>' : 'controlMCUSlugsMKIINewNav/Update PWM Commands and Send Telemetry/PTZ Unit'
 * '<S444>' : 'controlMCUSlugsMKIINewNav/Update PWM Commands and Send Telemetry/Generate PWM Signals Based on the Control Type/If  Control Type Is Manual'
 * '<S445>' : 'controlMCUSlugsMKIINewNav/Update PWM Commands and Send Telemetry/Generate PWM Signals Based on the Control Type/If  Control Type Is Passthrough'
 * '<S446>' : 'controlMCUSlugsMKIINewNav/Update PWM Commands and Send Telemetry/Generate PWM Signals Based on the Control Type/If  Control Type Is Selective Passthrough'
 * '<S447>' : 'controlMCUSlugsMKIINewNav/Update PWM Commands and Send Telemetry/Generate PWM Signals Based on the Control Type/If  Control Type Is Wp, ISR, LP or  Mid Lvl Cmds'
 * '<S448>' : 'controlMCUSlugsMKIINewNav/Update PWM Commands and Send Telemetry/Generate PWM Signals Based on the Control Type/If  Control Type Is Selective Passthrough/Convert to PWM'
 * '<S449>' : 'controlMCUSlugsMKIINewNav/Update PWM Commands and Send Telemetry/Generate PWM Signals Based on the Control Type/If  Control Type Is Selective Passthrough/Convert to PWM1'
 * '<S450>' : 'controlMCUSlugsMKIINewNav/Update PWM Commands and Send Telemetry/Generate PWM Signals Based on the Control Type/If  Control Type Is Selective Passthrough/Convert to PWM2'
 * '<S451>' : 'controlMCUSlugsMKIINewNav/Update PWM Commands and Send Telemetry/Generate PWM Signals Based on the Control Type/If  Control Type Is Selective Passthrough/Convert to PWM3'
 * '<S452>' : 'controlMCUSlugsMKIINewNav/Update PWM Commands and Send Telemetry/Generate PWM Signals Based on the Control Type/If  Control Type Is Selective Passthrough/myMux Fun1'
 * '<S453>' : 'controlMCUSlugsMKIINewNav/Update PWM Commands and Send Telemetry/Generate PWM Signals Based on the Control Type/If  Control Type Is Wp, ISR, LP or  Mid Lvl Cmds/Convert to PWM'
 * '<S454>' : 'controlMCUSlugsMKIINewNav/Update PWM Commands and Send Telemetry/Generate PWM Signals Based on the Control Type/If  Control Type Is Wp, ISR, LP or  Mid Lvl Cmds/Convert to PWM1'
 * '<S455>' : 'controlMCUSlugsMKIINewNav/Update PWM Commands and Send Telemetry/Generate PWM Signals Based on the Control Type/If  Control Type Is Wp, ISR, LP or  Mid Lvl Cmds/Convert to PWM2'
 * '<S456>' : 'controlMCUSlugsMKIINewNav/Update PWM Commands and Send Telemetry/Generate PWM Signals Based on the Control Type/If  Control Type Is Wp, ISR, LP or  Mid Lvl Cmds/Convert to PWM3'
 * '<S457>' : 'controlMCUSlugsMKIINewNav/Update PWM Commands and Send Telemetry/Generate PWM Signals Based on the Control Type/If  Control Type Is Wp, ISR, LP or  Mid Lvl Cmds/myMux Fun1'
 * '<S458>' : 'controlMCUSlugsMKIINewNav/Update PWM Commands and Send Telemetry/PTZ Unit/AutoTrack ISR?'
 * '<S459>' : 'controlMCUSlugsMKIINewNav/Update PWM Commands and Send Telemetry/PTZ Unit/Camera Stab'
 * '<S460>' : 'controlMCUSlugsMKIINewNav/Update PWM Commands and Send Telemetry/PTZ Unit/Camera Track'
 * '<S461>' : 'controlMCUSlugsMKIINewNav/Update PWM Commands and Send Telemetry/PTZ Unit/Pan'
 * '<S462>' : 'controlMCUSlugsMKIINewNav/Update PWM Commands and Send Telemetry/PTZ Unit/Tilt'
 * '<S463>' : 'controlMCUSlugsMKIINewNav/Update PWM Commands and Send Telemetry/PTZ Unit/myMux Fun1'
 * '<S464>' : 'controlMCUSlugsMKIINewNav/Update PWM Commands and Send Telemetry/PTZ Unit/Camera Stab/Convert to PWM3'
 * '<S465>' : 'controlMCUSlugsMKIINewNav/Update PWM Commands and Send Telemetry/PTZ Unit/Camera Stab/Roll Compensation'
 * '<S466>' : 'controlMCUSlugsMKIINewNav/Update PWM Commands and Send Telemetry/PTZ Unit/Camera Stab/Tustin Lowpass, Auto Initial Condition1'
 * '<S467>' : 'controlMCUSlugsMKIINewNav/Update PWM Commands and Send Telemetry/PTZ Unit/Camera Stab/Roll Compensation/Compare To 20 Deg'
 * '<S468>' : 'controlMCUSlugsMKIINewNav/Update PWM Commands and Send Telemetry/PTZ Unit/Camera Stab/Roll Compensation/Convert to OC1'
 * '<S469>' : 'controlMCUSlugsMKIINewNav/Update PWM Commands and Send Telemetry/PTZ Unit/Camera Stab/Roll Compensation/Convert to PWM2'
 * '<S470>' : 'controlMCUSlugsMKIINewNav/Update PWM Commands and Send Telemetry/PTZ Unit/Camera Stab/Roll Compensation/Tustin Lowpass, Auto Initial Condition2'
 * '<S471>' : 'controlMCUSlugsMKIINewNav/Update PWM Commands and Send Telemetry/PTZ Unit/Camera Stab/Roll Compensation/dsPIC_ABS'
 * '<S472>' : 'controlMCUSlugsMKIINewNav/Update PWM Commands and Send Telemetry/PTZ Unit/Camera Stab/Roll Compensation/Tustin Lowpass, Auto Initial Condition2/Embedded MATLAB Function'
 * '<S473>' : 'controlMCUSlugsMKIINewNav/Update PWM Commands and Send Telemetry/PTZ Unit/Camera Stab/Roll Compensation/dsPIC_ABS/Environment Controller'
 * '<S474>' : 'controlMCUSlugsMKIINewNav/Update PWM Commands and Send Telemetry/PTZ Unit/Camera Stab/Tustin Lowpass, Auto Initial Condition1/Embedded MATLAB Function'
 * '<S475>' : 'controlMCUSlugsMKIINewNav/Update PWM Commands and Send Telemetry/PTZ Unit/Camera Track/Convert to PWM2'
 * '<S476>' : 'controlMCUSlugsMKIINewNav/Update PWM Commands and Send Telemetry/PTZ Unit/Camera Track/Convert to PWM4'
 * '<S477>' : 'controlMCUSlugsMKIINewNav/Update PWM Commands and Send Telemetry/PTZ Unit/Camera Track/LPF the Euler and Construct Matrix'
 * '<S478>' : 'controlMCUSlugsMKIINewNav/Update PWM Commands and Send Telemetry/PTZ Unit/Camera Track/Transpose1'
 * '<S479>' : 'controlMCUSlugsMKIINewNav/Update PWM Commands and Send Telemetry/PTZ Unit/Camera Track/Vector Norm'
 * '<S480>' : 'controlMCUSlugsMKIINewNav/Update PWM Commands and Send Telemetry/PTZ Unit/Camera Track/dsPIC_asin'
 * '<S481>' : 'controlMCUSlugsMKIINewNav/Update PWM Commands and Send Telemetry/PTZ Unit/Camera Track/dsPIC_atan2'
 * '<S482>' : 'controlMCUSlugsMKIINewNav/Update PWM Commands and Send Telemetry/PTZ Unit/Camera Track/LPF the Euler and Construct Matrix/Euler Angles to  Direction Cosine Matrix'
 * '<S483>' : 'controlMCUSlugsMKIINewNav/Update PWM Commands and Send Telemetry/PTZ Unit/Camera Track/LPF the Euler and Construct Matrix/Tustin Lowpass, Auto Initial Condition1'
 * '<S484>' : 'controlMCUSlugsMKIINewNav/Update PWM Commands and Send Telemetry/PTZ Unit/Camera Track/LPF the Euler and Construct Matrix/Tustin Lowpass, Auto Initial Condition2'
 * '<S485>' : 'controlMCUSlugsMKIINewNav/Update PWM Commands and Send Telemetry/PTZ Unit/Camera Track/LPF the Euler and Construct Matrix/Tustin Lowpass, Auto Initial Condition3'
 * '<S486>' : 'controlMCUSlugsMKIINewNav/Update PWM Commands and Send Telemetry/PTZ Unit/Camera Track/LPF the Euler and Construct Matrix/myMux Fun5'
 * '<S487>' : 'controlMCUSlugsMKIINewNav/Update PWM Commands and Send Telemetry/PTZ Unit/Camera Track/LPF the Euler and Construct Matrix/Euler Angles to  Direction Cosine Matrix/A11'
 * '<S488>' : 'controlMCUSlugsMKIINewNav/Update PWM Commands and Send Telemetry/PTZ Unit/Camera Track/LPF the Euler and Construct Matrix/Euler Angles to  Direction Cosine Matrix/A12'
 * '<S489>' : 'controlMCUSlugsMKIINewNav/Update PWM Commands and Send Telemetry/PTZ Unit/Camera Track/LPF the Euler and Construct Matrix/Euler Angles to  Direction Cosine Matrix/A13'
 * '<S490>' : 'controlMCUSlugsMKIINewNav/Update PWM Commands and Send Telemetry/PTZ Unit/Camera Track/LPF the Euler and Construct Matrix/Euler Angles to  Direction Cosine Matrix/A21'
 * '<S491>' : 'controlMCUSlugsMKIINewNav/Update PWM Commands and Send Telemetry/PTZ Unit/Camera Track/LPF the Euler and Construct Matrix/Euler Angles to  Direction Cosine Matrix/A22'
 * '<S492>' : 'controlMCUSlugsMKIINewNav/Update PWM Commands and Send Telemetry/PTZ Unit/Camera Track/LPF the Euler and Construct Matrix/Euler Angles to  Direction Cosine Matrix/A23'
 * '<S493>' : 'controlMCUSlugsMKIINewNav/Update PWM Commands and Send Telemetry/PTZ Unit/Camera Track/LPF the Euler and Construct Matrix/Euler Angles to  Direction Cosine Matrix/A31'
 * '<S494>' : 'controlMCUSlugsMKIINewNav/Update PWM Commands and Send Telemetry/PTZ Unit/Camera Track/LPF the Euler and Construct Matrix/Euler Angles to  Direction Cosine Matrix/A32'
 * '<S495>' : 'controlMCUSlugsMKIINewNav/Update PWM Commands and Send Telemetry/PTZ Unit/Camera Track/LPF the Euler and Construct Matrix/Euler Angles to  Direction Cosine Matrix/A33'
 * '<S496>' : 'controlMCUSlugsMKIINewNav/Update PWM Commands and Send Telemetry/PTZ Unit/Camera Track/LPF the Euler and Construct Matrix/Euler Angles to  Direction Cosine Matrix/Create Transformation Matrix'
 * '<S497>' : 'controlMCUSlugsMKIINewNav/Update PWM Commands and Send Telemetry/PTZ Unit/Camera Track/LPF the Euler and Construct Matrix/Tustin Lowpass, Auto Initial Condition1/Embedded MATLAB Function'
 * '<S498>' : 'controlMCUSlugsMKIINewNav/Update PWM Commands and Send Telemetry/PTZ Unit/Camera Track/LPF the Euler and Construct Matrix/Tustin Lowpass, Auto Initial Condition2/Embedded MATLAB Function'
 * '<S499>' : 'controlMCUSlugsMKIINewNav/Update PWM Commands and Send Telemetry/PTZ Unit/Camera Track/LPF the Euler and Construct Matrix/Tustin Lowpass, Auto Initial Condition3/Embedded MATLAB Function'
 * '<S500>' : 'controlMCUSlugsMKIINewNav/Update PWM Commands and Send Telemetry/PTZ Unit/Camera Track/Vector Norm/dsPIC Dot Product'
 * '<S501>' : 'controlMCUSlugsMKIINewNav/Update PWM Commands and Send Telemetry/PTZ Unit/Camera Track/Vector Norm/dsPIC_SQRT'
 * '<S502>' : 'controlMCUSlugsMKIINewNav/Update PWM Commands and Send Telemetry/PTZ Unit/Camera Track/Vector Norm/dsPIC Dot Product/Embedded MATLAB Function'
 * '<S503>' : 'controlMCUSlugsMKIINewNav/Update PWM Commands and Send Telemetry/PTZ Unit/Camera Track/Vector Norm/dsPIC Dot Product/Environment Controller'
 * '<S504>' : 'controlMCUSlugsMKIINewNav/Update PWM Commands and Send Telemetry/PTZ Unit/Camera Track/Vector Norm/dsPIC_SQRT/Environment Controller'
 * '<S505>' : 'controlMCUSlugsMKIINewNav/Update PWM Commands and Send Telemetry/PTZ Unit/Camera Track/Vector Norm/dsPIC_SQRT/negprotect'
 * '<S506>' : 'controlMCUSlugsMKIINewNav/Update PWM Commands and Send Telemetry/PTZ Unit/Camera Track/dsPIC_asin/Environment Controller'
 * '<S507>' : 'controlMCUSlugsMKIINewNav/Update PWM Commands and Send Telemetry/PTZ Unit/Camera Track/dsPIC_atan2/Environment Controller'
 * '<S508>' : 'controlMCUSlugsMKIINewNav/Update PWM Commands and Send Telemetry/PTZ Unit/Pan/Angle Conversion'
 * '<S509>' : 'controlMCUSlugsMKIINewNav/Update PWM Commands and Send Telemetry/PTZ Unit/Pan/Convert to OC1'
 * '<S510>' : 'controlMCUSlugsMKIINewNav/Update PWM Commands and Send Telemetry/PTZ Unit/Tilt/Angle Conversion'
 * '<S511>' : 'controlMCUSlugsMKIINewNav/Update PWM Commands and Send Telemetry/PTZ Unit/Tilt/Convert to OC1'
 * '<S512>' : 'controlMCUSlugsMKIINewNav/get Nav Vars/Angle Conversion'
 * '<S513>' : 'controlMCUSlugsMKIINewNav/get Nav Vars/Direction Cosine Matrix to Rotation Angles'
 * '<S514>' : 'controlMCUSlugsMKIINewNav/get Nav Vars/Embedded MATLAB Function1'
 * '<S515>' : 'controlMCUSlugsMKIINewNav/get Nav Vars/Euler Angles to  Direction Cosine Matrix1'
 * '<S516>' : 'controlMCUSlugsMKIINewNav/get Nav Vars/Euler Angles to  Direction Cosine Matrix2'
 * '<S517>' : 'controlMCUSlugsMKIINewNav/get Nav Vars/myMux Fun1'
 * '<S518>' : 'controlMCUSlugsMKIINewNav/get Nav Vars/myMux Fun2'
 * '<S519>' : 'controlMCUSlugsMKIINewNav/get Nav Vars/Direction Cosine Matrix to Rotation Angles/AxisRotDefault'
 * '<S520>' : 'controlMCUSlugsMKIINewNav/get Nav Vars/Direction Cosine Matrix to Rotation Angles/AxisRotZeroR3'
 * '<S521>' : 'controlMCUSlugsMKIINewNav/get Nav Vars/Direction Cosine Matrix to Rotation Angles/Get DCM Values'
 * '<S522>' : 'controlMCUSlugsMKIINewNav/get Nav Vars/Euler Angles to  Direction Cosine Matrix1/A11'
 * '<S523>' : 'controlMCUSlugsMKIINewNav/get Nav Vars/Euler Angles to  Direction Cosine Matrix1/A12'
 * '<S524>' : 'controlMCUSlugsMKIINewNav/get Nav Vars/Euler Angles to  Direction Cosine Matrix1/A13'
 * '<S525>' : 'controlMCUSlugsMKIINewNav/get Nav Vars/Euler Angles to  Direction Cosine Matrix1/A21'
 * '<S526>' : 'controlMCUSlugsMKIINewNav/get Nav Vars/Euler Angles to  Direction Cosine Matrix1/A22'
 * '<S527>' : 'controlMCUSlugsMKIINewNav/get Nav Vars/Euler Angles to  Direction Cosine Matrix1/A23'
 * '<S528>' : 'controlMCUSlugsMKIINewNav/get Nav Vars/Euler Angles to  Direction Cosine Matrix1/A31'
 * '<S529>' : 'controlMCUSlugsMKIINewNav/get Nav Vars/Euler Angles to  Direction Cosine Matrix1/A32'
 * '<S530>' : 'controlMCUSlugsMKIINewNav/get Nav Vars/Euler Angles to  Direction Cosine Matrix1/A33'
 * '<S531>' : 'controlMCUSlugsMKIINewNav/get Nav Vars/Euler Angles to  Direction Cosine Matrix1/Create Transformation Matrix'
 * '<S532>' : 'controlMCUSlugsMKIINewNav/get Nav Vars/Euler Angles to  Direction Cosine Matrix2/A11'
 * '<S533>' : 'controlMCUSlugsMKIINewNav/get Nav Vars/Euler Angles to  Direction Cosine Matrix2/A12'
 * '<S534>' : 'controlMCUSlugsMKIINewNav/get Nav Vars/Euler Angles to  Direction Cosine Matrix2/A13'
 * '<S535>' : 'controlMCUSlugsMKIINewNav/get Nav Vars/Euler Angles to  Direction Cosine Matrix2/A21'
 * '<S536>' : 'controlMCUSlugsMKIINewNav/get Nav Vars/Euler Angles to  Direction Cosine Matrix2/A22'
 * '<S537>' : 'controlMCUSlugsMKIINewNav/get Nav Vars/Euler Angles to  Direction Cosine Matrix2/A23'
 * '<S538>' : 'controlMCUSlugsMKIINewNav/get Nav Vars/Euler Angles to  Direction Cosine Matrix2/A31'
 * '<S539>' : 'controlMCUSlugsMKIINewNav/get Nav Vars/Euler Angles to  Direction Cosine Matrix2/A32'
 * '<S540>' : 'controlMCUSlugsMKIINewNav/get Nav Vars/Euler Angles to  Direction Cosine Matrix2/A33'
 * '<S541>' : 'controlMCUSlugsMKIINewNav/get Nav Vars/Euler Angles to  Direction Cosine Matrix2/Create Transformation Matrix'
 */
#endif                                 /* RTW_HEADER_controlMCUSlugsMKIINewNav_h_ */

/* File trailer for Real-Time Workshop generated code.
 *
 * [EOF]
 */
