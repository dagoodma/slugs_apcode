/*
 * File: controlMCUSlugsMKIINewNav.h
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

/* Block signals for system '<S113>/negprotect' */
typedef struct {
  real32_T zpVal;                      /* '<S113>/negprotect' */
} rtB_negprotect_controlMCUSlugsM;

/* Block signals for system '<S399>/Embedded MATLAB Function' */
typedef struct {
  real32_T xDoty;                      /* '<S399>/Embedded MATLAB Function' */
} rtB_EmbeddedMATLABFunction_co_h;

/* Block signals for system '<S135>/Zero out Z1' */
typedef struct {
  real32_T P[3];                       /* '<S135>/Zero out Z1' */
} rtB_ZerooutZ1_controlMCUSlugsMK;

/* Block signals for system '<S263>/Embedded MATLAB Function' */
typedef struct {
  real32_T xDoty;                      /* '<S263>/Embedded MATLAB Function' */
} rtB_EmbeddedMATLABFunction_co_k;

/* Block signals for system '<S271>/Select N  Terms' */
typedef struct {
  real32_T N[3];                       /* '<S271>/Select N  Terms' */
} rtB_SelectNTerms_controlMCUSlug;

/* Block signals for system '<S158>/negprotect3' */
typedef struct {
  real32_T zpVal;                      /* '<S158>/negprotect3' */
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
  real32_T CFunctionCall_a;            /* '<S210>/C Function Call' */
  real32_T Merge1;                     /* '<S127>/Merge1' */
  real32_T Merge2;                     /* '<S127>/Merge2' */
  real32_T Merge[3];                   /* '<S127>/Merge' */
  real32_T CFunctionCall_p;            /* '<S229>/C Function Call' */
  real32_T Switch;                     /* '<S215>/Switch' */
  real32_T Switch2;                    /* '<S215>/Switch2' */
  real32_T CFunctionCall_k;            /* '<S219>/C Function Call' */
  real32_T CFunctionCall_i;            /* '<S218>/C Function Call' */
  real32_T Product;                    /* '<S217>/Product' */
  real32_T CFunctionCall_pf;           /* '<S235>/C Function Call' */
  real32_T y_d[3];                     /* '<S127>/myMux Fun2' */
  real32_T y_k[4];                     /* '<S127>/myMux Fun1' */
  real32_T GettheGSLocationupdateControlMC[3];/* '<S405>/Get the GS Location [updateControlMCUState.c]' */
  real32_T NumericalUnity[3];          /* '<S408>/Numerical Unity' */
  real32_T DataTypeConversion_ks[2];   /* '<S405>/Data Type Conversion' */
  real32_T DataTypeConversion1[3];     /* '<S405>/Data Type Conversion1' */
  real32_T UEN2NEU[3];                 /* '<S150>/UEN 2 NEU' */
  real32_T CFunctionCall_mk;           /* '<S197>/C Function Call' */
  real32_T Sum2;                       /* '<S128>/Sum2' */
  real32_T CFunctionCall_ox;           /* '<S149>/C Function Call' */
  real32_T CFunctionCall_p5;           /* '<S163>/C Function Call' */
  real32_T CFunctionCall_op;           /* '<S171>/C Function Call' */
  real32_T u1_h;                       /* '<S158>/[-1 1]' */
  real32_T CFunctionCall_l;            /* '<S168>/C Function Call' */
  real32_T CFunctionCall_aw;           /* '<S184>/C Function Call' */
  real32_T u1_m;                       /* '<S159>/[-1 1]' */
  real32_T CFunctionCall_j;            /* '<S181>/C Function Call' */
  real32_T CFunctionCall_pg;           /* '<S162>/C Function Call' */
  real32_T CFunctionCall_n;            /* '<S161>/C Function Call' */
  real32_T CFunctionCall_b;            /* '<S242>/C Function Call' */
  real32_T CFunctionCall_f5;           /* '<S384>/C Function Call' */
  real32_T CFunctionCall_bg;           /* '<S257>/C Function Call' */
  real32_T CFunctionCall_b2;           /* '<S258>/C Function Call' */
  real32_T GetWaypointCoordinatesnavSuppor[3];/* '<S330>/Get Waypoint Coordinates [navSupport.c]' */
  real32_T GetWaypointCoordinatesnavSupp_j[3];/* '<S331>/Get Waypoint Coordinates [navSupport.c]' */
  real32_T CFunctionCall_kc;           /* '<S337>/C Function Call' */
  real32_T CFunctionCall_mu;           /* '<S344>/C Function Call' */
  real32_T WP0L2IPT1[3];               /* '<S250>/Subtract' */
  real32_T GetWaypointCoordinatesnavSupp_n[3];/* '<S275>/Get Waypoint Coordinates [navSupport.c]' */
  real32_T UEN2NEU_f[3];               /* '<S295>/UEN 2 NEU' */
  real32_T GetWaypointCoordinatesnavSupp_c[3];/* '<S276>/Get Waypoint Coordinates [navSupport.c]' */
  real32_T UEN2NEU_l[3];               /* '<S312>/UEN 2 NEU' */
  real32_T CFunctionCall_av;           /* '<S282>/C Function Call' */
  real32_T CFunctionCall_ml;           /* '<S289>/C Function Call' */
  real32_T Product1_e;                 /* '<S249>/Product1' */
  real32_T CFunctionCall_fw;           /* '<S400>/C Function Call' */
  real32_T Reshape1[3];                /* '<S393>/Reshape1' */
  real32_T Add_c;                      /* '<S91>/Add' */
  real32_T CFunctionCall_mj;           /* '<S107>/C Function Call' */
  real32_T CFunctionCall_d;            /* '<S113>/C Function Call' */
  real32_T Switch3_as;                 /* '<S99>/Switch3' */
  real32_T GettheGSLocationupdateControl_g[3];/* '<S17>/Get the GS Location [updateControlMCUState.c]' */
  real32_T Switch3_bm;                 /* '<S100>/Switch3' */
  real32_T u060;                       /* '<S87>/[-60 60]' */
  real32_T apUtilsc_b;                 /* '<S102>/[apUtils.c]' */
  real32_T bankLimit;                  /* '<S82>/bank Limit' */
  real32_T CFunctionCall_nj;           /* '<S85>/C Function Call' */
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
  real32_T Switch3_o;                  /* '<S27>/Switch3' */
  real32_T Subtract_f;                 /* '<S29>/Subtract' */
  real32_T Merge_e;                    /* '<S26>/Merge' */
  real32_T PsiDotLimit;                /* '<S15>/Psi Dot  Limit' */
  real32_T Switch3_a3;                 /* '<S28>/Switch3' */
  real32_T Switch3_fi;                 /* '<S46>/Switch3' */
  real32_T Switch3_ik;                 /* '<S47>/Switch3' */
  real32_T T;                          /* '<S41>/-T' */
  real32_T CFunctionCall_dv;           /* '<S43>/C Function Call' */
  real32_T NumericalUnity_f;           /* '<S44>/Numerical Unity' */
  real32_T c_i;                        /* '<S41>/1-c' */
  real32_T T_m;                        /* '<S31>/-T' */
  real32_T CFunctionCall_ai;           /* '<S33>/C Function Call' */
  real32_T NumericalUnity_p;           /* '<S34>/Numerical Unity' */
  real32_T c_k;                        /* '<S31>/1-c' */
  int16_T y_o[3];                      /* '<S443>/myMux Fun1' */
  int16_T GettheAidePanFCBEX1000c;     /* '<S460>/Get the Aide Pan [FCBEX1000c]' */
  int16_T GettheAideTiltFCBEX1000c;    /* '<S460>/Get the Aide Tilt [FCBEX1000.c]' */
  int16_T y_e[3];                      /* '<S127>/myMux Fun4' */
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
  uint8_T DataTypeConversion1_n;       /* '<S406>/Data Type Conversion1' */
  uint8_T DetectRisingTransitionnavSupp_n;/* '<S406>/Detect Rising Transition [navSupport.c]' */
  uint8_T IC;                          /* '<S134>/IC' */
  uint8_T DetectRisingTransitionnavSupp_g;/* '<S251>/Detect Rising Transition [navSupport.c]' */
  uint8_T WP0;                         /* '<S134>/computeCurrentWP' */
  uint8_T WP1;                         /* '<S134>/computeCurrentWP' */
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
  rtB_myMuxFun1_controlMCUSlugsMK sf_myMuxFun5_e;/* '<S127>/myMux Fun5' */
  rtB_ZerooutZ1_controlMCUSlugsMK sf_ZerooutZ2;/* '<S127>/Zero out Z2' */
  rtB_ZerooutZ1_controlMCUSlugsMK sf_ZerooutZ1;/* '<S127>/Zero out Z1' */
  rtB_negprotect3_controlMCUSlugs sf_negprotect_n4;/* '<S216>/negprotect' */
  rtB_negprotect3_controlMCUSlugs sf_negprotect_d;/* '<S215>/negprotect' */
  rtB_ZerooutZ1_controlMCUSlugsMK sf_ZerooutZ1_g;/* '<S223>/Zero out Z1' */
  rtB_negprotect_controlMCUSlugsM sf_negprotect_i;/* '<S229>/negprotect' */
  rtB_EmbeddedMATLABFunction_co_h sf_EmbeddedMATLABFunction_d;/* '<S228>/Embedded MATLAB Function' */
  rtB_negprotect_controlMCUSlugsM sf_negprotect_p;/* '<S210>/negprotect' */
  rtB_EmbeddedMATLABFunction_co_h sf_EmbeddedMATLABFunction_et;/* '<S209>/Embedded MATLAB Function' */
  rtB_negprotect_controlMCUSlugsM sf_negprotect_dl;/* '<S197>/negprotect' */
  rtB_EmbeddedMATLABFunction_co_h sf_EmbeddedMATLABFunction_p;/* '<S196>/Embedded MATLAB Function' */
  rtB_negprotect3_controlMCUSlugs sf_negprotect2;/* '<S146>/negprotect2' */
  rtB_negprotect3_controlMCUSlugs sf_negprotect1;/* '<S146>/negprotect1' */
  rtB_negprotect_controlMCUSlugsM sf_negprotect_dz;/* '<S163>/negprotect' */
  rtB_negprotect3_controlMCUSlugs sf_negprotect3_b;/* '<S159>/negprotect3' */
  rtB_EmbeddedMATLABFunction_co_k sf_EmbeddedMATLABFunction_ip;/* '<S180>/Embedded MATLAB Function' */
  rtB_negprotect_controlMCUSlugsM sf_negprotect_l0;/* '<S184>/negprotect' */
  rtB_EmbeddedMATLABFunction_co_h sf_EmbeddedMATLABFunction_py;/* '<S183>/Embedded MATLAB Function' */
  rtB_negprotect3_controlMCUSlugs sf_negprotect3;/* '<S158>/negprotect3' */
  rtB_EmbeddedMATLABFunction_co_k sf_EmbeddedMATLABFunction_ls;/* '<S167>/Embedded MATLAB Function' */
  rtB_negprotect_controlMCUSlugsM sf_negprotect_ht;/* '<S171>/negprotect' */
  rtB_EmbeddedMATLABFunction_co_h sf_EmbeddedMATLABFunction_eq;/* '<S170>/Embedded MATLAB Function' */
  rtB_ZerooutZ1_controlMCUSlugsMK sf_ZerooutZ2_a;/* '<S144>/Zero out Z2' */
  rtB_negprotect_controlMCUSlugsM sf_negprotect_a;/* '<S242>/negprotect' */
  rtB_EmbeddedMATLABFunction_co_h sf_EmbeddedMATLABFunction_dr;/* '<S241>/Embedded MATLAB Function' */
  rtB_ZerooutZ1_controlMCUSlugsMK sf_ZerooutZ2_e;/* '<S134>/Zero out Z2' */
  rtB_negprotect_controlMCUSlugsM sf_negprotect_dg;/* '<S384>/negprotect' */
  rtB_EmbeddedMATLABFunction_co_h sf_EmbeddedMATLABFunction_n;/* '<S383>/Embedded MATLAB Function' */
  rtB_SelectNTerms_controlMCUSlug sf_SelectNTerms_a;/* '<S329>/Select N  Terms' */
  rtB_negprotect_controlMCUSlugsM sf_negprotect_h;/* '<S344>/negprotect' */
  rtB_EmbeddedMATLABFunction_co_h sf_EmbeddedMATLABFunction_l;/* '<S343>/Embedded MATLAB Function' */
  rtB_negprotect_controlMCUSlugsM sf_negprotect_l;/* '<S337>/negprotect' */
  rtB_EmbeddedMATLABFunction_co_h sf_EmbeddedMATLABFunction_f;/* '<S336>/Embedded MATLAB Function' */
  rtB_ZerooutZ1_controlMCUSlugsMK sf_ZerooutZ3;/* '<S249>/Zero out Z3' */
  rtB_ZerooutZ1_controlMCUSlugsMK sf_ZerooutZ2_k;/* '<S249>/Zero out Z2' */
  rtB_ZerooutZ1_controlMCUSlugsMK sf_ZerooutZ1_l;/* '<S249>/Zero out Z1' */
  rtB_SelectNTerms_controlMCUSlug sf_SelectNTerms;/* '<S271>/Select N  Terms' */
  rtB_negprotect_controlMCUSlugsM sf_negprotect_k;/* '<S289>/negprotect' */
  rtB_EmbeddedMATLABFunction_co_h sf_EmbeddedMATLABFunction_j5;/* '<S288>/Embedded MATLAB Function' */
  rtB_negprotect_controlMCUSlugsM sf_negprotect_nk;/* '<S282>/negprotect' */
  rtB_EmbeddedMATLABFunction_co_h sf_EmbeddedMATLABFunction_i4;/* '<S281>/Embedded MATLAB Function' */
  rtB_negprotect_controlMCUSlugsM sf_negprotect_d0;/* '<S258>/negprotect' */
  rtB_EmbeddedMATLABFunction_co_k sf_EmbeddedMATLABFunction_bh;/* '<S256>/Embedded MATLAB Function' */
  rtB_EmbeddedMATLABFunction_co_k sf_EmbeddedMATLABFunction_do;/* '<S263>/Embedded MATLAB Function' */
  rtB_ZerooutZ1_controlMCUSlugsMK sf_ZerooutZ1_o;/* '<S135>/Zero out Z1' */
  rtB_negprotect_controlMCUSlugsM sf_negprotect_nu;/* '<S400>/negprotect' */
  rtB_EmbeddedMATLABFunction_co_h sf_EmbeddedMATLABFunction_o;/* '<S399>/Embedded MATLAB Function' */
  rtB_myMuxFun1_controlMCUSlugsMK sf_myMuxFun1_i;/* '<S17>/myMux Fun1' */
  rtB_negprotect_controlMCUSlugsM sf_negprotect_n;/* '<S113>/negprotect' */
  rtB_EmbeddedMATLABFunction_cont sf_EmbeddedMATLABFunction_a;/* '<S90>/Embedded MATLAB Function' */
  rtB_EmbeddedMATLABFunction_cont sf_EmbeddedMATLABFunction_e;/* '<S89>/Embedded MATLAB Function' */
  rtB_myMuxFun1_controlMCUSlugsMK sf_myMuxFun1_f;/* '<S16>/myMux Fun1' */
  rtB_EmbeddedMATLABFunction_cont sf_EmbeddedMATLABFunction_b;/* '<S51>/Embedded MATLAB Function' */
} BlockIO_controlMCUSlugsMKIINewN;

/* Block states (auto storage) for system '<Root>' */
typedef struct {
  real_T Add3_DWORK1;                  /* '<S94>/Add3' */
  real32_T IntegerDelay3_DSTATE;       /* '<S19>/Integer Delay3' */
  real32_T IntegerDelay3_DSTATE_i;     /* '<S20>/Integer Delay3' */
  real32_T IntegerDelay1_DSTATE;       /* '<S144>/Integer Delay1' */
  real32_T IntegerDelay_DSTATE;        /* '<S128>/Integer Delay' */
  real32_T IntegerDelay3_DSTATE_l;     /* '<S262>/Integer Delay3' */
  real32_T IntegerDelay1_DSTATE_p;     /* '<S249>/Integer Delay1' */
  real32_T DiscreteTimeIntegrator_DSTATE[3];/* '<S390>/Discrete-Time Integrator' */
  real32_T IntegerDelay3_DSTATE_k;     /* '<S99>/Integer Delay3' */
  real32_T FixPtUnitDelay1_DSTATE;     /* '<S125>/FixPt Unit Delay1' */
  real32_T IntegerDelay3_DSTATE_c;     /* '<S97>/Integer Delay3' */
  real32_T IntegerDelay2_DSTATE;       /* '<S117>/Integer Delay2' */
  real32_T IntegerDelay3_DSTATE_o;     /* '<S119>/Integer Delay3' */
  real32_T IntegerDelay_DSTATE_c;      /* '<S117>/Integer Delay' */
  real32_T IntegerDelay1_DSTATE_j[2];  /* '<S117>/Integer Delay1' */
  real32_T IntegerDelay3_DSTATE_lo;    /* '<S118>/Integer Delay3' */
  real32_T IntegerDelay3_DSTATE_i4;    /* '<S100>/Integer Delay3' */
  real32_T IntegerDelay3_DSTATE_g;     /* '<S96>/Integer Delay3' */
  real32_T NDelays_DSTATE[5];          /* '<S92>/NDelays' */
  real32_T IntegerDelay2_DSTATE_b;     /* '<S110>/Integer Delay2' */
  real32_T IntegerDelay3_DSTATE_oy;    /* '<S111>/Integer Delay3' */
  real32_T IntegerDelay_DSTATE_o;      /* '<S110>/Integer Delay' */
  real32_T IntegerDelay1_DSTATE_m[2];  /* '<S110>/Integer Delay1' */
  real32_T IntegerDelay3_DSTATE_d;     /* '<S112>/Integer Delay3' */
  real32_T IntegerDelay3_DSTATE_cv;    /* '<S98>/Integer Delay3' */
  real32_T NDelays_DSTATE_n[5];        /* '<S95>/NDelays' */
  real32_T IntegerDelay2_DSTATE_g;     /* '<S121>/Integer Delay2' */
  real32_T IntegerDelay3_DSTATE_f;     /* '<S122>/Integer Delay3' */
  real32_T IntegerDelay_DSTATE_i;      /* '<S121>/Integer Delay' */
  real32_T IntegerDelay1_DSTATE_g[2];  /* '<S121>/Integer Delay1' */
  real32_T IntegerDelay3_DSTATE_n;     /* '<S123>/Integer Delay3' */
  real32_T IntegerDelay3_DSTATE_oh;    /* '<S83>/Integer Delay3' */
  real32_T IntegerDelay3_DSTATE_a;     /* '<S84>/Integer Delay3' */
  real32_T IntegerDelay3_DSTATE_h;     /* '<S54>/Integer Delay3' */
  real32_T NDelays_DSTATE_j[5];        /* '<S57>/NDelays' */
  real32_T IntegerDelay2_DSTATE_o;     /* '<S77>/Integer Delay2' */
  real32_T IntegerDelay3_DSTATE_m;     /* '<S78>/Integer Delay3' */
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
  real32_T Memory1_PreviousInput;      /* '<S94>/Memory1' */
  real32_T Memory1_PreviousInput_c;    /* '<S92>/Memory1' */
  real32_T Memory1_PreviousInput_b;    /* '<S95>/Memory1' */
  real32_T Memory1_PreviousInput_n;    /* '<S57>/Memory1' */
  real32_T Memory1_PreviousInput_g;    /* '<S55>/Memory1' */
  uint8_T IntegerDelay_DSTATE_d;       /* '<S127>/Integer Delay' */
  uint8_T IntegerDelay1_DSTATE_j2;     /* '<S127>/Integer Delay1' */
  uint8_T IntegerDelay_DSTATE_p;       /* '<S134>/Integer Delay' */
  uint8_T FixPtUnitDelay2_DSTATE;      /* '<S125>/FixPt Unit Delay2' */
  uint8_T fromWp;                      /* '<S134>/computeCurrentWP' */
  uint8_T toWp;                        /* '<S134>/computeCurrentWP' */
  uint8_T persistentDidReachIP;        /* '<S134>/Embedded MATLAB Function' */
  uint8_T DiscreteTimeIntegrator_IC_LOADI;/* '<S390>/Discrete-Time Integrator' */
  boolean_T IC1_FirstOutputTime;       /* '<S418>/IC1' */
  boolean_T IC1_FirstOutputTime_p;     /* '<S136>/IC1' */
  boolean_T IC2_FirstOutputTime;       /* '<S136>/IC2' */
  boolean_T IC_FirstOutputTime;        /* '<S133>/IC' */
  boolean_T IC_FirstOutputTime_l;      /* '<S134>/IC' */
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
  rtDW_EmbeddedMATLABFunction_con sf_EmbeddedMATLABFunction_a;/* '<S90>/Embedded MATLAB Function' */
  rtDW_EmbeddedMATLABFunction_con sf_EmbeddedMATLABFunction_e;/* '<S89>/Embedded MATLAB Function' */
  rtDW_EmbeddedMATLABFunction_con sf_EmbeddedMATLABFunction_b;/* '<S51>/Embedded MATLAB Function' */
} D_Work_controlMCUSlugsMKIINewNa;

/* Invariant block signals (auto storage) */
typedef struct {
  const real32_T e1;                   /* '<S207>/e^1' */
  const real32_T u;                    /* '<S207>/ ' */
  const real32_T Sum5;                 /* '<S207>/Sum5' */
  const real32_T e2;                   /* '<S204>/Sum3' */
  const real32_T e1_d;                 /* '<S410>/e^1' */
  const real32_T u_g;                  /* '<S410>/ ' */
  const real32_T Sum5_i;               /* '<S410>/Sum5' */
  const real32_T e2_g;                 /* '<S409>/Sum3' */
  const real32_T e1_n;                 /* '<S156>/e^1' */
  const real32_T u_o;                  /* '<S156>/ ' */
  const real32_T Sum5_h;               /* '<S156>/Sum5' */
  const real32_T e2_n;                 /* '<S153>/Sum3' */
  const real32_T e1_m;                 /* '<S356>/e^1' */
  const real32_T u_m;                  /* '<S356>/ ' */
  const real32_T Sum5_c;               /* '<S356>/Sum5' */
  const real32_T e2_b;                 /* '<S353>/Sum3' */
  const real32_T e1_dl;                /* '<S373>/e^1' */
  const real32_T u_e;                  /* '<S373>/ ' */
  const real32_T Sum5_l;               /* '<S373>/Sum5' */
  const real32_T e2_h;                 /* '<S370>/Sum3' */
  const real32_T e1_i;                 /* '<S301>/e^1' */
  const real32_T u_h;                  /* '<S301>/ ' */
  const real32_T Sum5_d;               /* '<S301>/Sum5' */
  const real32_T e2_o;                 /* '<S298>/Sum3' */
  const real32_T e1_i5;                /* '<S318>/e^1' */
  const real32_T u_b;                  /* '<S318>/ ' */
  const real32_T Sum5_n;               /* '<S318>/Sum5' */
  const real32_T e2_ge;                /* '<S315>/Sum3' */
  const real32_T e1_e;                 /* '<S398>/e^1' */
  const real32_T u_i;                  /* '<S398>/ ' */
  const real32_T Sum5_ib;              /* '<S398>/Sum5' */
  const real32_T e2_e;                 /* '<S395>/Sum3' */
  const real32_T Add3;                 /* '<S94>/Add3' */
  const real32_T Add4;                 /* '<S94>/Add4' */
  const real32_T Divide1;              /* '<S40>/Divide1' */
  const real32_T Divide2;              /* '<S40>/Divide2' */
  const boolean_T InitializeSPISSLasInput;/* '<Root>/Initialize SPI SSL as  Input' */
} ConstBlockIO_controlMCUSlugsMKI;

/* Constant parameters (auto storage) */
typedef struct {
  /* Pooled Parameter (Expression: [0 0 1;0 1 0;1 0 0])
   * Referenced by:
   *   '<S129>/UEN 2 NEU'
   *   '<S150>/UEN 2 NEU'
   *   '<S393>/UEN 2 NEU'
   *   '<S295>/UEN 2 NEU'
   *   '<S312>/UEN 2 NEU'
   *   '<S350>/UEN 2 NEU'
   *   '<S367>/UEN 2 NEU'
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
 * '<S50>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Lateral Channel Encaps/psi_dot_c Select logic'
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
 * '<S82>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Lateral Channel Encaps/psi_dot_c Select logic/Bank to Psi Dot'
 * '<S83>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Lateral Channel Encaps/psi_dot_c Select logic/Bank to Psi Dot/Protect NaNs'
 * '<S84>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Lateral Channel Encaps/psi_dot_c Select logic/Bank to Psi Dot/Protect NaNs1'
 * '<S85>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Lateral Channel Encaps/psi_dot_c Select logic/Bank to Psi Dot/dsPIC_tan'
 * '<S86>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Lateral Channel Encaps/psi_dot_c Select logic/Bank to Psi Dot/dsPIC_tan/Environment Controller'
 * '<S87>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Longitudinal Channel Encaps/Longitudinal Channel'
 * '<S88>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Longitudinal Channel Encaps/myMux Fun1'
 * '<S89>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Longitudinal Channel Encaps/Longitudinal Channel/ 0.3 sec tau for high speeds'
 * '<S90>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Longitudinal Channel Encaps/Longitudinal Channel/0.5 sec  tau for Turns'
 * '<S91>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Longitudinal Channel Encaps/Longitudinal Channel/Air Density from Height'
 * '<S92>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Longitudinal Channel Encaps/Longitudinal Channel/Airspeed Hold [PID]'
 * '<S93>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Longitudinal Channel Encaps/Longitudinal Channel/Compute Airspeed'
 * '<S94>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Longitudinal Channel Encaps/Longitudinal Channel/Height Error to Pitch  Command [PI]'
 * '<S95>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Longitudinal Channel Encaps/Longitudinal Channel/Pitch To Elevator [PID]'
 * '<S96>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Longitudinal Channel Encaps/Longitudinal Channel/Protect NaNs'
 * '<S97>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Longitudinal Channel Encaps/Longitudinal Channel/Protect NaNs1'
 * '<S98>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Longitudinal Channel Encaps/Longitudinal Channel/Protect NaNs2'
 * '<S99>'  : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Longitudinal Channel Encaps/Longitudinal Channel/Protect NaNs3'
 * '<S100>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Longitudinal Channel Encaps/Longitudinal Channel/Protect NaNs4'
 * '<S101>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Longitudinal Channel Encaps/Longitudinal Channel/Rate Limiter Dynamic with External IC'
 * '<S102>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Longitudinal Channel Encaps/Longitudinal Channel/dsPIC_cos'
 * '<S103>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Longitudinal Channel Encaps/Longitudinal Channel/lowpass limit'
 * '<S104>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Longitudinal Channel Encaps/Longitudinal Channel/ 0.3 sec tau for high speeds/Embedded MATLAB Function'
 * '<S105>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Longitudinal Channel Encaps/Longitudinal Channel/0.5 sec  tau for Turns/Embedded MATLAB Function'
 * '<S106>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Longitudinal Channel Encaps/Longitudinal Channel/Air Density from Height/Length Conversion'
 * '<S107>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Longitudinal Channel Encaps/Longitudinal Channel/Air Density from Height/dsPIC Power'
 * '<S108>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Longitudinal Channel Encaps/Longitudinal Channel/Air Density from Height/dsPIC Power/Environment Controller'
 * '<S109>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Longitudinal Channel Encaps/Longitudinal Channel/Airspeed Hold [PID]/Saturation Dynamic'
 * '<S110>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Longitudinal Channel Encaps/Longitudinal Channel/Airspeed Hold [PID]/Trapezoidal Integrator'
 * '<S111>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Longitudinal Channel Encaps/Longitudinal Channel/Airspeed Hold [PID]/Trapezoidal Integrator/Protect NaNs '
 * '<S112>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Longitudinal Channel Encaps/Longitudinal Channel/Airspeed Hold [PID]/Trapezoidal Integrator/Protect NaNs1'
 * '<S113>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Longitudinal Channel Encaps/Longitudinal Channel/Compute Airspeed/dsPIC_SQRT'
 * '<S114>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Longitudinal Channel Encaps/Longitudinal Channel/Compute Airspeed/dsPIC_SQRT/Environment Controller'
 * '<S115>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Longitudinal Channel Encaps/Longitudinal Channel/Compute Airspeed/dsPIC_SQRT/negprotect'
 * '<S116>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Longitudinal Channel Encaps/Longitudinal Channel/Height Error to Pitch  Command [PI]/Saturation Dynamic'
 * '<S117>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Longitudinal Channel Encaps/Longitudinal Channel/Height Error to Pitch  Command [PI]/Trapezoidal Integrator'
 * '<S118>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Longitudinal Channel Encaps/Longitudinal Channel/Height Error to Pitch  Command [PI]/Trapezoidal Integrator/Protect NaNs'
 * '<S119>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Longitudinal Channel Encaps/Longitudinal Channel/Height Error to Pitch  Command [PI]/Trapezoidal Integrator/Protect NaNs '
 * '<S120>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Longitudinal Channel Encaps/Longitudinal Channel/Pitch To Elevator [PID]/Saturation Dynamic'
 * '<S121>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Longitudinal Channel Encaps/Longitudinal Channel/Pitch To Elevator [PID]/Trapezoidal Integrator'
 * '<S122>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Longitudinal Channel Encaps/Longitudinal Channel/Pitch To Elevator [PID]/Trapezoidal Integrator/Protect NaNs '
 * '<S123>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Longitudinal Channel Encaps/Longitudinal Channel/Pitch To Elevator [PID]/Trapezoidal Integrator/Protect NaNs1'
 * '<S124>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Longitudinal Channel Encaps/Longitudinal Channel/Rate Limiter Dynamic with External IC/Saturation Dynamic'
 * '<S125>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Longitudinal Channel Encaps/Longitudinal Channel/Rate Limiter Dynamic with External IC/Unit Delay External IC'
 * '<S126>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Longitudinal Channel Encaps/Longitudinal Channel/dsPIC_cos/Environment Controller1'
 * '<S127>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation'
 * '<S128>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation'
 * '<S129>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Geod2LTP'
 * '<S130>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Get 2D Groundspeed'
 * '<S131>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/L2+'
 * '<S132>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Line Segment'
 * '<S133>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Minimum Turn Radius'
 * '<S134>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation'
 * '<S135>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/RTB//Follow Mobile Navigation'
 * '<S136>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Recent Enable'
 * '<S137>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Zero out Z1'
 * '<S138>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Zero out Z2'
 * '<S139>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/myMux Fun1'
 * '<S140>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/myMux Fun2'
 * '<S141>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/myMux Fun3'
 * '<S142>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/myMux Fun4'
 * '<S143>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/myMux Fun5'
 * '<S144>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Get SP Location'
 * '<S145>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Inside the Circle,  Keep Straight until  intersection'
 * '<S146>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Intersection. Circular Navigation'
 * '<S147>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/No intersection,  Navigate to ISR'
 * '<S148>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Vector Norm'
 * '<S149>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/dsPIC_ABS'
 * '<S150>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Get SP Location/Geod2LTP'
 * '<S151>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Get SP Location/Zero out Z2'
 * '<S152>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Get SP Location/Geod2LTP/ECEF2LTP (UEN)'
 * '<S153>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Get SP Location/Geod2LTP/Geod2ECEF1'
 * '<S154>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Get SP Location/Geod2LTP/ECEF2LTP (UEN)/From Body to Universal1'
 * '<S155>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Get SP Location/Geod2LTP/ECEF2LTP (UEN)/From Body to Universal1/Rotation Matrix from Long////Lat'
 * '<S156>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Get SP Location/Geod2LTP/Geod2ECEF1/eccentricity^2'
 * '<S157>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Inside the Circle,  Keep Straight until  intersection/Compute Head of Circle'
 * '<S158>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Intersection. Circular Navigation/Compute Angle Between Vectors1'
 * '<S159>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Intersection. Circular Navigation/Compute Angle Between Vectors2'
 * '<S160>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Intersection. Circular Navigation/Embedded MATLAB Function'
 * '<S161>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Intersection. Circular Navigation/dsPIC_ABS'
 * '<S162>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Intersection. Circular Navigation/dsPIC_ABS1'
 * '<S163>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Intersection. Circular Navigation/dsPIC_SQRT'
 * '<S164>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Intersection. Circular Navigation/negprotect1'
 * '<S165>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Intersection. Circular Navigation/negprotect2'
 * '<S166>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Intersection. Circular Navigation/Compute Angle Between Vectors1/Vector Norm'
 * '<S167>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Intersection. Circular Navigation/Compute Angle Between Vectors1/dsPIC Dot Product'
 * '<S168>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Intersection. Circular Navigation/Compute Angle Between Vectors1/dsPIC_acos'
 * '<S169>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Intersection. Circular Navigation/Compute Angle Between Vectors1/negprotect3'
 * '<S170>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Intersection. Circular Navigation/Compute Angle Between Vectors1/Vector Norm/dsPIC Dot Product'
 * '<S171>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Intersection. Circular Navigation/Compute Angle Between Vectors1/Vector Norm/dsPIC_SQRT'
 * '<S172>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Intersection. Circular Navigation/Compute Angle Between Vectors1/Vector Norm/dsPIC Dot Product/Embedded MATLAB Function'
 * '<S173>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Intersection. Circular Navigation/Compute Angle Between Vectors1/Vector Norm/dsPIC Dot Product/Environment Controller'
 * '<S174>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Intersection. Circular Navigation/Compute Angle Between Vectors1/Vector Norm/dsPIC_SQRT/Environment Controller'
 * '<S175>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Intersection. Circular Navigation/Compute Angle Between Vectors1/Vector Norm/dsPIC_SQRT/negprotect'
 * '<S176>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Intersection. Circular Navigation/Compute Angle Between Vectors1/dsPIC Dot Product/Embedded MATLAB Function'
 * '<S177>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Intersection. Circular Navigation/Compute Angle Between Vectors1/dsPIC Dot Product/Environment Controller'
 * '<S178>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Intersection. Circular Navigation/Compute Angle Between Vectors1/dsPIC_acos/Environment Controller'
 * '<S179>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Intersection. Circular Navigation/Compute Angle Between Vectors2/Vector Norm'
 * '<S180>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Intersection. Circular Navigation/Compute Angle Between Vectors2/dsPIC Dot Product'
 * '<S181>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Intersection. Circular Navigation/Compute Angle Between Vectors2/dsPIC_acos'
 * '<S182>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Intersection. Circular Navigation/Compute Angle Between Vectors2/negprotect3'
 * '<S183>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Intersection. Circular Navigation/Compute Angle Between Vectors2/Vector Norm/dsPIC Dot Product'
 * '<S184>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Intersection. Circular Navigation/Compute Angle Between Vectors2/Vector Norm/dsPIC_SQRT'
 * '<S185>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Intersection. Circular Navigation/Compute Angle Between Vectors2/Vector Norm/dsPIC Dot Product/Embedded MATLAB Function'
 * '<S186>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Intersection. Circular Navigation/Compute Angle Between Vectors2/Vector Norm/dsPIC Dot Product/Environment Controller'
 * '<S187>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Intersection. Circular Navigation/Compute Angle Between Vectors2/Vector Norm/dsPIC_SQRT/Environment Controller'
 * '<S188>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Intersection. Circular Navigation/Compute Angle Between Vectors2/Vector Norm/dsPIC_SQRT/negprotect'
 * '<S189>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Intersection. Circular Navigation/Compute Angle Between Vectors2/dsPIC Dot Product/Embedded MATLAB Function'
 * '<S190>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Intersection. Circular Navigation/Compute Angle Between Vectors2/dsPIC Dot Product/Environment Controller'
 * '<S191>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Intersection. Circular Navigation/Compute Angle Between Vectors2/dsPIC_acos/Environment Controller'
 * '<S192>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Intersection. Circular Navigation/dsPIC_ABS/Environment Controller'
 * '<S193>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Intersection. Circular Navigation/dsPIC_ABS1/Environment Controller'
 * '<S194>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Intersection. Circular Navigation/dsPIC_SQRT/Environment Controller'
 * '<S195>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Intersection. Circular Navigation/dsPIC_SQRT/negprotect'
 * '<S196>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Vector Norm/dsPIC Dot Product'
 * '<S197>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Vector Norm/dsPIC_SQRT'
 * '<S198>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Vector Norm/dsPIC Dot Product/Embedded MATLAB Function'
 * '<S199>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Vector Norm/dsPIC Dot Product/Environment Controller'
 * '<S200>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Vector Norm/dsPIC_SQRT/Environment Controller'
 * '<S201>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/Vector Norm/dsPIC_SQRT/negprotect'
 * '<S202>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Circle Navigation/dsPIC_ABS/Environment Controller'
 * '<S203>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Geod2LTP/ECEF2LTP (UEN)'
 * '<S204>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Geod2LTP/Geod2ECEF1'
 * '<S205>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Geod2LTP/ECEF2LTP (UEN)/From Body to Universal1'
 * '<S206>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Geod2LTP/ECEF2LTP (UEN)/From Body to Universal1/Rotation Matrix from Long////Lat'
 * '<S207>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Geod2LTP/Geod2ECEF1/eccentricity^2'
 * '<S208>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Get 2D Groundspeed/Vector Norm'
 * '<S209>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Get 2D Groundspeed/Vector Norm/dsPIC Dot Product'
 * '<S210>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Get 2D Groundspeed/Vector Norm/dsPIC_SQRT'
 * '<S211>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Get 2D Groundspeed/Vector Norm/dsPIC Dot Product/Embedded MATLAB Function'
 * '<S212>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Get 2D Groundspeed/Vector Norm/dsPIC Dot Product/Environment Controller'
 * '<S213>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Get 2D Groundspeed/Vector Norm/dsPIC_SQRT/Environment Controller'
 * '<S214>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Get 2D Groundspeed/Vector Norm/dsPIC_SQRT/negprotect'
 * '<S215>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/L2+/Get Sin, Cos eta'
 * '<S216>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/L2+/L2+'
 * '<S217>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/L2+/Subsystem'
 * '<S218>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/L2+/dsPIC_ABS'
 * '<S219>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/L2+/dsPIC_atan2'
 * '<S220>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/L2+/if sin(eta) < 0 - maxAcc else                 maxAcc '
 * '<S221>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/L2+/Get Sin, Cos eta/2D Dot Product '
 * '<S222>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/L2+/Get Sin, Cos eta/Compare To Zero'
 * '<S223>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/L2+/Get Sin, Cos eta/Get  Groundspeed'
 * '<S224>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/L2+/Get Sin, Cos eta/Get ZProd'
 * '<S225>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/L2+/Get Sin, Cos eta/negprotect'
 * '<S226>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/L2+/Get Sin, Cos eta/Get  Groundspeed/Vector Norm'
 * '<S227>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/L2+/Get Sin, Cos eta/Get  Groundspeed/Zero out Z1'
 * '<S228>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/L2+/Get Sin, Cos eta/Get  Groundspeed/Vector Norm/dsPIC Dot Product'
 * '<S229>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/L2+/Get Sin, Cos eta/Get  Groundspeed/Vector Norm/dsPIC_SQRT'
 * '<S230>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/L2+/Get Sin, Cos eta/Get  Groundspeed/Vector Norm/dsPIC Dot Product/Embedded MATLAB Function'
 * '<S231>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/L2+/Get Sin, Cos eta/Get  Groundspeed/Vector Norm/dsPIC Dot Product/Environment Controller'
 * '<S232>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/L2+/Get Sin, Cos eta/Get  Groundspeed/Vector Norm/dsPIC_SQRT/Environment Controller'
 * '<S233>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/L2+/Get Sin, Cos eta/Get  Groundspeed/Vector Norm/dsPIC_SQRT/negprotect'
 * '<S234>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/L2+/L2+/negprotect'
 * '<S235>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/L2+/Subsystem/dsPIC_atan'
 * '<S236>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/L2+/Subsystem/dsPIC_atan/Environment Controller'
 * '<S237>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/L2+/dsPIC_ABS/Environment Controller'
 * '<S238>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/L2+/dsPIC_atan2/Environment Controller'
 * '<S239>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/L2+/if sin(eta) < 0 - maxAcc else                 maxAcc /Compare To Zero'
 * '<S240>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Line Segment/Vector Norm'
 * '<S241>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Line Segment/Vector Norm/dsPIC Dot Product'
 * '<S242>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Line Segment/Vector Norm/dsPIC_SQRT'
 * '<S243>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Line Segment/Vector Norm/dsPIC Dot Product/Embedded MATLAB Function'
 * '<S244>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Line Segment/Vector Norm/dsPIC Dot Product/Environment Controller'
 * '<S245>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Line Segment/Vector Norm/dsPIC_SQRT/Environment Controller'
 * '<S246>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Line Segment/Vector Norm/dsPIC_SQRT/negprotect'
 * '<S247>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Compute WP Switching Trigger'
 * '<S248>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Embedded MATLAB Function'
 * '<S249>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet'
 * '<S250>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable'
 * '<S251>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Subsystem'
 * '<S252>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Vector Norm'
 * '<S253>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Zero out Z2'
 * '<S254>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/computeCurrentWP'
 * '<S255>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Compute WP Switching Trigger/Compute  Projection'
 * '<S256>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Compute WP Switching Trigger/dsPIC Dot Product'
 * '<S257>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Compute WP Switching Trigger/dsPIC_ABS'
 * '<S258>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Compute WP Switching Trigger/dsPIC_SQRT'
 * '<S259>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Compute WP Switching Trigger/max'
 * '<S260>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Compute WP Switching Trigger/max1'
 * '<S261>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Compute WP Switching Trigger/min'
 * '<S262>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Compute WP Switching Trigger/Compute  Projection/Protect NaNs'
 * '<S263>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Compute WP Switching Trigger/Compute  Projection/dsPIC Dot Product'
 * '<S264>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Compute WP Switching Trigger/Compute  Projection/dsPIC Dot Product/Embedded MATLAB Function'
 * '<S265>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Compute WP Switching Trigger/Compute  Projection/dsPIC Dot Product/Environment Controller'
 * '<S266>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Compute WP Switching Trigger/dsPIC Dot Product/Embedded MATLAB Function'
 * '<S267>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Compute WP Switching Trigger/dsPIC Dot Product/Environment Controller'
 * '<S268>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Compute WP Switching Trigger/dsPIC_ABS/Environment Controller'
 * '<S269>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Compute WP Switching Trigger/dsPIC_SQRT/Environment Controller'
 * '<S270>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Compute WP Switching Trigger/dsPIC_SQRT/negprotect'
 * '<S271>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/Compute Frenet'
 * '<S272>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/Zero out Z1'
 * '<S273>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/Zero out Z2'
 * '<S274>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/Zero out Z3'
 * '<S275>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index1'
 * '<S276>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index2'
 * '<S277>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/Compute Frenet/Normalize Vector'
 * '<S278>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/Compute Frenet/Normalize Vector '
 * '<S279>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/Compute Frenet/Select N  Terms'
 * '<S280>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/Compute Frenet/Normalize Vector/Vector Norm'
 * '<S281>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/Compute Frenet/Normalize Vector/Vector Norm/dsPIC Dot Product'
 * '<S282>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/Compute Frenet/Normalize Vector/Vector Norm/dsPIC_SQRT'
 * '<S283>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/Compute Frenet/Normalize Vector/Vector Norm/dsPIC Dot Product/Embedded MATLAB Function'
 * '<S284>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/Compute Frenet/Normalize Vector/Vector Norm/dsPIC Dot Product/Environment Controller'
 * '<S285>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/Compute Frenet/Normalize Vector/Vector Norm/dsPIC_SQRT/Environment Controller'
 * '<S286>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/Compute Frenet/Normalize Vector/Vector Norm/dsPIC_SQRT/negprotect'
 * '<S287>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/Compute Frenet/Normalize Vector /Vector Norm'
 * '<S288>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/Compute Frenet/Normalize Vector /Vector Norm/dsPIC Dot Product'
 * '<S289>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/Compute Frenet/Normalize Vector /Vector Norm/dsPIC_SQRT'
 * '<S290>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/Compute Frenet/Normalize Vector /Vector Norm/dsPIC Dot Product/Embedded MATLAB Function'
 * '<S291>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/Compute Frenet/Normalize Vector /Vector Norm/dsPIC Dot Product/Environment Controller'
 * '<S292>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/Compute Frenet/Normalize Vector /Vector Norm/dsPIC_SQRT/Environment Controller'
 * '<S293>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/Compute Frenet/Normalize Vector /Vector Norm/dsPIC_SQRT/negprotect'
 * '<S294>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index1/Environment Controller1'
 * '<S295>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index1/Geod2LTP2'
 * '<S296>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index1/Get WP from idx SIM'
 * '<S297>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index1/Geod2LTP2/ECEF2LTP (UEN)'
 * '<S298>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index1/Geod2LTP2/Geod2ECEF1'
 * '<S299>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index1/Geod2LTP2/ECEF2LTP (UEN)/From Body to Universal1'
 * '<S300>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index1/Geod2LTP2/ECEF2LTP (UEN)/From Body to Universal1/Rotation Matrix from Long////Lat'
 * '<S301>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index1/Geod2LTP2/Geod2ECEF1/eccentricity^2'
 * '<S302>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index1/Get WP from idx SIM/Geod2LTP1'
 * '<S303>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index1/Get WP from idx SIM/Zero out Z1'
 * '<S304>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index1/Get WP from idx SIM/Zero out Z2'
 * '<S305>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index1/Get WP from idx SIM/Zero out Z3'
 * '<S306>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index1/Get WP from idx SIM/Geod2LTP1/ECEF2LTP (UEN)'
 * '<S307>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index1/Get WP from idx SIM/Geod2LTP1/Geod2ECEF1'
 * '<S308>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index1/Get WP from idx SIM/Geod2LTP1/ECEF2LTP (UEN)/From Body to Universal1'
 * '<S309>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index1/Get WP from idx SIM/Geod2LTP1/ECEF2LTP (UEN)/From Body to Universal1/Rotation Matrix from Long////Lat'
 * '<S310>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index1/Get WP from idx SIM/Geod2LTP1/Geod2ECEF1/eccentricity^2'
 * '<S311>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index2/Environment Controller1'
 * '<S312>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index2/Geod2LTP2'
 * '<S313>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index2/Get WP from idx SIM'
 * '<S314>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index2/Geod2LTP2/ECEF2LTP (UEN)'
 * '<S315>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index2/Geod2LTP2/Geod2ECEF1'
 * '<S316>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index2/Geod2LTP2/ECEF2LTP (UEN)/From Body to Universal1'
 * '<S317>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index2/Geod2LTP2/ECEF2LTP (UEN)/From Body to Universal1/Rotation Matrix from Long////Lat'
 * '<S318>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index2/Geod2LTP2/Geod2ECEF1/eccentricity^2'
 * '<S319>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index2/Get WP from idx SIM/Geod2LTP1'
 * '<S320>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index2/Get WP from idx SIM/Zero out Z1'
 * '<S321>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index2/Get WP from idx SIM/Zero out Z2'
 * '<S322>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index2/Get WP from idx SIM/Zero out Z3'
 * '<S323>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index2/Get WP from idx SIM/Geod2LTP1/ECEF2LTP (UEN)'
 * '<S324>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index2/Get WP from idx SIM/Geod2LTP1/Geod2ECEF1'
 * '<S325>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index2/Get WP from idx SIM/Geod2LTP1/ECEF2LTP (UEN)/From Body to Universal1'
 * '<S326>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index2/Get WP from idx SIM/Geod2LTP1/ECEF2LTP (UEN)/From Body to Universal1/Rotation Matrix from Long////Lat'
 * '<S327>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Get Frenet/get WP Coord from Index2/Get WP from idx SIM/Geod2LTP1/Geod2ECEF1/eccentricity^2'
 * '<S328>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem'
 * '<S329>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/Compute Frenet'
 * '<S330>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index'
 * '<S331>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index1'
 * '<S332>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/Compute Frenet/Normalize Vector'
 * '<S333>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/Compute Frenet/Normalize Vector '
 * '<S334>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/Compute Frenet/Select N  Terms'
 * '<S335>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/Compute Frenet/Normalize Vector/Vector Norm'
 * '<S336>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/Compute Frenet/Normalize Vector/Vector Norm/dsPIC Dot Product'
 * '<S337>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/Compute Frenet/Normalize Vector/Vector Norm/dsPIC_SQRT'
 * '<S338>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/Compute Frenet/Normalize Vector/Vector Norm/dsPIC Dot Product/Embedded MATLAB Function'
 * '<S339>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/Compute Frenet/Normalize Vector/Vector Norm/dsPIC Dot Product/Environment Controller'
 * '<S340>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/Compute Frenet/Normalize Vector/Vector Norm/dsPIC_SQRT/Environment Controller'
 * '<S341>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/Compute Frenet/Normalize Vector/Vector Norm/dsPIC_SQRT/negprotect'
 * '<S342>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/Compute Frenet/Normalize Vector /Vector Norm'
 * '<S343>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/Compute Frenet/Normalize Vector /Vector Norm/dsPIC Dot Product'
 * '<S344>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/Compute Frenet/Normalize Vector /Vector Norm/dsPIC_SQRT'
 * '<S345>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/Compute Frenet/Normalize Vector /Vector Norm/dsPIC Dot Product/Embedded MATLAB Function'
 * '<S346>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/Compute Frenet/Normalize Vector /Vector Norm/dsPIC Dot Product/Environment Controller'
 * '<S347>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/Compute Frenet/Normalize Vector /Vector Norm/dsPIC_SQRT/Environment Controller'
 * '<S348>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/Compute Frenet/Normalize Vector /Vector Norm/dsPIC_SQRT/negprotect'
 * '<S349>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index/Environment Controller1'
 * '<S350>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index/Geod2LTP2'
 * '<S351>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index/Get WP from idx SIM'
 * '<S352>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index/Geod2LTP2/ECEF2LTP (UEN)'
 * '<S353>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index/Geod2LTP2/Geod2ECEF1'
 * '<S354>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index/Geod2LTP2/ECEF2LTP (UEN)/From Body to Universal1'
 * '<S355>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index/Geod2LTP2/ECEF2LTP (UEN)/From Body to Universal1/Rotation Matrix from Long////Lat'
 * '<S356>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index/Geod2LTP2/Geod2ECEF1/eccentricity^2'
 * '<S357>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index/Get WP from idx SIM/Geod2LTP1'
 * '<S358>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index/Get WP from idx SIM/Zero out Z1'
 * '<S359>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index/Get WP from idx SIM/Zero out Z2'
 * '<S360>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index/Get WP from idx SIM/Zero out Z3'
 * '<S361>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index/Get WP from idx SIM/Geod2LTP1/ECEF2LTP (UEN)'
 * '<S362>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index/Get WP from idx SIM/Geod2LTP1/Geod2ECEF1'
 * '<S363>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index/Get WP from idx SIM/Geod2LTP1/ECEF2LTP (UEN)/From Body to Universal1'
 * '<S364>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index/Get WP from idx SIM/Geod2LTP1/ECEF2LTP (UEN)/From Body to Universal1/Rotation Matrix from Long////Lat'
 * '<S365>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index/Get WP from idx SIM/Geod2LTP1/Geod2ECEF1/eccentricity^2'
 * '<S366>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index1/Environment Controller1'
 * '<S367>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index1/Geod2LTP2'
 * '<S368>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index1/Get WP from idx SIM'
 * '<S369>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index1/Geod2LTP2/ECEF2LTP (UEN)'
 * '<S370>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index1/Geod2LTP2/Geod2ECEF1'
 * '<S371>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index1/Geod2LTP2/ECEF2LTP (UEN)/From Body to Universal1'
 * '<S372>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index1/Geod2LTP2/ECEF2LTP (UEN)/From Body to Universal1/Rotation Matrix from Long////Lat'
 * '<S373>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index1/Geod2LTP2/Geod2ECEF1/eccentricity^2'
 * '<S374>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index1/Get WP from idx SIM/Geod2LTP1'
 * '<S375>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index1/Get WP from idx SIM/Zero out Z1'
 * '<S376>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index1/Get WP from idx SIM/Zero out Z2'
 * '<S377>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index1/Get WP from idx SIM/Zero out Z3'
 * '<S378>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index1/Get WP from idx SIM/Geod2LTP1/ECEF2LTP (UEN)'
 * '<S379>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index1/Get WP from idx SIM/Geod2LTP1/Geod2ECEF1'
 * '<S380>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index1/Get WP from idx SIM/Geod2LTP1/ECEF2LTP (UEN)/From Body to Universal1'
 * '<S381>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index1/Get WP from idx SIM/Geod2LTP1/ECEF2LTP (UEN)/From Body to Universal1/Rotation Matrix from Long////Lat'
 * '<S382>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/On WP Enable/Subsystem/get WP Coord from Index1/Get WP from idx SIM/Geod2LTP1/Geod2ECEF1/eccentricity^2'
 * '<S383>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Vector Norm/dsPIC Dot Product'
 * '<S384>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Vector Norm/dsPIC_SQRT'
 * '<S385>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Vector Norm/dsPIC Dot Product/Embedded MATLAB Function'
 * '<S386>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Vector Norm/dsPIC Dot Product/Environment Controller'
 * '<S387>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Vector Norm/dsPIC_SQRT/Environment Controller'
 * '<S388>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Normal WP  Navigation/Vector Norm/dsPIC_SQRT/negprotect'
 * '<S389>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/RTB//Follow Mobile Navigation/Compute Mobile Location'
 * '<S390>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/RTB//Follow Mobile Navigation/Simple Predictor'
 * '<S391>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/RTB//Follow Mobile Navigation/Vector Norm'
 * '<S392>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/RTB//Follow Mobile Navigation/Zero out Z1'
 * '<S393>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/RTB//Follow Mobile Navigation/Compute Mobile Location/Geod2LTP'
 * '<S394>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/RTB//Follow Mobile Navigation/Compute Mobile Location/Geod2LTP/ECEF2LTP (UEN)'
 * '<S395>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/RTB//Follow Mobile Navigation/Compute Mobile Location/Geod2LTP/Geod2ECEF1'
 * '<S396>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/RTB//Follow Mobile Navigation/Compute Mobile Location/Geod2LTP/ECEF2LTP (UEN)/From Body to Universal1'
 * '<S397>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/RTB//Follow Mobile Navigation/Compute Mobile Location/Geod2LTP/ECEF2LTP (UEN)/From Body to Universal1/Rotation Matrix from Long////Lat'
 * '<S398>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/RTB//Follow Mobile Navigation/Compute Mobile Location/Geod2LTP/Geod2ECEF1/eccentricity^2'
 * '<S399>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/RTB//Follow Mobile Navigation/Vector Norm/dsPIC Dot Product'
 * '<S400>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/RTB//Follow Mobile Navigation/Vector Norm/dsPIC_SQRT'
 * '<S401>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/RTB//Follow Mobile Navigation/Vector Norm/dsPIC Dot Product/Embedded MATLAB Function'
 * '<S402>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/RTB//Follow Mobile Navigation/Vector Norm/dsPIC Dot Product/Environment Controller'
 * '<S403>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/RTB//Follow Mobile Navigation/Vector Norm/dsPIC_SQRT/Environment Controller'
 * '<S404>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/RTB//Follow Mobile Navigation/Vector Norm/dsPIC_SQRT/negprotect'
 * '<S405>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Recent Enable/Grab Upon Enable'
 * '<S406>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Recent Enable/Subsystem'
 * '<S407>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Recent Enable/Grab Upon Enable/Compute GS Location'
 * '<S408>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Recent Enable/Grab Upon Enable/Environment Controller'
 * '<S409>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Recent Enable/Grab Upon Enable/Compute GS Location/Geod2ECEF1'
 * '<S410>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Recent Enable/Grab Upon Enable/Compute GS Location/Geod2ECEF1/eccentricity^2'
 * '<S411>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Recent Enable/Subsystem/Detect Rising Edge'
 * '<S412>' : 'controlMCUSlugsMKIINewNav/Inner Loop// Navigation/Navigation Encaps/Navigation/Recent Enable/Subsystem/Environment Controller2'
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
