/*
 * File: sensorMCUSlugsMKII_data.c
 *
 * Real-Time Workshop code generated for Simulink model sensorMCUSlugsMKII.
 *
 * Model version                        : 1.202
 * Real-Time Workshop file version      : 8.1 (R2011b) 08-Jul-2011
 * Real-Time Workshop file generated on : Sat Jan 31 12:07:47 2015
 * TLC version                          : 8.1 (Jul  9 2011)
 * C source code generated on           : Sat Jan 31 12:07:48 2015
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

#include "sensorMCUSlugsMKII.h"
#include "sensorMCUSlugsMKII_private.h"

/* Block parameters (auto storage) */
Parameters_sensorMCUSlugsMKII sensorMCUSlugsMKII_P = {
  0.01,                                /* Expression: apSampleTime
                                        * Referenced by: '<S87>/Embedded MATLAB Function3'
                                        */
  0.97044334975369462,                 /* Computed Parameter: DiscreteZeroPole_A
                                        * Referenced by: '<S168>/Discrete Zero-Pole'
                                        */
  0.029119852459414206,                /* Computed Parameter: DiscreteZeroPole_C
                                        * Referenced by: '<S168>/Discrete Zero-Pole'
                                        */
  0.014778325123152709,                /* Computed Parameter: DiscreteZeroPole_D
                                        * Referenced by: '<S168>/Discrete Zero-Pole'
                                        */
  0.01,                                /* Expression: T
                                        * Referenced by: '<S157>/Constant'
                                        */
  0.02,                                /* Expression: f
                                        * Referenced by: '<S157>/Constant1'
                                        */
  0.01,                                /* Expression: T
                                        * Referenced by: '<S160>/Constant'
                                        */
  4.0,                                 /* Expression: f
                                        * Referenced by: '<S160>/Constant1'
                                        */
  0.01,                                /* Expression: T
                                        * Referenced by: '<S98>/Constant'
                                        */
  0.31830988618379069,                 /* Expression: f
                                        * Referenced by: '<S98>/Constant1'
                                        */
  0.01,                                /* Expression: T
                                        * Referenced by: '<S125>/Constant'
                                        */
  40.0,                                /* Expression: f
                                        * Referenced by: '<S125>/Constant1'
                                        */
  0.01,                                /* Expression: T
                                        * Referenced by: '<S125>/Constant2'
                                        */
  40.0,                                /* Expression: f
                                        * Referenced by: '<S125>/Constant3'
                                        */
  0.01,                                /* Expression: T
                                        * Referenced by: '<S125>/Constant4'
                                        */
  40.0,                                /* Expression: f
                                        * Referenced by: '<S125>/Constant5'
                                        */
  -0.99960423832914991,                /* Computed Parameter: DiscreteZeroPole_A_j
                                        * Referenced by: '<S100>/Discrete Zero-Pole'
                                        */
  3.9156825028515473E-10,              /* Computed Parameter: DiscreteZeroPole_C_e
                                        * Referenced by: '<S100>/Discrete Zero-Pole'
                                        */
  9.8940417712526327E-7,               /* Computed Parameter: DiscreteZeroPole_D_j
                                        * Referenced by: '<S100>/Discrete Zero-Pole'
                                        */
  -0.99960423832914991,                /* Computed Parameter: DiscreteZeroPole_A_l
                                        * Referenced by: '<S101>/Discrete Zero-Pole'
                                        */
  3.9156825028515473E-10,              /* Computed Parameter: DiscreteZeroPole_C_o
                                        * Referenced by: '<S101>/Discrete Zero-Pole'
                                        */
  9.8940417712526327E-7,               /* Computed Parameter: DiscreteZeroPole_D_ju
                                        * Referenced by: '<S101>/Discrete Zero-Pole'
                                        */
  -0.99960423832914991,                /* Computed Parameter: DiscreteZeroPole_A_i
                                        * Referenced by: '<S102>/Discrete Zero-Pole'
                                        */
  3.9156825028515473E-10,              /* Computed Parameter: DiscreteZeroPole_C_k
                                        * Referenced by: '<S102>/Discrete Zero-Pole'
                                        */
  9.8940417712526327E-7,               /* Computed Parameter: DiscreteZeroPole_D_p
                                        * Referenced by: '<S102>/Discrete Zero-Pole'
                                        */
  1.0,                                 /* Computed Parameter: DiscreteZeroPole_A_c
                                        * Referenced by: '<S32>/Discrete Zero-Pole'
                                        */
  0.01,                                /* Computed Parameter: DiscreteZeroPole_C_c
                                        * Referenced by: '<S32>/Discrete Zero-Pole'
                                        */
  0.005,                               /* Computed Parameter: DiscreteZeroPole_D_jb
                                        * Referenced by: '<S32>/Discrete Zero-Pole'
                                        */
  1.0,                                 /* Computed Parameter: DiscreteZeroPole_A_i3
                                        * Referenced by: '<S33>/Discrete Zero-Pole'
                                        */
  0.01,                                /* Computed Parameter: DiscreteZeroPole_C_i
                                        * Referenced by: '<S33>/Discrete Zero-Pole'
                                        */
  0.005,                               /* Computed Parameter: DiscreteZeroPole_D_o
                                        * Referenced by: '<S33>/Discrete Zero-Pole'
                                        */
  1.0,                                 /* Computed Parameter: DiscreteZeroPole_A_e
                                        * Referenced by: '<S34>/Discrete Zero-Pole'
                                        */
  0.01,                                /* Computed Parameter: DiscreteZeroPole_C_p
                                        * Referenced by: '<S34>/Discrete Zero-Pole'
                                        */
  0.005,                               /* Computed Parameter: DiscreteZeroPole_D_l
                                        * Referenced by: '<S34>/Discrete Zero-Pole'
                                        */
  1.0,                                 /* Computed Parameter: DiscreteZeroPole_A_a
                                        * Referenced by: '<S105>/Discrete Zero-Pole'
                                        */
  0.01,                                /* Computed Parameter: DiscreteZeroPole_C_cr
                                        * Referenced by: '<S105>/Discrete Zero-Pole'
                                        */
  0.005,                               /* Computed Parameter: DiscreteZeroPole_D_c
                                        * Referenced by: '<S105>/Discrete Zero-Pole'
                                        */
  1.0,                                 /* Computed Parameter: DiscreteZeroPole_A_aw
                                        * Referenced by: '<S107>/Discrete Zero-Pole'
                                        */
  0.01,                                /* Computed Parameter: DiscreteZeroPole_C_b
                                        * Referenced by: '<S107>/Discrete Zero-Pole'
                                        */
  0.005,                               /* Computed Parameter: DiscreteZeroPole_D_co
                                        * Referenced by: '<S107>/Discrete Zero-Pole'
                                        */
  0.01,                                /* Expression: T
                                        * Referenced by: '<S126>/Constant'
                                        */
  40.0,                                /* Expression: f
                                        * Referenced by: '<S126>/Constant1'
                                        */
  0.01,                                /* Expression: T
                                        * Referenced by: '<S126>/Constant2'
                                        */
  40.0,                                /* Expression: f
                                        * Referenced by: '<S126>/Constant3'
                                        */
  0.01,                                /* Expression: T
                                        * Referenced by: '<S126>/Constant4'
                                        */
  40.0,                                /* Expression: f
                                        * Referenced by: '<S126>/Constant5'
                                        */
  0.01,                                /* Expression: T
                                        * Referenced by: '<S124>/Constant'
                                        */
  40.0,                                /* Expression: f
                                        * Referenced by: '<S124>/Constant1'
                                        */
  0.01,                                /* Expression: T
                                        * Referenced by: '<S124>/Constant2'
                                        */
  40.0,                                /* Expression: f
                                        * Referenced by: '<S124>/Constant3'
                                        */
  0.01,                                /* Expression: T
                                        * Referenced by: '<S124>/Constant4'
                                        */
  40.0,                                /* Expression: f
                                        * Referenced by: '<S124>/Constant5'
                                        */
  1.0,                                 /* Computed Parameter: DiscreteZeroPole_A_f
                                        * Referenced by: '<S109>/Discrete Zero-Pole'
                                        */
  0.01,                                /* Computed Parameter: DiscreteZeroPole_C_m
                                        * Referenced by: '<S109>/Discrete Zero-Pole'
                                        */
  0.005,                               /* Computed Parameter: DiscreteZeroPole_D_d
                                        * Referenced by: '<S109>/Discrete Zero-Pole'
                                        */
  0.01,                                /* Expression: T
                                        * Referenced by: '<S159>/Constant'
                                        */
  4.0,                                 /* Expression: f
                                        * Referenced by: '<S159>/Constant1'
                                        */
  100.0,                               /* Expression: 100
                                        * Referenced by: '<S118>/Gain'
                                        */
  0.01,                                /* Expression: T
                                        * Referenced by: '<S158>/Constant'
                                        */
  0.02,                                /* Expression: f
                                        * Referenced by: '<S158>/Constant1'
                                        */
  0.0F,                                /* Computed Parameter: Vn_fil_Y0
                                        * Referenced by: '<S94>/Vn_fil'
                                        */
  0.0F,                                /* Computed Parameter: Ve_fil_Y0
                                        * Referenced by: '<S94>/Ve_fil'
                                        */

  /*  Computed Parameter: Constant1_Value_a
   * Referenced by: '<S3>/Constant1'
   */
  { 0.0F, 0.0F, 0.0F },

  /*  Computed Parameter: Constant_Value_ij
   * Referenced by: '<S3>/Constant'
   */
  { 0.0F, 0.0F, 0.0F },
  143.543F,                            /* Computed Parameter: Out1_Y0
                                        * Referenced by: '<S162>/Out1'
                                        */
  120000.0F,                           /* Computed Parameter: u0k120k_UpperSat
                                        * Referenced by: '<S165>/[80k - 120k]'
                                        */
  80000.0F,                            /* Computed Parameter: u0k120k_LowerSat
                                        * Referenced by: '<S165>/[80k - 120k]'
                                        */
  0.0F,                                /* Computed Parameter: IntegerDelay_InitialCondition
                                        * Referenced by: '<S166>/Integer Delay'
                                        */
  -0.0950433F,                         /* Computed Parameter: gains_Value
                                        * Referenced by: '<S170>/gains'
                                        */
  293.053F,                            /* Computed Parameter: MeanTemperatureforCalibration_V
                                        * Referenced by: '<S170>/Mean Temperature for Calibration'
                                        */
  -0.0552923F,                         /* Computed Parameter: gains_Value_k
                                        * Referenced by: '<S171>/gains'
                                        */
  -202.93F,                            /* Computed Parameter: MeanTemperatureforCalibration_i
                                        * Referenced by: '<S171>/Mean Temperature for Calibration'
                                        */
  -41.0F,                              /* Computed Parameter: Constant_Value_i5
                                        * Referenced by: '<S171>/Constant'
                                        */
  -6.0F,                               /* Computed Parameter: Constant_Value_h
                                        * Referenced by: '<S173>/Constant'
                                        */
  0.0207608F,                          /* Computed Parameter: gains_Value_m
                                        * Referenced by: '<S173>/gains'
                                        */
  347.23F,                             /* Computed Parameter: MeanTemperatureforCalibration_m
                                        * Referenced by: '<S173>/Mean Temperature for Calibration'
                                        */
  -0.0102663F,                         /* Computed Parameter: gains_Value_c
                                        * Referenced by: '<S174>/gains'
                                        */
  -161.3F,                             /* Computed Parameter: MeanTemperatureforCalibration_c
                                        * Referenced by: '<S174>/Mean Temperature for Calibration'
                                        */
  0.01F,                               /* Computed Parameter: DiscreteTimeIntegrator1_gainval
                                        * Referenced by: '<S12>/Discrete-Time Integrator1'
                                        */

  /*  Computed Parameter: DiscreteTimeIntegrator1_IC
   * Referenced by: '<S12>/Discrete-Time Integrator1'
   */
  { 1.0F, 0.0F, 0.0F, 0.0F },

  /*  Computed Parameter: DiscreteTimeIntegrator1_UpperSa
   * Referenced by: '<S12>/Discrete-Time Integrator1'
   */
  { 1.0F, 1.0F, 1.0F, 1.0F },

  /*  Computed Parameter: DiscreteTimeIntegrator1_LowerSa
   * Referenced by: '<S12>/Discrete-Time Integrator1'
   */
  { -1.0F, -1.0F, -1.0F, -1.0F },
  2.0F,                                /* Computed Parameter: Gain_Gain_e
                                        * Referenced by: '<S60>/Gain'
                                        */
  2.0F,                                /* Computed Parameter: Gain_Gain_d
                                        * Referenced by: '<S63>/Gain'
                                        */
  2.0F,                                /* Computed Parameter: Gain_Gain_j
                                        * Referenced by: '<S58>/Gain'
                                        */
  2.0F,                                /* Computed Parameter: Gain_Gain_l
                                        * Referenced by: '<S64>/Gain'
                                        */
  2.0F,                                /* Computed Parameter: Gain_Gain_p
                                        * Referenced by: '<S59>/Gain'
                                        */
  2.0F,                                /* Computed Parameter: Gain_Gain_k
                                        * Referenced by: '<S62>/Gain'
                                        */
  0.0F,                                /* Computed Parameter: Constant1_Value_f
                                        * Referenced by: '<S12>/Constant1'
                                        */
  0.0F,                                /* Computed Parameter: ZeroBound_UpperSat
                                        * Referenced by: '<S27>/Zero Bound'
                                        */
  0.001F,                              /* Computed Parameter: ZeroBound_LowerSat
                                        * Referenced by: '<S27>/Zero Bound'
                                        */
  0.0174532924F,                       /* Computed Parameter: Deg2R_Gain
                                        * Referenced by: '<S81>/Deg2R'
                                        */
  6.378137E+6F,                        /* Computed Parameter: Reequatorialradius_Value
                                        * Referenced by: '<S82>/Re=equatorial radius'
                                        */
  1.0F,                                /* Computed Parameter: const_Value
                                        * Referenced by: '<S82>/const'
                                        */
  1.0F,                                /* Computed Parameter: const2_Value
                                        * Referenced by: '<S85>/const2'
                                        */
  6.3567525E+6F,                       /* Computed Parameter: Rppolarradius_Value
                                        * Referenced by: '<S85>/Rp=polar radius'
                                        */
  6.378137E+6F,                        /* Computed Parameter: Reequatorialradius1_Value
                                        * Referenced by: '<S85>/Re=equatorial radius1'
                                        */
  0.0174532924F,                       /* Computed Parameter: Deg2R_Gain_c
                                        * Referenced by: '<S82>/Deg2R'
                                        */
  0.0174532924F,                       /* Computed Parameter: Deg2R1_Gain
                                        * Referenced by: '<S82>/Deg2R1'
                                        */
  1.0F,                                /* Computed Parameter: const1_Value
                                        * Referenced by: '<S82>/const1'
                                        */
  6.378137E+6F,                        /* Computed Parameter: Reequatorialradius_Value_b
                                        * Referenced by: '<S73>/Re=equatorial radius'
                                        */
  1.0F,                                /* Computed Parameter: const_Value_a
                                        * Referenced by: '<S73>/const'
                                        */
  1.0F,                                /* Computed Parameter: const2_Value_b
                                        * Referenced by: '<S74>/const2'
                                        */
  6.3567525E+6F,                       /* Computed Parameter: Rppolarradius_Value_f
                                        * Referenced by: '<S74>/Rp=polar radius'
                                        */
  6.378137E+6F,                        /* Computed Parameter: Reequatorialradius1_Value_j
                                        * Referenced by: '<S74>/Re=equatorial radius1'
                                        */
  0.0174532924F,                       /* Computed Parameter: Deg2R_Gain_g
                                        * Referenced by: '<S73>/Deg2R'
                                        */
  0.0174532924F,                       /* Computed Parameter: Deg2R1_Gain_k
                                        * Referenced by: '<S73>/Deg2R1'
                                        */
  1.0F,                                /* Computed Parameter: const1_Value_i
                                        * Referenced by: '<S73>/const1'
                                        */

  /*  Computed Parameter: UEN2NEU_Gain
   * Referenced by: '<S17>/UEN 2 NEU'
   */
  { 0.0F, 0.0F, 1.0F, 0.0F, 1.0F, 0.0F, 1.0F, 0.0F, 0.0F },
  0.0174532924F,                       /* Computed Parameter: UnitConversion_Gain
                                        * Referenced by: '<S70>/Unit Conversion'
                                        */
  0.0F,                                /* Computed Parameter: Constant_Value_hh
                                        * Referenced by: '<S13>/Constant'
                                        */
  30.0F,                               /* Computed Parameter: Gain1_Gain
                                        * Referenced by: '<S12>/Gain1'
                                        */
  0.0F,                                /* Computed Parameter: IntegerDelay_InitialCondition_c
                                        * Referenced by: '<S12>/Integer Delay'
                                        */
  0.0F,                                /* Computed Parameter: UD_X0
                                        * Referenced by: '<S78>/UD'
                                        */
  1.0F,                                /* Computed Parameter: Saturation1_UpperSat
                                        * Referenced by: '<S75>/Saturation1'
                                        */
  0.0F,                                /* Computed Parameter: Saturation1_LowerSat
                                        * Referenced by: '<S75>/Saturation1'
                                        */
  0.0F,                                /* Computed Parameter: UD_X0_g
                                        * Referenced by: '<S79>/UD'
                                        */
  1.0F,                                /* Computed Parameter: Saturation1_UpperSat_d
                                        * Referenced by: '<S76>/Saturation1'
                                        */
  0.0F,                                /* Computed Parameter: Saturation1_LowerSat_m
                                        * Referenced by: '<S76>/Saturation1'
                                        */
  0.0F,                                /* Computed Parameter: UD_X0_f
                                        * Referenced by: '<S80>/UD'
                                        */
  1.0F,                                /* Computed Parameter: Saturation1_UpperSat_m
                                        * Referenced by: '<S77>/Saturation1'
                                        */
  0.0F,                                /* Computed Parameter: Saturation1_LowerSat_j
                                        * Referenced by: '<S77>/Saturation1'
                                        */
  1.0F,                                /* Computed Parameter: Saturation1_UpperSat_e
                                        * Referenced by: '<S16>/Saturation1'
                                        */
  0.0F,                                /* Computed Parameter: Saturation1_LowerSat_a
                                        * Referenced by: '<S16>/Saturation1'
                                        */
  7.0F,                                /* Computed Parameter: Constant_Value_jv
                                        * Referenced by: '<S92>/Constant'
                                        */

  /*  Computed Parameter: u11_Gain
   * Referenced by: '<S2>/[1 1 -1]'
   */
  { 1.0F, 1.0F, -1.0F },
  0.5F,                                /* Computed Parameter: Gain_Gain_ks
                                        * Referenced by: '<S87>/Gain'
                                        */
  27.127F,                             /* Computed Parameter: Gains_Value
                                        * Referenced by: '<S151>/Gains'
                                        */
  1.51128531F,                         /* Computed Parameter: Gains_Value_e
                                        * Referenced by: '<S156>/Gains'
                                        */
  -1605.28198F,                        /* Computed Parameter: Bias_Value
                                        * Referenced by: '<S156>/Bias'
                                        */
  -50.0F,                              /* Computed Parameter: Constant_Value_dn
                                        * Referenced by: '<S172>/Constant'
                                        */
  9444.44434F,                         /* Computed Parameter: Bias_Value_l
                                        * Referenced by: '<S151>/Bias'
                                        */
  0.093502529F,                        /* Computed Parameter: Constant3_Value_n
                                        * Referenced by: '<S161>/Constant3'
                                        */
  -0.188893303F,                       /* Computed Parameter: Constant4_Value_d
                                        * Referenced by: '<S161>/Constant4'
                                        */
  2.18031291E-5F,                      /* Computed Parameter: Constant5_Value_j
                                        * Referenced by: '<S161>/Constant5'
                                        */
  145473.5F,                           /* Computed Parameter: Constant2_Value_cf
                                        * Referenced by: '<S161>/Constant2'
                                        */
  0.3048F,                             /* Computed Parameter: UnitConversion_Gain_c
                                        * Referenced by: '<S167>/Unit Conversion'
                                        */
  -1.0F,                               /* Computed Parameter: Gain_Gain_a
                                        * Referenced by: '<S86>/Gain'
                                        */
  0.0F,                                /* Computed Parameter: UnitDelay_X0
                                        * Referenced by: '<S91>/Unit Delay'
                                        */
  2.0F,                                /* Computed Parameter: Gain_Gain_g
                                        * Referenced by: '<S91>/Gain'
                                        */
  19.5F,                               /* Computed Parameter: IntegerDelay1_InitialCondition
                                        * Referenced by: '<S23>/Integer Delay1'
                                        */
  20.0F,                               /* Computed Parameter: Gain2_Gain
                                        * Referenced by: '<S23>/Gain2'
                                        */
  0.0326839499F,                       /* Computed Parameter: GyroGains1_Value
                                        * Referenced by: '<S117>/Gyro Gains1'
                                        */
  19.5F,                               /* Computed Parameter: IntegerDelay1_InitialConditio_b
                                        * Referenced by: '<S93>/Integer Delay1'
                                        */
  20.0F,                               /* Computed Parameter: Gain2_Gain_l
                                        * Referenced by: '<S93>/Gain2'
                                        */
  9.896E-9F,                           /* Computed Parameter: RateLimiter_RisingLim
                                        * Referenced by: '<S86>/Rate Limiter'
                                        */
  -9.896E-9F,                          /* Computed Parameter: RateLimiter_FallingLim
                                        * Referenced by: '<S86>/Rate Limiter'
                                        */
  0.0F,                                /* Computed Parameter: RateLimiter_IC
                                        * Referenced by: '<S86>/Rate Limiter'
                                        */
  0.0F,                                /* Computed Parameter: ZeroBound_UpperSat_i
                                        * Referenced by: '<S26>/Zero Bound'
                                        */
  0.001F,                              /* Computed Parameter: ZeroBound_LowerSat_n
                                        * Referenced by: '<S26>/Zero Bound'
                                        */
  2.5572E-9F,                          /* Computed Parameter: BiasRateLimiter_RisingLim
                                        * Referenced by: '<S12>/Bias Rate Limiter'
                                        */
  -2.5572E-9F,                         /* Computed Parameter: BiasRateLimiter_FallingLim
                                        * Referenced by: '<S12>/Bias Rate Limiter'
                                        */
  0.0F,                                /* Computed Parameter: BiasRateLimiter_IC
                                        * Referenced by: '<S12>/Bias Rate Limiter'
                                        */
  0.0F,                                /* Computed Parameter: UnitDelay_X0_k
                                        * Referenced by: '<S89>/Unit Delay'
                                        */
  10.0F,                               /* Computed Parameter: Gain_Gain_b
                                        * Referenced by: '<S89>/Gain'
                                        */
  0.0F,                                /* Computed Parameter: UnitDelay_X0_l
                                        * Referenced by: '<S90>/Unit Delay'
                                        */
  10.0F,                               /* Computed Parameter: Gain_Gain_gt
                                        * Referenced by: '<S90>/Gain'
                                        */
  -1.0F,                               /* Computed Parameter: Gain1_Gain_a
                                        * Referenced by: '<S86>/Gain1'
                                        */
  0.5F,                                /* Computed Parameter: GyroGains2_Value
                                        * Referenced by: '<S117>/Gyro Gains2'
                                        */

  /*  Computed Parameter: u11_Gain_o
   * Referenced by: '<S129>/[ -1 -1 -1]'
   */
  { -1.0F, -1.0F, -1.0F },
  0.000872664619F,                     /* Computed Parameter: GyroGains_Value
                                        * Referenced by: '<S117>/Gyro Gains'
                                        */

  /*  Computed Parameter: u11_Gain_j
   * Referenced by: '<S127>/[ -1 -1 -1]'
   */
  { -1.0F, -1.0F, -1.0F },

  /*  Computed Parameter: Gain1_Gain_b
   * Referenced by: '<S44>/Gain1'
   */
  { 1.0F, 1.0F, -1.0F },

  /*  Computed Parameter: Gain2_Gain_o
   * Referenced by: '<S44>/Gain2'
   */
  { 1.0F, 1.0F },

  /*  Computed Parameter: Gain3_Gain
   * Referenced by: '<S44>/Gain3'
   */
  { -1.0F, 1.0F },
  0.1F,                                /* Computed Parameter: Gain_Gain_o
                                        * Referenced by: '<S12>/Gain'
                                        */
  6921.4F,                             /* Computed Parameter: Gains_Value_k
                                        * Referenced by: '<S152>/Gains'
                                        */
  -130.0F,                             /* Computed Parameter: Constant_Value_hm
                                        * Referenced by: '<S169>/Constant'
                                        */
  -9.2183E+6F,                         /* Computed Parameter: Bias_Value_b
                                        * Referenced by: '<S152>/Bias'
                                        */
  3000.0F,                             /* Computed Parameter: u001maxDynPress_UpperSat
                                        * Referenced by: '<S121>/[0.001  maxDynPress]'
                                        */
  0.001F,                              /* Computed Parameter: u001maxDynPress_LowerSat
                                        * Referenced by: '<S121>/[0.001  maxDynPress]'
                                        */
  3.17606163F,                         /* Computed Parameter: Gains_Value_k1
                                        * Referenced by: '<S153>/Gains'
                                        */
  911.698242F,                         /* Computed Parameter: Bias_Value_la
                                        * Referenced by: '<S153>/Bias'
                                        */
  0U,                                  /* Computed Parameter: Constant_Value_e
                                        * Referenced by: '<S114>/Constant'
                                        */
  0U,                                  /* Computed Parameter: Output_X0
                                        * Referenced by: '<S112>/Output'
                                        */
  1U,                                  /* Computed Parameter: FixPtConstant_Value
                                        * Referenced by: '<S113>/FixPt Constant'
                                        */
  4294967295U,                         /* Computed Parameter: FixPtSwitch_Threshold
                                        * Referenced by: '<S114>/FixPt Switch'
                                        */
  1U,                                  /* Computed Parameter: IntegerDelay_DelayLength
                                        * Referenced by: '<S166>/Integer Delay'
                                        */
  1U,                                  /* Computed Parameter: IntegerDelay_DelayLength_k
                                        * Referenced by: '<S12>/Integer Delay'
                                        */
  5U,                                  /* Computed Parameter: IntegerDelay1_DelayLength
                                        * Referenced by: '<S23>/Integer Delay1'
                                        */
  5U,                                  /* Computed Parameter: IntegerDelay1_DelayLength_j
                                        * Referenced by: '<S93>/Integer Delay1'
                                        */
  52429U,                              /* Computed Parameter: ConverttoMicroseconds_Gain
                                        * Referenced by: '<S10>/Convert to  Microseconds'
                                        */
  0U,                                  /* Computed Parameter: Switch2_Threshold
                                        * Referenced by: '<S119>/Switch2'
                                        */
  0U,                                  /* Computed Parameter: Switch_Threshold
                                        * Referenced by: '<S3>/Switch'
                                        */
  0U,                                  /* Computed Parameter: Switch6_Threshold
                                        * Referenced by: '<S3>/Switch6'
                                        */
  0U,                                  /* Computed Parameter: Switch4_Threshold
                                        * Referenced by: '<S3>/Switch4'
                                        */
  0U,                                  /* Computed Parameter: Switch1_Threshold
                                        * Referenced by: '<S3>/Switch1'
                                        */
  0U,                                  /* Computed Parameter: Switch5_Threshold
                                        * Referenced by: '<S3>/Switch5'
                                        */
  0U,                                  /* Computed Parameter: Switch2_Threshold_g
                                        * Referenced by: '<S3>/Switch2'
                                        */
  0U,                                  /* Computed Parameter: Switch3_Threshold
                                        * Referenced by: '<S3>/Switch3'
                                        */
  1,                                   /* Computed Parameter: Constant_Value_n3
                                        * Referenced by: '<Root>/Constant'
                                        */
  1,                                   /* Computed Parameter: Constant2_Value_e
                                        * Referenced by: '<S179>/Constant2'
                                        */
  0,                                   /* Computed Parameter: Constant1_Value_kt
                                        * Referenced by: '<S179>/Constant1'
                                        */

  /* Start of '<S94>/Embedded MATLAB Function2' */
  {
    0.01                               /* Expression: apSampleTime
                                        * Referenced by: '<S94>/Embedded MATLAB Function2'
                                        */
  }
  /* End of '<S94>/Embedded MATLAB Function2' */
  ,

  /* Start of '<S94>/Embedded MATLAB Function1' */
  {
    0.01                               /* Expression: apSampleTime
                                        * Referenced by: '<S94>/Embedded MATLAB Function1'
                                        */
  }
  /* End of '<S94>/Embedded MATLAB Function1' */
};

/* File trailer for Real-Time Workshop generated code.
 *
 * [EOF]
 */
