/*
 * File: sensorMCUSlugsMKII.c
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

#include "sensorMCUSlugsMKII.h"
#include "sensorMCUSlugsMKII_private.h"

/* Block signals (auto storage) */
BlockIO_sensorMCUSlugsMKII sensorMCUSlugsMKII_B;

/* Block states (auto storage) */
D_Work_sensorMCUSlugsMKII sensorMCUSlugsMKII_DWork;

/* Real-time model */
RT_MODEL_sensorMCUSlugsMKII sensorMCUSlugsMKII_M_;
RT_MODEL_sensorMCUSlugsMKII *const sensorMCUSlugsMKII_M = &sensorMCUSlugsMKII_M_;

/*
 * Output and update for atomic system:
 *    '<S19>/myMux Fun1'
 *    '<S88>/myMux Fun1'
 */
void sensorMCUSlugsMKII_myMuxFun1(real_T rtu_u1, real_T rtu_u2, real_T rtu_u3,
  rtB_myMuxFun1_sensorMCUSlugsMKI *localB)
{
  /* MATLAB Function 'Position and Attitude Filter/Attitude Complimentary Filter COG/3D Filter/myMux Fun1': '<S35>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S35>:1:5' */
  localB->y[0] = rtu_u1;
  localB->y[1] = rtu_u2;
  localB->y[2] = rtu_u3;
}

/*
 * Output and update for atomic system:
 *    '<S46>/negprotect'
 *    '<S51>/negprotect'
 */
void sensorMCUSlugsMKII_negprotect(real32_T rtu_val,
  rtB_negprotect_sensorMCUSlugsMK *localB)
{
  /* MATLAB Function 'Position and Attitude Filter/Attitude Complimentary Filter COG/Normalize Vector1/Norm/Subsystem2/negprotect': '<S48>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  if (rtu_val >= 0.001F) {
    /* '<S48>:1:5' */
    /* '<S48>:1:6' */
    localB->zpVal = rtu_val;
  } else {
    /* '<S48>:1:8' */
    localB->zpVal = 0.001F;
  }
}

/*
 * Output and update for atomic system:
 *    '<S47>/Embedded MATLAB Function'
 *    '<S52>/Embedded MATLAB Function'
 */
void sensorMC_EmbeddedMATLABFunction(const real32_T rtu_x[3],
  rtB_EmbeddedMATLABFunction_sens *localB)
{
  /* MATLAB Function 'Position and Attitude Filter/Attitude Complimentary Filter COG/Normalize Vector1/Norm/Subsystem3/Embedded MATLAB Function': '<S49>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S49>:1:5' */
  localB->xDoty = (rtu_x[0] * rtu_x[0] + rtu_x[1] * rtu_x[1]) + rtu_x[2] *
    rtu_x[2];
}

/*
 * Initial conditions for atomic system:
 *    '<S94>/Embedded MATLAB Function1'
 *    '<S94>/Embedded MATLAB Function2'
 */
void se_EmbeddedMATLABFunction1_Init(rtDW_EmbeddedMATLABFunction1_se *localDW)
{
  localDW->LastU_not_empty = FALSE;
}

/*
 * Output and update for atomic system:
 *    '<S94>/Embedded MATLAB Function1'
 *    '<S94>/Embedded MATLAB Function2'
 */
void sensorM_EmbeddedMATLABFunction1(real32_T rtu_u, uint8_T rtu_NewGPS,
  rtB_EmbeddedMATLABFunction1_sen *localB, rtDW_EmbeddedMATLABFunction1_se
  *localDW, rtP_EmbeddedMATLABFunction1_sen *localP)
{
  /* MATLAB Function 'Position and Attitude Filter/Position Filter/BlendPosVel/Subsystem/Embedded MATLAB Function1': '<S110>:1' */
  if (!localDW->LastU_not_empty) {
    /* '<S110>:1:7' */
    /* '<S110>:1:8' */
    localDW->LastU = 0.0F;
    localDW->LastU_not_empty = TRUE;

    /* '<S110>:1:9' */
    localDW->TimeSinceLast = (real32_T)localP->SFunction_p1;

    /* '<S110>:1:10' */
    localDW->rate = 0.0F;

    /* '<S110>:1:11' */
    localDW->OldRate = 0.0F;
  }

  if (rtu_NewGPS != 0) {
    /* '<S110>:1:15' */
    localDW->OldRate = localDW->rate;

    /* '<S110>:1:16' */
    localDW->rate = (rtu_u - localDW->LastU) / localDW->TimeSinceLast;

    /* '<S110>:1:17' */
    localDW->TimeSinceLast = 0.0F;

    /* '<S110>:1:18' */
    localDW->LastU = rtu_u;
  }

  /* '<S110>:1:20' */
  localB->y = (0.5F * localDW->rate + 0.5F * localDW->OldRate) *
    localDW->TimeSinceLast + rtu_u;

  /* '<S110>:1:21' */
  localDW->TimeSinceLast = localDW->TimeSinceLast + (real32_T)
    localP->SFunction_p1;
}

/*
 * Output and update for atomic system:
 *    '<S86>/myMux Fun1'
 *    '<S86>/myMux Fun2'
 */
void sensorMCUSlugsMKII_myMuxFun1_l(real32_T rtu_u1, real32_T rtu_u2, real32_T
  rtu_u3, rtB_myMuxFun1_sensorMCUSlugsM_n *localB)
{
  /* MATLAB Function 'Position and Attitude Filter/Position Filter/BlendPosVel/myMux Fun1': '<S95>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S95>:1:5' */
  localB->y[0] = rtu_u1;
  localB->y[1] = rtu_u2;
  localB->y[2] = rtu_u3;
}

/*
 * Initial conditions for atomic system:
 *    '<S124>/Embedded MATLAB Function'
 *    '<S124>/Embedded MATLAB Function1'
 *    '<S124>/Embedded MATLAB Function2'
 *    '<S125>/Embedded MATLAB Function'
 *    '<S125>/Embedded MATLAB Function1'
 *    '<S125>/Embedded MATLAB Function2'
 *    '<S126>/Embedded MATLAB Function'
 *    '<S126>/Embedded MATLAB Function1'
 *    '<S126>/Embedded MATLAB Function2'
 *    '<S159>/Embedded MATLAB Function'
 *    ...
 */
void s_EmbeddedMATLABFunction_p_Init(rtDW_EmbeddedMATLABFunction_s_i *localDW)
{
  localDW->a_not_empty = FALSE;
}

/*
 * Output and update for atomic system:
 *    '<S124>/Embedded MATLAB Function'
 *    '<S124>/Embedded MATLAB Function1'
 *    '<S124>/Embedded MATLAB Function2'
 *    '<S125>/Embedded MATLAB Function'
 *    '<S125>/Embedded MATLAB Function1'
 *    '<S125>/Embedded MATLAB Function2'
 *    '<S126>/Embedded MATLAB Function'
 *    '<S126>/Embedded MATLAB Function1'
 *    '<S126>/Embedded MATLAB Function2'
 *    '<S159>/Embedded MATLAB Function'
 *    ...
 */
void sensor_EmbeddedMATLABFunction_c(real_T rtu_u, real_T rtu_T, real_T rtu_f,
  rtB_EmbeddedMATLABFunction_se_k *localB, rtDW_EmbeddedMATLABFunction_s_i
  *localDW)
{
  real_T omega;

  /* MATLAB Function 'Sensor Data/Sensor Suite/ADIS16405 Signal Cond /3D Tustin Lowpass, Auto Initial Condition/Embedded MATLAB Function': '<S130>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  if (!localDW->a_not_empty) {
    /* '<S130>:1:9' */
    /* tf = [T*omega/(2+T*omega) T*omega/(2+T*omega)],[1 (T*omega-2)/(T*omega+2)] */
    /* tf = [a a],[1 -b] */
    /* '<S130>:1:12' */
    omega = 6.2831853071795862 * rtu_f;

    /* '<S130>:1:13' */
    localDW->a = rtu_T * omega / (rtu_T * omega + 2.0);
    localDW->a_not_empty = TRUE;

    /* '<S130>:1:14' */
    localDW->b = -(rtu_T * omega - 2.0) / (rtu_T * omega + 2.0);

    /* '<S130>:1:15' */
    localDW->y_km1 = rtu_u;

    /* '<S130>:1:16' */
    localDW->u_km1 = rtu_u;
  }

  /* '<S130>:1:19' */
  localB->y = (rtu_u + localDW->u_km1) * localDW->a + localDW->b *
    localDW->y_km1;

  /* '<S130>:1:20' */
  localDW->y_km1 = localB->y;

  /* '<S130>:1:21' */
  localDW->u_km1 = rtu_u;
}

/*
 * Output and update for atomic system:
 *    '<S124>/myMux Fun'
 *    '<S125>/myMux Fun'
 *    '<S126>/myMux Fun'
 */
void sensorMCUSlugsMKII_myMuxFun(real_T rtu_u1, real_T rtu_u2, real_T rtu_u3,
  rtB_myMuxFun_sensorMCUSlugsMKII *localB)
{
  /* MATLAB Function 'Sensor Data/Sensor Suite/ADIS16405 Signal Cond /3D Tustin Lowpass, Auto Initial Condition/myMux Fun': '<S133>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S133>:1:5' */
  localB->y[0] = rtu_u1;
  localB->y[1] = rtu_u2;
  localB->y[2] = rtu_u3;
}

/*
 * Output and update for atomic system:
 *    '<S119>/Embedded MATLAB Function'
 *    '<S119>/Embedded MATLAB Function1'
 *    '<S119>/Embedded MATLAB Function2'
 *    '<S119>/Embedded MATLAB Function3'
 */
void sensor_EmbeddedMATLABFunction_g(const uint16_T rtu_u[5],
  rtB_EmbeddedMATLABFunction_se_l *localB)
{
  uint32_T tmp;

  /* MATLAB Function 'Sensor Data/Sensor Suite/If no HIL then Read all the Sensors/Embedded MATLAB Function': '<S142>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S142>:1:5' */
  tmp = (uint32_T)rtu_u[0] + (uint32_T)rtu_u[1];
  if ((int32_T)tmp > 65535L) {
    tmp = 65535UL;
  }

  tmp += (uint32_T)rtu_u[2];
  if ((int32_T)tmp > 65535L) {
    tmp = 65535UL;
  }

  tmp += (uint32_T)rtu_u[3];
  if ((int32_T)tmp > 65535L) {
    tmp = 65535UL;
  }

  tmp += (uint32_T)rtu_u[4];
  if ((int32_T)tmp > 65535L) {
    tmp = 65535UL;
  }

  localB->y = (uint16_T)(int16_T)floor((real_T)tmp * 0.2 + 0.5);
}

/*
 * Output and update for atomic system:
 *    '<Root>/myMux Fun2'
 *    '<Root>/myMux Fun3'
 *    '<Root>/myMux Fun4'
 */
void sensorMCUSlugsMKII_myMuxFun2(const real32_T rtu_u1[3], const real32_T
  rtu_u2[3], rtB_myMuxFun2_sensorMCUSlugsMKI *localB)
{
  /* MATLAB Function 'myMux Fun2': '<S7>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S7>:1:5' */
  localB->y[0] = rtu_u1[0];
  localB->y[1] = rtu_u1[1];
  localB->y[2] = rtu_u1[2];
  localB->y[3] = rtu_u2[0];
  localB->y[4] = rtu_u2[1];
  localB->y[5] = rtu_u2[2];
}

real32_T rt_atan2f_snf(real32_T u0, real32_T u1)
{
  real32_T y;
  if (rtIsNaNF(u0) || rtIsNaNF(u1)) {
    y = (rtNaNF);
  } else if (rtIsInfF(u0) && rtIsInfF(u1)) {
    y = (real32_T)atan2(u0 > 0.0F ? 1.0F : -1.0F, u1 > 0.0F ? 1.0F : -1.0F);
  } else if (u1 == 0.0F) {
    if (u0 > 0.0F) {
      y = RT_PIF / 2.0F;
    } else if (u0 < 0.0F) {
      y = -(RT_PIF / 2.0F);
    } else {
      y = 0.0F;
    }
  } else {
    y = (real32_T)atan2(u0, u1);
  }

  return y;
}

/* Model step function */
void sensorMCUSlugsMKII_step(void)
{
  /* local block i/o variables */
  real_T rtb_DataTypeConversion5;
  real_T rtb_DataTypeConversion19;
  real_T rtb_DataTypeConversion1[3];
  real_T rtb_DataTypeConversion2[3];
  real_T rtb_DiscreteZeroPole;
  real_T rtb_DiscreteZeroPole_g;
  real_T rtb_DiscreteZeroPole_p;
  real_T rtb_DataTypeConversion2_d[3];
  real_T rtb_DiscreteZeroPole_gx;
  real_T rtb_DiscreteZeroPole_pz;
  real_T rtb_DiscreteZeroPole_n;
  real_T rtb_DataTypeConversion2_h;
  real_T rtb_DataTypeConversion2_i;
  real_T rtb_DataTypeConversion5_e[3];
  real_T rtb_DataTypeConversion2_g[3];
  real_T rtb_DataTypeConversion2_c;
  real_T rtb_DataTypeConversion3;
  real_T rtb_DataTypeConversion6;
  real_T rtb_DataTypeConversion;
  real_T rtb_tOut;
  real_T rtb_DiscreteZeroPole_h;
  real_T rtb_RoundingFunction;
  real32_T rtb_Product1_c;
  real32_T rtb_Product_j;
  real32_T rtb_Gain_a;
  real32_T rtb_Add[3];
  real32_T rtb_Switch[3];
  real32_T rtb_Switch6[3];
  real32_T rtb_DataTypeConversion1_f;
  real32_T rtb_DataTypeConversion1_ff;
  real32_T rtb_Gain1_p;
  real32_T rtb_Switch4[3];
  real32_T rtb_Switch1[3];
  real32_T rtb_Switch2[3];
  real32_T rtb_Switch3[3];
  real32_T rtb_y_a[3];
  uint8_T rtb_DataTypeConversion2_n;
  real_T omega;
  uint8_T rtb_Product_b;
  real32_T rtb_Product_d;
  real32_T rtb_Product1_h;
  real32_T rtb_Product2_d;
  real32_T rtb_Product3_o;
  real32_T rtb_RhhcosphisinlambYe;
  real32_T rtb_RhhcosphicoslambXe;
  real32_T rtb_Rh;
  int32_T yIdx;
  int16_T colIdx;
  real32_T rtb_jxi;
  real32_T rtb_ixk;
  real32_T rtb_kxj;
  real32_T rtb_ixj;
  real32_T rtb_DataTypeConversion1_l;
  boolean_T rtb_LogicalOperator;
  real32_T rtb_Sum_l;
  int16_T rtb_Switch_k[13];
  uint8_T rtb_Compare_j;
  uint32_T rtb_Output;
  int16_T rtb_DataTypeConversion2_m;
  boolean_T rtb_LogicalOperator_f;
  real32_T rtb_UEN2NEU[3];
  real32_T rtb_q_dot[4];
  real32_T Product[3];
  real32_T Product1_h[3];
  real32_T tmp[9];
  real32_T tmp_0[3];
  real32_T rtb_TmpSignalConversionAtSFun_0[9];
  real32_T rtb_Deg2R_idx;
  real32_T rtb_Deg2R_idx_0;
  real32_T rtb_GyroErr_idx;
  real32_T rtb_GyroErr_idx_0;
  real32_T rtb_GyroErr_idx_1;
  real32_T rtb_Sum4_g_idx;
  real32_T rtb_Sum4_g_idx_0;
  real32_T rtb_DataTypeConversion3_e_idx;
  real32_T rtb_Sum4_g_idx_1;
  real32_T rtb_DataTypeConversion3_e_idx_0;
  real32_T rtb_DataTypeConversion3_e_idx_1;
  real32_T rtb_u11_idx;
  real32_T rtb_u11_idx_0;
  real32_T rtb_Merge_idx;

  /* S-Function "dsPIC_Digital_OutputWrite" Block: <Root>/Drive SPI SSL High */
  LATBbits.LATB2 = sensorMCUSlugsMKII_P.Constant_Value_n3;

  /* S-Function "dsPIC_Digital_Input" Block: <S4>/Enable HIL from  Control MCU */
  sensorMCUSlugsMKII_B.EnableHILfromControlMCU = PORTDbits.RD2;

  /* DataTypeConversion: '<S4>/Data Type Conversion' */
  sensorMCUSlugsMKII_B.DataTypeConversion_e =
    sensorMCUSlugsMKII_B.EnableHILfromControlMCU;

  /* S-Function "dsPIC_Digital_Input" Block: <S3>/Enable HIL Attitude from Control MCU */
  sensorMCUSlugsMKII_B.EnableHILAttitudefromControlMCU = PORTAbits.RA14;

  /* Product: '<S3>/Product' incorporates:
   *  DataTypeConversion: '<S3>/Data Type Conversion'
   */
  rtb_Product_b = (uint8_T)((uint16_T)sensorMCUSlugsMKII_B.DataTypeConversion_e *
                            (uint16_T)
    sensorMCUSlugsMKII_B.EnableHILAttitudefromControlMCU);

  /* Sqrt: '<S55>/sqrt' incorporates:
   *  DiscreteIntegrator: '<S12>/Discrete-Time Integrator1'
   *  Product: '<S56>/Product'
   *  Product: '<S56>/Product1'
   *  Product: '<S56>/Product2'
   *  Product: '<S56>/Product3'
   *  Sum: '<S56>/Sum'
   */
  rtb_Merge_idx = (real32_T)sqrt
    (((sensorMCUSlugsMKII_DWork.DiscreteTimeIntegrator1_DSTATE[0] *
       sensorMCUSlugsMKII_DWork.DiscreteTimeIntegrator1_DSTATE[0] +
       sensorMCUSlugsMKII_DWork.DiscreteTimeIntegrator1_DSTATE[1] *
       sensorMCUSlugsMKII_DWork.DiscreteTimeIntegrator1_DSTATE[1]) +
      sensorMCUSlugsMKII_DWork.DiscreteTimeIntegrator1_DSTATE[2] *
      sensorMCUSlugsMKII_DWork.DiscreteTimeIntegrator1_DSTATE[2]) +
     sensorMCUSlugsMKII_DWork.DiscreteTimeIntegrator1_DSTATE[3] *
     sensorMCUSlugsMKII_DWork.DiscreteTimeIntegrator1_DSTATE[3]);

  /* Product: '<S28>/Product' incorporates:
   *  DiscreteIntegrator: '<S12>/Discrete-Time Integrator1'
   */
  rtb_Product_d = sensorMCUSlugsMKII_DWork.DiscreteTimeIntegrator1_DSTATE[0] /
    rtb_Merge_idx;

  /* Product: '<S28>/Product1' incorporates:
   *  DiscreteIntegrator: '<S12>/Discrete-Time Integrator1'
   */
  rtb_Product1_h = sensorMCUSlugsMKII_DWork.DiscreteTimeIntegrator1_DSTATE[1] /
    rtb_Merge_idx;

  /* Product: '<S28>/Product2' incorporates:
   *  DiscreteIntegrator: '<S12>/Discrete-Time Integrator1'
   */
  rtb_Product2_d = sensorMCUSlugsMKII_DWork.DiscreteTimeIntegrator1_DSTATE[2] /
    rtb_Merge_idx;

  /* Product: '<S28>/Product3' incorporates:
   *  DiscreteIntegrator: '<S12>/Discrete-Time Integrator1'
   */
  rtb_Product3_o = sensorMCUSlugsMKII_DWork.DiscreteTimeIntegrator1_DSTATE[3] /
    rtb_Merge_idx;

  /* Sqrt: '<S68>/sqrt' incorporates:
   *  Product: '<S69>/Product'
   *  Product: '<S69>/Product1'
   *  Product: '<S69>/Product2'
   *  Product: '<S69>/Product3'
   *  Sum: '<S69>/Sum'
   */
  rtb_Merge_idx = (real32_T)sqrt(((rtb_Product_d * rtb_Product_d +
    rtb_Product1_h * rtb_Product1_h) + rtb_Product2_d * rtb_Product2_d) +
    rtb_Product3_o * rtb_Product3_o);

  /* Product: '<S67>/Product' */
  rtb_RhhcosphisinlambYe = rtb_Product_d / rtb_Merge_idx;

  /* Product: '<S67>/Product1' */
  rtb_RhhcosphicoslambXe = rtb_Product1_h / rtb_Merge_idx;

  /* Product: '<S67>/Product2' */
  rtb_Rh = rtb_Product2_d / rtb_Merge_idx;

  /* Product: '<S67>/Product3' */
  rtb_Merge_idx = rtb_Product3_o / rtb_Merge_idx;

  /* Sum: '<S57>/Sum' incorporates:
   *  Product: '<S57>/Product'
   *  Product: '<S57>/Product1'
   *  Product: '<S57>/Product2'
   *  Product: '<S57>/Product3'
   */
  sensorMCUSlugsMKII_B.VectorConcatenate[0] = ((rtb_RhhcosphisinlambYe *
    rtb_RhhcosphisinlambYe + rtb_RhhcosphicoslambXe * rtb_RhhcosphicoslambXe) -
    rtb_Rh * rtb_Rh) - rtb_Merge_idx * rtb_Merge_idx;

  /* Gain: '<S60>/Gain' incorporates:
   *  Product: '<S60>/Product2'
   *  Product: '<S60>/Product3'
   *  Sum: '<S60>/Sum'
   */
  sensorMCUSlugsMKII_B.VectorConcatenate[1] = (rtb_RhhcosphicoslambXe * rtb_Rh -
    rtb_Merge_idx * rtb_RhhcosphisinlambYe) * sensorMCUSlugsMKII_P.Gain_Gain_e;

  /* Gain: '<S63>/Gain' incorporates:
   *  Product: '<S63>/Product1'
   *  Product: '<S63>/Product2'
   *  Sum: '<S63>/Sum'
   */
  sensorMCUSlugsMKII_B.VectorConcatenate[2] = (rtb_RhhcosphisinlambYe * rtb_Rh +
    rtb_RhhcosphicoslambXe * rtb_Merge_idx) * sensorMCUSlugsMKII_P.Gain_Gain_d;

  /* Gain: '<S58>/Gain' incorporates:
   *  Product: '<S58>/Product2'
   *  Product: '<S58>/Product3'
   *  Sum: '<S58>/Sum'
   */
  sensorMCUSlugsMKII_B.VectorConcatenate[3] = (rtb_Merge_idx *
    rtb_RhhcosphisinlambYe + rtb_RhhcosphicoslambXe * rtb_Rh) *
    sensorMCUSlugsMKII_P.Gain_Gain_j;

  /* Sum: '<S61>/Sum' incorporates:
   *  Product: '<S61>/Product'
   *  Product: '<S61>/Product1'
   *  Product: '<S61>/Product2'
   *  Product: '<S61>/Product3'
   */
  sensorMCUSlugsMKII_B.VectorConcatenate[4] = ((rtb_RhhcosphisinlambYe *
    rtb_RhhcosphisinlambYe - rtb_RhhcosphicoslambXe * rtb_RhhcosphicoslambXe) +
    rtb_Rh * rtb_Rh) - rtb_Merge_idx * rtb_Merge_idx;

  /* Gain: '<S64>/Gain' incorporates:
   *  Product: '<S64>/Product1'
   *  Product: '<S64>/Product2'
   *  Sum: '<S64>/Sum'
   */
  sensorMCUSlugsMKII_B.VectorConcatenate[5] = (rtb_Rh * rtb_Merge_idx -
    rtb_RhhcosphisinlambYe * rtb_RhhcosphicoslambXe) *
    sensorMCUSlugsMKII_P.Gain_Gain_l;

  /* Gain: '<S59>/Gain' incorporates:
   *  Product: '<S59>/Product1'
   *  Product: '<S59>/Product2'
   *  Sum: '<S59>/Sum'
   */
  sensorMCUSlugsMKII_B.VectorConcatenate[6] = (rtb_RhhcosphicoslambXe *
    rtb_Merge_idx - rtb_RhhcosphisinlambYe * rtb_Rh) *
    sensorMCUSlugsMKII_P.Gain_Gain_p;

  /* Gain: '<S62>/Gain' incorporates:
   *  Product: '<S62>/Product1'
   *  Product: '<S62>/Product2'
   *  Sum: '<S62>/Sum'
   */
  sensorMCUSlugsMKII_B.VectorConcatenate[7] = (rtb_RhhcosphisinlambYe *
    rtb_RhhcosphicoslambXe + rtb_Rh * rtb_Merge_idx) *
    sensorMCUSlugsMKII_P.Gain_Gain_k;

  /* Sum: '<S65>/Sum' incorporates:
   *  Product: '<S65>/Product'
   *  Product: '<S65>/Product1'
   *  Product: '<S65>/Product2'
   *  Product: '<S65>/Product3'
   */
  sensorMCUSlugsMKII_B.VectorConcatenate[8] = ((rtb_RhhcosphisinlambYe *
    rtb_RhhcosphisinlambYe - rtb_RhhcosphicoslambXe * rtb_RhhcosphicoslambXe) -
    rtb_Rh * rtb_Rh) + rtb_Merge_idx * rtb_Merge_idx;

  /* S-Function (sdspsubmtrx): '<S12>/Submatrix1' */
  yIdx = 0L;
  for (colIdx = 0; colIdx < 3; colIdx++) {
    sensorMCUSlugsMKII_B.Submatrix1[yIdx] =
      sensorMCUSlugsMKII_B.VectorConcatenate[(int32_T)(colIdx * 3)];
    yIdx++;
  }

  /* End of S-Function (sdspsubmtrx): '<S12>/Submatrix1' */

  /* MATLAB Function: '<S12>/myMux Fun2' incorporates:
   *  Constant: '<S12>/Constant1'
   */
  /* MATLAB Function 'Position and Attitude Filter/Attitude Complimentary Filter COG/myMux Fun2': '<S30>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S30>:1:5' */
  rtb_y_a[0] = sensorMCUSlugsMKII_B.Submatrix1[0];
  rtb_y_a[1] = sensorMCUSlugsMKII_B.Submatrix1[1];
  rtb_y_a[2] = sensorMCUSlugsMKII_P.Constant1_Value_f;

  /* MATLAB Function: '<S52>/Embedded MATLAB Function' */
  sensorMC_EmbeddedMATLABFunction(rtb_y_a,
    &sensorMCUSlugsMKII_B.sf_EmbeddedMATLABFunction);

  /* MATLAB Function: '<S51>/negprotect' */
  sensorMCUSlugsMKII_negprotect
    (sensorMCUSlugsMKII_B.sf_EmbeddedMATLABFunction.xDoty,
     &sensorMCUSlugsMKII_B.sf_negprotect);

  /* S-Function "dsPIC_C_function_Call" Block: <S51>/C Function Call */
  sensorMCUSlugsMKII_B.CFunctionCall = mySqrt
    (sensorMCUSlugsMKII_B.sf_negprotect.zpVal);

  /* Saturate: '<S27>/Zero Bound' */
  rtb_Merge_idx = sensorMCUSlugsMKII_B.CFunctionCall >=
    sensorMCUSlugsMKII_P.ZeroBound_UpperSat ?
    sensorMCUSlugsMKII_P.ZeroBound_UpperSat : sensorMCUSlugsMKII_B.CFunctionCall
    <= sensorMCUSlugsMKII_P.ZeroBound_LowerSat ?
    sensorMCUSlugsMKII_P.ZeroBound_LowerSat : sensorMCUSlugsMKII_B.CFunctionCall;

  /* Product: '<S27>/Divide' */
  rtb_GyroErr_idx_1 = rtb_y_a[0] / rtb_Merge_idx;
  rtb_GyroErr_idx = rtb_y_a[1] / rtb_Merge_idx;
  rtb_GyroErr_idx_0 = rtb_y_a[2] / rtb_Merge_idx;

  /* S-Function "dsPIC_C_function_Call" Block: <S2>/Get the GS Location [updateSensorMCUState.c] */
  getGSLocation(&sensorMCUSlugsMKII_B.GettheGSLocationupdateSensorMCU[0]);

  /* Gain: '<S81>/Deg2R' */
  rtb_Deg2R_idx = sensorMCUSlugsMKII_P.Deg2R_Gain *
    sensorMCUSlugsMKII_B.GettheGSLocationupdateSensorMCU[2];
  rtb_Deg2R_idx_0 = sensorMCUSlugsMKII_P.Deg2R_Gain *
    sensorMCUSlugsMKII_B.GettheGSLocationupdateSensorMCU[1];

  /* Logic: '<S116>/Logical Operator' */
  rtb_LogicalOperator = !sensorMCUSlugsMKII_B.EnableHILfromControlMCU;

  /* Outputs for Enabled SubSystem: '<S116>/If no HIL then Read all the Sensors' incorporates:
   *  EnablePort: '<S119>/Enable'
   */
  if (rtb_LogicalOperator) {
    /* S-Function "ADC" Block: <S119>/ADC Input */
    {
      uint16_T i;
      for (i = 0; i < 5; i++) {
        sensorMCUSlugsMKII_B.Baro[i] = ADCBuffChannel0[i];
        sensorMCUSlugsMKII_B.Pitot[i] = ADCBuffChannel1[i];
        sensorMCUSlugsMKII_B.Power[i] = ADCBuffChannel2[i];
        sensorMCUSlugsMKII_B.Thermistor[i] = ADCBuffChannel3[i];
      }
    }

    /* MATLAB Function: '<S119>/Embedded MATLAB Function' */
    sensor_EmbeddedMATLABFunction_g(sensorMCUSlugsMKII_B.Baro,
      &sensorMCUSlugsMKII_B.sf_EmbeddedMATLABFunction_gn);

    /* DataTypeConversion: '<S119>/Data Type Conversion' */
    colIdx = (int16_T)sensorMCUSlugsMKII_B.sf_EmbeddedMATLABFunction_gn.y;

    /* S-Function "dsPIC_C_function_Call" Block: <S119>/Read the Pressure Data [pressure.c] */
    getPressure(&sensorMCUSlugsMKII_B.ReadthePressureDatapressurec[0]);

    /* MATLAB Function: '<S119>/Embedded MATLAB Function2' */
    sensor_EmbeddedMATLABFunction_g(sensorMCUSlugsMKII_B.Power,
      &sensorMCUSlugsMKII_B.sf_EmbeddedMATLABFunction2_dh);

    /* DataTypeConversion: '<S119>/Data Type Conversion2' */
    rtb_DataTypeConversion2_m = (int16_T)
      sensorMCUSlugsMKII_B.sf_EmbeddedMATLABFunction2_dh.y;

    /* MATLAB Function: '<S119>/myMux Fun' */
    /* MATLAB Function 'Sensor Data/Sensor Suite/If no HIL then Read all the Sensors/myMux Fun': '<S148>:1' */
    /*  This block supports an embeddable subset of the MATLAB language. */
    /*  See the help menu for details.  */
    /* '<S148>:1:5' */
    sensorMCUSlugsMKII_B.y_nt[0] = colIdx;
    sensorMCUSlugsMKII_B.y_nt[1] =
      sensorMCUSlugsMKII_B.ReadthePressureDatapressurec[0];
    sensorMCUSlugsMKII_B.y_nt[2] = rtb_DataTypeConversion2_m;
    sensorMCUSlugsMKII_B.y_nt[3] =
      sensorMCUSlugsMKII_B.ReadthePressureDatapressurec[1];

    /* S-Function "dsPIC_C_function_Call" Block: <S119>/Update State AP ADC Data [updateSensorMcuState.c] */
    updateRawADCData(sensorMCUSlugsMKII_B.y_nt);

    /* MATLAB Function: '<S119>/Embedded MATLAB Function1' */
    sensor_EmbeddedMATLABFunction_g(sensorMCUSlugsMKII_B.Pitot,
      &sensorMCUSlugsMKII_B.sf_EmbeddedMATLABFunction1_b);

    /* MATLAB Function: '<S119>/Embedded MATLAB Function3' */
    sensor_EmbeddedMATLABFunction_g(sensorMCUSlugsMKII_B.Thermistor,
      &sensorMCUSlugsMKII_B.sf_EmbeddedMATLABFunction3_c);

    /* S-Function "dsPIC_C_function_Call" Block: <S119>/Read the Cube Data [adisCube16405.c] */
    getCubeData(&sensorMCUSlugsMKII_B.ReadtheCubeDataadisCube16405c[0]);

    /* MATLAB Function: '<S119>/myMux Fun4' */
    /* MATLAB Function 'Sensor Data/Sensor Suite/If no HIL then Read all the Sensors/myMux Fun4': '<S149>:1' */
    /*  This block supports an embeddable subset of the MATLAB language. */
    /*  See the help menu for details.  */
    /* '<S149>:1:5' */
    sensorMCUSlugsMKII_B.y_h[0] =
      sensorMCUSlugsMKII_B.ReadtheCubeDataadisCube16405c[0];
    sensorMCUSlugsMKII_B.y_h[1] =
      sensorMCUSlugsMKII_B.ReadtheCubeDataadisCube16405c[1];
    sensorMCUSlugsMKII_B.y_h[2] =
      sensorMCUSlugsMKII_B.ReadtheCubeDataadisCube16405c[2];
    sensorMCUSlugsMKII_B.y_h[3] =
      sensorMCUSlugsMKII_B.ReadtheCubeDataadisCube16405c[3];
    sensorMCUSlugsMKII_B.y_h[4] =
      sensorMCUSlugsMKII_B.ReadtheCubeDataadisCube16405c[4];
    sensorMCUSlugsMKII_B.y_h[5] =
      sensorMCUSlugsMKII_B.ReadtheCubeDataadisCube16405c[5];
    sensorMCUSlugsMKII_B.y_h[6] =
      sensorMCUSlugsMKII_B.ReadtheCubeDataadisCube16405c[6];
    sensorMCUSlugsMKII_B.y_h[7] =
      sensorMCUSlugsMKII_B.ReadtheCubeDataadisCube16405c[7];
    sensorMCUSlugsMKII_B.y_h[8] =
      sensorMCUSlugsMKII_B.ReadtheCubeDataadisCube16405c[8];
    sensorMCUSlugsMKII_B.y_h[9] = colIdx;
    sensorMCUSlugsMKII_B.y_h[10] =
      sensorMCUSlugsMKII_B.ReadthePressureDatapressurec[0];
    sensorMCUSlugsMKII_B.y_h[11] = rtb_DataTypeConversion2_m;
    sensorMCUSlugsMKII_B.y_h[12] =
      sensorMCUSlugsMKII_B.ReadthePressureDatapressurec[1];

    /* S-Function "dsPIC_C_function_Call" Block: <S119>/Is the GPS Novatel or Ublox? [gpsPort.c] */
    sensorMCUSlugsMKII_B.IstheGPSNovatelorUbloxgpsPortc = isGPSNovatel();

    /* Logic: '<S119>/Logical Operator' */
    rtb_LogicalOperator_f =
      !(sensorMCUSlugsMKII_B.IstheGPSNovatelorUbloxgpsPortc != 0);

    /* Outputs for Enabled SubSystem: '<S119>/if GPS is Novatel' incorporates:
     *  EnablePort: '<S146>/Enable'
     */
    if (sensorMCUSlugsMKII_B.IstheGPSNovatelorUbloxgpsPortc > 0) {
      /* S-Function "dsPIC_C_function_Call" Block: <S146>/Read the Raw Data from GPS [gpsPort.c] */
      getGPSRawData(&sensorMCUSlugsMKII_B.ReadtheRawDatafromGPSgpsPortc[0]);

      /* S-Function "dsPIC_C_function_Call" Block: <S146>/Parse the GPS RAW Data [gps.c//novatel.c] */
      gpsParse(sensorMCUSlugsMKII_B.ReadtheRawDatafromGPSgpsPortc);

      /* S-Function "dsPIC_C_function_Call" Block: <S146>/Produce the GPS Main Data and update the AP State (lat lon hei cog sog) [gps.c//novatel.c] */
      getGpsMainData(&sensorMCUSlugsMKII_B.ProducetheGPSMainDataandupdat_c[0]);
    }

    /* End of Outputs for SubSystem: '<S119>/if GPS is Novatel' */

    /* Outputs for Enabled SubSystem: '<S119>/if GPS is Ublox' incorporates:
     *  EnablePort: '<S147>/Enable'
     */
    if (rtb_LogicalOperator_f) {
      /* S-Function "dsPIC_C_function_Call" Block: <S147>/Produce the GPS Main Data and update the AP State (lat lon hei cog sog) [gpsUblox.c] */
      getGpsUbloxMainData(&sensorMCUSlugsMKII_B.ProducetheGPSMainDataandupdatet
                          [0]);
    }

    /* End of Outputs for SubSystem: '<S119>/if GPS is Ublox' */

    /* Switch: '<S119>/Switch2' */
    for (colIdx = 0; colIdx < 5; colIdx++) {
      if (sensorMCUSlugsMKII_B.IstheGPSNovatelorUbloxgpsPortc >
          sensorMCUSlugsMKII_P.Switch2_Threshold) {
        sensorMCUSlugsMKII_B.Switch2[colIdx] =
          sensorMCUSlugsMKII_B.ProducetheGPSMainDataandupdat_c[colIdx];
      } else {
        sensorMCUSlugsMKII_B.Switch2[colIdx] =
          sensorMCUSlugsMKII_B.ProducetheGPSMainDataandupdatet[colIdx];
      }
    }

    /* End of Switch: '<S119>/Switch2' */
  }

  /* End of Outputs for SubSystem: '<S116>/If no HIL then Read all the Sensors' */

  /* Product: '<S85>/e^1' incorporates:
   *  Constant: '<S85>/Re=equatorial radius1'
   *  Constant: '<S85>/Rp=polar radius'
   */
  rtb_DataTypeConversion1_l = sensorMCUSlugsMKII_P.Rppolarradius_Value /
    sensorMCUSlugsMKII_P.Reequatorialradius1_Value;

  /* Sum: '<S85>/Sum5' incorporates:
   *  Constant: '<S85>/const2'
   *  Product: '<S85>/ '
   */
  rtb_DataTypeConversion1_l = sensorMCUSlugsMKII_P.const2_Value -
    rtb_DataTypeConversion1_l * rtb_DataTypeConversion1_l;

  /* Gain: '<S82>/Deg2R' */
  rtb_ixj = sensorMCUSlugsMKII_P.Deg2R_Gain_c * sensorMCUSlugsMKII_B.Switch2[0];

  /* Trigonometry: '<S82>/sin(phi)' */
  rtb_kxj = (real32_T)sin(rtb_ixj);

  /* Sum: '<S82>/Sum1' incorporates:
   *  Constant: '<S82>/const'
   *  Product: '<S82>/Product1'
   *  Product: '<S82>/sin(phi)^2'
   */
  rtb_Merge_idx = sensorMCUSlugsMKII_P.const_Value - rtb_kxj * rtb_kxj *
    rtb_DataTypeConversion1_l;

  /* Fcn: '<S82>/f' */
  if (rtb_Merge_idx < 0.0F) {
    rtb_Sum_l = -(real32_T)sqrt(-rtb_Merge_idx);
  } else {
    rtb_Sum_l = (real32_T)sqrt(rtb_Merge_idx);
  }

  /* End of Fcn: '<S82>/f' */

  /* Product: '<S82>/Rh' incorporates:
   *  Constant: '<S82>/Re=equatorial radius'
   */
  rtb_ixk = sensorMCUSlugsMKII_P.Reequatorialradius_Value / rtb_Sum_l;

  /* Sum: '<S82>/Sum2' */
  rtb_jxi = sensorMCUSlugsMKII_B.Switch2[2] + rtb_ixk;

  /* Trigonometry: '<S82>/cos(phi)' */
  rtb_ixj = (real32_T)cos(rtb_ixj);

  /* Gain: '<S82>/Deg2R1' */
  rtb_Rh = sensorMCUSlugsMKII_P.Deg2R1_Gain * sensorMCUSlugsMKII_B.Switch2[1];

  /* Product: '<S82>/(Rh+h)cos(phi)*cos(lamb)=Xe' incorporates:
   *  Trigonometry: '<S82>/cos(lamb)'
   */
  rtb_RhhcosphicoslambXe = rtb_jxi * rtb_ixj * (real32_T)cos(rtb_Rh);

  /* Product: '<S82>/(Rh+h)cos(phi)*sin(lamb)=Ye' incorporates:
   *  Trigonometry: '<S82>/sin(lamb)'
   */
  rtb_jxi = rtb_jxi * rtb_ixj * (real32_T)sin(rtb_Rh);

  /* Product: '<S82>/Ze' incorporates:
   *  Constant: '<S82>/const1'
   *  Product: '<S82>/Rh(1-e^2)'
   *  Sum: '<S82>/Sum3'
   *  Sum: '<S82>/Sum4'
   */
  rtb_kxj *= (sensorMCUSlugsMKII_P.const1_Value - rtb_DataTypeConversion1_l) *
    rtb_ixk + sensorMCUSlugsMKII_B.Switch2[2];

  /* Product: '<S74>/e^1' incorporates:
   *  Constant: '<S74>/Re=equatorial radius1'
   *  Constant: '<S74>/Rp=polar radius'
   */
  rtb_DataTypeConversion1_l = sensorMCUSlugsMKII_P.Rppolarradius_Value_f /
    sensorMCUSlugsMKII_P.Reequatorialradius1_Value_j;

  /* Sum: '<S74>/Sum5' incorporates:
   *  Constant: '<S74>/const2'
   *  Product: '<S74>/ '
   */
  rtb_DataTypeConversion1_l = sensorMCUSlugsMKII_P.const2_Value_b -
    rtb_DataTypeConversion1_l * rtb_DataTypeConversion1_l;

  /* Gain: '<S73>/Deg2R' */
  rtb_ixj = sensorMCUSlugsMKII_P.Deg2R_Gain_g *
    sensorMCUSlugsMKII_B.GettheGSLocationupdateSensorMCU[1];

  /* Trigonometry: '<S73>/sin(phi)' */
  rtb_ixk = (real32_T)sin(rtb_ixj);

  /* Sum: '<S73>/Sum1' incorporates:
   *  Constant: '<S73>/const'
   *  Product: '<S73>/Product1'
   *  Product: '<S73>/sin(phi)^2'
   */
  rtb_Merge_idx = sensorMCUSlugsMKII_P.const_Value_a - rtb_ixk * rtb_ixk *
    rtb_DataTypeConversion1_l;

  /* Fcn: '<S73>/f' */
  if (rtb_Merge_idx < 0.0F) {
    rtb_Sum_l = -(real32_T)sqrt(-rtb_Merge_idx);
  } else {
    rtb_Sum_l = (real32_T)sqrt(rtb_Merge_idx);
  }

  /* End of Fcn: '<S73>/f' */

  /* Product: '<S73>/Rh' incorporates:
   *  Constant: '<S73>/Re=equatorial radius'
   */
  rtb_Rh = sensorMCUSlugsMKII_P.Reequatorialradius_Value_b / rtb_Sum_l;

  /* Sum: '<S73>/Sum2' */
  rtb_RhhcosphisinlambYe = sensorMCUSlugsMKII_B.GettheGSLocationupdateSensorMCU
    [0] + rtb_Rh;

  /* Trigonometry: '<S73>/cos(phi)' */
  rtb_ixj = (real32_T)cos(rtb_ixj);

  /* Gain: '<S73>/Deg2R1' */
  rtb_Merge_idx = sensorMCUSlugsMKII_P.Deg2R1_Gain_k *
    sensorMCUSlugsMKII_B.GettheGSLocationupdateSensorMCU[2];

  /* Product: '<S73>/(Rh+h)cos(phi)*cos(lamb)=Xe' incorporates:
   *  Trigonometry: '<S73>/cos(lamb)'
   */
  rtb_Sum_l = rtb_RhhcosphisinlambYe * rtb_ixj * (real32_T)cos(rtb_Merge_idx);

  /* Product: '<S73>/(Rh+h)cos(phi)*sin(lamb)=Ye' incorporates:
   *  Trigonometry: '<S73>/sin(lamb)'
   */
  rtb_RhhcosphisinlambYe = rtb_RhhcosphisinlambYe * rtb_ixj * (real32_T)sin
    (rtb_Merge_idx);

  /* Product: '<S73>/Ze' incorporates:
   *  Constant: '<S73>/const1'
   *  Product: '<S73>/Rh(1-e^2)'
   *  Sum: '<S73>/Sum3'
   *  Sum: '<S73>/Sum4'
   */
  rtb_ixk *= (sensorMCUSlugsMKII_P.const1_Value_i - rtb_DataTypeConversion1_l) *
    rtb_Rh + sensorMCUSlugsMKII_B.GettheGSLocationupdateSensorMCU[0];

  /* SignalConversion: '<S81>/TmpSignal ConversionAtProduct1Inport1' incorporates:
   *  Fcn: '<S84>/11'
   *  Fcn: '<S84>/12'
   *  Fcn: '<S84>/13'
   *  Fcn: '<S84>/21'
   *  Fcn: '<S84>/22'
   *  Fcn: '<S84>/31'
   *  Fcn: '<S84>/32'
   *  Fcn: '<S84>/33'
   */
  tmp[0L] = (real32_T)cos(rtb_Deg2R_idx) * (real32_T)cos(rtb_Deg2R_idx_0);
  tmp[1L] = -(real32_T)sin(rtb_Deg2R_idx);
  tmp[2L] = -(real32_T)sin(rtb_Deg2R_idx_0) * (real32_T)cos(rtb_Deg2R_idx);
  tmp[3L] = (real32_T)sin(rtb_Deg2R_idx) * (real32_T)cos(rtb_Deg2R_idx_0);
  tmp[4L] = (real32_T)cos(rtb_Deg2R_idx);
  tmp[5L] = -(real32_T)sin(rtb_Deg2R_idx) * (real32_T)sin(rtb_Deg2R_idx_0);
  tmp[6L] = (real32_T)sin(rtb_Deg2R_idx_0);
  tmp[7L] = 0.0F;
  tmp[8L] = (real32_T)cos(rtb_Deg2R_idx_0);

  /* Sum: '<S17>/Sum1' */
  rtb_Deg2R_idx_0 = rtb_RhhcosphicoslambXe - rtb_Sum_l;
  rtb_Sum_l = rtb_jxi - rtb_RhhcosphisinlambYe;
  rtb_Deg2R_idx = rtb_kxj - rtb_ixk;

  /* Product: '<S81>/Product1' incorporates:
   *  Gain: '<S17>/UEN 2 NEU'
   */
  for (colIdx = 0; colIdx < 3; colIdx++) {
    tmp_0[colIdx] = tmp[colIdx + 6] * rtb_Deg2R_idx + (tmp[colIdx + 3] *
      rtb_Sum_l + tmp[colIdx] * rtb_Deg2R_idx_0);
  }

  /* End of Product: '<S81>/Product1' */

  /* Gain: '<S17>/UEN 2 NEU' */
  for (colIdx = 0; colIdx < 3; colIdx++) {
    rtb_UEN2NEU[colIdx] = sensorMCUSlugsMKII_P.UEN2NEU_Gain[colIdx + 6] * tmp_0
      [2] + (sensorMCUSlugsMKII_P.UEN2NEU_Gain[colIdx + 3] * tmp_0[1] +
             sensorMCUSlugsMKII_P.UEN2NEU_Gain[colIdx] * tmp_0[0]);
  }

  /* S-Function "dsPIC_C_function_Call" Block: <S2>/Checks if FixType is 3 [updateSensorMCUState.c] */
  sensorMCUSlugsMKII_B.ChecksifFixTypeis3updateSensorM = isFixValid();

  /* Outputs for Enabled SubSystem: '<S2>/Enabled Subsystem' incorporates:
   *  EnablePort: '<S15>/Enable'
   */
  if (sensorMCUSlugsMKII_B.ChecksifFixTypeis3updateSensorM > 0) {
    /* Inport: '<S15>/In1' */
    sensorMCUSlugsMKII_B.In1_p[0] = rtb_UEN2NEU[0];
    sensorMCUSlugsMKII_B.In1_p[1] = rtb_UEN2NEU[1];
    sensorMCUSlugsMKII_B.In1_p[2] = rtb_UEN2NEU[2];

    /* Inport: '<S15>/In2' */
    sensorMCUSlugsMKII_B.In2 = sensorMCUSlugsMKII_B.Switch2[3];

    /* Inport: '<S15>/In3' */
    sensorMCUSlugsMKII_B.In3 = sensorMCUSlugsMKII_B.Switch2[4];
  }

  /* End of Outputs for SubSystem: '<S2>/Enabled Subsystem' */

  /* Gain: '<S70>/Unit Conversion' */
  sensorMCUSlugsMKII_B.UnitConversion = sensorMCUSlugsMKII_P.UnitConversion_Gain
    * sensorMCUSlugsMKII_B.In2;

  /* S-Function "dsPIC_C_function_Call" Block: <S71>/C Function Call1 */
  sensorMCUSlugsMKII_B.CFunctionCall1 = myCos
    (sensorMCUSlugsMKII_B.UnitConversion);

  /* S-Function "dsPIC_C_function_Call" Block: <S71>/C Function Call */
  sensorMCUSlugsMKII_B.CFunctionCall_k = mySin
    (sensorMCUSlugsMKII_B.UnitConversion);

  /* MATLAB Function: '<S13>/myMux Fun2' incorporates:
   *  Constant: '<S13>/Constant'
   */
  /* MATLAB Function 'Position and Attitude Filter/COG.SOG2V/myMux Fun2': '<S72>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S72>:1:5' */
  rtb_UEN2NEU[0] = sensorMCUSlugsMKII_B.CFunctionCall1;
  rtb_UEN2NEU[1] = sensorMCUSlugsMKII_B.CFunctionCall_k;
  rtb_UEN2NEU[2] = sensorMCUSlugsMKII_P.Constant_Value_hh;

  /* Sum: '<S22>/Sum' incorporates:
   *  Product: '<S40>/i x j'
   *  Product: '<S40>/j x k'
   *  Product: '<S40>/k x i'
   *  Product: '<S41>/i x k'
   *  Product: '<S41>/j x i'
   *  Product: '<S41>/k x j'
   */
  rtb_Deg2R_idx = rtb_GyroErr_idx * rtb_UEN2NEU[2] - rtb_GyroErr_idx_0 *
    rtb_UEN2NEU[1];
  rtb_Sum_l = rtb_GyroErr_idx_0 * rtb_UEN2NEU[0] - rtb_GyroErr_idx_1 *
    rtb_UEN2NEU[2];
  rtb_GyroErr_idx_0 = rtb_GyroErr_idx_1 * rtb_UEN2NEU[1] - rtb_GyroErr_idx *
    rtb_UEN2NEU[0];

  /* Product: '<S12>/Product2' incorporates:
   *  Gain: '<S12>/Gain1'
   */
  for (colIdx = 0; colIdx < 3; colIdx++) {
    tmp_0[colIdx] = sensorMCUSlugsMKII_B.VectorConcatenate[colIdx + 6] *
      rtb_GyroErr_idx_0 + (sensorMCUSlugsMKII_B.VectorConcatenate[colIdx + 3] *
      rtb_Sum_l + sensorMCUSlugsMKII_B.VectorConcatenate[colIdx] * rtb_Deg2R_idx);
  }

  /* End of Product: '<S12>/Product2' */

  /* Gain: '<S12>/Gain1' */
  rtb_jxi = sensorMCUSlugsMKII_P.Gain1_Gain * tmp_0[0];
  rtb_GyroErr_idx = sensorMCUSlugsMKII_P.Gain1_Gain * tmp_0[1];
  rtb_GyroErr_idx_1 = sensorMCUSlugsMKII_P.Gain1_Gain * tmp_0[2];

  /* Delay: '<S12>/Integer Delay' */
  rtb_Sum4_g_idx_1 = sensorMCUSlugsMKII_DWork.IntegerDelay_DSTATE[0];
  rtb_Sum4_g_idx = sensorMCUSlugsMKII_DWork.IntegerDelay_DSTATE[1];
  rtb_Sum4_g_idx_0 = sensorMCUSlugsMKII_DWork.IntegerDelay_DSTATE[2];

  /* Product: '<S13>/Product1' */
  rtb_Product1_c = sensorMCUSlugsMKII_B.In3 *
    sensorMCUSlugsMKII_B.CFunctionCall1;

  /* Product: '<S13>/Product' */
  rtb_Product_j = sensorMCUSlugsMKII_B.CFunctionCall_k *
    sensorMCUSlugsMKII_B.In3;

  /* Abs: '<S75>/Abs1' incorporates:
   *  Sum: '<S78>/Diff'
   *  UnitDelay: '<S78>/UD'
   */
  rtb_Deg2R_idx = (real32_T)fabs(sensorMCUSlugsMKII_B.In1_p[0] -
    sensorMCUSlugsMKII_DWork.UD_DSTATE);

  /* Abs: '<S76>/Abs1' incorporates:
   *  Sum: '<S79>/Diff'
   *  UnitDelay: '<S79>/UD'
   */
  rtb_Sum_l = (real32_T)fabs(sensorMCUSlugsMKII_B.In1_p[1] -
    sensorMCUSlugsMKII_DWork.UD_DSTATE_e);

  /* Abs: '<S77>/Abs1' incorporates:
   *  Sum: '<S80>/Diff'
   *  UnitDelay: '<S80>/UD'
   */
  rtb_Deg2R_idx_0 = (real32_T)fabs(sensorMCUSlugsMKII_B.In1_p[2] -
    sensorMCUSlugsMKII_DWork.UD_DSTATE_k);

  /* Sum: '<S16>/Sum' incorporates:
   *  Saturate: '<S75>/Saturation1'
   *  Saturate: '<S76>/Saturation1'
   *  Saturate: '<S77>/Saturation1'
   */
  rtb_Deg2R_idx = ((rtb_Deg2R_idx >= sensorMCUSlugsMKII_P.Saturation1_UpperSat ?
                    sensorMCUSlugsMKII_P.Saturation1_UpperSat : rtb_Deg2R_idx <=
                    sensorMCUSlugsMKII_P.Saturation1_LowerSat ?
                    sensorMCUSlugsMKII_P.Saturation1_LowerSat : rtb_Deg2R_idx) +
                   (rtb_Sum_l >= sensorMCUSlugsMKII_P.Saturation1_UpperSat_d ?
                    sensorMCUSlugsMKII_P.Saturation1_UpperSat_d : rtb_Sum_l <=
                    sensorMCUSlugsMKII_P.Saturation1_LowerSat_m ?
                    sensorMCUSlugsMKII_P.Saturation1_LowerSat_m : rtb_Sum_l)) +
    (rtb_Deg2R_idx_0 >= sensorMCUSlugsMKII_P.Saturation1_UpperSat_m ?
     sensorMCUSlugsMKII_P.Saturation1_UpperSat_m : rtb_Deg2R_idx_0 <=
     sensorMCUSlugsMKII_P.Saturation1_LowerSat_j ?
     sensorMCUSlugsMKII_P.Saturation1_LowerSat_j : rtb_Deg2R_idx_0);

  /* DataTypeConversion: '<S16>/Data Type Conversion2' incorporates:
   *  Saturate: '<S16>/Saturation1'
   */
  rtb_Deg2R_idx_0 = (real32_T)floor(rtb_Deg2R_idx >=
    sensorMCUSlugsMKII_P.Saturation1_UpperSat_e ?
    sensorMCUSlugsMKII_P.Saturation1_UpperSat_e : rtb_Deg2R_idx <=
    sensorMCUSlugsMKII_P.Saturation1_LowerSat_a ?
    sensorMCUSlugsMKII_P.Saturation1_LowerSat_a : rtb_Deg2R_idx);
  if (rtIsNaNF(rtb_Deg2R_idx_0) || rtIsInfF(rtb_Deg2R_idx_0)) {
    rtb_Deg2R_idx_0 = 0.0F;
  } else {
    rtb_Deg2R_idx_0 = (real32_T)fmod(rtb_Deg2R_idx_0, 256.0F);
  }

  rtb_DataTypeConversion2_n = (uint8_T)(rtb_Deg2R_idx_0 < 0.0F ? (uint8_T)
    (int16_T)(int8_T)-(int8_T)(uint8_T)-rtb_Deg2R_idx_0 : (uint8_T)
    rtb_Deg2R_idx_0);

  /* End of DataTypeConversion: '<S16>/Data Type Conversion2' */

  /* Outputs for Enabled SubSystem: '<S86>/Subsystem' incorporates:
   *  EnablePort: '<S94>/Enable'
   */
  /* RelationalOperator: '<S92>/Compare' incorporates:
   *  Constant: '<S92>/Constant'
   */
  if ((sensorMCUSlugsMKII_B.In3 > sensorMCUSlugsMKII_P.Constant_Value_jv) > 0) {
    if (!sensorMCUSlugsMKII_DWork.Subsystem_MODE) {
      sensorMCUSlugsMKII_DWork.Subsystem_MODE = TRUE;
    }

    /* MATLAB Function: '<S94>/Embedded MATLAB Function1' */
    sensorM_EmbeddedMATLABFunction1(rtb_Product_j, rtb_DataTypeConversion2_n,
      &sensorMCUSlugsMKII_B.sf_EmbeddedMATLABFunction1_a,
      &sensorMCUSlugsMKII_DWork.sf_EmbeddedMATLABFunction1_a,
      (rtP_EmbeddedMATLABFunction1_sen *)
      &sensorMCUSlugsMKII_P.sf_EmbeddedMATLABFunction1_a);

    /* MATLAB Function: '<S94>/Embedded MATLAB Function2' */
    sensorM_EmbeddedMATLABFunction1(rtb_Product1_c, rtb_DataTypeConversion2_n,
      &sensorMCUSlugsMKII_B.sf_EmbeddedMATLABFunction2_d,
      &sensorMCUSlugsMKII_DWork.sf_EmbeddedMATLABFunction2_d,
      (rtP_EmbeddedMATLABFunction1_sen *)
      &sensorMCUSlugsMKII_P.sf_EmbeddedMATLABFunction2_d);
  } else {
    if (sensorMCUSlugsMKII_DWork.Subsystem_MODE) {
      /* Disable for Outport: '<S94>/Vn_fil' */
      sensorMCUSlugsMKII_B.sf_EmbeddedMATLABFunction2_d.y =
        sensorMCUSlugsMKII_P.Vn_fil_Y0;

      /* Disable for Outport: '<S94>/Ve_fil' */
      sensorMCUSlugsMKII_B.sf_EmbeddedMATLABFunction1_a.y =
        sensorMCUSlugsMKII_P.Ve_fil_Y0;
      sensorMCUSlugsMKII_DWork.Subsystem_MODE = FALSE;
    }
  }

  /* End of RelationalOperator: '<S92>/Compare' */
  /* End of Outputs for SubSystem: '<S86>/Subsystem' */

  /* Gain: '<S2>/[1 1 -1]' */
  rtb_u11_idx = sensorMCUSlugsMKII_P.u11_Gain[0] * sensorMCUSlugsMKII_B.In1_p[0];
  rtb_u11_idx_0 = sensorMCUSlugsMKII_P.u11_Gain[1] * sensorMCUSlugsMKII_B.In1_p
    [1];
  rtb_Sum_l = sensorMCUSlugsMKII_P.u11_Gain[2] * sensorMCUSlugsMKII_B.In1_p[2];

  /* MATLAB Function: '<S87>/Embedded MATLAB Function3' */
  /* MATLAB Function 'Position and Attitude Filter/Position Filter/BlendPosVel/2 FIlter Blend/Embedded MATLAB Function3': '<S97>:1' */
  /*  persistent lastH; */
  if (!sensorMCUSlugsMKII_DWork.lastGps_h_not_empty) {
    /* '<S97>:1:7' */
    /* '<S97>:1:8' */
    sensorMCUSlugsMKII_DWork.lastGps_h = 0.0F;
    sensorMCUSlugsMKII_DWork.lastGps_h_not_empty = TRUE;

    /* '<S97>:1:9' */
    sensorMCUSlugsMKII_DWork.TimeSinceLast = (real32_T)
      sensorMCUSlugsMKII_P.SFunction_p1;

    /* '<S97>:1:10' */
    /*      lastH           = single(-baseHeight); */
    /*      h_rate          = single(0); */
  }

  if (rtb_DataTypeConversion2_n != 0) {
    /* '<S97>:1:16' */
    rtb_Merge_idx = (rtb_Sum_l - sensorMCUSlugsMKII_DWork.lastGps_h) /
      sensorMCUSlugsMKII_DWork.TimeSinceLast;

    /* '<S97>:1:17' */
    sensorMCUSlugsMKII_DWork.TimeSinceLast = 0.0F;
    if (!(((real_T)rtb_Merge_idx > -10.6) && ((real_T)rtb_Merge_idx < 17.67))) {
      /*          h = lastH; */
      /* '<S97>:1:23' */
      rtb_Merge_idx = 0.0F;
    } else {
      /* '<S97>:1:18' */
      /*          h = lastH + gps_h - lastGps_h; */
      /* '<S97>:1:20' */
    }

    /* '<S97>:1:25' */
    sensorMCUSlugsMKII_DWork.lastGps_h = rtb_Sum_l;
  } else {
    /*      h = lastH; */
    /* '<S97>:1:28' */
    rtb_Merge_idx = 0.0F;
  }

  /* '<S97>:1:31' */
  sensorMCUSlugsMKII_DWork.TimeSinceLast =
    sensorMCUSlugsMKII_DWork.TimeSinceLast + (real32_T)
    sensorMCUSlugsMKII_P.SFunction_p1;

  /* End of MATLAB Function: '<S87>/Embedded MATLAB Function3' */

  /* Gain: '<S87>/Gain' */
  /*  lastH = h; */
  /* %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5 */
  /*  persistent lastGps_h; */
  /*  persistent TimeSinceLast; */
  /*  persistent rate; */
  /*  persistent lastH; */
  /*   */
  /*  if (isempty(lastGps_h)) */
  /*      lastGps_h       = single(0.0); */
  /*      TimeSinceLast   = single(apSampleTime); */
  /*      rate            = single(0); */
  /*      lastH           = single(-baseHeight); */
  /*      h_rate          = single(0); */
  /*  end */
  /*   */
  /*  if (NewGPS) */
  /*      rate = single((gps_h - lastGps_h)/TimeSinceLast); */
  /*      TimeSinceLast = single(0); */
  /*      if (rate > -10.6) && (rate < 17.67) */
  /*          h = lastH + gps_h - lastGps_h; */
  /*  %         h_rate = rate; */
  /*      else */
  /*          h = lastH; */
  /*  %         h_rate = single(0); */
  /*      end */
  /*      lastGps_h = gps_h; */
  /*  else */
  /*      h = lastH; */
  /*  %     h_rate = single(0); */
  /*  end */
  /*   */
  /*  TimeSinceLast = TimeSinceLast + apSampleTime; */
  /*  lastH = h; */
  rtb_Merge_idx *= sensorMCUSlugsMKII_P.Gain_Gain_ks;

  /* S-Function "dsPIC_C_function_Call" Block: <S166>/Get the GS Location [updateSensorMCUState.c] */
  getGSLocation(&sensorMCUSlugsMKII_B.GettheGSLocationupdateSensorM_m[0]);

  /* MATLAB Function: '<S150>/Enables//Disables the Computation of  initial Baro Bias' */
  /* MATLAB Function 'Sensor Data/Sensor Suite/Low Pass Filtering,Temperature Compensation and Calibration/Baro Altimeter/Enables/Disables the Computation of  initial Baro Bias': '<S165>:1' */
  if (sensorMCUSlugsMKII_DWork.aveCount < 2500.0) {
    /* '<S165>:1:11' */
    /* '<S165>:1:12' */
    sensorMCUSlugsMKII_DWork.aveCount = sensorMCUSlugsMKII_DWork.aveCount + 1.0;
  }

  if (sensorMCUSlugsMKII_DWork.aveCount == 2500.0) {
    /* '<S165>:1:15' */
    /* '<S165>:1:16' */
    sensorMCUSlugsMKII_DWork.tIni = 0.0;

    /* '<S165>:1:17' */
    sensorMCUSlugsMKII_DWork.aveCount = sensorMCUSlugsMKII_DWork.aveCount + 1.0;
  }

  /* '<S165>:1:20' */
  rtb_tOut = sensorMCUSlugsMKII_DWork.tIni;

  /* End of MATLAB Function: '<S150>/Enables//Disables the Computation of  initial Baro Bias' */

  /* Outputs for Enabled SubSystem: '<S150>/Zero Out Height' incorporates:
   *  EnablePort: '<S168>/Enable'
   */
  /* if aveCount > 500 */
  /*     tIni = 0; */
  /* end */
  /*  if isempty(aveCount) */
  /*      aveCount =1; */
  /*      initBias = [0.0 0.0 0.0]'; */
  /*  end; */
  /*       */
  /*  if (aveCount<10) */
  /*      initBias = initBias+ Raw; */
  /*      bias = initBias/aveCount; */
  /*      aveCount = aveCount +1; */
  /*  end; */
  /*   */
  /*  if (aveCount == 10) */
  /*      initBias = initBias/(aveCount-1); */
  /*      aveCount = 100; */
  /*  end; */
  /*   */
  /*  if (aveCount == 100) */
  /*      bias = initBias; */
  /*  end; */
  if (rtb_tOut > 0.0) {
    /* Sum: '<S168>/Sum' incorporates:
     *  Delay: '<S168>/Integer Delay'
     */
    sensorMCUSlugsMKII_B.Sum =
      sensorMCUSlugsMKII_B.GettheGSLocationupdateSensorM_m[0] -
      sensorMCUSlugsMKII_DWork.IntegerDelay_DSTATE_m;
  }

  /* End of Outputs for SubSystem: '<S150>/Zero Out Height' */

  /* Outputs for Enabled SubSystem: '<S4>/Raw HIL  Readings' incorporates:
   *  EnablePort: '<S115>/Enable'
   */
  if (sensorMCUSlugsMKII_B.EnableHILfromControlMCU) {
    /* S-Function "dsPIC_C_function_Call" Block: <S115>/Data from HIL [hil.c] */
    hilRead(&sensorMCUSlugsMKII_B.DatafromHILhilc[0]);

    /* S-Function "dsPIC_C_function_Call" Block: <S115>/HIL Messages  Parser//Decoder [hil.c] */
    protDecodeHil(sensorMCUSlugsMKII_B.DatafromHILhilc);

    /* S-Function "dsPIC_C_function_Call" Block: <S115>/HIL Raw Readings [hil.c] */
    hil_getRawRead(&sensorMCUSlugsMKII_B.HILRawReadingshilc[0]);
  }

  /* End of Outputs for SubSystem: '<S4>/Raw HIL  Readings' */

  /* Switch: '<S116>/Switch' */
  for (colIdx = 0; colIdx < 13; colIdx++) {
    if (sensorMCUSlugsMKII_B.EnableHILfromControlMCU) {
      rtb_Switch_k[colIdx] = sensorMCUSlugsMKII_B.HILRawReadingshilc[colIdx];
    } else {
      rtb_Switch_k[colIdx] = sensorMCUSlugsMKII_B.y_h[colIdx];
    }
  }

  /* End of Switch: '<S116>/Switch' */

  /* DataTypeConversion: '<S121>/Data Type Conversion5' */
  rtb_DataTypeConversion5 = (real_T)rtb_Switch_k[12];

  /* MATLAB Function: '<S159>/Embedded MATLAB Function' */
  sensor_EmbeddedMATLABFunction_c(rtb_DataTypeConversion5,
    sensorMCUSlugsMKII_P.Constant_Value, sensorMCUSlugsMKII_P.Constant1_Value,
    &sensorMCUSlugsMKII_B.sf_EmbeddedMATLABFunction_b,
    &sensorMCUSlugsMKII_DWork.sf_EmbeddedMATLABFunction_b);

  /* Sum: '<S157>/Sum' incorporates:
   *  Constant: '<S157>/Bias'
   *  Constant: '<S157>/Gains'
   *  Constant: '<S158>/Bias'
   *  Constant: '<S158>/Gains'
   *  Product: '<S157>/Divide'
   *  Product: '<S158>/Divide'
   *  Sum: '<S158>/Sum'
   */
  rtb_RhhcosphicoslambXe = ((real32_T)((real_T)
    sensorMCUSlugsMKII_P.Gains_Value_c *
    sensorMCUSlugsMKII_B.sf_EmbeddedMATLABFunction_b.y) +
    sensorMCUSlugsMKII_P.Bias_Value_i) * sensorMCUSlugsMKII_P.Gains_Value_m +
    sensorMCUSlugsMKII_P.Bias_Value_b;

  /* RelationalOperator: '<S171>/Compare' incorporates:
   *  Constant: '<S171>/Constant'
   */
  rtb_Compare_j = (rtb_RhhcosphicoslambXe <
                   sensorMCUSlugsMKII_P.Constant_Value_dn);

  /* Outputs for Enabled SubSystem: '<S156>/Hi Temp Compensation' incorporates:
   *  EnablePort: '<S172>/Enable'
   */
  /* Logic: '<S156>/Logical Operator' */
  if (!(rtb_Compare_j != 0)) {
    /* Sum: '<S172>/Add' incorporates:
     *  Constant: '<S172>/Constant'
     *  Constant: '<S172>/Mean Temperature for Calibration'
     *  Constant: '<S172>/gains'
     *  Product: '<S172>/Divide1'
     *  Sum: '<S172>/Sum1'
     *  Sum: '<S172>/Sum2'
     */
    sensorMCUSlugsMKII_B.Merge = ((real32_T)rtb_Switch_k[9] -
      (rtb_RhhcosphicoslambXe -
       sensorMCUSlugsMKII_P.MeanTemperatureforCalibration_V) *
      sensorMCUSlugsMKII_P.gains_Value) + sensorMCUSlugsMKII_P.Constant_Value_h;
  }

  /* End of Logic: '<S156>/Logical Operator' */
  /* End of Outputs for SubSystem: '<S156>/Hi Temp Compensation' */

  /* Outputs for Enabled SubSystem: '<S156>/Lo Temp Compensation' incorporates:
   *  EnablePort: '<S173>/Enable'
   */
  if (rtb_Compare_j > 0) {
    /* Sum: '<S173>/Sum2' incorporates:
     *  Constant: '<S173>/Mean Temperature for Calibration'
     *  Constant: '<S173>/gains'
     *  Product: '<S173>/Divide1'
     *  Sum: '<S173>/Sum1'
     */
    sensorMCUSlugsMKII_B.Merge = (real32_T)rtb_Switch_k[9] -
      (rtb_RhhcosphicoslambXe -
       sensorMCUSlugsMKII_P.MeanTemperatureforCalibration_c) *
      sensorMCUSlugsMKII_P.gains_Value_c;
  }

  /* End of Outputs for SubSystem: '<S156>/Lo Temp Compensation' */

  /* DataTypeConversion: '<S121>/Data Type Conversion19' */
  rtb_DataTypeConversion19 = sensorMCUSlugsMKII_B.Merge;

  /* MATLAB Function: '<S162>/Embedded MATLAB Function' */
  sensor_EmbeddedMATLABFunction_c(rtb_DataTypeConversion19,
    sensorMCUSlugsMKII_P.Constant_Value_j,
    sensorMCUSlugsMKII_P.Constant1_Value_d,
    &sensorMCUSlugsMKII_B.sf_EmbeddedMATLABFunction_e,
    &sensorMCUSlugsMKII_DWork.sf_EmbeddedMATLABFunction_e);

  /* Sum: '<S151>/Sum' incorporates:
   *  Constant: '<S151>/Bias'
   *  Constant: '<S151>/Gains'
   *  DataTypeConversion: '<S121>/Data Type Conversion1'
   *  Product: '<S151>/Divide'
   */
  rtb_ixk = sensorMCUSlugsMKII_P.Gains_Value * (real32_T)
    sensorMCUSlugsMKII_B.sf_EmbeddedMATLABFunction_e.y +
    sensorMCUSlugsMKII_P.Bias_Value_l;

  /* Outputs for Enabled SubSystem: '<S150>/Initial Baro Bias' incorporates:
   *  EnablePort: '<S167>/Enable'
   */
  if (rtb_tOut > 0.0) {
    /* DataTypeConversion: '<S167>/Data Type Conversion' */
    rtb_DataTypeConversion = rtb_ixk;

    /* DiscreteZeroPole: '<S170>/Discrete Zero-Pole' */
    {
      rtb_DiscreteZeroPole_h = sensorMCUSlugsMKII_P.DiscreteZeroPole_D*
        rtb_DataTypeConversion;
      rtb_DiscreteZeroPole_h += sensorMCUSlugsMKII_P.DiscreteZeroPole_C*
        sensorMCUSlugsMKII_DWork.DiscreteZeroPole_DSTATE_p;
    }

    /* Saturate: '<S167>/[80k - 120k]' incorporates:
     *  DataTypeConversion: '<S167>/Data Type Conversion1'
     */
    sensorMCUSlugsMKII_B.u0k120k = (real32_T)rtb_DiscreteZeroPole_h >=
      sensorMCUSlugsMKII_P.u0k120k_UpperSat ?
      sensorMCUSlugsMKII_P.u0k120k_UpperSat : (real32_T)rtb_DiscreteZeroPole_h <=
      sensorMCUSlugsMKII_P.u0k120k_LowerSat ?
      sensorMCUSlugsMKII_P.u0k120k_LowerSat : (real32_T)rtb_DiscreteZeroPole_h;

    /* Update for DiscreteZeroPole: '<S170>/Discrete Zero-Pole' */
    {
      sensorMCUSlugsMKII_DWork.DiscreteZeroPole_DSTATE_p =
        rtb_DataTypeConversion + sensorMCUSlugsMKII_P.DiscreteZeroPole_A*
        sensorMCUSlugsMKII_DWork.DiscreteZeroPole_DSTATE_p;
    }
  }

  /* End of Outputs for SubSystem: '<S150>/Initial Baro Bias' */

  /* Product: '<S163>/Divide' incorporates:
   *  Sum: '<S163>/Sum2'
   */
  rtb_Sum_l = (rtb_ixk - sensorMCUSlugsMKII_B.u0k120k) /
    sensorMCUSlugsMKII_B.u0k120k;

  /* Sum: '<S163>/Sum1' incorporates:
   *  Constant: '<S163>/Constant2'
   *  Constant: '<S163>/Constant3'
   *  Constant: '<S163>/Constant4'
   *  Constant: '<S163>/Constant5'
   *  Gain: '<S169>/Unit Conversion'
   *  Product: '<S163>/Divide1'
   *  Product: '<S163>/Divide2'
   *  Product: '<S163>/Divide3'
   *  Product: '<S163>/Divide4'
   *  Sum: '<S163>/Sum3'
   */
  rtb_Rh = ((rtb_Sum_l * rtb_Sum_l * sensorMCUSlugsMKII_P.Constant3_Value_n +
             rtb_Sum_l * sensorMCUSlugsMKII_P.Constant4_Value_d) +
            sensorMCUSlugsMKII_P.Constant5_Value_j) *
    sensorMCUSlugsMKII_P.Constant2_Value_cf *
    sensorMCUSlugsMKII_P.UnitConversion_Gain_c +
    sensorMCUSlugsMKII_B.GettheGSLocationupdateSensorM_m[0];

  /* Outputs for Enabled SubSystem: '<S150>/Enabled Subsystem' incorporates:
   *  EnablePort: '<S164>/Enable'
   */
  /* Logic: '<S150>/Logical Operator' incorporates:
   *  Inport: '<S164>/In1'
   *  Sum: '<S150>/Sum1'
   */
  if (!(rtb_tOut != 0.0)) {
    sensorMCUSlugsMKII_B.In1 = sensorMCUSlugsMKII_B.Sum + rtb_Rh;
  }

  /* End of Logic: '<S150>/Logical Operator' */
  /* End of Outputs for SubSystem: '<S150>/Enabled Subsystem' */

  /* Sum: '<S87>/Sum' incorporates:
   *  Gain: '<S86>/Gain'
   */
  rtb_Merge_idx += sensorMCUSlugsMKII_P.Gain_Gain_a * sensorMCUSlugsMKII_B.In1;

  /* MATLAB Function: '<S98>/Embedded MATLAB Function' incorporates:
   *  Constant: '<S98>/Constant'
   *  Constant: '<S98>/Constant1'
   */
  /* MATLAB Function 'Position and Attitude Filter/Position Filter/BlendPosVel/2 FIlter Blend/Tustin Lowpass,  Auto Initial Condition4/Embedded MATLAB Function': '<S99>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  if (!sensorMCUSlugsMKII_DWork.a_not_empty) {
    /* '<S99>:1:9' */
    /* tf = [T*omega/(2+T*omega) T*omega/(2+T*omega)],[1 (T*omega-2)/(T*omega+2)] */
    /* tf = [a a],[1 -b] */
    /* '<S99>:1:12' */
    omega = 6.2831853071795862 * sensorMCUSlugsMKII_P.Constant1_Value_k;

    /* '<S99>:1:13' */
    sensorMCUSlugsMKII_DWork.a = sensorMCUSlugsMKII_P.Constant_Value_n * omega /
      (sensorMCUSlugsMKII_P.Constant_Value_n * omega + 2.0);
    sensorMCUSlugsMKII_DWork.a_not_empty = TRUE;

    /* '<S99>:1:14' */
    sensorMCUSlugsMKII_DWork.b = -(sensorMCUSlugsMKII_P.Constant_Value_n * omega
      - 2.0) / (sensorMCUSlugsMKII_P.Constant_Value_n * omega + 2.0);

    /* '<S99>:1:15' */
    sensorMCUSlugsMKII_DWork.y_km1 = rtb_Merge_idx;

    /* '<S99>:1:16' */
    sensorMCUSlugsMKII_DWork.u_km1 = rtb_Merge_idx;
  }

  /* '<S99>:1:19' */
  rtb_RhhcosphisinlambYe = (rtb_Merge_idx + sensorMCUSlugsMKII_DWork.u_km1) *
    (real32_T)sensorMCUSlugsMKII_DWork.a + (real32_T)sensorMCUSlugsMKII_DWork.b *
    sensorMCUSlugsMKII_DWork.y_km1;

  /* '<S99>:1:20' */
  sensorMCUSlugsMKII_DWork.y_km1 = rtb_RhhcosphisinlambYe;

  /* '<S99>:1:21' */
  sensorMCUSlugsMKII_DWork.u_km1 = rtb_Merge_idx;

  /* End of MATLAB Function: '<S98>/Embedded MATLAB Function' */

  /* Gain: '<S91>/Gain' incorporates:
   *  Sum: '<S91>/Sum1'
   *  UnitDelay: '<S91>/Unit Delay'
   */
  rtb_Gain_a = (rtb_RhhcosphisinlambYe -
                sensorMCUSlugsMKII_DWork.UnitDelay_DSTATE) *
    sensorMCUSlugsMKII_P.Gain_Gain_g;

  /* MATLAB Function: '<S86>/myMux Fun2' */
  sensorMCUSlugsMKII_myMuxFun1_l
    (sensorMCUSlugsMKII_B.sf_EmbeddedMATLABFunction2_d.y,
     sensorMCUSlugsMKII_B.sf_EmbeddedMATLABFunction1_a.y, rtb_Gain_a,
     &sensorMCUSlugsMKII_B.sf_myMuxFun2_f);

  /* Product: '<S12>/Product1' */
  for (colIdx = 0; colIdx < 3; colIdx++) {
    Product[colIdx] = sensorMCUSlugsMKII_B.VectorConcatenate[colIdx + 6] *
      sensorMCUSlugsMKII_B.sf_myMuxFun2_f.y[2] +
      (sensorMCUSlugsMKII_B.VectorConcatenate[colIdx + 3] *
       sensorMCUSlugsMKII_B.sf_myMuxFun2_f.y[1] +
       sensorMCUSlugsMKII_B.VectorConcatenate[colIdx] *
       sensorMCUSlugsMKII_B.sf_myMuxFun2_f.y[0]);
  }

  /* End of Product: '<S12>/Product1' */

  /* Gain: '<S23>/Gain2' incorporates:
   *  Delay: '<S23>/Integer Delay1'
   *  Sum: '<S23>/Sum5'
   */
  rtb_UEN2NEU[0] = (Product[0] - sensorMCUSlugsMKII_DWork.IntegerDelay1_DSTATE[0])
    * sensorMCUSlugsMKII_P.Gain2_Gain;
  rtb_UEN2NEU[1] = (Product[1] - sensorMCUSlugsMKII_DWork.IntegerDelay1_DSTATE[1])
    * sensorMCUSlugsMKII_P.Gain2_Gain;
  rtb_UEN2NEU[2] = (Product[2] - sensorMCUSlugsMKII_DWork.IntegerDelay1_DSTATE[2])
    * sensorMCUSlugsMKII_P.Gain2_Gain;

  /* DataTypeConversion: '<S117>/Data Type Conversion1' incorporates:
   *  Constant: '<S117>/Gyro Gains1'
   *  Product: '<S117>/Divide1'
   */
  rtb_DataTypeConversion1[0] = (real32_T)rtb_Switch_k[4] *
    sensorMCUSlugsMKII_P.GyroGains1_Value;
  rtb_DataTypeConversion1[1] = (real32_T)rtb_Switch_k[3] *
    sensorMCUSlugsMKII_P.GyroGains1_Value;
  rtb_DataTypeConversion1[2] = (real32_T)rtb_Switch_k[5] *
    sensorMCUSlugsMKII_P.GyroGains1_Value;

  /* MATLAB Function: '<S125>/Embedded MATLAB Function' */
  sensor_EmbeddedMATLABFunction_c(rtb_DataTypeConversion1[0],
    sensorMCUSlugsMKII_P.Constant_Value_i,
    sensorMCUSlugsMKII_P.Constant1_Value_k2,
    &sensorMCUSlugsMKII_B.sf_EmbeddedMATLABFunction_p,
    &sensorMCUSlugsMKII_DWork.sf_EmbeddedMATLABFunction_p);

  /* MATLAB Function: '<S125>/Embedded MATLAB Function1' */
  sensor_EmbeddedMATLABFunction_c(rtb_DataTypeConversion1[1],
    sensorMCUSlugsMKII_P.Constant2_Value, sensorMCUSlugsMKII_P.Constant3_Value,
    &sensorMCUSlugsMKII_B.sf_EmbeddedMATLABFunction1,
    &sensorMCUSlugsMKII_DWork.sf_EmbeddedMATLABFunction1);

  /* MATLAB Function: '<S125>/Embedded MATLAB Function2' */
  sensor_EmbeddedMATLABFunction_c(rtb_DataTypeConversion1[2],
    sensorMCUSlugsMKII_P.Constant4_Value, sensorMCUSlugsMKII_P.Constant5_Value,
    &sensorMCUSlugsMKII_B.sf_EmbeddedMATLABFunction2,
    &sensorMCUSlugsMKII_DWork.sf_EmbeddedMATLABFunction2);

  /* MATLAB Function: '<S125>/myMux Fun' */
  sensorMCUSlugsMKII_myMuxFun(sensorMCUSlugsMKII_B.sf_EmbeddedMATLABFunction_p.y,
    sensorMCUSlugsMKII_B.sf_EmbeddedMATLABFunction1.y,
    sensorMCUSlugsMKII_B.sf_EmbeddedMATLABFunction2.y,
    &sensorMCUSlugsMKII_B.sf_myMuxFun);

  /* DataTypeConversion: '<S117>/Data Type Conversion3' */
  rtb_DataTypeConversion3_e_idx = (real32_T)sensorMCUSlugsMKII_B.sf_myMuxFun.y[0];
  rtb_DataTypeConversion3_e_idx_0 = (real32_T)
    sensorMCUSlugsMKII_B.sf_myMuxFun.y[1];
  rtb_DataTypeConversion3_e_idx_1 = (real32_T)
    sensorMCUSlugsMKII_B.sf_myMuxFun.y[2];

  /* Product: '<S86>/Product1' */
  for (colIdx = 0; colIdx < 3; colIdx++) {
    Product1_h[colIdx] = sensorMCUSlugsMKII_B.VectorConcatenate[colIdx + 6] *
      sensorMCUSlugsMKII_B.sf_myMuxFun2_f.y[2] +
      (sensorMCUSlugsMKII_B.VectorConcatenate[colIdx + 3] *
       sensorMCUSlugsMKII_B.sf_myMuxFun2_f.y[1] +
       sensorMCUSlugsMKII_B.VectorConcatenate[colIdx] *
       sensorMCUSlugsMKII_B.sf_myMuxFun2_f.y[0]);
  }

  /* End of Product: '<S86>/Product1' */

  /* Sum: '<S86>/Sum' incorporates:
   *  DataTypeConversion: '<S117>/Data Type Conversion3'
   *  Delay: '<S93>/Integer Delay1'
   *  Gain: '<S93>/Gain2'
   *  Sum: '<S93>/Sum5'
   */
  rtb_kxj = (real32_T)sensorMCUSlugsMKII_B.sf_myMuxFun.y[0] - (Product1_h[0] -
    sensorMCUSlugsMKII_DWork.IntegerDelay1_DSTATE_b[0]) *
    sensorMCUSlugsMKII_P.Gain2_Gain_l;
  rtb_GyroErr_idx_0 = (real32_T)sensorMCUSlugsMKII_B.sf_myMuxFun.y[1] -
    (Product1_h[1] - sensorMCUSlugsMKII_DWork.IntegerDelay1_DSTATE_b[1]) *
    sensorMCUSlugsMKII_P.Gain2_Gain_l;
  rtb_Sum_l = (real32_T)sensorMCUSlugsMKII_B.sf_myMuxFun.y[2] - (Product1_h[2] -
    sensorMCUSlugsMKII_DWork.IntegerDelay1_DSTATE_b[2]) *
    sensorMCUSlugsMKII_P.Gain2_Gain_l;

  /* RateLimiter: '<S86>/Rate Limiter' */
  rtb_Deg2R_idx_0 = rtb_kxj - sensorMCUSlugsMKII_DWork.PrevY[0];
  rtb_DataTypeConversion1_l = rtb_GyroErr_idx_0 -
    sensorMCUSlugsMKII_DWork.PrevY[1];
  rtb_ixj = rtb_Sum_l - sensorMCUSlugsMKII_DWork.PrevY[2];
  rtb_Deg2R_idx = rtb_kxj;
  if (rtb_Deg2R_idx_0 > sensorMCUSlugsMKII_P.RateLimiter_RisingLim) {
    rtb_Deg2R_idx = sensorMCUSlugsMKII_DWork.PrevY[0] +
      sensorMCUSlugsMKII_P.RateLimiter_RisingLim;
  } else {
    if (rtb_Deg2R_idx_0 < sensorMCUSlugsMKII_P.RateLimiter_FallingLim) {
      rtb_Deg2R_idx = sensorMCUSlugsMKII_DWork.PrevY[0] +
        sensorMCUSlugsMKII_P.RateLimiter_FallingLim;
    }
  }

  rtb_kxj = rtb_Deg2R_idx;

  /* RateLimiter: '<S86>/Rate Limiter' */
  rtb_Deg2R_idx = rtb_GyroErr_idx_0;
  if (rtb_DataTypeConversion1_l > sensorMCUSlugsMKII_P.RateLimiter_RisingLim) {
    rtb_Deg2R_idx = sensorMCUSlugsMKII_DWork.PrevY[1] +
      sensorMCUSlugsMKII_P.RateLimiter_RisingLim;
  } else {
    if (rtb_DataTypeConversion1_l < sensorMCUSlugsMKII_P.RateLimiter_FallingLim)
    {
      rtb_Deg2R_idx = sensorMCUSlugsMKII_DWork.PrevY[1] +
        sensorMCUSlugsMKII_P.RateLimiter_FallingLim;
    }
  }

  rtb_GyroErr_idx_0 = rtb_Deg2R_idx;

  /* RateLimiter: '<S86>/Rate Limiter' */
  rtb_Deg2R_idx = rtb_Sum_l;
  if (rtb_ixj > sensorMCUSlugsMKII_P.RateLimiter_RisingLim) {
    rtb_Deg2R_idx = sensorMCUSlugsMKII_DWork.PrevY[2] +
      sensorMCUSlugsMKII_P.RateLimiter_RisingLim;
  } else {
    if (rtb_ixj < sensorMCUSlugsMKII_P.RateLimiter_FallingLim) {
      rtb_Deg2R_idx = sensorMCUSlugsMKII_DWork.PrevY[2] +
        sensorMCUSlugsMKII_P.RateLimiter_FallingLim;
    }
  }

  sensorMCUSlugsMKII_DWork.PrevY[0] = rtb_kxj;
  sensorMCUSlugsMKII_DWork.PrevY[1] = rtb_GyroErr_idx_0;
  sensorMCUSlugsMKII_DWork.PrevY[2] = rtb_Deg2R_idx;

  /* DataTypeConversion: '<S88>/Data Type Conversion2' */
  rtb_DataTypeConversion2[0] = rtb_kxj;
  rtb_DataTypeConversion2[1] = rtb_GyroErr_idx_0;
  rtb_DataTypeConversion2[2] = rtb_Deg2R_idx;

  /* DiscreteZeroPole: '<S100>/Discrete Zero-Pole' */
  {
    rtb_DiscreteZeroPole = sensorMCUSlugsMKII_P.DiscreteZeroPole_D_j*
      rtb_DataTypeConversion2[0];
    rtb_DiscreteZeroPole += sensorMCUSlugsMKII_P.DiscreteZeroPole_C_e*
      sensorMCUSlugsMKII_DWork.DiscreteZeroPole_DSTATE;
  }

  /* DiscreteZeroPole: '<S101>/Discrete Zero-Pole' */
  {
    rtb_DiscreteZeroPole_g = sensorMCUSlugsMKII_P.DiscreteZeroPole_D_ju*
      rtb_DataTypeConversion2[1];
    rtb_DiscreteZeroPole_g += sensorMCUSlugsMKII_P.DiscreteZeroPole_C_o*
      sensorMCUSlugsMKII_DWork.DiscreteZeroPole_DSTATE_k;
  }

  /* DiscreteZeroPole: '<S102>/Discrete Zero-Pole' */
  {
    rtb_DiscreteZeroPole_p = sensorMCUSlugsMKII_P.DiscreteZeroPole_D_p*
      rtb_DataTypeConversion2[2];
    rtb_DiscreteZeroPole_p += sensorMCUSlugsMKII_P.DiscreteZeroPole_C_k*
      sensorMCUSlugsMKII_DWork.DiscreteZeroPole_DSTATE_e;
  }

  /* MATLAB Function: '<S88>/myMux Fun1' */
  sensorMCUSlugsMKII_myMuxFun1(rtb_DiscreteZeroPole, rtb_DiscreteZeroPole_g,
    rtb_DiscreteZeroPole_p, &sensorMCUSlugsMKII_B.sf_myMuxFun1);

  /* DataTypeConversion: '<S88>/Data Type Conversion1' */
  rtb_kxj = (real32_T)sensorMCUSlugsMKII_B.sf_myMuxFun1.y[0];
  rtb_GyroErr_idx_0 = (real32_T)sensorMCUSlugsMKII_B.sf_myMuxFun1.y[1];
  rtb_Sum_l = (real32_T)sensorMCUSlugsMKII_B.sf_myMuxFun1.y[2];

  /* Sum: '<S12>/Add' incorporates:
   *  DataTypeConversion: '<S88>/Data Type Conversion1'
   *  Product: '<S38>/i x j'
   *  Product: '<S38>/j x k'
   *  Product: '<S38>/k x i'
   *  Product: '<S39>/i x k'
   *  Product: '<S39>/j x i'
   *  Product: '<S39>/k x j'
   *  Sum: '<S21>/Sum'
   */
  rtb_Add[0] = (((rtb_Sum4_g_idx * Product[2] - rtb_Sum4_g_idx_0 * Product[1]) +
                 rtb_UEN2NEU[0]) - rtb_DataTypeConversion3_e_idx) - (real32_T)
    sensorMCUSlugsMKII_B.sf_myMuxFun1.y[0];
  rtb_Add[1] = (((rtb_Sum4_g_idx_0 * Product[0] - rtb_Sum4_g_idx_1 * Product[2])
                 + rtb_UEN2NEU[1]) - rtb_DataTypeConversion3_e_idx_0) -
    (real32_T)sensorMCUSlugsMKII_B.sf_myMuxFun1.y[1];
  rtb_Add[2] = (((rtb_Sum4_g_idx_1 * Product[1] - rtb_Sum4_g_idx * Product[0]) +
                 rtb_UEN2NEU[2]) - rtb_DataTypeConversion3_e_idx_1) - (real32_T)
    sensorMCUSlugsMKII_B.sf_myMuxFun1.y[2];

  /* MATLAB Function: '<S47>/Embedded MATLAB Function' */
  sensorMC_EmbeddedMATLABFunction(rtb_Add,
    &sensorMCUSlugsMKII_B.sf_EmbeddedMATLABFunction_g);

  /* MATLAB Function: '<S46>/negprotect' */
  sensorMCUSlugsMKII_negprotect
    (sensorMCUSlugsMKII_B.sf_EmbeddedMATLABFunction_g.xDoty,
     &sensorMCUSlugsMKII_B.sf_negprotect_m);

  /* S-Function "dsPIC_C_function_Call" Block: <S46>/C Function Call */
  sensorMCUSlugsMKII_B.CFunctionCall_kv = mySqrt
    (sensorMCUSlugsMKII_B.sf_negprotect_m.zpVal);

  /* Saturate: '<S26>/Zero Bound' */
  rtb_Merge_idx = sensorMCUSlugsMKII_B.CFunctionCall_kv >=
    sensorMCUSlugsMKII_P.ZeroBound_UpperSat_i ?
    sensorMCUSlugsMKII_P.ZeroBound_UpperSat_i :
    sensorMCUSlugsMKII_B.CFunctionCall_kv <=
    sensorMCUSlugsMKII_P.ZeroBound_LowerSat_n ?
    sensorMCUSlugsMKII_P.ZeroBound_LowerSat_n :
    sensorMCUSlugsMKII_B.CFunctionCall_kv;

  /* Product: '<S26>/Divide' */
  rtb_Sum4_g_idx_1 = rtb_Add[0] / rtb_Merge_idx;
  rtb_Sum4_g_idx = rtb_Add[1] / rtb_Merge_idx;
  rtb_Sum4_g_idx_0 = rtb_Add[2] / rtb_Merge_idx;

  /* S-Function (sdspsubmtrx): '<S12>/Submatrix' */
  yIdx = 0L;
  colIdx = 2;
  while (colIdx <= 2) {
    sensorMCUSlugsMKII_B.g_hat[yIdx] = sensorMCUSlugsMKII_B.VectorConcatenate[6];
    sensorMCUSlugsMKII_B.g_hat[1L + yIdx] =
      sensorMCUSlugsMKII_B.VectorConcatenate[7];
    sensorMCUSlugsMKII_B.g_hat[2L + yIdx] =
      sensorMCUSlugsMKII_B.VectorConcatenate[8];
    yIdx += 3L;
    colIdx = 3;
  }

  /* End of S-Function (sdspsubmtrx): '<S12>/Submatrix' */

  /* Sum: '<S20>/Sum' incorporates:
   *  Product: '<S36>/i x j'
   *  Product: '<S37>/i x k'
   *  Product: '<S37>/j x i'
   */
  rtb_Deg2R_idx = rtb_Sum4_g_idx_1 * sensorMCUSlugsMKII_B.g_hat[1];
  rtb_Deg2R_idx_0 = rtb_Sum4_g_idx_1 * sensorMCUSlugsMKII_B.g_hat[2];
  rtb_DataTypeConversion1_l = rtb_Sum4_g_idx * sensorMCUSlugsMKII_B.g_hat[0];

  /* Sum: '<S12>/Sum4' incorporates:
   *  Product: '<S36>/j x k'
   *  Product: '<S36>/k x i'
   *  Product: '<S37>/k x j'
   *  Sum: '<S20>/Sum'
   */
  rtb_Sum4_g_idx_1 = (rtb_Sum4_g_idx * sensorMCUSlugsMKII_B.g_hat[2] -
                      rtb_Sum4_g_idx_0 * sensorMCUSlugsMKII_B.g_hat[1]) +
    rtb_jxi;
  rtb_Sum4_g_idx = (rtb_Sum4_g_idx_0 * sensorMCUSlugsMKII_B.g_hat[0] -
                    rtb_Deg2R_idx_0) + rtb_GyroErr_idx;
  rtb_Sum4_g_idx_0 = (rtb_Deg2R_idx - rtb_DataTypeConversion1_l) +
    rtb_GyroErr_idx_1;

  /* DataTypeConversion: '<S19>/Data Type Conversion2' */
  rtb_DataTypeConversion2_d[0] = rtb_Sum4_g_idx_1;
  rtb_DataTypeConversion2_d[1] = rtb_Sum4_g_idx;
  rtb_DataTypeConversion2_d[2] = rtb_Sum4_g_idx_0;

  /* DiscreteZeroPole: '<S32>/Discrete Zero-Pole' */
  {
    rtb_DiscreteZeroPole_gx = sensorMCUSlugsMKII_P.DiscreteZeroPole_D_jb*
      rtb_DataTypeConversion2_d[0];
    rtb_DiscreteZeroPole_gx += sensorMCUSlugsMKII_P.DiscreteZeroPole_C_c*
      sensorMCUSlugsMKII_DWork.DiscreteZeroPole_DSTATE_kq;
  }

  /* DiscreteZeroPole: '<S33>/Discrete Zero-Pole' */
  {
    rtb_DiscreteZeroPole_pz = sensorMCUSlugsMKII_P.DiscreteZeroPole_D_o*
      rtb_DataTypeConversion2_d[1];
    rtb_DiscreteZeroPole_pz += sensorMCUSlugsMKII_P.DiscreteZeroPole_C_i*
      sensorMCUSlugsMKII_DWork.DiscreteZeroPole_DSTATE_g;
  }

  /* DiscreteZeroPole: '<S34>/Discrete Zero-Pole' */
  {
    rtb_DiscreteZeroPole_n = sensorMCUSlugsMKII_P.DiscreteZeroPole_D_l*
      rtb_DataTypeConversion2_d[2];
    rtb_DiscreteZeroPole_n += sensorMCUSlugsMKII_P.DiscreteZeroPole_C_p*
      sensorMCUSlugsMKII_DWork.DiscreteZeroPole_DSTATE_g3;
  }

  /* MATLAB Function: '<S19>/myMux Fun1' */
  sensorMCUSlugsMKII_myMuxFun1(rtb_DiscreteZeroPole_gx, rtb_DiscreteZeroPole_pz,
    rtb_DiscreteZeroPole_n, &sensorMCUSlugsMKII_B.sf_myMuxFun1_i);

  /* RateLimiter: '<S12>/Bias Rate Limiter' incorporates:
   *  DataTypeConversion: '<S19>/Data Type Conversion1'
   */
  rtb_Deg2R_idx_0 = (real32_T)sensorMCUSlugsMKII_B.sf_myMuxFun1_i.y[0] -
    sensorMCUSlugsMKII_DWork.PrevY_e[0];
  rtb_DataTypeConversion1_l = (real32_T)sensorMCUSlugsMKII_B.sf_myMuxFun1_i.y[1]
    - sensorMCUSlugsMKII_DWork.PrevY_e[1];
  rtb_ixj = (real32_T)sensorMCUSlugsMKII_B.sf_myMuxFun1_i.y[2] -
    sensorMCUSlugsMKII_DWork.PrevY_e[2];

  /* DataTypeConversion: '<S19>/Data Type Conversion1' */
  rtb_jxi = (real32_T)sensorMCUSlugsMKII_B.sf_myMuxFun1_i.y[0];

  /* RateLimiter: '<S12>/Bias Rate Limiter' */
  if (rtb_Deg2R_idx_0 > sensorMCUSlugsMKII_P.BiasRateLimiter_RisingLim) {
    rtb_jxi = sensorMCUSlugsMKII_DWork.PrevY_e[0] +
      sensorMCUSlugsMKII_P.BiasRateLimiter_RisingLim;
  } else {
    if (rtb_Deg2R_idx_0 < sensorMCUSlugsMKII_P.BiasRateLimiter_FallingLim) {
      rtb_jxi = sensorMCUSlugsMKII_DWork.PrevY_e[0] +
        sensorMCUSlugsMKII_P.BiasRateLimiter_FallingLim;
    }
  }

  rtb_GyroErr_idx_1 = rtb_jxi;

  /* DataTypeConversion: '<S19>/Data Type Conversion1' */
  rtb_jxi = (real32_T)sensorMCUSlugsMKII_B.sf_myMuxFun1_i.y[1];

  /* RateLimiter: '<S12>/Bias Rate Limiter' */
  if (rtb_DataTypeConversion1_l > sensorMCUSlugsMKII_P.BiasRateLimiter_RisingLim)
  {
    rtb_jxi = sensorMCUSlugsMKII_DWork.PrevY_e[1] +
      sensorMCUSlugsMKII_P.BiasRateLimiter_RisingLim;
  } else {
    if (rtb_DataTypeConversion1_l <
        sensorMCUSlugsMKII_P.BiasRateLimiter_FallingLim) {
      rtb_jxi = sensorMCUSlugsMKII_DWork.PrevY_e[1] +
        sensorMCUSlugsMKII_P.BiasRateLimiter_FallingLim;
    }
  }

  rtb_GyroErr_idx = rtb_jxi;

  /* DataTypeConversion: '<S19>/Data Type Conversion1' */
  rtb_jxi = (real32_T)sensorMCUSlugsMKII_B.sf_myMuxFun1_i.y[2];

  /* RateLimiter: '<S12>/Bias Rate Limiter' */
  if (rtb_ixj > sensorMCUSlugsMKII_P.BiasRateLimiter_RisingLim) {
    rtb_jxi = sensorMCUSlugsMKII_DWork.PrevY_e[2] +
      sensorMCUSlugsMKII_P.BiasRateLimiter_RisingLim;
  } else {
    if (rtb_ixj < sensorMCUSlugsMKII_P.BiasRateLimiter_FallingLim) {
      rtb_jxi = sensorMCUSlugsMKII_DWork.PrevY_e[2] +
        sensorMCUSlugsMKII_P.BiasRateLimiter_FallingLim;
    }
  }

  sensorMCUSlugsMKII_DWork.PrevY_e[0] = rtb_GyroErr_idx_1;
  sensorMCUSlugsMKII_DWork.PrevY_e[1] = rtb_GyroErr_idx;
  sensorMCUSlugsMKII_DWork.PrevY_e[2] = rtb_jxi;

  /* Switch: '<S3>/Switch' incorporates:
   *  Constant: '<S3>/Constant'
   */
  if (rtb_Product_b > sensorMCUSlugsMKII_P.Switch_Threshold) {
    rtb_Switch[0] = sensorMCUSlugsMKII_P.Constant_Value_ij[0];
    rtb_Switch[1] = sensorMCUSlugsMKII_P.Constant_Value_ij[1];
    rtb_Switch[2] = sensorMCUSlugsMKII_P.Constant_Value_ij[2];
  } else {
    rtb_Switch[0] = rtb_GyroErr_idx_1;
    rtb_Switch[1] = rtb_GyroErr_idx;
    rtb_Switch[2] = rtb_jxi;
  }

  /* End of Switch: '<S3>/Switch' */

  /* Switch: '<S3>/Switch6' incorporates:
   *  Constant: '<S3>/Constant1'
   */
  if (rtb_Product_b > sensorMCUSlugsMKII_P.Switch6_Threshold) {
    rtb_Switch6[0] = sensorMCUSlugsMKII_P.Constant1_Value_a[0];
    rtb_Switch6[1] = sensorMCUSlugsMKII_P.Constant1_Value_a[1];
    rtb_Switch6[2] = sensorMCUSlugsMKII_P.Constant1_Value_a[2];
  } else {
    rtb_Switch6[0] = rtb_kxj;
    rtb_Switch6[1] = rtb_GyroErr_idx_0;
    rtb_Switch6[2] = rtb_Sum_l;
  }

  /* End of Switch: '<S3>/Switch6' */

  /* MATLAB Function: '<Root>/myMux Fun4' */
  sensorMCUSlugsMKII_myMuxFun2(rtb_Switch, rtb_Switch6,
    &sensorMCUSlugsMKII_B.sf_myMuxFun4);

  /* S-Function "dsPIC_PWM_IC" Block: <S1>/Input Capture */
  sensorMCUSlugsMKII_B.dT = ic2up;
  sensorMCUSlugsMKII_B.dA = ic3up;
  sensorMCUSlugsMKII_B.dE = ic4up;
  sensorMCUSlugsMKII_B.dR = ic5up;
  sensorMCUSlugsMKII_B.dFailsafe = ic8up;

  /* DataTypeConversion: '<S10>/Data Type Conversion' incorporates:
   *  Gain: '<S10>/Convert to  Microseconds'
   *  MATLAB Function: '<S1>/myMux Fun5'
   */
  /* MATLAB Function 'Control Surface Input/myMux Fun5': '<S11>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S11>:1:5' */
  sensorMCUSlugsMKII_B.DataTypeConversion[0] = (uint16_T)((uint32_T)
    sensorMCUSlugsMKII_P.ConverttoMicroseconds_Gain * (uint32_T)
    sensorMCUSlugsMKII_B.dT >> 18);
  sensorMCUSlugsMKII_B.DataTypeConversion[1] = (uint16_T)((uint32_T)
    sensorMCUSlugsMKII_P.ConverttoMicroseconds_Gain * (uint32_T)
    sensorMCUSlugsMKII_B.dA >> 18);
  sensorMCUSlugsMKII_B.DataTypeConversion[2] = (uint16_T)((uint32_T)
    sensorMCUSlugsMKII_P.ConverttoMicroseconds_Gain * (uint32_T)
    sensorMCUSlugsMKII_B.dE >> 18);
  sensorMCUSlugsMKII_B.DataTypeConversion[3] = (uint16_T)((uint32_T)
    sensorMCUSlugsMKII_P.ConverttoMicroseconds_Gain * (uint32_T)
    sensorMCUSlugsMKII_B.dR >> 18);
  sensorMCUSlugsMKII_B.DataTypeConversion[4] = (uint16_T)((uint32_T)
    sensorMCUSlugsMKII_P.ConverttoMicroseconds_Gain * (uint32_T)
    sensorMCUSlugsMKII_B.dFailsafe >> 18);

  /* S-Function "dsPIC_C_function_Call" Block: <S3>/Read XYZ from HIL [hil.c] */
  hil_getXYZ(&sensorMCUSlugsMKII_B.ReadXYZfromHILhilc[0]);

  /* DataTypeConversion: '<S104>/Data Type Conversion2' incorporates:
   *  Gain: '<S89>/Gain'
   *  Sum: '<S89>/Sum'
   *  Sum: '<S89>/Sum1'
   *  UnitDelay: '<S89>/Unit Delay'
   */
  rtb_DataTypeConversion2_h = (rtb_u11_idx -
    sensorMCUSlugsMKII_DWork.UnitDelay_DSTATE_p) *
    sensorMCUSlugsMKII_P.Gain_Gain_b + rtb_Product1_c;

  /* DiscreteZeroPole: '<S105>/Discrete Zero-Pole' */
  {
    rtb_RoundingFunction = sensorMCUSlugsMKII_P.DiscreteZeroPole_D_c*
      rtb_DataTypeConversion2_h;
    rtb_RoundingFunction += sensorMCUSlugsMKII_P.DiscreteZeroPole_C_cr*
      sensorMCUSlugsMKII_DWork.DiscreteZeroPole_DSTATE_f;
  }

  /* DataTypeConversion: '<S104>/Data Type Conversion1' */
  rtb_DataTypeConversion1_f = (real32_T)rtb_RoundingFunction;

  /* DataTypeConversion: '<S106>/Data Type Conversion2' incorporates:
   *  Gain: '<S90>/Gain'
   *  Sum: '<S90>/Sum'
   *  Sum: '<S90>/Sum1'
   *  UnitDelay: '<S90>/Unit Delay'
   */
  rtb_DataTypeConversion2_i = (rtb_u11_idx_0 -
    sensorMCUSlugsMKII_DWork.UnitDelay_DSTATE_f) *
    sensorMCUSlugsMKII_P.Gain_Gain_gt + rtb_Product_j;

  /* DiscreteZeroPole: '<S107>/Discrete Zero-Pole' */
  {
    rtb_RoundingFunction = sensorMCUSlugsMKII_P.DiscreteZeroPole_D_co*
      rtb_DataTypeConversion2_i;
    rtb_RoundingFunction += sensorMCUSlugsMKII_P.DiscreteZeroPole_C_b*
      sensorMCUSlugsMKII_DWork.DiscreteZeroPole_DSTATE_a;
  }

  /* DataTypeConversion: '<S106>/Data Type Conversion1' */
  rtb_DataTypeConversion1_ff = (real32_T)rtb_RoundingFunction;

  /* Gain: '<S86>/Gain1' */
  rtb_Gain1_p = sensorMCUSlugsMKII_P.Gain1_Gain_a * rtb_RhhcosphisinlambYe;

  /* MATLAB Function: '<S86>/myMux Fun1' */
  sensorMCUSlugsMKII_myMuxFun1_l(rtb_DataTypeConversion1_f,
    rtb_DataTypeConversion1_ff, rtb_Gain1_p,
    &sensorMCUSlugsMKII_B.sf_myMuxFun1_l);

  /* Switch: '<S3>/Switch4' */
  if (rtb_Product_b > sensorMCUSlugsMKII_P.Switch4_Threshold) {
    rtb_Switch4[0] = sensorMCUSlugsMKII_B.ReadXYZfromHILhilc[0];
    rtb_Switch4[1] = sensorMCUSlugsMKII_B.ReadXYZfromHILhilc[1];
    rtb_Switch4[2] = sensorMCUSlugsMKII_B.ReadXYZfromHILhilc[2];
  } else {
    rtb_Switch4[0] = sensorMCUSlugsMKII_B.sf_myMuxFun1_l.y[0];
    rtb_Switch4[1] = sensorMCUSlugsMKII_B.sf_myMuxFun1_l.y[1];
    rtb_Switch4[2] = sensorMCUSlugsMKII_B.sf_myMuxFun1_l.y[2];
  }

  /* End of Switch: '<S3>/Switch4' */
  /* S-Function "dsPIC_C_function_Call" Block: <S3>/Read the Vned  from HIL [hil.c] */
  hil_getVned(&sensorMCUSlugsMKII_B.ReadtheVnedfromHILhilc[0]);

  /* Switch: '<S3>/Switch1' */
  if (rtb_Product_b > sensorMCUSlugsMKII_P.Switch1_Threshold) {
    rtb_Switch1[0] = sensorMCUSlugsMKII_B.ReadtheVnedfromHILhilc[0];
    rtb_Switch1[1] = sensorMCUSlugsMKII_B.ReadtheVnedfromHILhilc[1];
    rtb_Switch1[2] = sensorMCUSlugsMKII_B.ReadtheVnedfromHILhilc[2];
  } else {
    rtb_Switch1[0] = sensorMCUSlugsMKII_B.sf_myMuxFun2_f.y[0];
    rtb_Switch1[1] = sensorMCUSlugsMKII_B.sf_myMuxFun2_f.y[1];
    rtb_Switch1[2] = sensorMCUSlugsMKII_B.sf_myMuxFun2_f.y[2];
  }

  /* End of Switch: '<S3>/Switch1' */

  /* MATLAB Function: '<Root>/myMux Fun3' */
  sensorMCUSlugsMKII_myMuxFun2(rtb_Switch4, rtb_Switch1,
    &sensorMCUSlugsMKII_B.sf_myMuxFun3);

  /* DataTypeConversion: '<S117>/Data Type Conversion5' incorporates:
   *  Constant: '<S117>/Gyro Gains2'
   *  Gain: '<S129>/[ -1 -1 -1]'
   *  Product: '<S117>/Divide2'
   */
  rtb_DataTypeConversion5_e[0] = (real32_T)rtb_Switch_k[7] *
    sensorMCUSlugsMKII_P.GyroGains2_Value * sensorMCUSlugsMKII_P.u11_Gain_o[0];
  rtb_DataTypeConversion5_e[1] = (real32_T)rtb_Switch_k[6] *
    sensorMCUSlugsMKII_P.GyroGains2_Value * sensorMCUSlugsMKII_P.u11_Gain_o[1];
  rtb_DataTypeConversion5_e[2] = (real32_T)rtb_Switch_k[8] *
    sensorMCUSlugsMKII_P.GyroGains2_Value * sensorMCUSlugsMKII_P.u11_Gain_o[2];

  /* MATLAB Function: '<S126>/Embedded MATLAB Function' */
  sensor_EmbeddedMATLABFunction_c(rtb_DataTypeConversion5_e[0],
    sensorMCUSlugsMKII_P.Constant_Value_g,
    sensorMCUSlugsMKII_P.Constant1_Value_l,
    &sensorMCUSlugsMKII_B.sf_EmbeddedMATLABFunction_d,
    &sensorMCUSlugsMKII_DWork.sf_EmbeddedMATLABFunction_d);

  /* MATLAB Function: '<S126>/Embedded MATLAB Function1' */
  sensor_EmbeddedMATLABFunction_c(rtb_DataTypeConversion5_e[1],
    sensorMCUSlugsMKII_P.Constant2_Value_c,
    sensorMCUSlugsMKII_P.Constant3_Value_d,
    &sensorMCUSlugsMKII_B.sf_EmbeddedMATLABFunction1_o,
    &sensorMCUSlugsMKII_DWork.sf_EmbeddedMATLABFunction1_o);

  /* MATLAB Function: '<S126>/Embedded MATLAB Function2' */
  sensor_EmbeddedMATLABFunction_c(rtb_DataTypeConversion5_e[2],
    sensorMCUSlugsMKII_P.Constant4_Value_h,
    sensorMCUSlugsMKII_P.Constant5_Value_f,
    &sensorMCUSlugsMKII_B.sf_EmbeddedMATLABFunction2_b,
    &sensorMCUSlugsMKII_DWork.sf_EmbeddedMATLABFunction2_b);

  /* MATLAB Function: '<S126>/myMux Fun' */
  sensorMCUSlugsMKII_myMuxFun(sensorMCUSlugsMKII_B.sf_EmbeddedMATLABFunction_d.y,
    sensorMCUSlugsMKII_B.sf_EmbeddedMATLABFunction1_o.y,
    sensorMCUSlugsMKII_B.sf_EmbeddedMATLABFunction2_b.y,
    &sensorMCUSlugsMKII_B.sf_myMuxFun_f);

  /* DataTypeConversion: '<S117>/Data Type Conversion6' */
  rtb_u11_idx = (real32_T)sensorMCUSlugsMKII_B.sf_myMuxFun_f.y[0];
  rtb_u11_idx_0 = (real32_T)sensorMCUSlugsMKII_B.sf_myMuxFun_f.y[1];
  rtb_Sum_l = (real32_T)sensorMCUSlugsMKII_B.sf_myMuxFun_f.y[2];

  /* Product: '<S117>/Divide' incorporates:
   *  Constant: '<S117>/Gyro Gains'
   */
  rtb_UEN2NEU[0] = (real32_T)rtb_Switch_k[0] *
    sensorMCUSlugsMKII_P.GyroGains_Value;
  rtb_UEN2NEU[1] = (real32_T)rtb_Switch_k[1] *
    sensorMCUSlugsMKII_P.GyroGains_Value;
  rtb_UEN2NEU[2] = (real32_T)rtb_Switch_k[2] *
    sensorMCUSlugsMKII_P.GyroGains_Value;

  /* DataTypeConversion: '<S117>/Data Type Conversion2' incorporates:
   *  Gain: '<S127>/[ -1 -1 -1]'
   */
  rtb_DataTypeConversion2_g[0] = sensorMCUSlugsMKII_P.u11_Gain_j[0] *
    rtb_UEN2NEU[1];
  rtb_DataTypeConversion2_g[1] = sensorMCUSlugsMKII_P.u11_Gain_j[1] *
    rtb_UEN2NEU[0];
  rtb_DataTypeConversion2_g[2] = sensorMCUSlugsMKII_P.u11_Gain_j[2] *
    rtb_UEN2NEU[2];

  /* MATLAB Function: '<S124>/Embedded MATLAB Function' */
  sensor_EmbeddedMATLABFunction_c(rtb_DataTypeConversion2_g[0],
    sensorMCUSlugsMKII_P.Constant_Value_d,
    sensorMCUSlugsMKII_P.Constant1_Value_h,
    &sensorMCUSlugsMKII_B.sf_EmbeddedMATLABFunction_c,
    &sensorMCUSlugsMKII_DWork.sf_EmbeddedMATLABFunction_c);

  /* MATLAB Function: '<S124>/Embedded MATLAB Function1' */
  sensor_EmbeddedMATLABFunction_c(rtb_DataTypeConversion2_g[1],
    sensorMCUSlugsMKII_P.Constant2_Value_j,
    sensorMCUSlugsMKII_P.Constant3_Value_j,
    &sensorMCUSlugsMKII_B.sf_EmbeddedMATLABFunction1_f,
    &sensorMCUSlugsMKII_DWork.sf_EmbeddedMATLABFunction1_f);

  /* MATLAB Function: '<S124>/Embedded MATLAB Function2' */
  sensor_EmbeddedMATLABFunction_c(rtb_DataTypeConversion2_g[2],
    sensorMCUSlugsMKII_P.Constant4_Value_l,
    sensorMCUSlugsMKII_P.Constant5_Value_d,
    &sensorMCUSlugsMKII_B.sf_EmbeddedMATLABFunction2_l,
    &sensorMCUSlugsMKII_DWork.sf_EmbeddedMATLABFunction2_l);

  /* MATLAB Function: '<S124>/myMux Fun' */
  sensorMCUSlugsMKII_myMuxFun(sensorMCUSlugsMKII_B.sf_EmbeddedMATLABFunction_c.y,
    sensorMCUSlugsMKII_B.sf_EmbeddedMATLABFunction1_f.y,
    sensorMCUSlugsMKII_B.sf_EmbeddedMATLABFunction2_l.y,
    &sensorMCUSlugsMKII_B.sf_myMuxFun_c);

  /* DataTypeConversion: '<S117>/Data Type Conversion7' */
  rtb_UEN2NEU[0] = (real32_T)sensorMCUSlugsMKII_B.sf_myMuxFun_c.y[0];
  rtb_UEN2NEU[1] = (real32_T)sensorMCUSlugsMKII_B.sf_myMuxFun_c.y[1];
  rtb_UEN2NEU[2] = (real32_T)sensorMCUSlugsMKII_B.sf_myMuxFun_c.y[2];

  /* MATLAB Function: '<Root>/myMux Fun1' */
  /* MATLAB Function 'myMux Fun1': '<S6>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S6>:1:5' */
  sensorMCUSlugsMKII_B.y[0] = rtb_DataTypeConversion3_e_idx;
  sensorMCUSlugsMKII_B.y[1] = rtb_DataTypeConversion3_e_idx_0;
  sensorMCUSlugsMKII_B.y[2] = rtb_DataTypeConversion3_e_idx_1;
  sensorMCUSlugsMKII_B.y[3] = rtb_u11_idx;
  sensorMCUSlugsMKII_B.y[4] = rtb_u11_idx_0;
  sensorMCUSlugsMKII_B.y[5] = rtb_Sum_l;
  sensorMCUSlugsMKII_B.y[6] = rtb_UEN2NEU[0];
  sensorMCUSlugsMKII_B.y[7] = rtb_UEN2NEU[1];
  sensorMCUSlugsMKII_B.y[8] = rtb_UEN2NEU[2];

  /* S-Function "dsPIC_C_function_Call" Block: <S3>/Read timestamp from HIL [hil.c] */
  sensorMCUSlugsMKII_B.ReadtimestampfromHILhilc = hil_getTs();

  /* UnitDelay: '<S112>/Output' */
  rtb_Output = sensorMCUSlugsMKII_DWork.Output_DSTATE;

  /* Switch: '<S3>/Switch5' incorporates:
   *  UnitDelay: '<S112>/Output'
   */
  if (rtb_Product_b > sensorMCUSlugsMKII_P.Switch5_Threshold) {
    sensorMCUSlugsMKII_B.Switch5 = (uint32_T)
      sensorMCUSlugsMKII_B.ReadtimestampfromHILhilc;
  } else {
    sensorMCUSlugsMKII_B.Switch5 = sensorMCUSlugsMKII_DWork.Output_DSTATE;
  }

  /* End of Switch: '<S3>/Switch5' */
  /* S-Function "dsPIC_C_function_Call" Block: <S3>/Read Euler  from HIL [hil.c] */
  hil_getEuler(&sensorMCUSlugsMKII_B.ReadEulerfromHILhilc[0]);

  /* Gain: '<S44>/Gain1' incorporates:
   *  Selector: '<S44>/Selector1'
   */
  rtb_Deg2R_idx = sensorMCUSlugsMKII_P.Gain1_Gain_b[2] *
    sensorMCUSlugsMKII_B.VectorConcatenate[6];

  /* If: '<S24>/If' */
  if ((rtb_Deg2R_idx >= 1.0F) || (rtb_Deg2R_idx <= -1.0F)) {
    /* Outputs for IfAction SubSystem: '<S24>/AxisRotZeroR3' incorporates:
     *  ActionPort: '<S43>/Action Port'
     */
    /* Fcn: '<S43>/Fcn1' incorporates:
     *  Gain: '<S44>/Gain3'
     *  Selector: '<S44>/Selector3'
     */
    rtb_Merge_idx = rt_atan2f_snf(sensorMCUSlugsMKII_P.Gain3_Gain[0] *
      sensorMCUSlugsMKII_B.VectorConcatenate[1],
      sensorMCUSlugsMKII_P.Gain3_Gain[1] *
      sensorMCUSlugsMKII_B.VectorConcatenate[4]);

    /* Fcn: '<S43>/Fcn2' */
    rtb_Deg2R_idx_0 = (real32_T)asin(rtb_Deg2R_idx >= 1.0F ? 1.0F :
      rtb_Deg2R_idx <= -1.0F ? -1.0F : rtb_Deg2R_idx);

    /* Fcn: '<S43>/Fcn3' */
    rtb_Deg2R_idx = 0.0F;

    /* End of Outputs for SubSystem: '<S24>/AxisRotZeroR3' */
  } else {
    /* Outputs for IfAction SubSystem: '<S24>/AxisRotDefault' incorporates:
     *  ActionPort: '<S42>/Action Port'
     */
    /* Fcn: '<S42>/Fcn1' incorporates:
     *  Gain: '<S44>/Gain1'
     *  Selector: '<S44>/Selector1'
     */
    rtb_Merge_idx = rt_atan2f_snf(sensorMCUSlugsMKII_P.Gain1_Gain_b[0] *
      sensorMCUSlugsMKII_B.VectorConcatenate[3],
      sensorMCUSlugsMKII_P.Gain1_Gain_b[1] *
      sensorMCUSlugsMKII_B.VectorConcatenate[0]);

    /* Fcn: '<S42>/Fcn2' */
    rtb_Deg2R_idx_0 = (real32_T)asin(rtb_Deg2R_idx >= 1.0F ? 1.0F :
      rtb_Deg2R_idx <= -1.0F ? -1.0F : rtb_Deg2R_idx);

    /* Fcn: '<S42>/Fcn3' incorporates:
     *  Gain: '<S44>/Gain2'
     *  Selector: '<S44>/Selector2'
     */
    rtb_Deg2R_idx = rt_atan2f_snf(sensorMCUSlugsMKII_P.Gain2_Gain_o[0] *
      sensorMCUSlugsMKII_B.VectorConcatenate[7],
      sensorMCUSlugsMKII_P.Gain2_Gain_o[1] *
      sensorMCUSlugsMKII_B.VectorConcatenate[8]);

    /* End of Outputs for SubSystem: '<S24>/AxisRotDefault' */
  }

  /* End of If: '<S24>/If' */

  /* MATLAB Function: '<S12>/Embedded MATLAB Function' */
  /* MATLAB Function 'Position and Attitude Filter/Attitude Complimentary Filter COG/Embedded MATLAB Function': '<S25>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  if (rtb_Merge_idx < 0.0F) {
    /* '<S25>:1:5' */
    /* '<S25>:1:6' */
    rtb_Merge_idx += 6.28318548F;
  } else {
    /* '<S25>:1:8' */
  }

  /* End of MATLAB Function: '<S12>/Embedded MATLAB Function' */

  /* Switch: '<S3>/Switch2' */
  if (rtb_Product_b > sensorMCUSlugsMKII_P.Switch2_Threshold_g) {
    rtb_Switch2[0] = sensorMCUSlugsMKII_B.ReadEulerfromHILhilc[0];
  } else {
    rtb_Switch2[0] = rtb_Deg2R_idx;
  }

  if (rtb_Product_b > sensorMCUSlugsMKII_P.Switch2_Threshold_g) {
    rtb_Switch2[1] = sensorMCUSlugsMKII_B.ReadEulerfromHILhilc[1];
  } else {
    rtb_Switch2[1] = rtb_Deg2R_idx_0;
  }

  if (rtb_Product_b > sensorMCUSlugsMKII_P.Switch2_Threshold_g) {
    rtb_Switch2[2] = sensorMCUSlugsMKII_B.ReadEulerfromHILhilc[2];
  } else {
    rtb_Switch2[2] = rtb_Merge_idx;
  }

  /* End of Switch: '<S3>/Switch2' */
  /* S-Function "dsPIC_C_function_Call" Block: <S3>/Read PQR from HIL [hil.c] */
  hil_getRates(&sensorMCUSlugsMKII_B.ReadPQRfromHILhilc[0]);

  /* Sum: '<S12>/Sum3' */
  rtb_GyroErr_idx_1 += rtb_UEN2NEU[0];
  rtb_GyroErr_idx += rtb_UEN2NEU[1];
  rtb_jxi += rtb_UEN2NEU[2];

  /* Switch: '<S3>/Switch3' */
  if (rtb_Product_b > sensorMCUSlugsMKII_P.Switch3_Threshold) {
    rtb_Switch3[0] = sensorMCUSlugsMKII_B.ReadPQRfromHILhilc[0];
    rtb_Switch3[1] = sensorMCUSlugsMKII_B.ReadPQRfromHILhilc[1];
    rtb_Switch3[2] = sensorMCUSlugsMKII_B.ReadPQRfromHILhilc[2];
  } else {
    rtb_Switch3[0] = rtb_GyroErr_idx_1;
    rtb_Switch3[1] = rtb_GyroErr_idx;
    rtb_Switch3[2] = rtb_jxi;
  }

  /* End of Switch: '<S3>/Switch3' */

  /* MATLAB Function: '<Root>/myMux Fun2' */
  sensorMCUSlugsMKII_myMuxFun2(rtb_Switch2, rtb_Switch3,
    &sensorMCUSlugsMKII_B.sf_myMuxFun2_l);

  /* S-Function "dsPIC_C_function_Call" Block: <S5>/Update Euler and PQR [updateSensorMcuState.c] */
  updateAttitude(sensorMCUSlugsMKII_B.sf_myMuxFun2_l.y);

  /* S-Function "dsPIC_C_function_Call" Block: <S5>/Update the Time Stamp [updateSensorMcuState.c] */
  updateTimeStamp(sensorMCUSlugsMKII_B.Switch5);

  /* S-Function "dsPIC_C_function_Call" Block: <S5>/Update Accels and Mags [updateSensorMcuState.c] */
  updateSensorData(sensorMCUSlugsMKII_B.y);

  /* S-Function "dsPIC_C_function_Call" Block: <S5>/Update XYZ and Vned [updateSensorMcuState.c] */
  updatePosition(sensorMCUSlugsMKII_B.sf_myMuxFun3.y);

  /* S-Function "dsPIC_C_function_Call" Block: <S5>/Update Pilot Console [updateSensorMcuState.c] */
  updatePilotConsole(sensorMCUSlugsMKII_B.DataTypeConversion);

  /* S-Function "dsPIC_C_function_Call" Block: <S5>/Update Biases [updateSensorMcuState.c] */
  updateBias(sensorMCUSlugsMKII_B.sf_myMuxFun4.y);

  /* S-Function "dsPIC_C_function_Call" Block: <S5>/Send Debug  Data to Serial Port and Prepare for SPI [IPCScheduler.c] */
  scheduleData(sensorMCUSlugsMKII_B.DataTypeConversion_e,
               &sensorMCUSlugsMKII_B.SendDebugDatatoSerialPortandPre[0]);

  /* S-Function "dsPIC_Digital_OutputWrite" Block: <S178>/Drive SSL Low */
  LATBbits.LATB2 = sensorMCUSlugsMKII_P.Constant1_Value_kt;

  /* S-Function "dsPIC_Nop" Block: <S178>/Nop */
  asm ("nop");
  asm ("nop");
  asm ("nop");
  asm ("nop");
  asm ("nop");
  asm ("nop");
  asm ("nop");
  asm ("nop");
  asm ("nop");
  asm ("nop");

  /* S-Function "dsPIC_C_function_Call" Block: <S178>/Send Data Via SPI [interProcCommMaster.c] */
  spiSend(sensorMCUSlugsMKII_B.SendDebugDatatoSerialPortandPre);

  /* S-Function "dsPIC_Nop" Block: <S178>/Nop1 */
  asm ("nop");
  asm ("nop");
  asm ("nop");
  asm ("nop");
  asm ("nop");
  asm ("nop");
  asm ("nop");
  asm ("nop");
  asm ("nop");
  asm ("nop");

  /* S-Function "dsPIC_Digital_OutputWrite" Block: <S178>/Drive SSL High */
  LATBbits.LATB2 = sensorMCUSlugsMKII_P.Constant2_Value_e;

  /* Sum: '<S12>/Sum1' incorporates:
   *  Gain: '<S12>/Gain'
   */
  rtb_GyroErr_idx_1 += sensorMCUSlugsMKII_P.Gain_Gain_o * rtb_Sum4_g_idx_1;
  rtb_GyroErr_idx += sensorMCUSlugsMKII_P.Gain_Gain_o * rtb_Sum4_g_idx;
  rtb_jxi += sensorMCUSlugsMKII_P.Gain_Gain_o * rtb_Sum4_g_idx_0;

  /* MATLAB Function: '<S12>/q dot calc' incorporates:
   *  SignalConversion: '<S31>/TmpSignal ConversionAt SFunction Inport1'
   */
  /* MATLAB Function 'Position and Attitude Filter/Attitude Complimentary Filter COG/q dot calc': '<S31>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S31>:1:5' */
  /* '<S31>:1:6' */
  /* '<S31>:1:7' */
  rtb_q_dot[0] = 0.0F;
  rtb_q_dot[1] = 0.0F;
  rtb_q_dot[2] = 0.0F;
  rtb_q_dot[3] = 0.0F;

  /* '<S31>:1:8' */
  rtb_q_dot[0] = (-0.5F * rtb_Product1_h * rtb_GyroErr_idx_1 + -0.5F *
                  rtb_Product2_d * rtb_GyroErr_idx) + -0.5F * rtb_Product3_o *
    rtb_jxi;

  /* '<S31>:1:9' */
  rtb_TmpSignalConversionAtSFun_0[0] = rtb_Product_d;
  rtb_TmpSignalConversionAtSFun_0[3] = -rtb_Product3_o;
  rtb_TmpSignalConversionAtSFun_0[6] = rtb_Product2_d;
  rtb_TmpSignalConversionAtSFun_0[1] = rtb_Product3_o;
  rtb_TmpSignalConversionAtSFun_0[4] = rtb_Product_d;
  rtb_TmpSignalConversionAtSFun_0[7] = -rtb_Product1_h;
  rtb_TmpSignalConversionAtSFun_0[2] = -rtb_Product2_d;
  rtb_TmpSignalConversionAtSFun_0[5] = rtb_Product1_h;
  rtb_TmpSignalConversionAtSFun_0[8] = rtb_Product_d;
  for (colIdx = 0; colIdx < 3; colIdx++) {
    tmp[3 * colIdx] = rtb_TmpSignalConversionAtSFun_0[3 * colIdx] * 0.5F;
    tmp[1 + 3 * colIdx] = rtb_TmpSignalConversionAtSFun_0[3 * colIdx + 1] * 0.5F;
    tmp[2 + 3 * colIdx] = rtb_TmpSignalConversionAtSFun_0[3 * colIdx + 2] * 0.5F;
  }

  for (colIdx = 0; colIdx < 3; colIdx++) {
    rtb_q_dot[(int32_T)(1 + colIdx)] = 0.0F;
    rtb_q_dot[(int32_T)(1 + colIdx)] += tmp[colIdx] * rtb_GyroErr_idx_1;
    rtb_q_dot[(int32_T)(1 + colIdx)] += tmp[colIdx + 3] * rtb_GyroErr_idx;
    rtb_q_dot[(int32_T)(1 + colIdx)] += tmp[colIdx + 6] * rtb_jxi;
  }

  /* End of MATLAB Function: '<S12>/q dot calc' */

  /* DataTypeConversion: '<S108>/Data Type Conversion2' */
  rtb_DataTypeConversion2_c = rtb_Gain_a;

  /* DiscreteZeroPole: '<S109>/Discrete Zero-Pole' */
  {
    rtb_RoundingFunction = sensorMCUSlugsMKII_P.DiscreteZeroPole_D_d*
      rtb_DataTypeConversion2_c;
    rtb_RoundingFunction += sensorMCUSlugsMKII_P.DiscreteZeroPole_C_m*
      sensorMCUSlugsMKII_DWork.DiscreteZeroPole_DSTATE_n;
  }

  /* DataTypeConversion: '<S108>/Data Type Conversion1' */
  rtb_Merge_idx = (real32_T)rtb_RoundingFunction;

  /* Sum: '<S113>/FixPt Sum1' incorporates:
   *  Constant: '<S113>/FixPt Constant'
   */
  rtb_Output += sensorMCUSlugsMKII_P.FixPtConstant_Value;

  /* Switch: '<S114>/FixPt Switch' incorporates:
   *  Constant: '<S114>/Constant'
   */
  if (rtb_Output > sensorMCUSlugsMKII_P.FixPtSwitch_Threshold) {
    rtb_Output = sensorMCUSlugsMKII_P.Constant_Value_e;
  }

  /* End of Switch: '<S114>/FixPt Switch' */

  /* DataTypeConversion: '<S121>/Data Type Conversion3' */
  rtb_DataTypeConversion3 = (real_T)rtb_Switch_k[10];

  /* MATLAB Function: '<S161>/Embedded MATLAB Function' */
  sensor_EmbeddedMATLABFunction_c(rtb_DataTypeConversion3,
    sensorMCUSlugsMKII_P.Constant_Value_dj,
    sensorMCUSlugsMKII_P.Constant1_Value_j,
    &sensorMCUSlugsMKII_B.sf_EmbeddedMATLABFunction_f,
    &sensorMCUSlugsMKII_DWork.sf_EmbeddedMATLABFunction_f);

  /* Sum: '<S152>/Sum' incorporates:
   *  Abs: '<S121>/Abs'
   *  Constant: '<S152>/Bias'
   *  Constant: '<S152>/Gains'
   *  Constant: '<S153>/Gains'
   *  Constant: '<S154>/Bias'
   *  Constant: '<S154>/Gains'
   *  DataTypeConversion: '<S121>/Data Type Conversion4'
   *  Product: '<S152>/Divide'
   *  Product: '<S153>/Divide'
   *  Product: '<S154>/Divide'
   *  Sum: '<S154>/Sum'
   */
  rtb_RoundingFunction = (real_T)((real32_T)fabs
    ((sensorMCUSlugsMKII_P.Gains_Value_a * (real32_T)
      sensorMCUSlugsMKII_B.sf_EmbeddedMATLABFunction_f.y +
      sensorMCUSlugsMKII_P.Bias_Value_c) * sensorMCUSlugsMKII_P.Gains_Value_b) *
    sensorMCUSlugsMKII_P.Gains_Value_g) + sensorMCUSlugsMKII_P.Bias_Value;

  /* Saturate: '<S121>/[0.001  maxDynPress]' */
  omega = rtb_RoundingFunction >= sensorMCUSlugsMKII_P.u001maxDynPress_UpperSat ?
    sensorMCUSlugsMKII_P.u001maxDynPress_UpperSat : rtb_RoundingFunction <=
    sensorMCUSlugsMKII_P.u001maxDynPress_LowerSat ?
    sensorMCUSlugsMKII_P.u001maxDynPress_LowerSat : rtb_RoundingFunction;

  /* Outputs for Enabled SubSystem: '<S116>/If no HIL then update Air Data' incorporates:
   *  EnablePort: '<S120>/Enable'
   */
  /* MATLAB Function 'Sensor Data/Sensor Suite/myMux Fun': '<S122>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S122>:1:5' */
  if (rtb_LogicalOperator) {
    /* Inport: '<S120>/AirData' incorporates:
     *  MATLAB Function: '<S116>/myMux Fun'
     */
    sensorMCUSlugsMKII_B.AirData[0] = (real32_T)omega;
    sensorMCUSlugsMKII_B.AirData[1] = rtb_ixk;
    sensorMCUSlugsMKII_B.AirData[2] = rtb_RhhcosphicoslambXe;

    /* S-Function "dsPIC_C_function_Call" Block: <S120>/Update the Air Calibrated Data [updateSensorMcuState.c] */
    updateAirData(sensorMCUSlugsMKII_B.AirData);
  }

  /* End of Outputs for SubSystem: '<S116>/If no HIL then update Air Data' */

  /* MATLAB Function: '<S116>/myMux Fun1' incorporates:
   *  DataTypeConversion: '<S116>/Data Type Conversion5'
   */
  /* MATLAB Function 'Sensor Data/Sensor Suite/myMux Fun1': '<S123>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S123>:1:5' */
  sensorMCUSlugsMKII_B.y_n[0] = (real32_T)omega;
  sensorMCUSlugsMKII_B.y_n[1] = sensorMCUSlugsMKII_B.AirData[0];
  sensorMCUSlugsMKII_B.y_n[2] = 0.0F;
  sensorMCUSlugsMKII_B.y_n[3] = 0.0F;

  /* S-Function "dsPIC_C_function_Call" Block: <S116>/Sensor DSC Diag [updateSensorMcuState.c] */
  updateSensorDiag(sensorMCUSlugsMKII_B.y_n);

  /* S-Function "UART_RX" Block: <S118>/Calculus Time Step */
  sensorMCUSlugsMKII_B.CalculusTimeStep_o1 = CalculusTimeStep;
  sensorMCUSlugsMKII_B.CalculusTimeStep_o2 = PR1;

  /* Product: '<S118>/Divide' incorporates:
   *  DataTypeConversion: '<S118>/Data Type Conversion1'
   *  DataTypeConversion: '<S118>/Data Type Conversion2'
   */
  rtb_RoundingFunction = (real_T)sensorMCUSlugsMKII_B.CalculusTimeStep_o1 /
    (real_T)sensorMCUSlugsMKII_B.CalculusTimeStep_o2;

  /* Gain: '<S118>/Gain' */
  omega = sensorMCUSlugsMKII_P.Gain_Gain * rtb_RoundingFunction;

  /* Rounding: '<S118>/Rounding Function' */
  rtb_RoundingFunction = floor(omega);

  /* DataTypeConversion: '<S118>/Data Type Conversion12' */
  omega = floor(rtb_RoundingFunction);
  if (rtIsNaN(omega) || rtIsInf(omega)) {
    omega = 0.0;
  } else {
    omega = fmod(omega, 256.0);
  }

  sensorMCUSlugsMKII_B.DataTypeConversion12 = (uint8_T)(omega < 0.0 ? (uint8_T)
    (int16_T)(int8_T)-(int8_T)(uint8_T)-omega : (uint8_T)omega);

  /* End of DataTypeConversion: '<S118>/Data Type Conversion12' */

  /* DataTypeConversion: '<S121>/Data Type Conversion6' */
  rtb_DataTypeConversion6 = (real_T)rtb_Switch_k[11];

  /* MATLAB Function: '<S160>/Embedded MATLAB Function' */
  sensor_EmbeddedMATLABFunction_c(rtb_DataTypeConversion6,
    sensorMCUSlugsMKII_P.Constant_Value_a,
    sensorMCUSlugsMKII_P.Constant1_Value_k5,
    &sensorMCUSlugsMKII_B.sf_EmbeddedMATLABFunction_gc,
    &sensorMCUSlugsMKII_DWork.sf_EmbeddedMATLABFunction_gc);

  /* DataTypeConversion: '<S121>/Data Type Conversion8' incorporates:
   *  Constant: '<S155>/Bias'
   *  Constant: '<S155>/Gains'
   *  Product: '<S155>/Divide'
   *  Sum: '<S155>/Sum'
   */
  rtb_Deg2R_idx_0 = (real32_T)floor((real32_T)((real_T)
    sensorMCUSlugsMKII_P.Gains_Value_k *
    sensorMCUSlugsMKII_B.sf_EmbeddedMATLABFunction_gc.y) +
    sensorMCUSlugsMKII_P.Bias_Value_la);
  if (rtIsNaNF(rtb_Deg2R_idx_0) || rtIsInfF(rtb_Deg2R_idx_0)) {
    rtb_Deg2R_idx_0 = 0.0F;
  } else {
    rtb_Deg2R_idx_0 = (real32_T)fmod(rtb_Deg2R_idx_0, 65536.0F);
  }

  sensorMCUSlugsMKII_B.DataTypeConversion8 = rtb_Deg2R_idx_0 < 0.0F ? (uint16_T)
    -(int16_T)(uint16_T)-rtb_Deg2R_idx_0 : (uint16_T)rtb_Deg2R_idx_0;

  /* End of DataTypeConversion: '<S121>/Data Type Conversion8' */
  /* S-Function "dsPIC_C_function_Call" Block: <S116>/Update the Load and Power Data [updateSensorMcuState.c] */
  updateLoadData(sensorMCUSlugsMKII_B.DataTypeConversion12,
                 sensorMCUSlugsMKII_B.DataTypeConversion8);

  /* S-Function "dsPIC_C_function_Call" Block: <Root>/Initialize Sensor MCU [interProcCommMaster.c] */

  /* Update for DiscreteIntegrator: '<S12>/Discrete-Time Integrator1' */
  sensorMCUSlugsMKII_DWork.DiscreteTimeIntegrator1_DSTATE[0] =
    sensorMCUSlugsMKII_P.DiscreteTimeIntegrator1_gainval * rtb_q_dot[0] +
    sensorMCUSlugsMKII_DWork.DiscreteTimeIntegrator1_DSTATE[0];
  sensorMCUSlugsMKII_DWork.DiscreteTimeIntegrator1_DSTATE[1] =
    sensorMCUSlugsMKII_P.DiscreteTimeIntegrator1_gainval * rtb_q_dot[1] +
    sensorMCUSlugsMKII_DWork.DiscreteTimeIntegrator1_DSTATE[1];
  sensorMCUSlugsMKII_DWork.DiscreteTimeIntegrator1_DSTATE[2] =
    sensorMCUSlugsMKII_P.DiscreteTimeIntegrator1_gainval * rtb_q_dot[2] +
    sensorMCUSlugsMKII_DWork.DiscreteTimeIntegrator1_DSTATE[2];
  sensorMCUSlugsMKII_DWork.DiscreteTimeIntegrator1_DSTATE[3] =
    sensorMCUSlugsMKII_P.DiscreteTimeIntegrator1_gainval * rtb_q_dot[3] +
    sensorMCUSlugsMKII_DWork.DiscreteTimeIntegrator1_DSTATE[3];
  if (sensorMCUSlugsMKII_DWork.DiscreteTimeIntegrator1_DSTATE[0] >=
      sensorMCUSlugsMKII_P.DiscreteTimeIntegrator1_UpperSa[0]) {
    sensorMCUSlugsMKII_DWork.DiscreteTimeIntegrator1_DSTATE[0] =
      sensorMCUSlugsMKII_P.DiscreteTimeIntegrator1_UpperSa[0];
  } else {
    if (sensorMCUSlugsMKII_DWork.DiscreteTimeIntegrator1_DSTATE[0] <=
        sensorMCUSlugsMKII_P.DiscreteTimeIntegrator1_LowerSa[0]) {
      sensorMCUSlugsMKII_DWork.DiscreteTimeIntegrator1_DSTATE[0] =
        sensorMCUSlugsMKII_P.DiscreteTimeIntegrator1_LowerSa[0];
    }
  }

  if (sensorMCUSlugsMKII_DWork.DiscreteTimeIntegrator1_DSTATE[1] >=
      sensorMCUSlugsMKII_P.DiscreteTimeIntegrator1_UpperSa[1]) {
    sensorMCUSlugsMKII_DWork.DiscreteTimeIntegrator1_DSTATE[1] =
      sensorMCUSlugsMKII_P.DiscreteTimeIntegrator1_UpperSa[1];
  } else {
    if (sensorMCUSlugsMKII_DWork.DiscreteTimeIntegrator1_DSTATE[1] <=
        sensorMCUSlugsMKII_P.DiscreteTimeIntegrator1_LowerSa[1]) {
      sensorMCUSlugsMKII_DWork.DiscreteTimeIntegrator1_DSTATE[1] =
        sensorMCUSlugsMKII_P.DiscreteTimeIntegrator1_LowerSa[1];
    }
  }

  if (sensorMCUSlugsMKII_DWork.DiscreteTimeIntegrator1_DSTATE[2] >=
      sensorMCUSlugsMKII_P.DiscreteTimeIntegrator1_UpperSa[2]) {
    sensorMCUSlugsMKII_DWork.DiscreteTimeIntegrator1_DSTATE[2] =
      sensorMCUSlugsMKII_P.DiscreteTimeIntegrator1_UpperSa[2];
  } else {
    if (sensorMCUSlugsMKII_DWork.DiscreteTimeIntegrator1_DSTATE[2] <=
        sensorMCUSlugsMKII_P.DiscreteTimeIntegrator1_LowerSa[2]) {
      sensorMCUSlugsMKII_DWork.DiscreteTimeIntegrator1_DSTATE[2] =
        sensorMCUSlugsMKII_P.DiscreteTimeIntegrator1_LowerSa[2];
    }
  }

  if (sensorMCUSlugsMKII_DWork.DiscreteTimeIntegrator1_DSTATE[3] >=
      sensorMCUSlugsMKII_P.DiscreteTimeIntegrator1_UpperSa[3]) {
    sensorMCUSlugsMKII_DWork.DiscreteTimeIntegrator1_DSTATE[3] =
      sensorMCUSlugsMKII_P.DiscreteTimeIntegrator1_UpperSa[3];
  } else {
    if (sensorMCUSlugsMKII_DWork.DiscreteTimeIntegrator1_DSTATE[3] <=
        sensorMCUSlugsMKII_P.DiscreteTimeIntegrator1_LowerSa[3]) {
      sensorMCUSlugsMKII_DWork.DiscreteTimeIntegrator1_DSTATE[3] =
        sensorMCUSlugsMKII_P.DiscreteTimeIntegrator1_LowerSa[3];
    }
  }

  /* End of Update for DiscreteIntegrator: '<S12>/Discrete-Time Integrator1' */

  /* Update for Delay: '<S12>/Integer Delay' */
  sensorMCUSlugsMKII_DWork.IntegerDelay_DSTATE[0] = rtb_GyroErr_idx_1;
  sensorMCUSlugsMKII_DWork.IntegerDelay_DSTATE[1] = rtb_GyroErr_idx;
  sensorMCUSlugsMKII_DWork.IntegerDelay_DSTATE[2] = rtb_jxi;

  /* Update for UnitDelay: '<S78>/UD' */
  sensorMCUSlugsMKII_DWork.UD_DSTATE = sensorMCUSlugsMKII_B.In1_p[0];

  /* Update for UnitDelay: '<S79>/UD' */
  sensorMCUSlugsMKII_DWork.UD_DSTATE_e = sensorMCUSlugsMKII_B.In1_p[1];

  /* Update for UnitDelay: '<S80>/UD' */
  sensorMCUSlugsMKII_DWork.UD_DSTATE_k = sensorMCUSlugsMKII_B.In1_p[2];

  /* Update for Enabled SubSystem: '<S150>/Zero Out Height' incorporates:
   *  Update for EnablePort: '<S168>/Enable'
   */
  if (rtb_tOut > 0.0) {
    /* Update for Delay: '<S168>/Integer Delay' */
    sensorMCUSlugsMKII_DWork.IntegerDelay_DSTATE_m = rtb_Rh;
  }

  /* End of Update for SubSystem: '<S150>/Zero Out Height' */

  /* Update for UnitDelay: '<S91>/Unit Delay' */
  sensorMCUSlugsMKII_DWork.UnitDelay_DSTATE = rtb_Merge_idx;

  /* Update for Delay: '<S23>/Integer Delay1' */
  for (colIdx = 0; colIdx < 4; colIdx++) {
    sensorMCUSlugsMKII_DWork.IntegerDelay1_DSTATE[(uint16_T)colIdx * 3U] =
      sensorMCUSlugsMKII_DWork.IntegerDelay1_DSTATE[((uint16_T)colIdx + 1U) * 3U];
    sensorMCUSlugsMKII_DWork.IntegerDelay1_DSTATE[(uint16_T)colIdx * 3U + 1U] =
      sensorMCUSlugsMKII_DWork.IntegerDelay1_DSTATE[((uint16_T)colIdx + 1U) * 3U
      + 1U];
    sensorMCUSlugsMKII_DWork.IntegerDelay1_DSTATE[(uint16_T)colIdx * 3U + 2U] =
      sensorMCUSlugsMKII_DWork.IntegerDelay1_DSTATE[((uint16_T)colIdx + 1U) * 3U
      + 2U];
  }

  sensorMCUSlugsMKII_DWork.IntegerDelay1_DSTATE[12] = Product[0];
  sensorMCUSlugsMKII_DWork.IntegerDelay1_DSTATE[13] = Product[1];
  sensorMCUSlugsMKII_DWork.IntegerDelay1_DSTATE[14] = Product[2];

  /* End of Update for Delay: '<S23>/Integer Delay1' */

  /* Update for Delay: '<S93>/Integer Delay1' */
  for (colIdx = 0; colIdx < 4; colIdx++) {
    sensorMCUSlugsMKII_DWork.IntegerDelay1_DSTATE_b[(uint16_T)colIdx * 3U] =
      sensorMCUSlugsMKII_DWork.IntegerDelay1_DSTATE_b[((uint16_T)colIdx + 1U) *
      3U];
    sensorMCUSlugsMKII_DWork.IntegerDelay1_DSTATE_b[(uint16_T)colIdx * 3U + 1U] =
      sensorMCUSlugsMKII_DWork.IntegerDelay1_DSTATE_b[((uint16_T)colIdx + 1U) *
      3U + 1U];
    sensorMCUSlugsMKII_DWork.IntegerDelay1_DSTATE_b[(uint16_T)colIdx * 3U + 2U] =
      sensorMCUSlugsMKII_DWork.IntegerDelay1_DSTATE_b[((uint16_T)colIdx + 1U) *
      3U + 2U];
  }

  sensorMCUSlugsMKII_DWork.IntegerDelay1_DSTATE_b[12] = Product1_h[0];
  sensorMCUSlugsMKII_DWork.IntegerDelay1_DSTATE_b[13] = Product1_h[1];
  sensorMCUSlugsMKII_DWork.IntegerDelay1_DSTATE_b[14] = Product1_h[2];

  /* End of Update for Delay: '<S93>/Integer Delay1' */
  /* Update for DiscreteZeroPole: '<S100>/Discrete Zero-Pole' */
  {
    sensorMCUSlugsMKII_DWork.DiscreteZeroPole_DSTATE = rtb_DataTypeConversion2[0]
      + (sensorMCUSlugsMKII_P.DiscreteZeroPole_A_j)*
      sensorMCUSlugsMKII_DWork.DiscreteZeroPole_DSTATE;
  }

  /* Update for DiscreteZeroPole: '<S101>/Discrete Zero-Pole' */
  {
    sensorMCUSlugsMKII_DWork.DiscreteZeroPole_DSTATE_k =
      rtb_DataTypeConversion2[1] + (sensorMCUSlugsMKII_P.DiscreteZeroPole_A_l)*
      sensorMCUSlugsMKII_DWork.DiscreteZeroPole_DSTATE_k;
  }

  /* Update for DiscreteZeroPole: '<S102>/Discrete Zero-Pole' */
  {
    sensorMCUSlugsMKII_DWork.DiscreteZeroPole_DSTATE_e =
      rtb_DataTypeConversion2[2] + (sensorMCUSlugsMKII_P.DiscreteZeroPole_A_i)*
      sensorMCUSlugsMKII_DWork.DiscreteZeroPole_DSTATE_e;
  }

  /* Update for DiscreteZeroPole: '<S32>/Discrete Zero-Pole' */
  {
    sensorMCUSlugsMKII_DWork.DiscreteZeroPole_DSTATE_kq =
      rtb_DataTypeConversion2_d[0] + sensorMCUSlugsMKII_P.DiscreteZeroPole_A_c*
      sensorMCUSlugsMKII_DWork.DiscreteZeroPole_DSTATE_kq;
  }

  /* Update for DiscreteZeroPole: '<S33>/Discrete Zero-Pole' */
  {
    sensorMCUSlugsMKII_DWork.DiscreteZeroPole_DSTATE_g =
      rtb_DataTypeConversion2_d[1] + sensorMCUSlugsMKII_P.DiscreteZeroPole_A_i3*
      sensorMCUSlugsMKII_DWork.DiscreteZeroPole_DSTATE_g;
  }

  /* Update for DiscreteZeroPole: '<S34>/Discrete Zero-Pole' */
  {
    sensorMCUSlugsMKII_DWork.DiscreteZeroPole_DSTATE_g3 =
      rtb_DataTypeConversion2_d[2] + sensorMCUSlugsMKII_P.DiscreteZeroPole_A_e*
      sensorMCUSlugsMKII_DWork.DiscreteZeroPole_DSTATE_g3;
  }

  /* Update for UnitDelay: '<S89>/Unit Delay' */
  sensorMCUSlugsMKII_DWork.UnitDelay_DSTATE_p = rtb_DataTypeConversion1_f;

  /* Update for DiscreteZeroPole: '<S105>/Discrete Zero-Pole' */
  {
    sensorMCUSlugsMKII_DWork.DiscreteZeroPole_DSTATE_f =
      rtb_DataTypeConversion2_h + sensorMCUSlugsMKII_P.DiscreteZeroPole_A_a*
      sensorMCUSlugsMKII_DWork.DiscreteZeroPole_DSTATE_f;
  }

  /* Update for UnitDelay: '<S90>/Unit Delay' */
  sensorMCUSlugsMKII_DWork.UnitDelay_DSTATE_f = rtb_DataTypeConversion1_ff;

  /* Update for DiscreteZeroPole: '<S107>/Discrete Zero-Pole' */
  {
    sensorMCUSlugsMKII_DWork.DiscreteZeroPole_DSTATE_a =
      rtb_DataTypeConversion2_i + sensorMCUSlugsMKII_P.DiscreteZeroPole_A_aw*
      sensorMCUSlugsMKII_DWork.DiscreteZeroPole_DSTATE_a;
  }

  /* Update for UnitDelay: '<S112>/Output' */
  sensorMCUSlugsMKII_DWork.Output_DSTATE = rtb_Output;

  /* Update for DiscreteZeroPole: '<S109>/Discrete Zero-Pole' */
  {
    sensorMCUSlugsMKII_DWork.DiscreteZeroPole_DSTATE_n =
      rtb_DataTypeConversion2_c + sensorMCUSlugsMKII_P.DiscreteZeroPole_A_f*
      sensorMCUSlugsMKII_DWork.DiscreteZeroPole_DSTATE_n;
  }

  /* Update absolute time for base rate */
  /* The "clockTick0" counts the number of times the code of this task has
   * been executed. The resolution of this integer timer is 0.01, which is the step size
   * of the task. Size of "clockTick0" ensures timer will not overflow during the
   * application lifespan selected.
   */
  sensorMCUSlugsMKII_M->Timing.clockTick0++;
}

/* Model initialize function */
void sensorMCUSlugsMKII_initialize(boolean_T firstTime)
{
  (void)firstTime;

  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* non-finite (run-time) assignments */
  sensorMCUSlugsMKII_P.ZeroBound_UpperSat = rtInfF;
  sensorMCUSlugsMKII_P.ZeroBound_UpperSat_i = rtInfF;

  /* initialize real-time model */
  (void) memset((void *)sensorMCUSlugsMKII_M, 0,
                sizeof(RT_MODEL_sensorMCUSlugsMKII));

  /* block I/O */
  (void) memset(((void *) &sensorMCUSlugsMKII_B), 0,
                sizeof(BlockIO_sensorMCUSlugsMKII));

  /* states (dwork) */
  (void) memset((void *)&sensorMCUSlugsMKII_DWork, 0,
                sizeof(D_Work_sensorMCUSlugsMKII));

  /* S-Function "dsPIC_MASTER" initialization Block: <Root>/Master */
  /* Solver mode : SingleTasking */
  /* CONFIG TIMER 1 for scheduling steps 	*/
  ConfigIntTimer1(T1_INT_PRIOR_0 & T1_INT_OFF);
  T1CON = 0x8010 ;                     /* T1_PS_1_8	*/
  PR1 = 49999;

  /* Configuration TIMER 2 */
  ConfigIntTimer2(T2_INT_PRIOR_0 & T2_INT_OFF);
  OpenTimer2(T2_ON & T2_GATE_OFF &
             T2_PS_1_8 & T2_SOURCE_INT & T2_IDLE_CON , 65535);

  /* Configuration TRIS */
  TRISB = 65531;

  /* Configuration ADCHS */
  AD1PCFGL = (4U);
  AD2PCFGL = (4U);
  AD1PCFGH = ((0U) & 65535);

  /* S-Function "dsPIC_UART_config" initialization Block: <Root>/UART Configuration */
  /* set priority */
  _U1RXIP = 0x0007 & 2;
  _U1TXIP = 0x0007 & 1;

  /* enable/disable interrupt */
  _U1RXIE = (1 != 0);
  _U1TXIE = 0;
  U1MODE = 0x8080 + 1024;
  U1STA = 0x0 + 1 + 0 + 64 + (1<<12);
  U1BRG = 64;
  _U1TXIF = 0;

  /* Start for Enabled SubSystem: '<S116>/If no HIL then Read all the Sensors' */

  /* S-Function "dsPIC_ADC" initialization Block: <S119>/ADC Input */
  /* Initialise ADC converter */
  /* 0 mean each 1 conversion */
#define NoSamplesADC                   3
#define ADCSval                        63
#define SAMCval                        (31 << 8)
#define OutFormatBitsval               (0 << 8)
#define VoltRef                        (0 << 13)

  /* Use internal counter Trigger, continuous simulatenous sampling */
  AD1CON1 = 0x0004 | (7 << 5) | (1 << 10)| OutFormatBitsval;
  AD1CON2 = 0x0400 | (NoSamplesADC << 2) | VoltRef;
  AD1CSSL = (3840 & 65535);
  AD1CON3 = (ADCSval | SAMCval) & 0xff7f;
  AD1CHS0 = 0x0000;
  IPC2 = (IPC2 | (6 << 12));           /*Interrupt Priority : 6*/
  _AD1IF = 0;
  DMACS0 = 0;
  DMA1CON = 0;
  DMA1REQ = 13;
  DMA1STA = __builtin_dmaoffset(ADCBuffChannelDMA);
  DMA1PAD = (volatile uint16_T) &ADC1BUF0;
  DMA1CNT = 4-1;
  _DMA1IF = 0;
  _DMA1IE = 0;
  AD1CON1bits.ADON= 1;
  DMA1CONbits.CHEN = 1;
  _DMA1IE = 1;

  /* _AD1IE = 1; */
  /* use DMA interrupt ! */
  /* Clear Off Any Interrupt Due To Configuration */
  /* Turn on ADC Module */
  AD1CON1bits.ADON= 1;

  /* End of Start for SubSystem: '<S116>/If no HIL then Read all the Sensors' */

  /* Start for Enabled SubSystem: '<S86>/Subsystem' */
  /* InitializeConditions for MATLAB Function: '<S94>/Embedded MATLAB Function1' */
  se_EmbeddedMATLABFunction1_Init
    (&sensorMCUSlugsMKII_DWork.sf_EmbeddedMATLABFunction1_a);

  /* InitializeConditions for MATLAB Function: '<S94>/Embedded MATLAB Function2' */
  se_EmbeddedMATLABFunction1_Init
    (&sensorMCUSlugsMKII_DWork.sf_EmbeddedMATLABFunction2_d);

  /* VirtualOutportStart for Outport: '<S94>/Vn_fil' */
  sensorMCUSlugsMKII_B.sf_EmbeddedMATLABFunction2_d.y =
    sensorMCUSlugsMKII_P.Vn_fil_Y0;

  /* VirtualOutportStart for Outport: '<S94>/Ve_fil' */
  sensorMCUSlugsMKII_B.sf_EmbeddedMATLABFunction1_a.y =
    sensorMCUSlugsMKII_P.Ve_fil_Y0;

  /* End of Start for SubSystem: '<S86>/Subsystem' */
  /* Start for Enabled SubSystem: '<S150>/Zero Out Height' */
  /* InitializeConditions for Delay: '<S168>/Integer Delay' */
  sensorMCUSlugsMKII_DWork.IntegerDelay_DSTATE_m =
    sensorMCUSlugsMKII_P.IntegerDelay_InitialCondition;

  /* End of Start for SubSystem: '<S150>/Zero Out Height' */

  /* Start for Enabled SubSystem: '<S150>/Enabled Subsystem' */
  /* VirtualOutportStart for Outport: '<S164>/Out1' */
  sensorMCUSlugsMKII_B.In1 = sensorMCUSlugsMKII_P.Out1_Y0;

  /* End of Start for SubSystem: '<S150>/Enabled Subsystem' */

  /* S-Function "dsPIC_PWM_IC" initialization Block: <S1>/Input Capture */
  /* Initialise Input Capture */
  /* config capture */
  OpenCapture2(IC_IDLE_CON & IC_TIMER2_SRC & IC_INT_1CAPTURE &
               IC_EVERY_RISE_EDGE);
  ConfigIntCapture2(IC_INT_ON & IC_INT_PRIOR_6);

  /* config capture */
  OpenCapture3(IC_IDLE_CON & IC_TIMER2_SRC & IC_INT_1CAPTURE &
               IC_EVERY_RISE_EDGE);
  ConfigIntCapture3(IC_INT_ON & IC_INT_PRIOR_6);

  /* config capture */
  OpenCapture4(IC_IDLE_CON & IC_TIMER2_SRC & IC_INT_1CAPTURE &
               IC_EVERY_RISE_EDGE);
  ConfigIntCapture4(IC_INT_ON & IC_INT_PRIOR_6);

  /* config capture */
  OpenCapture5(IC_IDLE_CON & IC_TIMER2_SRC & IC_INT_1CAPTURE &
               IC_EVERY_RISE_EDGE);
  ConfigIntCapture5(IC_INT_ON & IC_INT_PRIOR_6);

  /* config capture */
  OpenCapture8(IC_IDLE_CON & IC_TIMER2_SRC & IC_INT_1CAPTURE &
               IC_EVERY_RISE_EDGE);
  ConfigIntCapture8(IC_INT_ON & IC_INT_PRIOR_6);

  /* Start for S-Function (dsPIC_C_function_Call): '<Root>/Initialize Sensor MCU [interProcCommMaster.c]' */
  sensorMCUInit();

  {
    int16_T i;

    /* InitializeConditions for DiscreteIntegrator: '<S12>/Discrete-Time Integrator1' */
    sensorMCUSlugsMKII_DWork.DiscreteTimeIntegrator1_DSTATE[0] =
      sensorMCUSlugsMKII_P.DiscreteTimeIntegrator1_IC[0];
    sensorMCUSlugsMKII_DWork.DiscreteTimeIntegrator1_DSTATE[1] =
      sensorMCUSlugsMKII_P.DiscreteTimeIntegrator1_IC[1];
    sensorMCUSlugsMKII_DWork.DiscreteTimeIntegrator1_DSTATE[2] =
      sensorMCUSlugsMKII_P.DiscreteTimeIntegrator1_IC[2];
    sensorMCUSlugsMKII_DWork.DiscreteTimeIntegrator1_DSTATE[3] =
      sensorMCUSlugsMKII_P.DiscreteTimeIntegrator1_IC[3];

    /* InitializeConditions for Delay: '<S12>/Integer Delay' */
    sensorMCUSlugsMKII_DWork.IntegerDelay_DSTATE[0] =
      sensorMCUSlugsMKII_P.IntegerDelay_InitialCondition_c;
    sensorMCUSlugsMKII_DWork.IntegerDelay_DSTATE[1] =
      sensorMCUSlugsMKII_P.IntegerDelay_InitialCondition_c;
    sensorMCUSlugsMKII_DWork.IntegerDelay_DSTATE[2] =
      sensorMCUSlugsMKII_P.IntegerDelay_InitialCondition_c;

    /* InitializeConditions for UnitDelay: '<S78>/UD' */
    sensorMCUSlugsMKII_DWork.UD_DSTATE = sensorMCUSlugsMKII_P.UD_X0;

    /* InitializeConditions for UnitDelay: '<S79>/UD' */
    sensorMCUSlugsMKII_DWork.UD_DSTATE_e = sensorMCUSlugsMKII_P.UD_X0_g;

    /* InitializeConditions for UnitDelay: '<S80>/UD' */
    sensorMCUSlugsMKII_DWork.UD_DSTATE_k = sensorMCUSlugsMKII_P.UD_X0_f;

    /* InitializeConditions for MATLAB Function: '<S87>/Embedded MATLAB Function3' */
    sensorMCUSlugsMKII_DWork.lastGps_h_not_empty = FALSE;

    /* InitializeConditions for MATLAB Function: '<S150>/Enables//Disables the Computation of  initial Baro Bias' */
    sensorMCUSlugsMKII_DWork.aveCount = 1.0;
    sensorMCUSlugsMKII_DWork.tIni = 1.0;

    /* InitializeConditions for MATLAB Function: '<S159>/Embedded MATLAB Function' */
    s_EmbeddedMATLABFunction_p_Init
      (&sensorMCUSlugsMKII_DWork.sf_EmbeddedMATLABFunction_b);

    /* InitializeConditions for MATLAB Function: '<S162>/Embedded MATLAB Function' */
    s_EmbeddedMATLABFunction_p_Init
      (&sensorMCUSlugsMKII_DWork.sf_EmbeddedMATLABFunction_e);

    /* InitializeConditions for MATLAB Function: '<S98>/Embedded MATLAB Function' */
    sensorMCUSlugsMKII_DWork.a_not_empty = FALSE;

    /* InitializeConditions for UnitDelay: '<S91>/Unit Delay' */
    sensorMCUSlugsMKII_DWork.UnitDelay_DSTATE =
      sensorMCUSlugsMKII_P.UnitDelay_X0;

    /* InitializeConditions for Delay: '<S23>/Integer Delay1' */
    for (i = 0; i < 15; i++) {
      sensorMCUSlugsMKII_DWork.IntegerDelay1_DSTATE[i] =
        sensorMCUSlugsMKII_P.IntegerDelay1_InitialCondition;
    }

    /* End of InitializeConditions for Delay: '<S23>/Integer Delay1' */

    /* InitializeConditions for MATLAB Function: '<S125>/Embedded MATLAB Function' */
    s_EmbeddedMATLABFunction_p_Init
      (&sensorMCUSlugsMKII_DWork.sf_EmbeddedMATLABFunction_p);

    /* InitializeConditions for MATLAB Function: '<S125>/Embedded MATLAB Function1' */
    s_EmbeddedMATLABFunction_p_Init
      (&sensorMCUSlugsMKII_DWork.sf_EmbeddedMATLABFunction1);

    /* InitializeConditions for MATLAB Function: '<S125>/Embedded MATLAB Function2' */
    s_EmbeddedMATLABFunction_p_Init
      (&sensorMCUSlugsMKII_DWork.sf_EmbeddedMATLABFunction2);

    /* InitializeConditions for Delay: '<S93>/Integer Delay1' */
    for (i = 0; i < 15; i++) {
      sensorMCUSlugsMKII_DWork.IntegerDelay1_DSTATE_b[i] =
        sensorMCUSlugsMKII_P.IntegerDelay1_InitialConditio_b;
    }

    /* End of InitializeConditions for Delay: '<S93>/Integer Delay1' */

    /* InitializeConditions for RateLimiter: '<S86>/Rate Limiter' */
    sensorMCUSlugsMKII_DWork.PrevY[0] = sensorMCUSlugsMKII_P.RateLimiter_IC;
    sensorMCUSlugsMKII_DWork.PrevY[1] = sensorMCUSlugsMKII_P.RateLimiter_IC;
    sensorMCUSlugsMKII_DWork.PrevY[2] = sensorMCUSlugsMKII_P.RateLimiter_IC;

    /* InitializeConditions for RateLimiter: '<S12>/Bias Rate Limiter' */
    sensorMCUSlugsMKII_DWork.PrevY_e[0] =
      sensorMCUSlugsMKII_P.BiasRateLimiter_IC;
    sensorMCUSlugsMKII_DWork.PrevY_e[1] =
      sensorMCUSlugsMKII_P.BiasRateLimiter_IC;
    sensorMCUSlugsMKII_DWork.PrevY_e[2] =
      sensorMCUSlugsMKII_P.BiasRateLimiter_IC;

    /* InitializeConditions for UnitDelay: '<S89>/Unit Delay' */
    sensorMCUSlugsMKII_DWork.UnitDelay_DSTATE_p =
      sensorMCUSlugsMKII_P.UnitDelay_X0_k;

    /* InitializeConditions for UnitDelay: '<S90>/Unit Delay' */
    sensorMCUSlugsMKII_DWork.UnitDelay_DSTATE_f =
      sensorMCUSlugsMKII_P.UnitDelay_X0_l;

    /* InitializeConditions for MATLAB Function: '<S126>/Embedded MATLAB Function' */
    s_EmbeddedMATLABFunction_p_Init
      (&sensorMCUSlugsMKII_DWork.sf_EmbeddedMATLABFunction_d);

    /* InitializeConditions for MATLAB Function: '<S126>/Embedded MATLAB Function1' */
    s_EmbeddedMATLABFunction_p_Init
      (&sensorMCUSlugsMKII_DWork.sf_EmbeddedMATLABFunction1_o);

    /* InitializeConditions for MATLAB Function: '<S126>/Embedded MATLAB Function2' */
    s_EmbeddedMATLABFunction_p_Init
      (&sensorMCUSlugsMKII_DWork.sf_EmbeddedMATLABFunction2_b);

    /* InitializeConditions for MATLAB Function: '<S124>/Embedded MATLAB Function' */
    s_EmbeddedMATLABFunction_p_Init
      (&sensorMCUSlugsMKII_DWork.sf_EmbeddedMATLABFunction_c);

    /* InitializeConditions for MATLAB Function: '<S124>/Embedded MATLAB Function1' */
    s_EmbeddedMATLABFunction_p_Init
      (&sensorMCUSlugsMKII_DWork.sf_EmbeddedMATLABFunction1_f);

    /* InitializeConditions for MATLAB Function: '<S124>/Embedded MATLAB Function2' */
    s_EmbeddedMATLABFunction_p_Init
      (&sensorMCUSlugsMKII_DWork.sf_EmbeddedMATLABFunction2_l);

    /* InitializeConditions for UnitDelay: '<S112>/Output' */
    sensorMCUSlugsMKII_DWork.Output_DSTATE = sensorMCUSlugsMKII_P.Output_X0;

    /* InitializeConditions for MATLAB Function: '<S161>/Embedded MATLAB Function' */
    s_EmbeddedMATLABFunction_p_Init
      (&sensorMCUSlugsMKII_DWork.sf_EmbeddedMATLABFunction_f);

    /* InitializeConditions for MATLAB Function: '<S160>/Embedded MATLAB Function' */
    s_EmbeddedMATLABFunction_p_Init
      (&sensorMCUSlugsMKII_DWork.sf_EmbeddedMATLABFunction_gc);
  }
}

/* File trailer for Real-Time Workshop generated code.
 *
 * [EOF]
 */
