/*
 * File: controlMCUSlugsMKIINewNav.c
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

#include "controlMCUSlugsMKIINewNav.h"
#include "controlMCUSlugsMKIINewNav_private.h"

/* Block signals (auto storage) */
BlockIO_controlMCUSlugsMKIINewN controlMCUSlugsMKIINewNav_B;

/* Block states (auto storage) */
D_Work_controlMCUSlugsMKIINewNa controlMCUSlugsMKIINewNav_DWork;

/* Real-time model */
RT_MODEL_controlMCUSlugsMKIINew controlMCUSlugsMKIINewNav_M_;
RT_MODEL_controlMCUSlugsMKIINew *const controlMCUSlugsMKIINewNav_M =
  &controlMCUSlugsMKIINewNav_M_;
static void rate_scheduler(void);

/*
 *   This function updates active task flag for each subrate.
 * The function is called at model base rate, hence the
 * generated code self-manages all its subrates.
 */
static void rate_scheduler(void)
{
  /* Compute which subrates run during the next base time step.  Subrates
   * are an integer multiple of the base rate counter.  Therefore, the subtask
   * counter is reset when it reaches its limit (zero means run).
   */
  (controlMCUSlugsMKIINewNav_M->Timing.TaskCounters.TID[1])++;
  if ((controlMCUSlugsMKIINewNav_M->Timing.TaskCounters.TID[1]) > 1) {/* Sample time: [0.02s, 0.0s] */
    controlMCUSlugsMKIINewNav_M->Timing.TaskCounters.TID[1] = 0;
  }

  (controlMCUSlugsMKIINewNav_M->Timing.TaskCounters.TID[2])++;
  if ((controlMCUSlugsMKIINewNav_M->Timing.TaskCounters.TID[2]) > 99) {/* Sample time: [1.0s, 0.0s] */
    controlMCUSlugsMKIINewNav_M->Timing.TaskCounters.TID[2] = 0;
  }
}

/*
 * Initial conditions for atomic system:
 *    '<S51>/Embedded MATLAB Function'
 *    '<S89>/Embedded MATLAB Function'
 *    '<S113>/Embedded MATLAB Function'
 *    '<S425>/Embedded MATLAB Function'
 *    '<S426>/Embedded MATLAB Function'
 *    '<S470>/Embedded MATLAB Function'
 *    '<S466>/Embedded MATLAB Function'
 *    '<S483>/Embedded MATLAB Function'
 *    '<S484>/Embedded MATLAB Function'
 *    '<S485>/Embedded MATLAB Function'
 */
void con_EmbeddedMATLABFunction_Init(rtDW_EmbeddedMATLABFunction_con *localDW)
{
  localDW->a_not_empty = FALSE;
}

/*
 * Output and update for atomic system:
 *    '<S51>/Embedded MATLAB Function'
 *    '<S89>/Embedded MATLAB Function'
 *    '<S113>/Embedded MATLAB Function'
 *    '<S425>/Embedded MATLAB Function'
 *    '<S426>/Embedded MATLAB Function'
 *    '<S470>/Embedded MATLAB Function'
 *    '<S466>/Embedded MATLAB Function'
 *    '<S483>/Embedded MATLAB Function'
 *    '<S484>/Embedded MATLAB Function'
 *    '<S485>/Embedded MATLAB Function'
 */
void controlM_EmbeddedMATLABFunction(real32_T rtu_u, real_T rtu_T, real_T rtu_f,
  rtB_EmbeddedMATLABFunction_cont *localB, rtDW_EmbeddedMATLABFunction_con
  *localDW)
{
  real_T omega;

  /* MATLAB Function 'Control Blocks/Lateral Channel/0.016 sec  tau/Embedded MATLAB Function': '<S59>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  if (!localDW->a_not_empty) {
    /* '<S59>:1:9' */
    /* tf = [T*omega/(2+T*omega) T*omega/(2+T*omega)],[1 (T*omega-2)/(T*omega+2)] */
    /* tf = [a a],[1 -b] */
    /* '<S59>:1:12' */
    omega = 6.2831853071795862 * rtu_f;

    /* '<S59>:1:13' */
    localDW->a = rtu_T * omega / (rtu_T * omega + 2.0);
    localDW->a_not_empty = TRUE;

    /* '<S59>:1:14' */
    localDW->b = -(rtu_T * omega - 2.0) / (rtu_T * omega + 2.0);

    /* '<S59>:1:15' */
    localDW->y_km1 = rtu_u;

    /* '<S59>:1:16' */
    localDW->u_km1 = rtu_u;
  }

  /* '<S59>:1:19' */
  localB->y = (rtu_u + localDW->u_km1) * (real32_T)localDW->a + (real32_T)
    localDW->b * localDW->y_km1;

  /* '<S59>:1:20' */
  localDW->y_km1 = localB->y;

  /* '<S59>:1:21' */
  localDW->u_km1 = rtu_u;
}

/*
 * Output and update for atomic system:
 *    '<S16>/myMux Fun1'
 *    '<S17>/myMux Fun1'
 *    '<S128>/myMux Fun2'
 *    '<S477>/myMux Fun5'
 */
void controlMCUSlugsMKIINe_myMuxFun1(real32_T rtu_u1, real32_T rtu_u2, real32_T
  rtu_u3, rtB_myMuxFun1_controlMCUSlugsMK *localB)
{
  /* MATLAB Function 'Inner Loop/ Navigation/Lateral Channel Encaps/myMux Fun1': '<S49>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S49>:1:5' */
  localB->y[0] = rtu_u1;
  localB->y[1] = rtu_u2;
  localB->y[2] = rtu_u3;
}

/*
 * Output and update for atomic system:
 *    '<S122>/negprotect'
 *    '<S401>/negprotect'
 *    '<S259>/negprotect'
 *    '<S283>/negprotect'
 *    '<S290>/negprotect'
 *    '<S338>/negprotect'
 *    '<S345>/negprotect'
 *    '<S385>/negprotect'
 *    '<S244>/negprotect'
 *    '<S173>/negprotect'
 *    ...
 */
void controlMCUSlugsMKIIN_negprotect(real32_T rtu_val,
  rtB_negprotect_controlMCUSlugsM *localB)
{
  /* MATLAB Function 'dsPIC_SQRT/negprotect': '<S124>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  if (rtu_val >= 0.001F) {
    /* '<S124>:1:5' */
    /* '<S124>:1:6' */
    localB->zpVal = rtu_val;
  } else {
    /* '<S124>:1:8' */
    localB->zpVal = 0.001F;
  }
}

/*
 * Output and update for atomic system:
 *    '<S400>/Embedded MATLAB Function'
 *    '<S282>/Embedded MATLAB Function'
 *    '<S289>/Embedded MATLAB Function'
 *    '<S337>/Embedded MATLAB Function'
 *    '<S344>/Embedded MATLAB Function'
 *    '<S384>/Embedded MATLAB Function'
 *    '<S243>/Embedded MATLAB Function'
 *    '<S172>/Embedded MATLAB Function'
 *    '<S185>/Embedded MATLAB Function'
 *    '<S198>/Embedded MATLAB Function'
 *    ...
 */
void contro_EmbeddedMATLABFunction_o(const real32_T rtu_x[3],
  rtB_EmbeddedMATLABFunction_co_h *localB)
{
  /* MATLAB Function 'dsPIC Dot Product/Embedded MATLAB Function': '<S402>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S402>:1:5' */
  localB->xDoty = (rtu_x[0] * rtu_x[0] + rtu_x[1] * rtu_x[1]) + rtu_x[2] *
    rtu_x[2];
}

/*
 * Output and update for atomic system:
 *    '<S136>/Zero out Z1'
 *    '<S251>/Zero out Z1'
 *    '<S251>/Zero out Z2'
 *    '<S251>/Zero out Z3'
 *    '<S135>/Zero out Z2'
 *    '<S146>/Zero out Z2'
 *    '<S225>/Zero out Z1'
 *    '<S128>/Zero out Z1'
 *    '<S128>/Zero out Z2'
 */
void controlMCUSlugsMKIINe_ZerooutZ1(const real32_T rtu_Pin[3],
  rtB_ZerooutZ1_controlMCUSlugsMK *localB)
{
  /* MATLAB Function 'Navigation/Navigation/RTB/Follow Mobile Navigation/Zero out Z1': '<S393>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S393>:1:5' */
  localB->P[0] = rtu_Pin[0];
  localB->P[1] = rtu_Pin[1];
  localB->P[2] = 0.0F;
}

/*
 * Output and update for atomic system:
 *    '<S264>/Embedded MATLAB Function'
 *    '<S257>/Embedded MATLAB Function'
 *    '<S169>/Embedded MATLAB Function'
 *    '<S182>/Embedded MATLAB Function'
 */
void contro_EmbeddedMATLABFunction_d(const real32_T rtu_x[3], const real32_T
  rtu_y[3], rtB_EmbeddedMATLABFunction_co_k *localB)
{
  /* MATLAB Function 'dsPIC Dot Product/Embedded MATLAB Function': '<S265>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S265>:1:5' */
  localB->xDoty = (rtu_x[0] * rtu_y[0] + rtu_x[1] * rtu_y[1]) + rtu_x[2] *
    rtu_y[2];
}

/*
 * Output and update for atomic system:
 *    '<S272>/Select N  Terms'
 *    '<S330>/Select N  Terms'
 */
void controlMCUSlugsMKI_SelectNTerms(const real32_T rtu_T[3],
  rtB_SelectNTerms_controlMCUSlug *localB)
{
  /* MATLAB Function 'Navigation/Compute Frenet/Select N  Terms': '<S280>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S280>:1:5' */
  localB->N[0] = -rtu_T[1];
  localB->N[1] = rtu_T[0];
  localB->N[2] = 0.0F;
}

/*
 * Output and update for atomic system:
 *    '<S160>/negprotect3'
 *    '<S161>/negprotect3'
 *    '<S148>/negprotect1'
 *    '<S148>/negprotect2'
 *    '<S217>/negprotect'
 *    '<S218>/negprotect'
 */
void controlMCUSlugsMKII_negprotect3(real32_T rtu_val,
  rtB_negprotect3_controlMCUSlugs *localB)
{
  /* MATLAB Function 'Compute Angle Between Vectors/negprotect3': '<S171>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  if (rtu_val >= 0.0001F) {
    /* '<S171>:1:5' */
    /* '<S171>:1:6' */
    localB->zpVal = rtu_val;
  } else {
    /* '<S171>:1:8' */
    localB->zpVal = 0.0001F;
  }
}

/*
 * Initial conditions for atomic system:
 *    '<S415>/Buffer IC Channel'
 *    '<S415>/Buffer IC Channel1'
 *    '<S415>/Buffer IC Channel2'
 *    '<S415>/Buffer IC Channel3'
 *    '<S420>/Buffer Failsafe Channel'
 */
void controlMCU_BufferICChannel_Init(rtDW_BufferICChannel_controlMCU *localDW)
{
  int16_T i;
  for (i = 0; i < 7; i++) {
    localDW->oldValues[i] = 0U;
  }
}

/*
 * Output and update for atomic system:
 *    '<S415>/Buffer IC Channel'
 *    '<S415>/Buffer IC Channel1'
 *    '<S415>/Buffer IC Channel2'
 *    '<S415>/Buffer IC Channel3'
 *    '<S420>/Buffer Failsafe Channel'
 */
void controlMCUSlugs_BufferICChannel(uint16_T rtu_latest,
  rtB_BufferICChannel_controlMCUS *localB, rtDW_BufferICChannel_controlMCU
  *localDW)
{
  int16_T i;

  /* MATLAB Function 'Trim Vals/Bufffer IC/Buffer IC Channel': '<S421>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S421>:1:11' */
  for (i = 0; i < 7; i++) {
    localB->history[i] = 0U;
  }

  /* '<S421>:1:13' */
  for (i = 0; i < 6; i++) {
    /* '<S421>:1:13' */
    /* '<S421>:1:14' */
    localDW->oldValues[(int32_T)(6 - i)] = localDW->oldValues[5 - i];

    /* '<S421>:1:15' */
    localB->history[(int32_T)(6 - i)] = localDW->oldValues[5 - i];

    /* '<S421>:1:13' */
  }

  /* '<S421>:1:18' */
  localDW->oldValues[0] = rtu_latest;

  /* '<S421>:1:19' */
  localB->history[0] = rtu_latest;
}

/*
 * Output and update for atomic system:
 *    '<S447>/myMux Fun1'
 *    '<S446>/myMux Fun1'
 */
void controlMCUSlugsMKII_myMuxFun1_e(uint16_T rtu_u1, uint16_T rtu_u2, uint16_T
  rtu_u3, uint16_T rtu_u4, uint16_T rty_y[4])
{
  /* MATLAB Function 'Update PWM Commands and Send Telemetry/Generate PWM Signals Based on the Control Type/If  Control Type Is Wp, ISR, LP or  Mid Lvl Cmds/myMux Fun1': '<S457>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S457>:1:5' */
  rty_y[0] = rtu_u1;
  rty_y[1] = rtu_u2;
  rty_y[2] = rtu_u3;
  rty_y[3] = rtu_u4;
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
void controlMCUSlugsMKIINewNav_step(int_T tid)
{
  /* local block i/o variables */
  real_T rtb_DataTypeConversion;
  real_T rtb_DataTypeConversion_p;
  real_T rtb_DataTypeConversion_i;
  real_T rtb_DataTypeConversion_m;
  real32_T rtb_DataTypeConversion2;
  real32_T rtb_DataTypeConversion3;
  real32_T rtb_Subtract2[3];
  real32_T rtb_Puav[3];
  real32_T rtb_Product1_l;
  real32_T rtb_Sum1_a[3];
  real32_T rtb_Subtract1_o;
  real32_T rtb_h;
  real32_T rtb_Subtract6[3];
  real32_T rtb_Product9;
  real32_T rtb_Subtract5[3];
  real32_T rtb_Product9_j;
  real32_T rtb_Gain1[3];
  real32_T rtb_PaHPPned[3];
  real32_T rtb_Subtract_c[3];
  real32_T rtb_Add_cj;
  real32_T rtb_Subtract_d[3];
  real32_T rtb_Add_e[3];
  real32_T rtb_Divide_d[3];
  real32_T rtb_Add_gw[3];
  real32_T rtb_Divide_d5[3];
  real32_T rtb_Divide_h[3];
  real32_T rtb_Subtract_o[3];
  real32_T rtb_Divide2_m;
  real32_T rtb_ScheduleLPF;
  real32_T rtb_DifferenceInputs2;
  real32_T rtb_Theta_cLimit;
  real32_T rtb_PsiDotLimit;
  real32_T rtb_BankLimitCommand_i;
  real32_T rtb_u020;
  uint16_T rtb_DataTypeConversion_l;
  uint16_T rtb_Switch_it;
  uint16_T rtb_Switch1_h;
  uint16_T rtb_Switch2;
  uint16_T rtb_Switch3;
  uint16_T rtb_DataTypeConversion_pv;
  uint16_T rtb_DataTypeConversion_lc;
  uint16_T rtb_DataTypeConversion_i3;
  uint16_T rtb_DataTypeConversion_b;
  uint16_T rtb_y_c[5];
  boolean_T rtb_IC1;
  real32_T rtb_VectorConcatenate_i[9];
  real32_T rtb_Product2_ep[3];
  uint16_T rtb_u2deg;
  uint8_T rtb_DataTypeConversion2_n;
  uint8_T rtb_IC2;
  real32_T rtb_BankLimitCommand;
  real32_T rtb_Add4_l;
  real32_T rtb_OnOff;
  real32_T rtb_ElevatorLimit;
  real32_T rtb_ThrottleLimit;
  real_T rtb_Add_k;
  real32_T rtb_cosphi;
  real32_T rtb_Ze;
  real32_T rtb_Deg2R1;
  real32_T rtb_RhhcosphisinlambYe;
  real32_T rtb_RhhcosphicoslambXe;
  real32_T rtb_Product6[3];
  real32_T rtb_Product5[3];
  real32_T rtb_MathFunction[9];
  real32_T rtb_Product_ib[9];
  real32_T Product;
  real32_T Switch;
  real32_T Switch3_m;
  real32_T Switch3_lh;
  real32_T Switch3_a;
  real32_T Switch3_l;
  real32_T Switch3_ae;
  real32_T tmp[3];
  int16_T i;
  real32_T tmp_0[3];
  int16_T i_0;
  real32_T rtb_Deg2R_m_idx;
  real32_T rtb_Deg2R_m_idx_0;
  real32_T rtb_Product3_p4_idx;
  real32_T rtb_Merge_idx;
  real_T tmp_1;

  /* S-Function "dsPIC_C_function_Call" Block: <S9>/Navigation Mode [navSupport.c] */
  controlMCUSlugsMKIINewNav_B.NavigationModenavSupportc = getNavMode();

  /* S-Function "dsPIC_PWM_IC" Block: <S416>/Input Capture */
  controlMCUSlugsMKIINewNav_B.InputCapture_o1 = ic1up;
  controlMCUSlugsMKIINewNav_B.InputCapture_o2 = ic2up;
  controlMCUSlugsMKIINewNav_B.InputCapture_o3 = ic3up;
  controlMCUSlugsMKIINewNav_B.InputCapture_o4 = ic4up;
  controlMCUSlugsMKIINewNav_B.InputCapture_o5 = ic5up;
  controlMCUSlugsMKIINewNav_B.InputCapture_o6 = ic7up;
  controlMCUSlugsMKIINewNav_B.InputCapture_o7 = ic8up;

  /* MATLAB Function: '<S416>/myMux Fun5' */
  /* MATLAB Function 'Trim Vals/Control Surface Input/myMux Fun5': '<S428>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S428>:1:5' */
  rtb_y_c[0] = controlMCUSlugsMKIINewNav_B.InputCapture_o2;
  rtb_y_c[1] = controlMCUSlugsMKIINewNav_B.InputCapture_o3;
  rtb_y_c[2] = controlMCUSlugsMKIINewNav_B.InputCapture_o4;
  rtb_y_c[3] = controlMCUSlugsMKIINewNav_B.InputCapture_o5;
  rtb_y_c[4] = controlMCUSlugsMKIINewNav_B.InputCapture_o7;

  /* DataTypeConversion: '<S417>/Data Type Conversion' incorporates:
   *  Gain: '<S417>/Convert to  Microseconds'
   */
  rtb_DataTypeConversion_l = (uint16_T)(52429UL * (uint32_T)rtb_y_c[4] >> 18);

  /* MATLAB Function: '<S420>/Buffer Failsafe Channel' */
  controlMCUSlugs_BufferICChannel(rtb_DataTypeConversion_l,
    &controlMCUSlugsMKIINewNav_B.sf_BufferFailsafeChannel,
    &controlMCUSlugsMKIINewNav_DWork.sf_BufferFailsafeChannel);

  /* S-Function "dsPIC_C_function_Call" Block: <S420>/Choose the Median [navSupport.c] */
  controlMCUSlugsMKIINewNav_B.ChoosetheMediannavSupportc = meanFilter5
    (controlMCUSlugsMKIINewNav_B.sf_BufferFailsafeChannel.history);

  /* S-Function "dsPIC_C_function_Call" Block: <S7>/Manual or Auto? [navSupport.c] */
  controlMCUSlugsMKIINewNav_B.ManualorAutonavSupportc = isApManual
    (controlMCUSlugsMKIINewNav_B.ChoosetheMediannavSupportc);

  /* S-Function "UART_RX" Block: <S13>/Calculus Time Step */
  controlMCUSlugsMKIINewNav_B.CalculusTimeStep_o1 = CalculusTimeStep;
  controlMCUSlugsMKIINewNav_B.CalculusTimeStep_o2 = PR1;

  /* DataTypeConversion: '<S13>/Data Type Conversion12' incorporates:
   *  DataTypeConversion: '<S13>/Data Type Conversion3'
   *  DataTypeConversion: '<S13>/Data Type Conversion4'
   *  Gain: '<S13>/Gain'
   *  Product: '<S13>/Divide'
   *  Rounding: '<S13>/Rounding Function'
   */
  tmp_1 = floor((real_T)controlMCUSlugsMKIINewNav_B.CalculusTimeStep_o1 /
                (real_T)controlMCUSlugsMKIINewNav_B.CalculusTimeStep_o2 * 100.0);
  if (rtIsNaN(tmp_1) || rtIsInf(tmp_1)) {
    tmp_1 = 0.0;
  } else {
    tmp_1 = fmod(tmp_1, 256.0);
  }

  controlMCUSlugsMKIINewNav_B.DataTypeConversion12 = (uint8_T)(tmp_1 < 0.0 ?
    (uint8_T)(int16_T)(int8_T)-(int8_T)(uint8_T)-tmp_1 : (uint8_T)tmp_1);

  /* End of DataTypeConversion: '<S13>/Data Type Conversion12' */
  /* S-Function "dsPIC_C_function_Call" Block: <S2>/Data from GS [mavlinkCommsControlMcu.c] */
  gsRead(&controlMCUSlugsMKIINewNav_B.DatafromGSmavlinkCommsControlMc[0]);

  /* S-Function "dsPIC_C_function_Call" Block: <S2>/Data from IPC [interProcCommSlave.c] */
  readIpc(&controlMCUSlugsMKIINewNav_B.DatafromIPCinterProcCommSlavec[0]);

  /* S-Function "dsPIC_C_function_Call" Block: <S8>/Parser//Decoder [mavlinkCommsControlMcu.c] */
  protDecodeMavlink(controlMCUSlugsMKIINewNav_B.DatafromIPCinterProcCommSlavec);

  /* S-Function "dsPIC_C_function_Call" Block: <S8>/GS Messages  Parser//Decoder [mavlinkCommsControlMcu.c] */
  protDecodeMavlink(controlMCUSlugsMKIINewNav_B.DatafromGSmavlinkCommsControlMc);

  /* S-Function "dsPIC_C_function_Call" Block: <S8>/update Load of  Control MCU [updateControlMcu.c] */
  updateLoad(controlMCUSlugsMKIINewNav_B.DataTypeConversion12);

  /* S-Function "dsPIC_C_function_Call" Block: <Root>/Get MidLevel Commands [navSupport.c] */
  getMidLevelCommands
    (&controlMCUSlugsMKIINewNav_B.GetMidLevelCommandsnavSupportc[0]);

  /* S-Function "dsPIC_C_function_Call" Block: <Root>/Get Dynamic P [navSupport.c] */
  controlMCUSlugsMKIINewNav_B.GetDynamicPnavSupportc = getDynamic();

  /* S-Function "dsPIC_C_function_Call" Block: <S4>/Get Range of Values [navSupport.c] */
  getRangeOfParams(((uint8_T)0U),((uint8_T)2U),
                   &controlMCUSlugsMKIINewNav_B.GetRangeofValuesnavSupportc[0]);

  /* MATLAB Function: '<S415>/Buffer IC Channel' */
  controlMCUSlugs_BufferICChannel(rtb_y_c[0],
    &controlMCUSlugsMKIINewNav_B.sf_BufferICChannel,
    &controlMCUSlugsMKIINewNav_DWork.sf_BufferICChannel);

  /* MATLAB Function: '<S415>/Buffer IC Channel1' */
  controlMCUSlugs_BufferICChannel(rtb_y_c[1],
    &controlMCUSlugsMKIINewNav_B.sf_BufferICChannel1,
    &controlMCUSlugsMKIINewNav_DWork.sf_BufferICChannel1);

  /* MATLAB Function: '<S415>/Buffer IC Channel2' */
  controlMCUSlugs_BufferICChannel(rtb_y_c[3],
    &controlMCUSlugsMKIINewNav_B.sf_BufferICChannel2,
    &controlMCUSlugsMKIINewNav_DWork.sf_BufferICChannel2);

  /* MATLAB Function: '<S415>/Buffer IC Channel3' */
  controlMCUSlugs_BufferICChannel(rtb_y_c[2],
    &controlMCUSlugsMKIINewNav_B.sf_BufferICChannel3,
    &controlMCUSlugsMKIINewNav_DWork.sf_BufferICChannel3);

  /* Logic: '<S418>/Logical Operator' */
  rtb_IC1 = !(controlMCUSlugsMKIINewNav_B.ManualorAutonavSupportc != 0);

  /* InitialCondition: '<S418>/IC1' */
  if (controlMCUSlugsMKIINewNav_DWork.IC1_FirstOutputTime) {
    controlMCUSlugsMKIINewNav_DWork.IC1_FirstOutputTime = FALSE;
    rtb_IC1 = FALSE;
  }

  /* End of InitialCondition: '<S418>/IC1' */

  /* DataTypeConversion: '<S431>/Data Type Conversion1' */
  controlMCUSlugsMKIINewNav_B.DataTypeConversion1_o = rtb_IC1;

  /* S-Function "dsPIC_C_function_Call" Block: <S431>/Detect Rising Transition [navSupport.c] */
  controlMCUSlugsMKIINewNav_B.DetectRisingTransitionnavSuppor = justEnabled
    (controlMCUSlugsMKIINewNav_B.DataTypeConversion1_o,((uint8_T)2U));

  /* Outputs for Enabled SubSystem: '<S7>/Grab I.C.' incorporates:
   *  EnablePort: '<S419>/Enable'
   */
  if (controlMCUSlugsMKIINewNav_B.DetectRisingTransitionnavSuppor > 0) {
    /* S-Function "dsPIC_C_function_Call" Block: <S419>/Choose the Median [navSupport.c] */
    controlMCUSlugsMKIINewNav_B.ChoosetheMediannavSupportc_c = meanFilter5
      (controlMCUSlugsMKIINewNav_B.sf_BufferICChannel.history);

    /* Saturate: '<S419>/[0.55 0.68]' */
    rtb_u2deg = controlMCUSlugsMKIINewNav_B.ChoosetheMediannavSupportc_c >=
      8085U ? 8085U : controlMCUSlugsMKIINewNav_B.ChoosetheMediannavSupportc_c <=
      7753U ? 7753U : controlMCUSlugsMKIINewNav_B.ChoosetheMediannavSupportc_c;

    /* DataTypeConversion: '<S432>/Data Type Conversion' incorporates:
     *  Gain: '<S432>/Convert to  Microseconds'
     */
    controlMCUSlugsMKIINewNav_B.DataTypeConversion_i = (uint16_T)(52429UL *
      (uint32_T)rtb_u2deg >> 18);

    /* S-Function "dsPIC_C_function_Call" Block: <S419>/Update dT Trim [updateControlMcu.c] */
    updatePWMTrim(controlMCUSlugsMKIINewNav_B.DataTypeConversion_i,((uint8_T)0U));

    /* DataTypeConversion: '<S436>/Data Type Conversion' incorporates:
     *  Constant: '<S436>/Constant1'
     *  Constant: '<S436>/Constant2'
     *  Product: '<S436>/Divide'
     *  Sum: '<S436>/Add'
     */
    controlMCUSlugsMKIINewNav_B.DataTypeConversion_j = (real32_T)((real_T)
      rtb_u2deg * 0.000392464678178964 + -2.4929356357927808);

    /* S-Function "dsPIC_C_function_Call" Block: <S419>/Choose the Median [navSupport.c]1 */
    controlMCUSlugsMKIINewNav_B.ChoosetheMediannavSupportc1 = meanFilter5
      (controlMCUSlugsMKIINewNav_B.sf_BufferICChannel1.history);

    /* Saturate: '<S419>/[-2  2] deg' */
    rtb_u2deg = controlMCUSlugsMKIINewNav_B.ChoosetheMediannavSupportc1 >= 7693U
      ? 7693U : controlMCUSlugsMKIINewNav_B.ChoosetheMediannavSupportc1 <= 7040U
      ? 7040U : controlMCUSlugsMKIINewNav_B.ChoosetheMediannavSupportc1;

    /* DataTypeConversion: '<S433>/Data Type Conversion' incorporates:
     *  Gain: '<S433>/Convert to  Microseconds'
     */
    controlMCUSlugsMKIINewNav_B.DataTypeConversion_f = (uint16_T)(52429UL *
      (uint32_T)rtb_u2deg >> 18);

    /* S-Function "dsPIC_C_function_Call" Block: <S419>/Update dA Trim [updateControlMcu.c] */
    updatePWMTrim(controlMCUSlugsMKIINewNav_B.DataTypeConversion_f,((uint8_T)1U));

    /* DataTypeConversion: '<S437>/Data Type Conversion' incorporates:
     *  Constant: '<S437>/Constant1'
     *  Constant: '<S437>/Constant2'
     *  Product: '<S437>/Divide'
     *  Sum: '<S437>/Add'
     */
    controlMCUSlugsMKIINewNav_B.DataTypeConversion_je = (real32_T)((real_T)
      rtb_u2deg * -0.00010684205998466119 + 0.787069841887004);

    /* S-Function "dsPIC_C_function_Call" Block: <S419>/Choose the Median [navSupport.c]2 */
    controlMCUSlugsMKIINewNav_B.ChoosetheMediannavSupportc2 = meanFilter5
      (controlMCUSlugsMKIINewNav_B.sf_BufferICChannel2.history);

    /* Saturate: '<S419>/[-2  2] deg ' */
    rtb_u2deg = controlMCUSlugsMKIINewNav_B.ChoosetheMediannavSupportc2 >= 7653U
      ? 7653U : controlMCUSlugsMKIINewNav_B.ChoosetheMediannavSupportc2 <= 7247U
      ? 7247U : controlMCUSlugsMKIINewNav_B.ChoosetheMediannavSupportc2;

    /* DataTypeConversion: '<S434>/Data Type Conversion' incorporates:
     *  Gain: '<S434>/Convert to  Microseconds'
     */
    controlMCUSlugsMKIINewNav_B.DataTypeConversion_n = (uint16_T)(52429UL *
      (uint32_T)rtb_u2deg >> 18);

    /* S-Function "dsPIC_C_function_Call" Block: <S419>/Update dR Trim [updateControlMcu.c] */
    updatePWMTrim(controlMCUSlugsMKIINewNav_B.DataTypeConversion_n,((uint8_T)2U));

    /* DataTypeConversion: '<S438>/Data Type Conversion' incorporates:
     *  Constant: '<S438>/Constant1'
     *  Constant: '<S438>/Constant2'
     *  Product: '<S438>/Divide'
     *  Sum: '<S438>/Add'
     */
    controlMCUSlugsMKIINewNav_B.DataTypeConversion_k = (real32_T)((real_T)
      rtb_u2deg * -0.00017229941427366325 + 1.2836306363387913);

    /* S-Function "dsPIC_C_function_Call" Block: <S419>/Choose the Median [navSupport.c]3 */
    controlMCUSlugsMKIINewNav_B.ChoosetheMediannavSupportc3 = meanFilter5
      (controlMCUSlugsMKIINewNav_B.sf_BufferICChannel3.history);

    /* Saturate: '<S419>/[-2  2] deg  ' */
    rtb_u2deg = controlMCUSlugsMKIINewNav_B.ChoosetheMediannavSupportc3 >= 7955U
      ? 7955U : controlMCUSlugsMKIINewNav_B.ChoosetheMediannavSupportc3 <= 7112U
      ? 7112U : controlMCUSlugsMKIINewNav_B.ChoosetheMediannavSupportc3;

    /* DataTypeConversion: '<S435>/Data Type Conversion' incorporates:
     *  Gain: '<S435>/Convert to  Microseconds'
     */
    controlMCUSlugsMKIINewNav_B.DataTypeConversion_iq = (uint16_T)(52429UL *
      (uint32_T)rtb_u2deg >> 18);

    /* S-Function "dsPIC_C_function_Call" Block: <S419>/Update dE Trim [updateControlMcu.c] */
    updatePWMTrim(controlMCUSlugsMKIINewNav_B.DataTypeConversion_iq,((uint8_T)3U));

    /* DataTypeConversion: '<S439>/Data Type Conversion' incorporates:
     *  Constant: '<S439>/Constant1'
     *  Constant: '<S439>/Constant2'
     *  Product: '<S439>/Divide'
     *  Sum: '<S439>/Add'
     */
    controlMCUSlugsMKIINewNav_B.DataTypeConversion_h = (real32_T)((real_T)
      rtb_u2deg * 8.2860394028366822E-5 + -0.62421496834703016);
  }

  /* End of Outputs for SubSystem: '<S7>/Grab I.C.' */
  /* S-Function "dsPIC_C_function_Call" Block: <Root>/is it in Mid Level, Pt or SPT? [navSupport.c]1 */
  controlMCUSlugsMKIINewNav_B.isitinMidLevelPtorSPTnavSupport = isPassthrough();

  /* S-Function "dsPIC_C_function_Call" Block: <Root>/Get XYZ [navSupport.c] */
  getXYZ(&controlMCUSlugsMKIINewNav_B.GetXYZnavSupportc[0]);

  /* S-Function "dsPIC_C_function_Call" Block: <Root>/Get Max WP [navSupport.c] */
  controlMCUSlugsMKIINewNav_B.GetMaxWPnavSupportc = getMaxWp();

  /* S-Function "dsPIC_C_function_Call" Block: <Root>/Get Vned [navSupport.c] */
  getVned(&controlMCUSlugsMKIINewNav_B.GetVnednavSupportc[0]);

  /* S-Function "dsPIC_C_function_Call" Block: <Root>/Get Range of Values [navSupport.c] */
  getRangeOfParams(((uint8_T)19U),((uint8_T)21U),
                   &controlMCUSlugsMKIINewNav_B.GetRangeofValuesnavSupportc_k[0]);

  /* S-Function "dsPIC_C_function_Call" Block: <Root>/wp Fly? [navSupport.c] */
  controlMCUSlugsMKIINewNav_B.wpFlynavSupportc = isWpFly();

  /* S-Function "dsPIC_C_function_Call" Block: <Root>/Get RTB Order [navSupport.c] */
  getRTB(&controlMCUSlugsMKIINewNav_B.GetRTBOrdernavSupportc[0]);

  /* S-Function "dsPIC_C_function_Call" Block: <Root>/Get Mobile Location [navSupport.c]  */
  getMobileLocation(&controlMCUSlugsMKIINewNav_B.GetMobileLocationnavSupportc[0]);

  /* S-Function "dsPIC_C_function_Call" Block: <Root>/Get Range of Values [navSupport.c]    */
  getRangeOfParams(((uint8_T)26U),((uint8_T)28U),
                   &controlMCUSlugsMKIINewNav_B.GetRangeofValuesnavSupportc_c[0]);

  /* Outputs for Atomic SubSystem: '<S3>/Navigation Encaps' */
  /* S-Function "dsPIC_C_function_Call" Block: <S18>/Get ISR Location [navSupport.c] */
  getISRLocation(&controlMCUSlugsMKIINewNav_B.GetISRLocationnavSupportc[0]);

  /* InitialCondition: '<S137>/IC1' incorporates:
   *  Logic: '<S128>/Logical Operator'
   *  Logic: '<S128>/Logical Operator1'
   */
  if (controlMCUSlugsMKIINewNav_DWork.IC1_FirstOutputTime_p) {
    controlMCUSlugsMKIINewNav_DWork.IC1_FirstOutputTime_p = FALSE;
    rtb_IC1 = FALSE;
  } else {
    rtb_IC1 = ((controlMCUSlugsMKIINewNav_B.wpFlynavSupportc != 0) &&
               (!(controlMCUSlugsMKIINewNav_B.ManualorAutonavSupportc != 0)));
  }

  /* End of InitialCondition: '<S137>/IC1' */

  /* DataTypeConversion: '<S407>/Data Type Conversion2' incorporates:
   *  Delay: '<S412>/Delay'
   *  Logic: '<S412>/Logical Operator'
   *  RelationalOperator: '<S412>/Relational Operator'
   */
  rtb_DataTypeConversion2_n = ((controlMCUSlugsMKIINewNav_DWork.Delay_DSTATE !=
    rtb_IC1) && rtb_IC1);

  /* InitialCondition: '<S137>/IC2' */
  if (controlMCUSlugsMKIINewNav_DWork.IC2_FirstOutputTime) {
    controlMCUSlugsMKIINewNav_DWork.IC2_FirstOutputTime = FALSE;
    rtb_IC2 = 0U;
  } else {
    rtb_IC2 = rtb_DataTypeConversion2_n;
  }

  /* End of InitialCondition: '<S137>/IC2' */

  /* Outputs for Enabled SubSystem: '<S137>/Grab Upon Enable' incorporates:
   *  EnablePort: '<S406>/Enable'
   */
  if (rtb_IC2 > 0) {
    /* S-Function "dsPIC_C_function_Call" Block: <S406>/Get the GS Location [updateControlMCUState.c] */
    getGSLocation(&controlMCUSlugsMKIINewNav_B.GettheGSLocationupdateControlMC[0]);

    /* SignalConversion: '<S409>/Numerical Unity' */
    controlMCUSlugsMKIINewNav_B.NumericalUnity[0] =
      controlMCUSlugsMKIINewNav_B.GettheGSLocationupdateControlMC[0];
    controlMCUSlugsMKIINewNav_B.NumericalUnity[1] =
      controlMCUSlugsMKIINewNav_B.GettheGSLocationupdateControlMC[1];
    controlMCUSlugsMKIINewNav_B.NumericalUnity[2] =
      controlMCUSlugsMKIINewNav_B.GettheGSLocationupdateControlMC[2];

    /* Gain: '<S410>/Deg2R' */
    rtb_cosphi = 0.0174532924F * controlMCUSlugsMKIINewNav_B.NumericalUnity[1];

    /* Trigonometry: '<S410>/sin(phi)' */
    rtb_Ze = (real32_T)sin(rtb_cosphi);

    /* Sum: '<S410>/Sum1' incorporates:
     *  Constant: '<S410>/const'
     *  Product: '<S410>/Product1'
     *  Product: '<S410>/sin(phi)^2'
     */
    rtb_Merge_idx = 1.0F - rtb_Ze * rtb_Ze *
      controlMCUSlugsMKIINewNa_ConstB.Sum5_i;

    /* Fcn: '<S410>/f' */
    if (rtb_Merge_idx < 0.0F) {
      rtb_Product3_p4_idx = -(real32_T)sqrt(-rtb_Merge_idx);
    } else {
      rtb_Product3_p4_idx = (real32_T)sqrt(rtb_Merge_idx);
    }

    /* End of Fcn: '<S410>/f' */

    /* Product: '<S410>/Rh' incorporates:
     *  Constant: '<S410>/Re=equatorial radius'
     */
    rtb_Merge_idx = 6.378137E+6F / rtb_Product3_p4_idx;

    /* Sum: '<S410>/Sum2' */
    rtb_RhhcosphisinlambYe = controlMCUSlugsMKIINewNav_B.NumericalUnity[0] +
      rtb_Merge_idx;

    /* Trigonometry: '<S410>/cos(phi)' */
    rtb_cosphi = (real32_T)cos(rtb_cosphi);

    /* Gain: '<S410>/Deg2R1' */
    rtb_Deg2R1 = 0.0174532924F * controlMCUSlugsMKIINewNav_B.NumericalUnity[2];

    /* Product: '<S410>/(Rh+h)cos(phi)*cos(lamb)=Xe' incorporates:
     *  Trigonometry: '<S410>/cos(lamb)'
     */
    rtb_RhhcosphicoslambXe = rtb_RhhcosphisinlambYe * rtb_cosphi * (real32_T)cos
      (rtb_Deg2R1);

    /* Product: '<S410>/(Rh+h)cos(phi)*sin(lamb)=Ye' incorporates:
     *  Trigonometry: '<S410>/sin(lamb)'
     */
    rtb_RhhcosphisinlambYe = rtb_RhhcosphisinlambYe * rtb_cosphi * (real32_T)sin
      (rtb_Deg2R1);

    /* Product: '<S410>/Ze' incorporates:
     *  Product: '<S410>/Rh(1-e^2)'
     *  Sum: '<S410>/Sum4'
     */
    rtb_Ze *= controlMCUSlugsMKIINewNa_ConstB.e2_g * rtb_Merge_idx +
      controlMCUSlugsMKIINewNav_B.NumericalUnity[0];

    /* DataTypeConversion: '<S406>/Data Type Conversion' */
    controlMCUSlugsMKIINewNav_B.DataTypeConversion_ks[0] =
      controlMCUSlugsMKIINewNav_B.NumericalUnity[2];
    controlMCUSlugsMKIINewNav_B.DataTypeConversion_ks[1] =
      controlMCUSlugsMKIINewNav_B.NumericalUnity[1];

    /* DataTypeConversion: '<S406>/Data Type Conversion1' */
    controlMCUSlugsMKIINewNav_B.DataTypeConversion1[0] = rtb_RhhcosphicoslambXe;
    controlMCUSlugsMKIINewNav_B.DataTypeConversion1[1] = rtb_RhhcosphisinlambYe;
    controlMCUSlugsMKIINewNav_B.DataTypeConversion1[2] = rtb_Ze;
  }

  /* End of Outputs for SubSystem: '<S137>/Grab Upon Enable' */

  /* MATLAB Function: '<S128>/myMux Fun2' */
  controlMCUSlugsMKIINe_myMuxFun1
    (controlMCUSlugsMKIINewNav_B.DataTypeConversion1[0],
     controlMCUSlugsMKIINewNav_B.DataTypeConversion1[1],
     controlMCUSlugsMKIINewNav_B.DataTypeConversion1[2],
     &controlMCUSlugsMKIINewNav_B.sf_myMuxFun2_c);

  /* S-Function "dsPIC_C_function_Call" Block: <S128>/Diagnostics Float [navSupport.c] */
  setDiagnosticFloat(controlMCUSlugsMKIINewNav_B.sf_myMuxFun2_c.y);

  /* S-Function "dsPIC_C_function_Call" Block: <S18>/Navigation Mode [navSupport.c] */
  controlMCUSlugsMKIINewNav_B.NavigationModenavSupportc_c = getNavMode();

  /* InitialCondition: '<S137>/IC3' */
  if (controlMCUSlugsMKIINewNav_DWork.IC3_FirstOutputTime) {
    controlMCUSlugsMKIINewNav_DWork.IC3_FirstOutputTime = FALSE;
    controlMCUSlugsMKIINewNav_B.IC3 = 0.0F;
  } else {
    controlMCUSlugsMKIINewNav_B.IC3 =
      controlMCUSlugsMKIINewNav_B.NumericalUnity[0];
  }

  /* End of InitialCondition: '<S137>/IC3' */

  /* MATLAB Function: '<S128>/myMux Fun4' incorporates:
   *  DataTypeConversion: '<S128>/Data Type Conversion3'
   */
  /* MATLAB Function 'Navigation/Navigation/myMux Fun4': '<S143>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S143>:1:5' */
  controlMCUSlugsMKIINewNav_B.y_eh[0] =
    controlMCUSlugsMKIINewNav_B.NavigationModenavSupportc_c;
  controlMCUSlugsMKIINewNav_B.y_eh[1] = 0;

  /* DataTypeConversion: '<S128>/Data Type Conversion7' */
  rtb_Product3_p4_idx = (real32_T)floor(controlMCUSlugsMKIINewNav_B.IC3);
  if (rtIsNaNF(rtb_Product3_p4_idx) || rtIsInfF(rtb_Product3_p4_idx)) {
    rtb_Product3_p4_idx = 0.0F;
  } else {
    rtb_Product3_p4_idx = (real32_T)fmod(rtb_Product3_p4_idx, 65536.0F);
  }

  /* MATLAB Function: '<S128>/myMux Fun4' incorporates:
   *  DataTypeConversion: '<S128>/Data Type Conversion7'
   */
  controlMCUSlugsMKIINewNav_B.y_eh[2] = rtb_Product3_p4_idx < 0.0F ? -(int16_T)
    (uint16_T)-rtb_Product3_p4_idx : (int16_T)(uint16_T)rtb_Product3_p4_idx;

  /* S-Function "dsPIC_C_function_Call" Block: <S128>/Diagnostics Short [navSupport.c] */
  setDiagnosticShort(controlMCUSlugsMKIINewNav_B.y_eh);

  /* InitialCondition: '<S137>/IC4' */
  if (controlMCUSlugsMKIINewNav_DWork.IC4_FirstOutputTime) {
    controlMCUSlugsMKIINewNav_DWork.IC4_FirstOutputTime = FALSE;
    controlMCUSlugsMKIINewNav_B.IC4[0] = 36.9885063F;
    controlMCUSlugsMKIINewNav_B.IC4[1] = -122.055305F;
  } else {
    controlMCUSlugsMKIINewNav_B.IC4[0] =
      controlMCUSlugsMKIINewNav_B.DataTypeConversion_ks[0];
    controlMCUSlugsMKIINewNav_B.IC4[1] =
      controlMCUSlugsMKIINewNav_B.DataTypeConversion_ks[1];
  }

  /* End of InitialCondition: '<S137>/IC4' */

  /* Inport: '<S18>/Puav' */
  rtb_Puav[0] = controlMCUSlugsMKIINewNav_B.GetXYZnavSupportc[0];
  rtb_Puav[1] = controlMCUSlugsMKIINewNav_B.GetXYZnavSupportc[1];
  rtb_Puav[2] = controlMCUSlugsMKIINewNav_B.GetXYZnavSupportc[2];

  /* MATLAB Function: '<S128>/Zero out Z2' */
  controlMCUSlugsMKIINe_ZerooutZ1(rtb_Puav,
    &controlMCUSlugsMKIINewNav_B.sf_ZerooutZ2);

  /* MATLAB Function: '<S128>/Zero out Z1' */
  controlMCUSlugsMKIINe_ZerooutZ1(controlMCUSlugsMKIINewNav_B.GetVnednavSupportc,
    &controlMCUSlugsMKIINewNav_B.sf_ZerooutZ1);

  /* MATLAB Function: '<S211>/Embedded MATLAB Function' */
  contro_EmbeddedMATLABFunction_o(controlMCUSlugsMKIINewNav_B.sf_ZerooutZ1.P,
    &controlMCUSlugsMKIINewNav_B.sf_EmbeddedMATLABFunction_e);

  /* MATLAB Function: '<S212>/negprotect' */
  controlMCUSlugsMKIIN_negprotect
    (controlMCUSlugsMKIINewNav_B.sf_EmbeddedMATLABFunction_e.xDoty,
     &controlMCUSlugsMKIINewNav_B.sf_negprotect_p);

  /* S-Function "dsPIC_C_function_Call" Block: <S212>/C Function Call */
  controlMCUSlugsMKIINewNav_B.CFunctionCall_a = mySqrt
    (controlMCUSlugsMKIINewNav_B.sf_negprotect_p.zpVal);

  /* InitialCondition: '<S134>/IC' */
  if (controlMCUSlugsMKIINewNav_DWork.IC_FirstOutputTime) {
    controlMCUSlugsMKIINewNav_DWork.IC_FirstOutputTime = FALSE;
    rtb_Merge_idx = 1.5F;
  } else {
    rtb_Merge_idx = controlMCUSlugsMKIINewNav_B.GetRangeofValuesnavSupportc_c[0];
  }

  /* End of InitialCondition: '<S134>/IC' */

  /* Product: '<S134>/Product3' incorporates:
   *  Constant: '<S134>/Constant1'
   *  Product: '<S134>/Product1'
   *  Product: '<S134>/Product2'
   *  Saturate: '<S134>/[1.5 10]'
   */
  Product = (rtb_Merge_idx >= 10.0F ? 10.0F : rtb_Merge_idx <= 1.5F ? 1.5F :
             rtb_Merge_idx) *
    (controlMCUSlugsMKIINewNav_B.GetMidLevelCommandsnavSupportc[0] *
     controlMCUSlugsMKIINewNav_B.GetMidLevelCommandsnavSupportc[0]) / 4.57681F;

  /* If: '<S128>/Determine Overall Nav by the Nav Mode' incorporates:
   *  Inport: '<S133>/MidLvl h_c'
   *  Inport: '<S136>/MidLvl h_c'
   */
  if (controlMCUSlugsMKIINewNav_B.GetRTBOrdernavSupportc[0] == 1) {
    /* Outputs for IfAction SubSystem: '<S128>/RTB//Follow Mobile Navigation' incorporates:
     *  ActionPort: '<S136>/Action Port'
     */
    /* Outputs for Enabled SubSystem: '<S136>/Compute Mobile Location' incorporates:
     *  EnablePort: '<S390>/Enable'
     */
    if (controlMCUSlugsMKIINewNav_B.GetRTBOrdernavSupportc[1] > 0) {
      /* Gain: '<S395>/Deg2R' */
      rtb_Deg2R_m_idx_0 = 0.0174532924F * controlMCUSlugsMKIINewNav_B.IC4[0];
      rtb_Deg2R_m_idx = 0.0174532924F * controlMCUSlugsMKIINewNav_B.IC4[1];

      /* Gain: '<S396>/Deg2R' */
      rtb_cosphi = 0.0174532924F *
        controlMCUSlugsMKIINewNav_B.GetMobileLocationnavSupportc[0];

      /* Trigonometry: '<S396>/sin(phi)' */
      rtb_Ze = (real32_T)sin(rtb_cosphi);

      /* Sum: '<S396>/Sum1' incorporates:
       *  Constant: '<S396>/const'
       *  Product: '<S396>/Product1'
       *  Product: '<S396>/sin(phi)^2'
       */
      rtb_Merge_idx = 1.0F - rtb_Ze * rtb_Ze *
        controlMCUSlugsMKIINewNa_ConstB.Sum5_ib;

      /* Fcn: '<S396>/f' */
      if (rtb_Merge_idx < 0.0F) {
        rtb_Product3_p4_idx = -(real32_T)sqrt(-rtb_Merge_idx);
      } else {
        rtb_Product3_p4_idx = (real32_T)sqrt(rtb_Merge_idx);
      }

      /* End of Fcn: '<S396>/f' */

      /* Product: '<S396>/Rh' incorporates:
       *  Constant: '<S396>/Re=equatorial radius'
       */
      rtb_Merge_idx = 6.378137E+6F / rtb_Product3_p4_idx;

      /* Trigonometry: '<S396>/cos(phi)' */
      rtb_cosphi = (real32_T)cos(rtb_cosphi);

      /* Gain: '<S396>/Deg2R1' */
      rtb_Deg2R1 = 0.0174532924F *
        controlMCUSlugsMKIINewNav_B.GetMobileLocationnavSupportc[1];

      /* Product: '<S396>/Ze' incorporates:
       *  Product: '<S396>/Rh(1-e^2)'
       */
      rtb_Ze *= controlMCUSlugsMKIINewNa_ConstB.e2_e * rtb_Merge_idx;

      /* SignalConversion: '<S395>/TmpSignal ConversionAtProduct1Inport1' incorporates:
       *  Fcn: '<S398>/11'
       *  Fcn: '<S398>/12'
       *  Fcn: '<S398>/13'
       *  Fcn: '<S398>/21'
       *  Fcn: '<S398>/22'
       *  Fcn: '<S398>/31'
       *  Fcn: '<S398>/32'
       *  Fcn: '<S398>/33'
       */
      rtb_VectorConcatenate_i[0L] = (real32_T)cos(rtb_Deg2R_m_idx_0) * (real32_T)
        cos(rtb_Deg2R_m_idx);
      rtb_VectorConcatenate_i[1L] = -(real32_T)sin(rtb_Deg2R_m_idx_0);
      rtb_VectorConcatenate_i[2L] = -(real32_T)sin(rtb_Deg2R_m_idx) * (real32_T)
        cos(rtb_Deg2R_m_idx_0);
      rtb_VectorConcatenate_i[3L] = (real32_T)sin(rtb_Deg2R_m_idx_0) * (real32_T)
        cos(rtb_Deg2R_m_idx);
      rtb_VectorConcatenate_i[4L] = (real32_T)cos(rtb_Deg2R_m_idx_0);
      rtb_VectorConcatenate_i[5L] = -(real32_T)sin(rtb_Deg2R_m_idx_0) *
        (real32_T)sin(rtb_Deg2R_m_idx);
      rtb_VectorConcatenate_i[6L] = (real32_T)sin(rtb_Deg2R_m_idx);
      rtb_VectorConcatenate_i[7L] = 0.0F;
      rtb_VectorConcatenate_i[8L] = (real32_T)cos(rtb_Deg2R_m_idx);

      /* Sum: '<S394>/Sum1' incorporates:
       *  Product: '<S396>/(Rh+h)cos(phi)*cos(lamb)=Xe'
       *  Product: '<S396>/(Rh+h)cos(phi)*sin(lamb)=Ye'
       *  Sum: '<S396>/Sum2'
       *  Trigonometry: '<S396>/cos(lamb)'
       *  Trigonometry: '<S396>/sin(lamb)'
       */
      rtb_Deg2R_m_idx = rtb_Merge_idx * rtb_cosphi * (real32_T)cos(rtb_Deg2R1) -
        controlMCUSlugsMKIINewNav_B.DataTypeConversion1[0];
      rtb_Deg2R_m_idx_0 = rtb_Merge_idx * rtb_cosphi * (real32_T)sin(rtb_Deg2R1)
        - controlMCUSlugsMKIINewNav_B.DataTypeConversion1[1];
      rtb_Product3_p4_idx = rtb_Ze -
        controlMCUSlugsMKIINewNav_B.DataTypeConversion1[2];

      /* Product: '<S395>/Product1' incorporates:
       *  Gain: '<S394>/UEN 2 NEU'
       */
      for (i = 0; i < 3; i++) {
        tmp[i] = rtb_VectorConcatenate_i[i + 6] * rtb_Product3_p4_idx +
          (rtb_VectorConcatenate_i[i + 3] * rtb_Deg2R_m_idx_0 +
           rtb_VectorConcatenate_i[i] * rtb_Deg2R_m_idx);
      }

      /* End of Product: '<S395>/Product1' */

      /* Reshape: '<S394>/Reshape1' incorporates:
       *  Gain: '<S394>/UEN 2 NEU'
       */
      for (i = 0; i < 3; i++) {
        controlMCUSlugsMKIINewNav_B.Reshape1[i] = 0.0F;
        controlMCUSlugsMKIINewNav_B.Reshape1[i] =
          controlMCUSlugsMKIINewNa_ConstP.pooled45[i] * tmp[0] +
          controlMCUSlugsMKIINewNav_B.Reshape1[i];
        controlMCUSlugsMKIINewNav_B.Reshape1[i] =
          controlMCUSlugsMKIINewNa_ConstP.pooled45[i + 3] * tmp[1] +
          controlMCUSlugsMKIINewNav_B.Reshape1[i];
        controlMCUSlugsMKIINewNav_B.Reshape1[i] =
          controlMCUSlugsMKIINewNa_ConstP.pooled45[i + 6] * tmp[2] +
          controlMCUSlugsMKIINewNav_B.Reshape1[i];
      }

      /* End of Reshape: '<S394>/Reshape1' */
    }

    /* End of Outputs for SubSystem: '<S136>/Compute Mobile Location' */

    /* MATLAB Function: '<S136>/Zero out Z1' */
    controlMCUSlugsMKIINe_ZerooutZ1(controlMCUSlugsMKIINewNav_B.Reshape1,
      &controlMCUSlugsMKIINewNav_B.sf_ZerooutZ1_o);

    /* Sum: '<S136>/Subtract' */
    rtb_Subtract_o[0] = controlMCUSlugsMKIINewNav_B.sf_ZerooutZ1_o.P[0] -
      controlMCUSlugsMKIINewNav_B.sf_ZerooutZ2.P[0];
    rtb_Subtract_o[1] = controlMCUSlugsMKIINewNav_B.sf_ZerooutZ1_o.P[1] -
      controlMCUSlugsMKIINewNav_B.sf_ZerooutZ2.P[1];
    rtb_Subtract_o[2] = controlMCUSlugsMKIINewNav_B.sf_ZerooutZ1_o.P[2] -
      controlMCUSlugsMKIINewNav_B.sf_ZerooutZ2.P[2];

    /* DiscreteIntegrator: '<S391>/Discrete-Time Integrator' */
    if (controlMCUSlugsMKIINewNav_DWork.DiscreteTimeIntegrator_IC_LOADI != 0) {
      controlMCUSlugsMKIINewNav_DWork.DiscreteTimeIntegrator_DSTATE[0] =
        rtb_Subtract_o[0];
      controlMCUSlugsMKIINewNav_DWork.DiscreteTimeIntegrator_DSTATE[1] =
        rtb_Subtract_o[1];
      controlMCUSlugsMKIINewNav_DWork.DiscreteTimeIntegrator_DSTATE[2] =
        rtb_Subtract_o[2];
    }

    /* Sum: '<S391>/Sum' incorporates:
     *  DiscreteIntegrator: '<S391>/Discrete-Time Integrator'
     */
    rtb_Product6[0] = rtb_Subtract_o[0] -
      controlMCUSlugsMKIINewNav_DWork.DiscreteTimeIntegrator_DSTATE[0];
    rtb_Product6[1] = rtb_Subtract_o[1] -
      controlMCUSlugsMKIINewNav_DWork.DiscreteTimeIntegrator_DSTATE[1];
    rtb_Product6[2] = rtb_Subtract_o[2] -
      controlMCUSlugsMKIINewNav_DWork.DiscreteTimeIntegrator_DSTATE[2];

    /* Sum: '<S391>/Sum1' incorporates:
     *  DiscreteIntegrator: '<S391>/Discrete-Time Integrator'
     *  Gain: '<S391>/Gain1'
     */
    controlMCUSlugsMKIINewNav_B.Merge[0] = 3.0F * rtb_Product6[0] +
      controlMCUSlugsMKIINewNav_DWork.DiscreteTimeIntegrator_DSTATE[0];
    controlMCUSlugsMKIINewNav_B.Merge[1] = 3.0F * rtb_Product6[1] +
      controlMCUSlugsMKIINewNav_DWork.DiscreteTimeIntegrator_DSTATE[1];
    controlMCUSlugsMKIINewNav_B.Merge[2] = 3.0F * rtb_Product6[2] +
      controlMCUSlugsMKIINewNav_DWork.DiscreteTimeIntegrator_DSTATE[2];

    /* MATLAB Function: '<S400>/Embedded MATLAB Function' */
    contro_EmbeddedMATLABFunction_o(rtb_Subtract_o,
      &controlMCUSlugsMKIINewNav_B.sf_EmbeddedMATLABFunction_o);

    /* MATLAB Function: '<S401>/negprotect' */
    controlMCUSlugsMKIIN_negprotect
      (controlMCUSlugsMKIINewNav_B.sf_EmbeddedMATLABFunction_o.xDoty,
       &controlMCUSlugsMKIINewNav_B.sf_negprotect_nu);

    /* S-Function "dsPIC_C_function_Call" Block: <S401>/C Function Call */
    controlMCUSlugsMKIINewNav_B.CFunctionCall_fw = mySqrt
      (controlMCUSlugsMKIINewNav_B.sf_negprotect_nu.zpVal);

    /* SignalConversion: '<S404>/Numerical Unity' */
    controlMCUSlugsMKIINewNav_B.Merge2 =
      controlMCUSlugsMKIINewNav_B.CFunctionCall_fw;
    controlMCUSlugsMKIINewNav_B.Merge1 =
      controlMCUSlugsMKIINewNav_B.GetMidLevelCommandsnavSupportc[1];

    /* Update for DiscreteIntegrator: '<S391>/Discrete-Time Integrator' incorporates:
     *  Inport: '<S136>/MidLvl h_c'
     */
    controlMCUSlugsMKIINewNav_DWork.DiscreteTimeIntegrator_IC_LOADI = 0U;
    controlMCUSlugsMKIINewNav_DWork.DiscreteTimeIntegrator_DSTATE[0] = 0.01F *
      rtb_Product6[0] +
      controlMCUSlugsMKIINewNav_DWork.DiscreteTimeIntegrator_DSTATE[0];
    controlMCUSlugsMKIINewNav_DWork.DiscreteTimeIntegrator_DSTATE[1] = 0.01F *
      rtb_Product6[1] +
      controlMCUSlugsMKIINewNav_DWork.DiscreteTimeIntegrator_DSTATE[1];
    controlMCUSlugsMKIINewNav_DWork.DiscreteTimeIntegrator_DSTATE[2] = 0.01F *
      rtb_Product6[2] +
      controlMCUSlugsMKIINewNav_DWork.DiscreteTimeIntegrator_DSTATE[2];

    /* End of Outputs for SubSystem: '<S128>/RTB//Follow Mobile Navigation' */
  } else if (controlMCUSlugsMKIINewNav_B.NavigationModenavSupportc_c == 3) {
    /* Outputs for IfAction SubSystem: '<S128>/Normal WP  Navigation' incorporates:
     *  ActionPort: '<S135>/Action Port'
     */
    /* Product: '<S135>/Product' */
    Product = controlMCUSlugsMKIINewNav_B.GetRangeofValuesnavSupportc_k[0] *
      controlMCUSlugsMKIINewNav_B.CFunctionCall_a;

    /* Outputs for Enabled SubSystem: '<S135>/On WP Enable' incorporates:
     *  EnablePort: '<S252>/Enable'
     */
    /* Gain: '<S353>/Deg2R' */
    rtb_Deg2R_m_idx_0 = 0.0174532924F * controlMCUSlugsMKIINewNav_B.IC4[0];
    rtb_Deg2R_m_idx = 0.0174532924F * controlMCUSlugsMKIINewNav_B.IC4[1];

    /* S-Function "dsPIC_C_function_Call" Block: <S331>/Get Waypoint Coordinates [navSupport.c] */
    getWP(((uint8_T)1U),
          &controlMCUSlugsMKIINewNav_B.GetWaypointCoordinatesnavSuppor[0]);

    /* Gain: '<S354>/Deg2R' */
    rtb_cosphi = 0.0174532924F *
      controlMCUSlugsMKIINewNav_B.GetWaypointCoordinatesnavSuppor[0];

    /* Trigonometry: '<S354>/sin(phi)' */
    rtb_Ze = (real32_T)sin(rtb_cosphi);

    /* Sum: '<S354>/Sum1' incorporates:
     *  Constant: '<S354>/const'
     *  Product: '<S354>/Product1'
     *  Product: '<S354>/sin(phi)^2'
     */
    rtb_Merge_idx = 1.0F - rtb_Ze * rtb_Ze *
      controlMCUSlugsMKIINewNa_ConstB.Sum5_c;

    /* Fcn: '<S354>/f' */
    if (rtb_Merge_idx < 0.0F) {
      rtb_Product3_p4_idx = -(real32_T)sqrt(-rtb_Merge_idx);
    } else {
      rtb_Product3_p4_idx = (real32_T)sqrt(rtb_Merge_idx);
    }

    /* End of Fcn: '<S354>/f' */

    /* Product: '<S354>/Rh' incorporates:
     *  Constant: '<S354>/Re=equatorial radius'
     */
    rtb_Merge_idx = 6.378137E+6F / rtb_Product3_p4_idx;

    /* Sum: '<S354>/Sum2' */
    rtb_RhhcosphisinlambYe =
      controlMCUSlugsMKIINewNav_B.GetWaypointCoordinatesnavSuppor[2] +
      rtb_Merge_idx;

    /* Trigonometry: '<S354>/cos(phi)' */
    rtb_cosphi = (real32_T)cos(rtb_cosphi);

    /* Gain: '<S354>/Deg2R1' */
    rtb_Deg2R1 = 0.0174532924F *
      controlMCUSlugsMKIINewNav_B.GetWaypointCoordinatesnavSuppor[1];

    /* Product: '<S354>/(Rh+h)cos(phi)*cos(lamb)=Xe' incorporates:
     *  Trigonometry: '<S354>/cos(lamb)'
     */
    rtb_RhhcosphicoslambXe = rtb_RhhcosphisinlambYe * rtb_cosphi * (real32_T)cos
      (rtb_Deg2R1);

    /* Product: '<S354>/(Rh+h)cos(phi)*sin(lamb)=Ye' incorporates:
     *  Trigonometry: '<S354>/sin(lamb)'
     */
    rtb_RhhcosphisinlambYe = rtb_RhhcosphisinlambYe * rtb_cosphi * (real32_T)sin
      (rtb_Deg2R1);

    /* Product: '<S354>/Ze' incorporates:
     *  Product: '<S354>/Rh(1-e^2)'
     *  Sum: '<S354>/Sum4'
     */
    rtb_Ze *= controlMCUSlugsMKIINewNa_ConstB.e2_b * rtb_Merge_idx +
      controlMCUSlugsMKIINewNav_B.GetWaypointCoordinatesnavSuppor[2];

    /* SignalConversion: '<S353>/TmpSignal ConversionAtProduct1Inport1' incorporates:
     *  Fcn: '<S356>/11'
     *  Fcn: '<S356>/12'
     *  Fcn: '<S356>/13'
     *  Fcn: '<S356>/21'
     *  Fcn: '<S356>/22'
     *  Fcn: '<S356>/31'
     *  Fcn: '<S356>/32'
     *  Fcn: '<S356>/33'
     */
    rtb_VectorConcatenate_i[0L] = (real32_T)cos(rtb_Deg2R_m_idx_0) * (real32_T)
      cos(rtb_Deg2R_m_idx);
    rtb_VectorConcatenate_i[1L] = -(real32_T)sin(rtb_Deg2R_m_idx_0);
    rtb_VectorConcatenate_i[2L] = -(real32_T)sin(rtb_Deg2R_m_idx) * (real32_T)
      cos(rtb_Deg2R_m_idx_0);
    rtb_VectorConcatenate_i[3L] = (real32_T)sin(rtb_Deg2R_m_idx_0) * (real32_T)
      cos(rtb_Deg2R_m_idx);
    rtb_VectorConcatenate_i[4L] = (real32_T)cos(rtb_Deg2R_m_idx_0);
    rtb_VectorConcatenate_i[5L] = -(real32_T)sin(rtb_Deg2R_m_idx_0) * (real32_T)
      sin(rtb_Deg2R_m_idx);
    rtb_VectorConcatenate_i[6L] = (real32_T)sin(rtb_Deg2R_m_idx);
    rtb_VectorConcatenate_i[7L] = 0.0F;
    rtb_VectorConcatenate_i[8L] = (real32_T)cos(rtb_Deg2R_m_idx);

    /* Sum: '<S351>/Sum1' */
    rtb_Deg2R_m_idx = rtb_RhhcosphicoslambXe -
      controlMCUSlugsMKIINewNav_B.DataTypeConversion1[0];
    rtb_Deg2R_m_idx_0 = rtb_RhhcosphisinlambYe -
      controlMCUSlugsMKIINewNav_B.DataTypeConversion1[1];
    rtb_Product3_p4_idx = rtb_Ze -
      controlMCUSlugsMKIINewNav_B.DataTypeConversion1[2];

    /* Product: '<S353>/Product1' incorporates:
     *  Gain: '<S351>/UEN 2 NEU'
     */
    for (i = 0; i < 3; i++) {
      tmp[i] = rtb_VectorConcatenate_i[i + 6] * rtb_Product3_p4_idx +
        (rtb_VectorConcatenate_i[i + 3] * rtb_Deg2R_m_idx_0 +
         rtb_VectorConcatenate_i[i] * rtb_Deg2R_m_idx);
    }

    /* End of Product: '<S353>/Product1' */

    /* Gain: '<S351>/UEN 2 NEU' */
    for (i = 0; i < 3; i++) {
      rtb_Product6[i] = controlMCUSlugsMKIINewNa_ConstP.pooled45[i + 6] * tmp[2]
        + (controlMCUSlugsMKIINewNa_ConstP.pooled45[i + 3] * tmp[1] +
           controlMCUSlugsMKIINewNa_ConstP.pooled45[i] * tmp[0]);
    }

    /* Gain: '<S370>/Deg2R' */
    rtb_Deg2R_m_idx_0 = 0.0174532924F * controlMCUSlugsMKIINewNav_B.IC4[0];
    rtb_Deg2R_m_idx = 0.0174532924F * controlMCUSlugsMKIINewNav_B.IC4[1];

    /* S-Function "dsPIC_C_function_Call" Block: <S332>/Get Waypoint Coordinates [navSupport.c] */
    getWP(((uint8_T)2U),
          &controlMCUSlugsMKIINewNav_B.GetWaypointCoordinatesnavSupp_j[0]);

    /* Gain: '<S371>/Deg2R' */
    rtb_cosphi = 0.0174532924F *
      controlMCUSlugsMKIINewNav_B.GetWaypointCoordinatesnavSupp_j[0];

    /* Trigonometry: '<S371>/sin(phi)' */
    rtb_Ze = (real32_T)sin(rtb_cosphi);

    /* Sum: '<S371>/Sum1' incorporates:
     *  Constant: '<S371>/const'
     *  Product: '<S371>/Product1'
     *  Product: '<S371>/sin(phi)^2'
     */
    rtb_Merge_idx = 1.0F - rtb_Ze * rtb_Ze *
      controlMCUSlugsMKIINewNa_ConstB.Sum5_l;

    /* Fcn: '<S371>/f' */
    if (rtb_Merge_idx < 0.0F) {
      rtb_Product3_p4_idx = -(real32_T)sqrt(-rtb_Merge_idx);
    } else {
      rtb_Product3_p4_idx = (real32_T)sqrt(rtb_Merge_idx);
    }

    /* End of Fcn: '<S371>/f' */

    /* Product: '<S371>/Rh' incorporates:
     *  Constant: '<S371>/Re=equatorial radius'
     */
    rtb_Merge_idx = 6.378137E+6F / rtb_Product3_p4_idx;

    /* Sum: '<S371>/Sum2' */
    rtb_RhhcosphisinlambYe =
      controlMCUSlugsMKIINewNav_B.GetWaypointCoordinatesnavSupp_j[2] +
      rtb_Merge_idx;

    /* Trigonometry: '<S371>/cos(phi)' */
    rtb_cosphi = (real32_T)cos(rtb_cosphi);

    /* Gain: '<S371>/Deg2R1' */
    rtb_Deg2R1 = 0.0174532924F *
      controlMCUSlugsMKIINewNav_B.GetWaypointCoordinatesnavSupp_j[1];

    /* Product: '<S371>/(Rh+h)cos(phi)*cos(lamb)=Xe' incorporates:
     *  Trigonometry: '<S371>/cos(lamb)'
     */
    rtb_RhhcosphicoslambXe = rtb_RhhcosphisinlambYe * rtb_cosphi * (real32_T)cos
      (rtb_Deg2R1);

    /* Product: '<S371>/(Rh+h)cos(phi)*sin(lamb)=Ye' incorporates:
     *  Trigonometry: '<S371>/sin(lamb)'
     */
    rtb_RhhcosphisinlambYe = rtb_RhhcosphisinlambYe * rtb_cosphi * (real32_T)sin
      (rtb_Deg2R1);

    /* Product: '<S371>/Ze' incorporates:
     *  Product: '<S371>/Rh(1-e^2)'
     *  Sum: '<S371>/Sum4'
     */
    rtb_Ze *= controlMCUSlugsMKIINewNa_ConstB.e2_h * rtb_Merge_idx +
      controlMCUSlugsMKIINewNav_B.GetWaypointCoordinatesnavSupp_j[2];

    /* SignalConversion: '<S370>/TmpSignal ConversionAtProduct1Inport1' incorporates:
     *  Fcn: '<S373>/11'
     *  Fcn: '<S373>/12'
     *  Fcn: '<S373>/13'
     *  Fcn: '<S373>/21'
     *  Fcn: '<S373>/22'
     *  Fcn: '<S373>/31'
     *  Fcn: '<S373>/32'
     *  Fcn: '<S373>/33'
     */
    rtb_VectorConcatenate_i[0L] = (real32_T)cos(rtb_Deg2R_m_idx_0) * (real32_T)
      cos(rtb_Deg2R_m_idx);
    rtb_VectorConcatenate_i[1L] = -(real32_T)sin(rtb_Deg2R_m_idx_0);
    rtb_VectorConcatenate_i[2L] = -(real32_T)sin(rtb_Deg2R_m_idx) * (real32_T)
      cos(rtb_Deg2R_m_idx_0);
    rtb_VectorConcatenate_i[3L] = (real32_T)sin(rtb_Deg2R_m_idx_0) * (real32_T)
      cos(rtb_Deg2R_m_idx);
    rtb_VectorConcatenate_i[4L] = (real32_T)cos(rtb_Deg2R_m_idx_0);
    rtb_VectorConcatenate_i[5L] = -(real32_T)sin(rtb_Deg2R_m_idx_0) * (real32_T)
      sin(rtb_Deg2R_m_idx);
    rtb_VectorConcatenate_i[6L] = (real32_T)sin(rtb_Deg2R_m_idx);
    rtb_VectorConcatenate_i[7L] = 0.0F;
    rtb_VectorConcatenate_i[8L] = (real32_T)cos(rtb_Deg2R_m_idx);

    /* Sum: '<S368>/Sum1' */
    rtb_Deg2R_m_idx = rtb_RhhcosphicoslambXe -
      controlMCUSlugsMKIINewNav_B.DataTypeConversion1[0];
    rtb_Deg2R_m_idx_0 = rtb_RhhcosphisinlambYe -
      controlMCUSlugsMKIINewNav_B.DataTypeConversion1[1];
    rtb_Product3_p4_idx = rtb_Ze -
      controlMCUSlugsMKIINewNav_B.DataTypeConversion1[2];

    /* Product: '<S370>/Product1' incorporates:
     *  Gain: '<S368>/UEN 2 NEU'
     */
    for (i = 0; i < 3; i++) {
      tmp[i] = rtb_VectorConcatenate_i[i + 6] * rtb_Product3_p4_idx +
        (rtb_VectorConcatenate_i[i + 3] * rtb_Deg2R_m_idx_0 +
         rtb_VectorConcatenate_i[i] * rtb_Deg2R_m_idx);
    }

    /* End of Product: '<S370>/Product1' */

    /* Gain: '<S368>/UEN 2 NEU' */
    for (i = 0; i < 3; i++) {
      tmp_0[i] = controlMCUSlugsMKIINewNa_ConstP.pooled45[i + 6] * tmp[2] +
        (controlMCUSlugsMKIINewNa_ConstP.pooled45[i + 3] * tmp[1] +
         controlMCUSlugsMKIINewNa_ConstP.pooled45[i] * tmp[0]);
    }

    /* Sum: '<S330>/Add' incorporates:
     *  Gain: '<S368>/UEN 2 NEU'
     */
    rtb_Add_e[0] = tmp_0[0] - rtb_Product6[0];
    rtb_Add_e[1] = tmp_0[1] - rtb_Product6[1];
    rtb_Add_e[2] = tmp_0[2] - rtb_Product6[2];

    /* MATLAB Function: '<S337>/Embedded MATLAB Function' */
    contro_EmbeddedMATLABFunction_o(rtb_Add_e,
      &controlMCUSlugsMKIINewNav_B.sf_EmbeddedMATLABFunction_f);

    /* MATLAB Function: '<S338>/negprotect' */
    controlMCUSlugsMKIIN_negprotect
      (controlMCUSlugsMKIINewNav_B.sf_EmbeddedMATLABFunction_f.xDoty,
       &controlMCUSlugsMKIINewNav_B.sf_negprotect_l);

    /* S-Function "dsPIC_C_function_Call" Block: <S338>/C Function Call */
    controlMCUSlugsMKIINewNav_B.CFunctionCall_kc = mySqrt
      (controlMCUSlugsMKIINewNav_B.sf_negprotect_l.zpVal);

    /* Saturate: '<S333>/Zero Bound' */
    rtb_Merge_idx = controlMCUSlugsMKIINewNav_B.CFunctionCall_kc >= 0.001F ?
      controlMCUSlugsMKIINewNav_B.CFunctionCall_kc : 0.001F;

    /* Product: '<S333>/Divide' */
    rtb_Divide_d[0] = rtb_Add_e[0] / rtb_Merge_idx;
    rtb_Divide_d[1] = rtb_Add_e[1] / rtb_Merge_idx;
    rtb_Divide_d[2] = rtb_Add_e[2] / rtb_Merge_idx;

    /* Product: '<S252>/Product' incorporates:
     *  Constant: '<S252>/Constant5'
     */
    rtb_Product5[0] = Product * rtb_Divide_d[0] * 2.0F;
    rtb_Product5[1] = Product * rtb_Divide_d[1] * 2.0F;
    rtb_Product5[2] = Product * rtb_Divide_d[2] * 2.0F;

    /* MATLAB Function: '<S330>/Select N  Terms' */
    controlMCUSlugsMKI_SelectNTerms(rtb_Divide_d,
      &controlMCUSlugsMKIINewNav_B.sf_SelectNTerms_a);

    /* MATLAB Function: '<S344>/Embedded MATLAB Function' */
    contro_EmbeddedMATLABFunction_o
      (controlMCUSlugsMKIINewNav_B.sf_SelectNTerms_a.N,
       &controlMCUSlugsMKIINewNav_B.sf_EmbeddedMATLABFunction_l);

    /* MATLAB Function: '<S345>/negprotect' */
    controlMCUSlugsMKIIN_negprotect
      (controlMCUSlugsMKIINewNav_B.sf_EmbeddedMATLABFunction_l.xDoty,
       &controlMCUSlugsMKIINewNav_B.sf_negprotect_h);

    /* S-Function "dsPIC_C_function_Call" Block: <S345>/C Function Call */
    controlMCUSlugsMKIINewNav_B.CFunctionCall_mu = mySqrt
      (controlMCUSlugsMKIINewNav_B.sf_negprotect_h.zpVal);

    /* DataTypeConversion: '<S352>/Data Type Conversion' incorporates:
     *  Constant: '<S329>/Constant'
     */
    rtb_DataTypeConversion = 1.0;

    /* MATLAB Function: '<S352>/Zero out Z1' */
    /* MATLAB Function: '<S352>/Zero out Z2' */
    /* MATLAB Function: '<S352>/Zero out Z3' */
    /* DataTypeConversion: '<S369>/Data Type Conversion' incorporates:
     *  Constant: '<S329>/Constant1'
     */
    rtb_DataTypeConversion_p = 2.0;

    /* MATLAB Function: '<S369>/Zero out Z1' */
    /* MATLAB Function: '<S369>/Zero out Z2' */
    /* MATLAB Function: '<S369>/Zero out Z3' */
    /* Sum: '<S252>/Subtract' */
    controlMCUSlugsMKIINewNav_B.WP0L2IPT1[0] = rtb_Product6[0] - rtb_Product5[0];
    controlMCUSlugsMKIINewNav_B.WP0L2IPT1[1] = rtb_Product6[1] - rtb_Product5[1];
    controlMCUSlugsMKIINewNav_B.WP0L2IPT1[2] = rtb_Product6[2] - rtb_Product5[2];

    /* End of Outputs for SubSystem: '<S135>/On WP Enable' */

    /* MATLAB Function: '<S135>/Zero out Z2' */
    controlMCUSlugsMKIINe_ZerooutZ1(controlMCUSlugsMKIINewNav_B.WP0L2IPT1,
      &controlMCUSlugsMKIINewNav_B.sf_ZerooutZ2_e);

    /* Sum: '<S135>/Add' */
    rtb_PaHPPned[0] = controlMCUSlugsMKIINewNav_B.sf_ZerooutZ2_e.P[0] -
      controlMCUSlugsMKIINewNav_B.sf_ZerooutZ2.P[0];
    rtb_PaHPPned[1] = controlMCUSlugsMKIINewNav_B.sf_ZerooutZ2_e.P[1] -
      controlMCUSlugsMKIINewNav_B.sf_ZerooutZ2.P[1];
    rtb_PaHPPned[2] = controlMCUSlugsMKIINewNav_B.sf_ZerooutZ2_e.P[2] -
      controlMCUSlugsMKIINewNav_B.sf_ZerooutZ2.P[2];

    /* Delay: '<S135>/Integer Delay' */
    rtb_IC2 = controlMCUSlugsMKIINewNav_DWork.IntegerDelay_DSTATE_p;

    /* MATLAB Function: '<S384>/Embedded MATLAB Function' */
    contro_EmbeddedMATLABFunction_o(rtb_PaHPPned,
      &controlMCUSlugsMKIINewNav_B.sf_EmbeddedMATLABFunction_n);

    /* MATLAB Function: '<S385>/negprotect' */
    controlMCUSlugsMKIIN_negprotect
      (controlMCUSlugsMKIINewNav_B.sf_EmbeddedMATLABFunction_n.xDoty,
       &controlMCUSlugsMKIINewNav_B.sf_negprotect_dg);

    /* S-Function "dsPIC_C_function_Call" Block: <S385>/C Function Call */
    controlMCUSlugsMKIINewNav_B.CFunctionCall_f5 = mySqrt
      (controlMCUSlugsMKIINewNav_B.sf_negprotect_dg.zpVal);

    /* MATLAB Function: '<S135>/Embedded MATLAB Function' incorporates:
     *  Gain: '<S135>/Gain'
     *  RelationalOperator: '<S135>/Relational Operator'
     */
    /* MATLAB Function 'Navigation/Navigation/Normal WP  Navigation/Embedded MATLAB Function': '<S250>:1' */
    /*  This functions keeps track if the IP has been reached and if so */
    /*  it disables homing of IP until WP is reset */
    /*  initialize the persistent and the return value to 1 */
    /*  so WP, and not homing, is active by default */
    /*  this is just turned on for one sample  */
    /*  inmediatley upon enabling the navigation. */
    /*  Reset the flag the the IP was reached */
    if (rtb_DataTypeConversion2_n != 0) {
      /* '<S250>:1:19' */
      controlMCUSlugsMKIINewNav_DWork.persistentDidReachIP = 0U;
    }

    /*  Once the IP is reached the the persistent variable */
    /*  preserves the values until reset */
    if (0.4F * Product > controlMCUSlugsMKIINewNav_B.CFunctionCall_f5) {
      /* '<S250>:1:24' */
      /* '<S250>:1:25' */
      controlMCUSlugsMKIINewNav_DWork.persistentDidReachIP = 1U;
    }

    /* InitialCondition: '<S135>/IC' incorporates:
     *  MATLAB Function: '<S135>/Embedded MATLAB Function'
     */
    /* '<S250>:1:28' */
    if (controlMCUSlugsMKIINewNav_DWork.IC_FirstOutputTime_l) {
      controlMCUSlugsMKIINewNav_DWork.IC_FirstOutputTime_l = FALSE;
      controlMCUSlugsMKIINewNav_B.IC = 0U;
    } else {
      controlMCUSlugsMKIINewNav_B.IC =
        controlMCUSlugsMKIINewNav_DWork.persistentDidReachIP;
    }

    /* End of InitialCondition: '<S135>/IC' */

    /* MATLAB Function: '<S135>/computeCurrentWP' */
    /* MATLAB Function 'Navigation/Navigation/Normal WP  Navigation/computeCurrentWP': '<S255>:1' */
    /*  This functions keeps track if the IP has been reached and if so */
    /*  it disables homing of IP until WP is reset */
    /*  initialize the persistent and the return value to 1 */
    /*  so WP, and not homing, is active by default */
    if (rtb_IC2 != 0) {
      /* '<S255>:1:16' */
      controlMCUSlugsMKIINewNav_DWork.fromWp =
        controlMCUSlugsMKIINewNav_DWork.toWp;

      /* '<S255>:1:17' */
      rtb_u2deg = (uint16_T)controlMCUSlugsMKIINewNav_DWork.toWp + 1U;
      if (rtb_u2deg > 255U) {
        rtb_u2deg = 255U;
      }

      controlMCUSlugsMKIINewNav_DWork.toWp = (uint8_T)rtb_u2deg;
      if (controlMCUSlugsMKIINewNav_DWork.toWp >
          controlMCUSlugsMKIINewNav_B.GetMaxWPnavSupportc) {
        /* '<S255>:1:18' */
        /* '<S255>:1:19' */
        controlMCUSlugsMKIINewNav_DWork.toWp = 1U;
      }
    }

    /*  this is jturned on long as we have not reached IP  */
    if (!(controlMCUSlugsMKIINewNav_B.IC != 0)) {
      /* '<S255>:1:25' */
      /* '<S255>:1:26' */
      controlMCUSlugsMKIINewNav_DWork.fromWp = 1U;

      /* '<S255>:1:27' */
      controlMCUSlugsMKIINewNav_DWork.toWp = 2U;
    }

    /* '<S255>:1:32' */
    controlMCUSlugsMKIINewNav_B.WP0 = controlMCUSlugsMKIINewNav_DWork.fromWp;

    /* '<S255>:1:33' */
    controlMCUSlugsMKIINewNav_B.WP1 = controlMCUSlugsMKIINewNav_DWork.toWp;

    /* End of MATLAB Function: '<S135>/computeCurrentWP' */

    /* Outputs for Enabled SubSystem: '<S135>/Get Frenet' incorporates:
     *  EnablePort: '<S251>/Enable'
     */
    /* Gain: '<S298>/Deg2R' */
    rtb_Deg2R_m_idx_0 = 0.0174532924F * controlMCUSlugsMKIINewNav_B.IC4[0];
    rtb_Deg2R_m_idx = 0.0174532924F * controlMCUSlugsMKIINewNav_B.IC4[1];

    /* S-Function "dsPIC_C_function_Call" Block: <S276>/Get Waypoint Coordinates [navSupport.c] */
    getWP(controlMCUSlugsMKIINewNav_B.WP0,
          &controlMCUSlugsMKIINewNav_B.GetWaypointCoordinatesnavSupp_n[0]);

    /* Gain: '<S299>/Deg2R' */
    rtb_cosphi = 0.0174532924F *
      controlMCUSlugsMKIINewNav_B.GetWaypointCoordinatesnavSupp_n[0];

    /* Trigonometry: '<S299>/sin(phi)' */
    rtb_Ze = (real32_T)sin(rtb_cosphi);

    /* Sum: '<S299>/Sum1' incorporates:
     *  Constant: '<S299>/const'
     *  Product: '<S299>/Product1'
     *  Product: '<S299>/sin(phi)^2'
     */
    rtb_Merge_idx = 1.0F - rtb_Ze * rtb_Ze *
      controlMCUSlugsMKIINewNa_ConstB.Sum5_d;

    /* Fcn: '<S299>/f' */
    if (rtb_Merge_idx < 0.0F) {
      rtb_Product3_p4_idx = -(real32_T)sqrt(-rtb_Merge_idx);
    } else {
      rtb_Product3_p4_idx = (real32_T)sqrt(rtb_Merge_idx);
    }

    /* End of Fcn: '<S299>/f' */

    /* Product: '<S299>/Rh' incorporates:
     *  Constant: '<S299>/Re=equatorial radius'
     */
    rtb_Merge_idx = 6.378137E+6F / rtb_Product3_p4_idx;

    /* Sum: '<S299>/Sum2' */
    rtb_RhhcosphisinlambYe =
      controlMCUSlugsMKIINewNav_B.GetWaypointCoordinatesnavSupp_n[2] +
      rtb_Merge_idx;

    /* Trigonometry: '<S299>/cos(phi)' */
    rtb_cosphi = (real32_T)cos(rtb_cosphi);

    /* Gain: '<S299>/Deg2R1' */
    rtb_Deg2R1 = 0.0174532924F *
      controlMCUSlugsMKIINewNav_B.GetWaypointCoordinatesnavSupp_n[1];

    /* Product: '<S299>/(Rh+h)cos(phi)*cos(lamb)=Xe' incorporates:
     *  Trigonometry: '<S299>/cos(lamb)'
     */
    rtb_RhhcosphicoslambXe = rtb_RhhcosphisinlambYe * rtb_cosphi * (real32_T)cos
      (rtb_Deg2R1);

    /* Product: '<S299>/(Rh+h)cos(phi)*sin(lamb)=Ye' incorporates:
     *  Trigonometry: '<S299>/sin(lamb)'
     */
    rtb_RhhcosphisinlambYe = rtb_RhhcosphisinlambYe * rtb_cosphi * (real32_T)sin
      (rtb_Deg2R1);

    /* Product: '<S299>/Ze' incorporates:
     *  Product: '<S299>/Rh(1-e^2)'
     *  Sum: '<S299>/Sum4'
     */
    rtb_Ze *= controlMCUSlugsMKIINewNa_ConstB.e2_o * rtb_Merge_idx +
      controlMCUSlugsMKIINewNav_B.GetWaypointCoordinatesnavSupp_n[2];

    /* SignalConversion: '<S298>/TmpSignal ConversionAtProduct1Inport1' incorporates:
     *  Fcn: '<S301>/11'
     *  Fcn: '<S301>/12'
     *  Fcn: '<S301>/13'
     *  Fcn: '<S301>/21'
     *  Fcn: '<S301>/22'
     *  Fcn: '<S301>/31'
     *  Fcn: '<S301>/32'
     *  Fcn: '<S301>/33'
     */
    rtb_VectorConcatenate_i[0L] = (real32_T)cos(rtb_Deg2R_m_idx_0) * (real32_T)
      cos(rtb_Deg2R_m_idx);
    rtb_VectorConcatenate_i[1L] = -(real32_T)sin(rtb_Deg2R_m_idx_0);
    rtb_VectorConcatenate_i[2L] = -(real32_T)sin(rtb_Deg2R_m_idx) * (real32_T)
      cos(rtb_Deg2R_m_idx_0);
    rtb_VectorConcatenate_i[3L] = (real32_T)sin(rtb_Deg2R_m_idx_0) * (real32_T)
      cos(rtb_Deg2R_m_idx);
    rtb_VectorConcatenate_i[4L] = (real32_T)cos(rtb_Deg2R_m_idx_0);
    rtb_VectorConcatenate_i[5L] = -(real32_T)sin(rtb_Deg2R_m_idx_0) * (real32_T)
      sin(rtb_Deg2R_m_idx);
    rtb_VectorConcatenate_i[6L] = (real32_T)sin(rtb_Deg2R_m_idx);
    rtb_VectorConcatenate_i[7L] = 0.0F;
    rtb_VectorConcatenate_i[8L] = (real32_T)cos(rtb_Deg2R_m_idx);

    /* Sum: '<S296>/Sum1' */
    rtb_Deg2R_m_idx = rtb_RhhcosphicoslambXe -
      controlMCUSlugsMKIINewNav_B.DataTypeConversion1[0];
    rtb_Deg2R_m_idx_0 = rtb_RhhcosphisinlambYe -
      controlMCUSlugsMKIINewNav_B.DataTypeConversion1[1];
    rtb_Product3_p4_idx = rtb_Ze -
      controlMCUSlugsMKIINewNav_B.DataTypeConversion1[2];

    /* Product: '<S298>/Product1' incorporates:
     *  Gain: '<S296>/UEN 2 NEU'
     */
    for (i = 0; i < 3; i++) {
      tmp[i] = rtb_VectorConcatenate_i[i + 6] * rtb_Product3_p4_idx +
        (rtb_VectorConcatenate_i[i + 3] * rtb_Deg2R_m_idx_0 +
         rtb_VectorConcatenate_i[i] * rtb_Deg2R_m_idx);
    }

    /* End of Product: '<S298>/Product1' */

    /* Gain: '<S296>/UEN 2 NEU' */
    for (i = 0; i < 3; i++) {
      rtb_Product6[i] = controlMCUSlugsMKIINewNa_ConstP.pooled45[i + 6] * tmp[2]
        + (controlMCUSlugsMKIINewNa_ConstP.pooled45[i + 3] * tmp[1] +
           controlMCUSlugsMKIINewNa_ConstP.pooled45[i] * tmp[0]);
    }

    /* Gain: '<S315>/Deg2R' */
    rtb_Deg2R_m_idx_0 = 0.0174532924F * controlMCUSlugsMKIINewNav_B.IC4[0];
    rtb_Deg2R_m_idx = 0.0174532924F * controlMCUSlugsMKIINewNav_B.IC4[1];

    /* S-Function "dsPIC_C_function_Call" Block: <S277>/Get Waypoint Coordinates [navSupport.c] */
    getWP(controlMCUSlugsMKIINewNav_B.WP1,
          &controlMCUSlugsMKIINewNav_B.GetWaypointCoordinatesnavSupp_c[0]);

    /* Gain: '<S316>/Deg2R' */
    rtb_cosphi = 0.0174532924F *
      controlMCUSlugsMKIINewNav_B.GetWaypointCoordinatesnavSupp_c[0];

    /* Trigonometry: '<S316>/sin(phi)' */
    rtb_Ze = (real32_T)sin(rtb_cosphi);

    /* Sum: '<S316>/Sum1' incorporates:
     *  Constant: '<S316>/const'
     *  Product: '<S316>/Product1'
     *  Product: '<S316>/sin(phi)^2'
     */
    rtb_Merge_idx = 1.0F - rtb_Ze * rtb_Ze *
      controlMCUSlugsMKIINewNa_ConstB.Sum5_n;

    /* Fcn: '<S316>/f' */
    if (rtb_Merge_idx < 0.0F) {
      rtb_Product3_p4_idx = -(real32_T)sqrt(-rtb_Merge_idx);
    } else {
      rtb_Product3_p4_idx = (real32_T)sqrt(rtb_Merge_idx);
    }

    /* End of Fcn: '<S316>/f' */

    /* Product: '<S316>/Rh' incorporates:
     *  Constant: '<S316>/Re=equatorial radius'
     */
    rtb_Merge_idx = 6.378137E+6F / rtb_Product3_p4_idx;

    /* Sum: '<S316>/Sum2' */
    rtb_RhhcosphisinlambYe =
      controlMCUSlugsMKIINewNav_B.GetWaypointCoordinatesnavSupp_c[2] +
      rtb_Merge_idx;

    /* Trigonometry: '<S316>/cos(phi)' */
    rtb_cosphi = (real32_T)cos(rtb_cosphi);

    /* Gain: '<S316>/Deg2R1' */
    rtb_Deg2R1 = 0.0174532924F *
      controlMCUSlugsMKIINewNav_B.GetWaypointCoordinatesnavSupp_c[1];

    /* Product: '<S316>/(Rh+h)cos(phi)*cos(lamb)=Xe' incorporates:
     *  Trigonometry: '<S316>/cos(lamb)'
     */
    rtb_RhhcosphicoslambXe = rtb_RhhcosphisinlambYe * rtb_cosphi * (real32_T)cos
      (rtb_Deg2R1);

    /* Product: '<S316>/(Rh+h)cos(phi)*sin(lamb)=Ye' incorporates:
     *  Trigonometry: '<S316>/sin(lamb)'
     */
    rtb_RhhcosphisinlambYe = rtb_RhhcosphisinlambYe * rtb_cosphi * (real32_T)sin
      (rtb_Deg2R1);

    /* Product: '<S316>/Ze' incorporates:
     *  Product: '<S316>/Rh(1-e^2)'
     *  Sum: '<S316>/Sum4'
     */
    rtb_Ze *= controlMCUSlugsMKIINewNa_ConstB.e2_ge * rtb_Merge_idx +
      controlMCUSlugsMKIINewNav_B.GetWaypointCoordinatesnavSupp_c[2];

    /* SignalConversion: '<S315>/TmpSignal ConversionAtProduct1Inport1' incorporates:
     *  Fcn: '<S318>/11'
     *  Fcn: '<S318>/12'
     *  Fcn: '<S318>/13'
     *  Fcn: '<S318>/21'
     *  Fcn: '<S318>/22'
     *  Fcn: '<S318>/31'
     *  Fcn: '<S318>/32'
     *  Fcn: '<S318>/33'
     */
    rtb_VectorConcatenate_i[0L] = (real32_T)cos(rtb_Deg2R_m_idx_0) * (real32_T)
      cos(rtb_Deg2R_m_idx);
    rtb_VectorConcatenate_i[1L] = -(real32_T)sin(rtb_Deg2R_m_idx_0);
    rtb_VectorConcatenate_i[2L] = -(real32_T)sin(rtb_Deg2R_m_idx) * (real32_T)
      cos(rtb_Deg2R_m_idx_0);
    rtb_VectorConcatenate_i[3L] = (real32_T)sin(rtb_Deg2R_m_idx_0) * (real32_T)
      cos(rtb_Deg2R_m_idx);
    rtb_VectorConcatenate_i[4L] = (real32_T)cos(rtb_Deg2R_m_idx_0);
    rtb_VectorConcatenate_i[5L] = -(real32_T)sin(rtb_Deg2R_m_idx_0) * (real32_T)
      sin(rtb_Deg2R_m_idx);
    rtb_VectorConcatenate_i[6L] = (real32_T)sin(rtb_Deg2R_m_idx);
    rtb_VectorConcatenate_i[7L] = 0.0F;
    rtb_VectorConcatenate_i[8L] = (real32_T)cos(rtb_Deg2R_m_idx);

    /* Sum: '<S313>/Sum1' */
    rtb_Deg2R_m_idx = rtb_RhhcosphicoslambXe -
      controlMCUSlugsMKIINewNav_B.DataTypeConversion1[0];
    rtb_Deg2R_m_idx_0 = rtb_RhhcosphisinlambYe -
      controlMCUSlugsMKIINewNav_B.DataTypeConversion1[1];
    rtb_Product3_p4_idx = rtb_Ze -
      controlMCUSlugsMKIINewNav_B.DataTypeConversion1[2];

    /* Product: '<S315>/Product1' incorporates:
     *  Gain: '<S313>/UEN 2 NEU'
     */
    for (i = 0; i < 3; i++) {
      tmp[i] = rtb_VectorConcatenate_i[i + 6] * rtb_Product3_p4_idx +
        (rtb_VectorConcatenate_i[i + 3] * rtb_Deg2R_m_idx_0 +
         rtb_VectorConcatenate_i[i] * rtb_Deg2R_m_idx);
    }

    /* End of Product: '<S315>/Product1' */

    /* Gain: '<S313>/UEN 2 NEU' */
    for (i = 0; i < 3; i++) {
      controlMCUSlugsMKIINewNav_B.UEN2NEU_l[i] = 0.0F;
      controlMCUSlugsMKIINewNav_B.UEN2NEU_l[i] =
        controlMCUSlugsMKIINewNa_ConstP.pooled45[i] * tmp[0] +
        controlMCUSlugsMKIINewNav_B.UEN2NEU_l[i];
      controlMCUSlugsMKIINewNav_B.UEN2NEU_l[i] =
        controlMCUSlugsMKIINewNa_ConstP.pooled45[i + 3] * tmp[1] +
        controlMCUSlugsMKIINewNav_B.UEN2NEU_l[i];
      controlMCUSlugsMKIINewNav_B.UEN2NEU_l[i] =
        controlMCUSlugsMKIINewNa_ConstP.pooled45[i + 6] * tmp[2] +
        controlMCUSlugsMKIINewNav_B.UEN2NEU_l[i];
    }

    /* Sum: '<S272>/Add' */
    rtb_Add_gw[0] = controlMCUSlugsMKIINewNav_B.UEN2NEU_l[0] - rtb_Product6[0];
    rtb_Add_gw[1] = controlMCUSlugsMKIINewNav_B.UEN2NEU_l[1] - rtb_Product6[1];
    rtb_Add_gw[2] = controlMCUSlugsMKIINewNav_B.UEN2NEU_l[2] - rtb_Product6[2];

    /* MATLAB Function: '<S282>/Embedded MATLAB Function' */
    contro_EmbeddedMATLABFunction_o(rtb_Add_gw,
      &controlMCUSlugsMKIINewNav_B.sf_EmbeddedMATLABFunction_i4);

    /* MATLAB Function: '<S283>/negprotect' */
    controlMCUSlugsMKIIN_negprotect
      (controlMCUSlugsMKIINewNav_B.sf_EmbeddedMATLABFunction_i4.xDoty,
       &controlMCUSlugsMKIINewNav_B.sf_negprotect_nk);

    /* S-Function "dsPIC_C_function_Call" Block: <S283>/C Function Call */
    controlMCUSlugsMKIINewNav_B.CFunctionCall_av = mySqrt
      (controlMCUSlugsMKIINewNav_B.sf_negprotect_nk.zpVal);

    /* Saturate: '<S278>/Zero Bound' */
    rtb_Merge_idx = controlMCUSlugsMKIINewNav_B.CFunctionCall_av >= 0.001F ?
      controlMCUSlugsMKIINewNav_B.CFunctionCall_av : 0.001F;

    /* Product: '<S278>/Divide' */
    rtb_Divide_d5[0] = rtb_Add_gw[0] / rtb_Merge_idx;
    rtb_Divide_d5[1] = rtb_Add_gw[1] / rtb_Merge_idx;
    rtb_Divide_d5[2] = rtb_Add_gw[2] / rtb_Merge_idx;

    /* MATLAB Function: '<S272>/Select N  Terms' */
    controlMCUSlugsMKI_SelectNTerms(rtb_Divide_d5,
      &controlMCUSlugsMKIINewNav_B.sf_SelectNTerms);

    /* MATLAB Function: '<S289>/Embedded MATLAB Function' */
    contro_EmbeddedMATLABFunction_o
      (controlMCUSlugsMKIINewNav_B.sf_SelectNTerms.N,
       &controlMCUSlugsMKIINewNav_B.sf_EmbeddedMATLABFunction_j5);

    /* MATLAB Function: '<S290>/negprotect' */
    controlMCUSlugsMKIIN_negprotect
      (controlMCUSlugsMKIINewNav_B.sf_EmbeddedMATLABFunction_j5.xDoty,
       &controlMCUSlugsMKIINewNav_B.sf_negprotect_k);

    /* S-Function "dsPIC_C_function_Call" Block: <S290>/C Function Call */
    controlMCUSlugsMKIINewNav_B.CFunctionCall_ml = mySqrt
      (controlMCUSlugsMKIINewNav_B.sf_negprotect_k.zpVal);

    /* Saturate: '<S279>/Zero Bound' */
    rtb_Merge_idx = controlMCUSlugsMKIINewNav_B.CFunctionCall_ml >= 0.001F ?
      controlMCUSlugsMKIINewNav_B.CFunctionCall_ml : 0.001F;

    /* Product: '<S279>/Divide' */
    rtb_Divide_h[0] = controlMCUSlugsMKIINewNav_B.sf_SelectNTerms.N[0] /
      rtb_Merge_idx;
    rtb_Divide_h[1] = controlMCUSlugsMKIINewNav_B.sf_SelectNTerms.N[1] /
      rtb_Merge_idx;
    rtb_Divide_h[2] = controlMCUSlugsMKIINewNav_B.sf_SelectNTerms.N[2] /
      rtb_Merge_idx;

    /* Delay: '<S251>/Integer Delay1' */
    controlMCUSlugsMKIINewNav_B.Merge1 =
      controlMCUSlugsMKIINewNav_DWork.IntegerDelay1_DSTATE_p;

    /* MATLAB Function: '<S251>/Zero out Z1' */
    controlMCUSlugsMKIINe_ZerooutZ1(rtb_Divide_h,
      &controlMCUSlugsMKIINewNav_B.sf_ZerooutZ1_l);

    /* MATLAB Function: '<S251>/Zero out Z2' */
    controlMCUSlugsMKIINe_ZerooutZ1(rtb_Divide_d5,
      &controlMCUSlugsMKIINewNav_B.sf_ZerooutZ2_k);

    /* MATLAB Function: '<S251>/Zero out Z3' */
    controlMCUSlugsMKIINe_ZerooutZ1(controlMCUSlugsMKIINewNav_B.UEN2NEU_l,
      &controlMCUSlugsMKIINewNav_B.sf_ZerooutZ3);

    /* DataTypeConversion: '<S297>/Data Type Conversion' */
    rtb_DataTypeConversion_i = (real_T)controlMCUSlugsMKIINewNav_B.WP0;

    /* MATLAB Function: '<S297>/Zero out Z1' */
    /* MATLAB Function: '<S297>/Zero out Z2' */
    /* MATLAB Function: '<S297>/Zero out Z3' */
    /* DataTypeConversion: '<S314>/Data Type Conversion' */
    rtb_DataTypeConversion_m = (real_T)controlMCUSlugsMKIINewNav_B.WP1;

    /* MATLAB Function: '<S314>/Zero out Z1' */
    /* MATLAB Function: '<S314>/Zero out Z2' */
    /* Product: '<S251>/Product1' */
    controlMCUSlugsMKIINewNav_B.Product1_e =
      controlMCUSlugsMKIINewNav_B.GetRangeofValuesnavSupportc_k[1] *
      controlMCUSlugsMKIINewNav_B.CFunctionCall_a;

    /* Update for Delay: '<S251>/Integer Delay1' */
    controlMCUSlugsMKIINewNav_DWork.IntegerDelay1_DSTATE_p =
      controlMCUSlugsMKIINewNav_B.UEN2NEU_l[2];

    /* End of Outputs for SubSystem: '<S135>/Get Frenet' */

    /* Sum: '<S249>/Subtract' */
    rtb_Subtract_c[0] = controlMCUSlugsMKIINewNav_B.sf_ZerooutZ2.P[0] -
      controlMCUSlugsMKIINewNav_B.sf_ZerooutZ3.P[0];
    rtb_Subtract_c[1] = controlMCUSlugsMKIINewNav_B.sf_ZerooutZ2.P[1] -
      controlMCUSlugsMKIINewNav_B.sf_ZerooutZ3.P[1];
    rtb_Subtract_c[2] = controlMCUSlugsMKIINewNav_B.sf_ZerooutZ2.P[2] -
      controlMCUSlugsMKIINewNav_B.sf_ZerooutZ3.P[2];

    /* MATLAB Function: '<S257>/Embedded MATLAB Function' */
    contro_EmbeddedMATLABFunction_d(rtb_Subtract_c,
      controlMCUSlugsMKIINewNav_B.sf_ZerooutZ1_l.P,
      &controlMCUSlugsMKIINewNav_B.sf_EmbeddedMATLABFunction_bh);

    /* S-Function "dsPIC_C_function_Call" Block: <S258>/C Function Call */
    controlMCUSlugsMKIINewNav_B.CFunctionCall_bg = myAbs
      (controlMCUSlugsMKIINewNav_B.sf_EmbeddedMATLABFunction_bh.xDoty);

    /* Sum: '<S249>/Add' incorporates:
     *  Product: '<S249>/Product'
     *  Product: '<S249>/Product1'
     */
    rtb_Add_cj = Product * Product -
      controlMCUSlugsMKIINewNav_B.CFunctionCall_bg *
      controlMCUSlugsMKIINewNav_B.CFunctionCall_bg;

    /* Sum: '<S256>/Subtract' */
    rtb_Subtract_d[0] = controlMCUSlugsMKIINewNav_B.sf_ZerooutZ3.P[0] -
      controlMCUSlugsMKIINewNav_B.sf_ZerooutZ2.P[0];
    rtb_Subtract_d[1] = controlMCUSlugsMKIINewNav_B.sf_ZerooutZ3.P[1] -
      controlMCUSlugsMKIINewNav_B.sf_ZerooutZ2.P[1];
    rtb_Subtract_d[2] = controlMCUSlugsMKIINewNav_B.sf_ZerooutZ3.P[2] -
      controlMCUSlugsMKIINewNav_B.sf_ZerooutZ2.P[2];

    /* MATLAB Function: '<S264>/Embedded MATLAB Function' */
    contro_EmbeddedMATLABFunction_d(rtb_Subtract_d,
      controlMCUSlugsMKIINewNav_B.sf_ZerooutZ2_k.P,
      &controlMCUSlugsMKIINewNav_B.sf_EmbeddedMATLABFunction_do);

    /* Switch: '<S263>/Switch3' incorporates:
     *  Delay: '<S263>/Integer Delay3'
     *  RelationalOperator: '<S263>/Relational Operator2'
     */
    if ((controlMCUSlugsMKIINewNav_B.sf_EmbeddedMATLABFunction_do.xDoty ==
         controlMCUSlugsMKIINewNav_B.sf_EmbeddedMATLABFunction_do.xDoty) > 0) {
      rtb_Deg2R1 =
        controlMCUSlugsMKIINewNav_B.sf_EmbeddedMATLABFunction_do.xDoty;
    } else {
      rtb_Deg2R1 = controlMCUSlugsMKIINewNav_DWork.IntegerDelay3_DSTATE_l;
    }

    /* End of Switch: '<S263>/Switch3' */

    /* Product: '<S249>/Divide1' */
    rtb_Merge_idx = Product;

    /* Product: '<S249>/Divide' incorporates:
     *  Constant: '<S249>/Constant4'
     */
    rtb_RhhcosphisinlambYe = controlMCUSlugsMKIINewNav_B.CFunctionCall_bg /
      0.577350259F;

    /* Switch: '<S262>/Switch' incorporates:
     *  Product: '<S249>/Divide1'
     *  RelationalOperator: '<S262>/Relational Operator'
     */
    if (!(Product < rtb_RhhcosphisinlambYe)) {
      rtb_Merge_idx = rtb_RhhcosphisinlambYe;
    }

    /* End of Switch: '<S262>/Switch' */

    /* MATLAB Function: '<S259>/negprotect' */
    controlMCUSlugsMKIIN_negprotect(rtb_Add_cj,
      &controlMCUSlugsMKIINewNav_B.sf_negprotect_d0);

    /* S-Function "dsPIC_C_function_Call" Block: <S259>/C Function Call */
    controlMCUSlugsMKIINewNav_B.CFunctionCall_b2 = mySqrt
      (controlMCUSlugsMKIINewNav_B.sf_negprotect_d0.zpVal);

    /* Switch: '<S135>/Switch' incorporates:
     *  Product: '<S249>/Product2'
     *  Sum: '<S249>/Add3'
     */
    if (controlMCUSlugsMKIINewNav_B.IC > 0) {
      /* Switch: '<S261>/Switch' incorporates:
       *  RelationalOperator: '<S249>/Relational Operator'
       *  RelationalOperator: '<S261>/Relational Operator'
       *  Switch: '<S249>/Switch1'
       */
      if ((!(controlMCUSlugsMKIINewNav_B.CFunctionCall_bg > Product)) &&
          (!(rtb_Merge_idx > controlMCUSlugsMKIINewNav_B.CFunctionCall_b2))) {
        rtb_Merge_idx = controlMCUSlugsMKIINewNav_B.CFunctionCall_b2;
      }

      /* End of Switch: '<S261>/Switch' */

      /* Sum: '<S249>/Add1' */
      rtb_Merge_idx = rtb_Deg2R1 - rtb_Merge_idx;

      /* Switch: '<S260>/Switch' incorporates:
       *  Constant: '<S249>/Constant1'
       *  RelationalOperator: '<S260>/Relational Operator'
       */
      if (!(rtb_Merge_idx > 0.0F)) {
        rtb_Merge_idx = 0.0F;
      }

      /* End of Switch: '<S260>/Switch' */
      controlMCUSlugsMKIINewNav_B.Merge[0] = (0.0F - rtb_Merge_idx *
        controlMCUSlugsMKIINewNav_B.sf_ZerooutZ2_k.P[0]) - rtb_Subtract_c[0];
      controlMCUSlugsMKIINewNav_B.Merge[1] = (0.0F - rtb_Merge_idx *
        controlMCUSlugsMKIINewNav_B.sf_ZerooutZ2_k.P[1]) - rtb_Subtract_c[1];
      controlMCUSlugsMKIINewNav_B.Merge[2] = (0.0F - rtb_Merge_idx *
        controlMCUSlugsMKIINewNav_B.sf_ZerooutZ2_k.P[2]) - rtb_Subtract_c[2];
    } else {
      controlMCUSlugsMKIINewNav_B.Merge[0] = rtb_PaHPPned[0];
      controlMCUSlugsMKIINewNav_B.Merge[1] = rtb_PaHPPned[1];
      controlMCUSlugsMKIINewNav_B.Merge[2] = rtb_PaHPPned[2];
    }

    /* End of Switch: '<S135>/Switch' */

    /* Switch: '<S135>/Switch1' */
    if (controlMCUSlugsMKIINewNav_B.IC > 0) {
      controlMCUSlugsMKIINewNav_B.Merge2 = rtb_Deg2R1;
    } else {
      controlMCUSlugsMKIINewNav_B.Merge2 =
        controlMCUSlugsMKIINewNav_B.CFunctionCall_f5;
    }

    /* End of Switch: '<S135>/Switch1' */

    /* Update for Delay: '<S135>/Integer Delay' incorporates:
     *  RelationalOperator: '<S249>/Switch Distance Less than?'
     */
    controlMCUSlugsMKIINewNav_DWork.IntegerDelay_DSTATE_p = (rtb_Deg2R1 <
      controlMCUSlugsMKIINewNav_B.Product1_e);

    /* Update for Delay: '<S263>/Integer Delay3' */
    controlMCUSlugsMKIINewNav_DWork.IntegerDelay3_DSTATE_l = rtb_Deg2R1;

    /* End of Outputs for SubSystem: '<S128>/Normal WP  Navigation' */
  } else if (controlMCUSlugsMKIINewNav_B.NavigationModenavSupportc_c == 10) {
    /* Outputs for IfAction SubSystem: '<S128>/Line Segment' incorporates:
     *  ActionPort: '<S133>/Action Port'
     */
    /* Gain: '<S133>/Gain' */
    controlMCUSlugsMKIINewNav_B.Merge[0] =
      -controlMCUSlugsMKIINewNav_B.sf_ZerooutZ2.P[0];
    controlMCUSlugsMKIINewNav_B.Merge[1] =
      -controlMCUSlugsMKIINewNav_B.sf_ZerooutZ2.P[1];
    controlMCUSlugsMKIINewNav_B.Merge[2] =
      -controlMCUSlugsMKIINewNav_B.sf_ZerooutZ2.P[2];

    /* Gain: '<S133>/Gain1' */
    rtb_Gain1[0] = -controlMCUSlugsMKIINewNav_B.sf_ZerooutZ2.P[0];
    rtb_Gain1[1] = -controlMCUSlugsMKIINewNav_B.sf_ZerooutZ2.P[1];
    rtb_Gain1[2] = -controlMCUSlugsMKIINewNav_B.sf_ZerooutZ2.P[2];

    /* MATLAB Function: '<S243>/Embedded MATLAB Function' */
    contro_EmbeddedMATLABFunction_o(rtb_Gain1,
      &controlMCUSlugsMKIINewNav_B.sf_EmbeddedMATLABFunction_dr);

    /* MATLAB Function: '<S244>/negprotect' */
    controlMCUSlugsMKIIN_negprotect
      (controlMCUSlugsMKIINewNav_B.sf_EmbeddedMATLABFunction_dr.xDoty,
       &controlMCUSlugsMKIINewNav_B.sf_negprotect_a);

    /* S-Function "dsPIC_C_function_Call" Block: <S244>/C Function Call */
    controlMCUSlugsMKIINewNav_B.CFunctionCall_b = mySqrt
      (controlMCUSlugsMKIINewNav_B.sf_negprotect_a.zpVal);

    /* SignalConversion: '<S247>/Numerical Unity' */
    controlMCUSlugsMKIINewNav_B.Merge2 =
      controlMCUSlugsMKIINewNav_B.CFunctionCall_b;
    controlMCUSlugsMKIINewNav_B.Merge1 =
      controlMCUSlugsMKIINewNav_B.GetMidLevelCommandsnavSupportc[1];

    /* End of Outputs for SubSystem: '<S128>/Line Segment' */
  } else {
    if (controlMCUSlugsMKIINewNav_B.NavigationModenavSupportc_c == 9) {
      /* Outputs for IfAction SubSystem: '<S128>/Circle Navigation' incorporates:
       *  ActionPort: '<S129>/Action Port'
       */
      /* Gain: '<S154>/Deg2R' */
      rtb_Deg2R_m_idx_0 = 0.0174532924F * controlMCUSlugsMKIINewNav_B.IC4[0];
      rtb_Deg2R_m_idx = 0.0174532924F * controlMCUSlugsMKIINewNav_B.IC4[1];

      /* Gain: '<S155>/Deg2R' */
      rtb_cosphi = 0.0174532924F *
        controlMCUSlugsMKIINewNav_B.GetISRLocationnavSupportc[0];

      /* Trigonometry: '<S155>/sin(phi)' */
      rtb_Ze = (real32_T)sin(rtb_cosphi);

      /* Sum: '<S155>/Sum1' incorporates:
       *  Constant: '<S155>/const'
       *  Product: '<S155>/Product1'
       *  Product: '<S155>/sin(phi)^2'
       */
      rtb_Merge_idx = 1.0F - rtb_Ze * rtb_Ze *
        controlMCUSlugsMKIINewNa_ConstB.Sum5_h;

      /* Fcn: '<S155>/f' */
      if (rtb_Merge_idx < 0.0F) {
        rtb_Product3_p4_idx = -(real32_T)sqrt(-rtb_Merge_idx);
      } else {
        rtb_Product3_p4_idx = (real32_T)sqrt(rtb_Merge_idx);
      }

      /* End of Fcn: '<S155>/f' */

      /* Product: '<S155>/Rh' incorporates:
       *  Constant: '<S155>/Re=equatorial radius'
       */
      rtb_Merge_idx = 6.378137E+6F / rtb_Product3_p4_idx;

      /* Sum: '<S155>/Sum2' */
      rtb_RhhcosphisinlambYe =
        controlMCUSlugsMKIINewNav_B.GetISRLocationnavSupportc[2] + rtb_Merge_idx;

      /* Trigonometry: '<S155>/cos(phi)' */
      rtb_cosphi = (real32_T)cos(rtb_cosphi);

      /* Gain: '<S155>/Deg2R1' */
      rtb_Deg2R1 = 0.0174532924F *
        controlMCUSlugsMKIINewNav_B.GetISRLocationnavSupportc[1];

      /* Product: '<S155>/(Rh+h)cos(phi)*cos(lamb)=Xe' incorporates:
       *  Trigonometry: '<S155>/cos(lamb)'
       */
      rtb_RhhcosphicoslambXe = rtb_RhhcosphisinlambYe * rtb_cosphi * (real32_T)
        cos(rtb_Deg2R1);

      /* Product: '<S155>/(Rh+h)cos(phi)*sin(lamb)=Ye' incorporates:
       *  Trigonometry: '<S155>/sin(lamb)'
       */
      rtb_RhhcosphisinlambYe = rtb_RhhcosphisinlambYe * rtb_cosphi * (real32_T)
        sin(rtb_Deg2R1);

      /* Product: '<S155>/Ze' incorporates:
       *  Product: '<S155>/Rh(1-e^2)'
       *  Sum: '<S155>/Sum4'
       */
      rtb_Ze *= controlMCUSlugsMKIINewNa_ConstB.e2_n * rtb_Merge_idx +
        controlMCUSlugsMKIINewNav_B.GetISRLocationnavSupportc[2];

      /* SignalConversion: '<S154>/TmpSignal ConversionAtProduct1Inport1' incorporates:
       *  Fcn: '<S157>/11'
       *  Fcn: '<S157>/12'
       *  Fcn: '<S157>/13'
       *  Fcn: '<S157>/21'
       *  Fcn: '<S157>/22'
       *  Fcn: '<S157>/31'
       *  Fcn: '<S157>/32'
       *  Fcn: '<S157>/33'
       */
      rtb_VectorConcatenate_i[0L] = (real32_T)cos(rtb_Deg2R_m_idx_0) * (real32_T)
        cos(rtb_Deg2R_m_idx);
      rtb_VectorConcatenate_i[1L] = -(real32_T)sin(rtb_Deg2R_m_idx_0);
      rtb_VectorConcatenate_i[2L] = -(real32_T)sin(rtb_Deg2R_m_idx) * (real32_T)
        cos(rtb_Deg2R_m_idx_0);
      rtb_VectorConcatenate_i[3L] = (real32_T)sin(rtb_Deg2R_m_idx_0) * (real32_T)
        cos(rtb_Deg2R_m_idx);
      rtb_VectorConcatenate_i[4L] = (real32_T)cos(rtb_Deg2R_m_idx_0);
      rtb_VectorConcatenate_i[5L] = -(real32_T)sin(rtb_Deg2R_m_idx_0) *
        (real32_T)sin(rtb_Deg2R_m_idx);
      rtb_VectorConcatenate_i[6L] = (real32_T)sin(rtb_Deg2R_m_idx);
      rtb_VectorConcatenate_i[7L] = 0.0F;
      rtb_VectorConcatenate_i[8L] = (real32_T)cos(rtb_Deg2R_m_idx);

      /* Sum: '<S152>/Sum1' */
      rtb_Deg2R_m_idx = rtb_RhhcosphicoslambXe -
        controlMCUSlugsMKIINewNav_B.DataTypeConversion1[0];
      rtb_Deg2R_m_idx_0 = rtb_RhhcosphisinlambYe -
        controlMCUSlugsMKIINewNav_B.DataTypeConversion1[1];
      rtb_Product3_p4_idx = rtb_Ze -
        controlMCUSlugsMKIINewNav_B.DataTypeConversion1[2];

      /* Product: '<S154>/Product1' incorporates:
       *  Gain: '<S152>/UEN 2 NEU'
       */
      for (i = 0; i < 3; i++) {
        tmp[i] = rtb_VectorConcatenate_i[i + 6] * rtb_Product3_p4_idx +
          (rtb_VectorConcatenate_i[i + 3] * rtb_Deg2R_m_idx_0 +
           rtb_VectorConcatenate_i[i] * rtb_Deg2R_m_idx);
      }

      /* End of Product: '<S154>/Product1' */

      /* Gain: '<S152>/UEN 2 NEU' */
      for (i = 0; i < 3; i++) {
        controlMCUSlugsMKIINewNav_B.UEN2NEU[i] = 0.0F;
        controlMCUSlugsMKIINewNav_B.UEN2NEU[i] =
          controlMCUSlugsMKIINewNa_ConstP.pooled45[i] * tmp[0] +
          controlMCUSlugsMKIINewNav_B.UEN2NEU[i];
        controlMCUSlugsMKIINewNav_B.UEN2NEU[i] =
          controlMCUSlugsMKIINewNa_ConstP.pooled45[i + 3] * tmp[1] +
          controlMCUSlugsMKIINewNav_B.UEN2NEU[i];
        controlMCUSlugsMKIINewNav_B.UEN2NEU[i] =
          controlMCUSlugsMKIINewNa_ConstP.pooled45[i + 6] * tmp[2] +
          controlMCUSlugsMKIINewNav_B.UEN2NEU[i];
      }

      /* Delay: '<S146>/Integer Delay1' */
      controlMCUSlugsMKIINewNav_B.Merge1 =
        controlMCUSlugsMKIINewNav_DWork.IntegerDelay1_DSTATE;

      /* MATLAB Function: '<S146>/Zero out Z2' */
      controlMCUSlugsMKIINe_ZerooutZ1(controlMCUSlugsMKIINewNav_B.UEN2NEU,
        &controlMCUSlugsMKIINewNav_B.sf_ZerooutZ2_a);

      /* Sum: '<S129>/Sum1' */
      rtb_Sum1_a[0] = controlMCUSlugsMKIINewNav_B.sf_ZerooutZ2.P[0] -
        controlMCUSlugsMKIINewNav_B.sf_ZerooutZ2_a.P[0];
      rtb_Sum1_a[1] = controlMCUSlugsMKIINewNav_B.sf_ZerooutZ2.P[1] -
        controlMCUSlugsMKIINewNav_B.sf_ZerooutZ2_a.P[1];
      rtb_Sum1_a[2] = controlMCUSlugsMKIINewNav_B.sf_ZerooutZ2.P[2] -
        controlMCUSlugsMKIINewNav_B.sf_ZerooutZ2_a.P[2];

      /* MATLAB Function: '<S198>/Embedded MATLAB Function' */
      contro_EmbeddedMATLABFunction_o(rtb_Sum1_a,
        &controlMCUSlugsMKIINewNav_B.sf_EmbeddedMATLABFunction_p);

      /* MATLAB Function: '<S199>/negprotect' */
      controlMCUSlugsMKIIN_negprotect
        (controlMCUSlugsMKIINewNav_B.sf_EmbeddedMATLABFunction_p.xDoty,
         &controlMCUSlugsMKIINewNav_B.sf_negprotect_dl);

      /* S-Function "dsPIC_C_function_Call" Block: <S199>/C Function Call */
      controlMCUSlugsMKIINewNav_B.CFunctionCall_mk = mySqrt
        (controlMCUSlugsMKIINewNav_B.sf_negprotect_dl.zpVal);

      /* Product: '<S129>/Product' */
      rtb_Merge_idx = controlMCUSlugsMKIINewNav_B.GetRangeofValuesnavSupportc_k
        [0] * controlMCUSlugsMKIINewNav_B.CFunctionCall_a;

      /* Sum: '<S129>/Sum2' */
      controlMCUSlugsMKIINewNav_B.Sum2 = Product - rtb_Merge_idx;

      /* S-Function "dsPIC_C_function_Call" Block: <S151>/C Function Call */
      controlMCUSlugsMKIINewNav_B.CFunctionCall_ox = myAbs
        (controlMCUSlugsMKIINewNav_B.Sum2);

      /* If: '<S129>/If' incorporates:
       *  Product: '<S148>/Product7'
       *  Sum: '<S129>/Sum'
       *  Sum: '<S148>/Subtract3'
       */
      if (controlMCUSlugsMKIINewNav_B.CFunctionCall_mk > rtb_Merge_idx + Product)
      {
        /* Outputs for IfAction SubSystem: '<S129>/No intersection,  Navigate to ISR' incorporates:
         *  ActionPort: '<S149>/Action Port'
         */
        /* Sum: '<S149>/Subtract' */
        controlMCUSlugsMKIINewNav_B.Merge[0] =
          controlMCUSlugsMKIINewNav_B.sf_ZerooutZ2_a.P[0] -
          controlMCUSlugsMKIINewNav_B.sf_ZerooutZ2.P[0];
        controlMCUSlugsMKIINewNav_B.Merge[1] =
          controlMCUSlugsMKIINewNav_B.sf_ZerooutZ2_a.P[1] -
          controlMCUSlugsMKIINewNav_B.sf_ZerooutZ2.P[1];
        controlMCUSlugsMKIINewNav_B.Merge[2] =
          controlMCUSlugsMKIINewNav_B.sf_ZerooutZ2_a.P[2] -
          controlMCUSlugsMKIINewNav_B.sf_ZerooutZ2.P[2];

        /* End of Outputs for SubSystem: '<S129>/No intersection,  Navigate to ISR' */
      } else if (controlMCUSlugsMKIINewNav_B.CFunctionCall_mk <
                 controlMCUSlugsMKIINewNav_B.CFunctionCall_ox) {
        /* Outputs for IfAction SubSystem: '<S129>/Inside the Circle,  Keep Straight until  intersection' incorporates:
         *  ActionPort: '<S147>/Action Port'
         */
        /* Sum: '<S147>/Subtract' incorporates:
         *  MATLAB Function: '<S147>/Compute Head of Circle'
         */
        /* MATLAB Function 'Navigation/Navigation/Circle Navigation/Inside the Circle,  Keep Straight until  intersection/Compute Head of Circle': '<S159>:1' */
        /* % Compute the top coordinate of the circle */
        /*  using the circle's parametric equations: */
        /*   x = a + r cos (t) */
        /*   y = b + r sin (t) */
        /*  */
        /*  @ t =0 */
        /* '<S159>:1:9' */
        /* '<S159>:1:11' */
        /* '<S159>:1:12' */
        /* '<S159>:1:13' */
        controlMCUSlugsMKIINewNav_B.Merge[0] = (real32_T)((real_T)
          (controlMCUSlugsMKIINewNav_B.sf_ZerooutZ2_a.P[0] + Product) - (real_T)
          controlMCUSlugsMKIINewNav_B.sf_ZerooutZ2.P[0]);
        controlMCUSlugsMKIINewNav_B.Merge[1] = (real32_T)((real_T)
          controlMCUSlugsMKIINewNav_B.sf_ZerooutZ2_a.P[1] - (real_T)
          controlMCUSlugsMKIINewNav_B.sf_ZerooutZ2.P[1]);
        controlMCUSlugsMKIINewNav_B.Merge[2] = (real32_T)((real_T)
          controlMCUSlugsMKIINewNav_B.sf_ZerooutZ2_a.P[2] - (real_T)
          controlMCUSlugsMKIINewNav_B.sf_ZerooutZ2.P[2]);

        /* End of Outputs for SubSystem: '<S129>/Inside the Circle,  Keep Straight until  intersection' */
      } else {
        /* Outputs for IfAction SubSystem: '<S129>/Intersection. Circular Navigation' incorporates:
         *  ActionPort: '<S148>/Action Port'
         */
        /* Product: '<S148>/Product' */
        rtb_Deg2R1 = Product * Product;

        /* Product: '<S148>/Product2' */
        rtb_RhhcosphisinlambYe = controlMCUSlugsMKIINewNav_B.CFunctionCall_mk *
          controlMCUSlugsMKIINewNav_B.CFunctionCall_mk;

        /* Sum: '<S148>/Subtract1' */
        rtb_Subtract1_o = controlMCUSlugsMKIINewNav_B.CFunctionCall_mk +
          controlMCUSlugsMKIINewNav_B.CFunctionCall_mk;

        /* MATLAB Function: '<S148>/negprotect2' */
        controlMCUSlugsMKII_negprotect3(rtb_Subtract1_o,
          &controlMCUSlugsMKIINewNav_B.sf_negprotect2);

        /* Product: '<S148>/Product3' incorporates:
         *  Product: '<S148>/Product1'
         *  Sum: '<S148>/Subtract'
         */
        rtb_Merge_idx = ((rtb_Deg2R1 - rtb_Merge_idx * rtb_Merge_idx) +
                         rtb_RhhcosphisinlambYe) /
          controlMCUSlugsMKIINewNav_B.sf_negprotect2.zpVal;

        /* Sum: '<S148>/Subtract2' incorporates:
         *  Product: '<S148>/Product4'
         */
        rtb_h = rtb_Deg2R1 - rtb_Merge_idx * rtb_Merge_idx;

        /* MATLAB Function: '<S165>/negprotect' */
        controlMCUSlugsMKIIN_negprotect(rtb_h,
          &controlMCUSlugsMKIINewNav_B.sf_negprotect_dz);

        /* S-Function "dsPIC_C_function_Call" Block: <S165>/C Function Call */
        controlMCUSlugsMKIINewNav_B.CFunctionCall_p5 = mySqrt
          (controlMCUSlugsMKIINewNav_B.sf_negprotect_dz.zpVal);

        /* MATLAB Function: '<S148>/negprotect1' */
        controlMCUSlugsMKII_negprotect3
          (controlMCUSlugsMKIINewNav_B.CFunctionCall_mk,
           &controlMCUSlugsMKIINewNav_B.sf_negprotect1);

        /* Product: '<S148>/Product6' */
        rtb_Product6[0] = rtb_Sum1_a[0] /
          controlMCUSlugsMKIINewNav_B.sf_negprotect1.zpVal;
        rtb_Product6[1] = rtb_Sum1_a[1] /
          controlMCUSlugsMKIINewNav_B.sf_negprotect1.zpVal;
        rtb_Product6[2] = rtb_Sum1_a[2] /
          controlMCUSlugsMKIINewNav_B.sf_negprotect1.zpVal;

        /* Product: '<S148>/Product5' */
        rtb_Product5[0] = controlMCUSlugsMKIINewNav_B.CFunctionCall_p5 *
          rtb_Product6[0];
        rtb_Product5[1] = controlMCUSlugsMKIINewNav_B.CFunctionCall_p5 *
          rtb_Product6[1];
        rtb_Product5[2] = controlMCUSlugsMKIINewNav_B.CFunctionCall_p5 *
          rtb_Product6[2];
        rtb_Product6[0] = rtb_Product6[0] * rtb_Merge_idx +
          controlMCUSlugsMKIINewNav_B.sf_ZerooutZ2_a.P[0];
        rtb_Product6[1] = rtb_Product6[1] * rtb_Merge_idx +
          controlMCUSlugsMKIINewNav_B.sf_ZerooutZ2_a.P[1];
        rtb_Product6[2] = rtb_Product6[2] * rtb_Merge_idx +
          controlMCUSlugsMKIINewNav_B.sf_ZerooutZ2_a.P[2];

        /* Sum: '<S148>/Subtract6' incorporates:
         *  MATLAB Function: '<S148>/Embedded MATLAB Function'
         *  Product: '<S148>/Product7'
         *  Sum: '<S148>/Subtract3'
         */
        /* MATLAB Function 'Navigation/Navigation/Circle Navigation/Intersection. Circular Navigation/Embedded MATLAB Function': '<S162>:1' */
        /*  This block supports an embeddable subset of the MATLAB language. */
        /*  See the help menu for details.  */
        /* '<S162>:1:5' */
        /* '<S162>:1:6' */
        /* '<S162>:1:8' */
        /* '<S162>:1:9' */
        /* '<S162>:1:11' */
        /* '<S162>:1:12' */
        rtb_Subtract6[0] = (rtb_Product6[0] - rtb_Product5[1]) -
          controlMCUSlugsMKIINewNav_B.sf_ZerooutZ2.P[0];
        rtb_Subtract6[1] = (rtb_Product6[1] + rtb_Product5[0]) -
          controlMCUSlugsMKIINewNav_B.sf_ZerooutZ2.P[1];
        rtb_Subtract6[2] = 0.0F - controlMCUSlugsMKIINewNav_B.sf_ZerooutZ2.P[2];

        /* MATLAB Function: '<S169>/Embedded MATLAB Function' */
        contro_EmbeddedMATLABFunction_d
          (controlMCUSlugsMKIINewNav_B.sf_ZerooutZ1.P, rtb_Subtract6,
           &controlMCUSlugsMKIINewNav_B.sf_EmbeddedMATLABFunction_ls);

        /* MATLAB Function: '<S172>/Embedded MATLAB Function' */
        contro_EmbeddedMATLABFunction_o(rtb_Subtract6,
          &controlMCUSlugsMKIINewNav_B.sf_EmbeddedMATLABFunction_eq);

        /* MATLAB Function: '<S173>/negprotect' */
        controlMCUSlugsMKIIN_negprotect
          (controlMCUSlugsMKIINewNav_B.sf_EmbeddedMATLABFunction_eq.xDoty,
           &controlMCUSlugsMKIINewNav_B.sf_negprotect_ht);

        /* S-Function "dsPIC_C_function_Call" Block: <S173>/C Function Call */
        controlMCUSlugsMKIINewNav_B.CFunctionCall_op = mySqrt
          (controlMCUSlugsMKIINewNav_B.sf_negprotect_ht.zpVal);

        /* Product: '<S160>/Product9' */
        rtb_Product9 = controlMCUSlugsMKIINewNav_B.CFunctionCall_op *
          controlMCUSlugsMKIINewNav_B.CFunctionCall_a;

        /* MATLAB Function: '<S160>/negprotect3' */
        controlMCUSlugsMKII_negprotect3(rtb_Product9,
          &controlMCUSlugsMKIINewNav_B.sf_negprotect3);

        /* Product: '<S160>/Product8' */
        rtb_Product3_p4_idx =
          controlMCUSlugsMKIINewNav_B.sf_EmbeddedMATLABFunction_ls.xDoty /
          controlMCUSlugsMKIINewNav_B.sf_negprotect3.zpVal;

        /* Saturate: '<S160>/[-1 1]' */
        controlMCUSlugsMKIINewNav_B.u1_h = rtb_Product3_p4_idx >= 1.0F ? 1.0F :
          rtb_Product3_p4_idx <= -1.0F ? -1.0F : rtb_Product3_p4_idx;

        /* S-Function "dsPIC_C_function_Call" Block: <S170>/C Function Call */
        controlMCUSlugsMKIINewNav_B.CFunctionCall_l = myAcos
          (controlMCUSlugsMKIINewNav_B.u1_h);

        /* Sum: '<S148>/Subtract5' incorporates:
         *  MATLAB Function: '<S148>/Embedded MATLAB Function'
         */
        rtb_Subtract5[0] = (rtb_Product6[0] + rtb_Product5[1]) -
          controlMCUSlugsMKIINewNav_B.sf_ZerooutZ2.P[0];
        rtb_Subtract5[1] = (rtb_Product6[1] - rtb_Product5[0]) -
          controlMCUSlugsMKIINewNav_B.sf_ZerooutZ2.P[1];
        rtb_Subtract5[2] = 0.0F - controlMCUSlugsMKIINewNav_B.sf_ZerooutZ2.P[2];

        /* MATLAB Function: '<S182>/Embedded MATLAB Function' */
        contro_EmbeddedMATLABFunction_d
          (controlMCUSlugsMKIINewNav_B.sf_ZerooutZ1.P, rtb_Subtract5,
           &controlMCUSlugsMKIINewNav_B.sf_EmbeddedMATLABFunction_ip);

        /* MATLAB Function: '<S185>/Embedded MATLAB Function' */
        contro_EmbeddedMATLABFunction_o(rtb_Subtract5,
          &controlMCUSlugsMKIINewNav_B.sf_EmbeddedMATLABFunction_py);

        /* MATLAB Function: '<S186>/negprotect' */
        controlMCUSlugsMKIIN_negprotect
          (controlMCUSlugsMKIINewNav_B.sf_EmbeddedMATLABFunction_py.xDoty,
           &controlMCUSlugsMKIINewNav_B.sf_negprotect_l0);

        /* S-Function "dsPIC_C_function_Call" Block: <S186>/C Function Call */
        controlMCUSlugsMKIINewNav_B.CFunctionCall_aw = mySqrt
          (controlMCUSlugsMKIINewNav_B.sf_negprotect_l0.zpVal);

        /* Product: '<S161>/Product9' */
        rtb_Product9_j = controlMCUSlugsMKIINewNav_B.CFunctionCall_aw *
          controlMCUSlugsMKIINewNav_B.CFunctionCall_a;

        /* MATLAB Function: '<S161>/negprotect3' */
        controlMCUSlugsMKII_negprotect3(rtb_Product9_j,
          &controlMCUSlugsMKIINewNav_B.sf_negprotect3_b);

        /* Product: '<S161>/Product8' */
        rtb_Product3_p4_idx =
          controlMCUSlugsMKIINewNav_B.sf_EmbeddedMATLABFunction_ip.xDoty /
          controlMCUSlugsMKIINewNav_B.sf_negprotect3_b.zpVal;

        /* Saturate: '<S161>/[-1 1]' */
        controlMCUSlugsMKIINewNav_B.u1_m = rtb_Product3_p4_idx >= 1.0F ? 1.0F :
          rtb_Product3_p4_idx <= -1.0F ? -1.0F : rtb_Product3_p4_idx;

        /* S-Function "dsPIC_C_function_Call" Block: <S183>/C Function Call */
        controlMCUSlugsMKIINewNav_B.CFunctionCall_j = myAcos
          (controlMCUSlugsMKIINewNav_B.u1_m);

        /* S-Function "dsPIC_C_function_Call" Block: <S164>/C Function Call */
        controlMCUSlugsMKIINewNav_B.CFunctionCall_pg = myAbs
          (controlMCUSlugsMKIINewNav_B.CFunctionCall_j);

        /* S-Function "dsPIC_C_function_Call" Block: <S163>/C Function Call */
        controlMCUSlugsMKIINewNav_B.CFunctionCall_n = myAbs
          (controlMCUSlugsMKIINewNav_B.CFunctionCall_l);

        /* Switch: '<S148>/Switch1' incorporates:
         *  RelationalOperator: '<S148>/Relational Operator'
         */
        if (controlMCUSlugsMKIINewNav_B.CFunctionCall_pg <=
            controlMCUSlugsMKIINewNav_B.CFunctionCall_n) {
          controlMCUSlugsMKIINewNav_B.Merge[0] = rtb_Subtract5[0];
          controlMCUSlugsMKIINewNav_B.Merge[1] = rtb_Subtract5[1];
          controlMCUSlugsMKIINewNav_B.Merge[2] = rtb_Subtract5[2];
        } else {
          controlMCUSlugsMKIINewNav_B.Merge[0] = rtb_Subtract6[0];
          controlMCUSlugsMKIINewNav_B.Merge[1] = rtb_Subtract6[1];
          controlMCUSlugsMKIINewNav_B.Merge[2] = rtb_Subtract6[2];
        }

        /* End of Switch: '<S148>/Switch1' */
        /* End of Outputs for SubSystem: '<S129>/Intersection. Circular Navigation' */
      }

      /* End of If: '<S129>/If' */

      /* Delay: '<S129>/Integer Delay' */
      controlMCUSlugsMKIINewNav_B.Merge2 =
        controlMCUSlugsMKIINewNav_DWork.IntegerDelay_DSTATE;

      /* Update for Delay: '<S146>/Integer Delay1' */
      controlMCUSlugsMKIINewNav_DWork.IntegerDelay1_DSTATE =
        controlMCUSlugsMKIINewNav_B.UEN2NEU[2];

      /* Update for Delay: '<S129>/Integer Delay' */
      controlMCUSlugsMKIINewNav_DWork.IntegerDelay_DSTATE = Product;

      /* End of Outputs for SubSystem: '<S128>/Circle Navigation' */
    }
  }

  /* End of If: '<S128>/Determine Overall Nav by the Nav Mode' */

  /* MATLAB Function: '<S225>/Zero out Z1' */
  controlMCUSlugsMKIINe_ZerooutZ1(controlMCUSlugsMKIINewNav_B.Merge,
    &controlMCUSlugsMKIINewNav_B.sf_ZerooutZ1_g);

  /* MATLAB Function: '<S230>/Embedded MATLAB Function' */
  contro_EmbeddedMATLABFunction_o(controlMCUSlugsMKIINewNav_B.sf_ZerooutZ1_g.P,
    &controlMCUSlugsMKIINewNav_B.sf_EmbeddedMATLABFunction_dv);

  /* MATLAB Function: '<S231>/negprotect' */
  controlMCUSlugsMKIIN_negprotect
    (controlMCUSlugsMKIINewNav_B.sf_EmbeddedMATLABFunction_dv.xDoty,
     &controlMCUSlugsMKIINewNav_B.sf_negprotect_i);

  /* S-Function "dsPIC_C_function_Call" Block: <S231>/C Function Call */
  controlMCUSlugsMKIINewNav_B.CFunctionCall_p = mySqrt
    (controlMCUSlugsMKIINewNav_B.sf_negprotect_i.zpVal);

  /* Product: '<S217>/Product1' */
  rtb_Product1_l = controlMCUSlugsMKIINewNav_B.CFunctionCall_p *
    controlMCUSlugsMKIINewNav_B.CFunctionCall_a;

  /* MATLAB Function: '<S217>/negprotect' */
  controlMCUSlugsMKII_negprotect3(rtb_Product1_l,
    &controlMCUSlugsMKIINewNav_B.sf_negprotect_d);

  /* DeadZone: '<S217>/Dead Zone' */
  if (controlMCUSlugsMKIINewNav_B.sf_negprotect_d.zpVal > 0.1F) {
    rtb_Product3_p4_idx = controlMCUSlugsMKIINewNav_B.sf_negprotect_d.zpVal -
      0.1F;
  } else if (controlMCUSlugsMKIINewNav_B.sf_negprotect_d.zpVal >= -0.1F) {
    rtb_Product3_p4_idx = 0.0F;
  } else {
    rtb_Product3_p4_idx = controlMCUSlugsMKIINewNav_B.sf_negprotect_d.zpVal -
      -0.1F;
  }

  /* End of DeadZone: '<S217>/Dead Zone' */

  /* Switch: '<S217>/Switch' incorporates:
   *  Constant: '<S217>/cos(pi//2)'
   *  Constant: '<S217>/sin(pi//2)'
   *  Constant: '<S224>/Constant'
   *  Product: '<S217>/Divide1'
   *  Product: '<S217>/Divide2'
   *  Product: '<S223>/Product'
   *  Product: '<S223>/Product1'
   *  Product: '<S226>/Product'
   *  Product: '<S226>/Product1'
   *  RelationalOperator: '<S224>/Compare'
   *  Sum: '<S223>/Subtract'
   *  Sum: '<S226>/Subtract'
   *  Switch: '<S217>/Switch2'
   */
  if ((rtb_Product3_p4_idx == 0.0F) > 0) {
    controlMCUSlugsMKIINewNav_B.Switch = 1.0F;
    controlMCUSlugsMKIINewNav_B.Switch2 = 0.0F;
  } else {
    controlMCUSlugsMKIINewNav_B.Switch = (controlMCUSlugsMKIINewNav_B.Merge[1] *
      controlMCUSlugsMKIINewNav_B.sf_ZerooutZ1.P[0] -
      controlMCUSlugsMKIINewNav_B.Merge[0] *
      controlMCUSlugsMKIINewNav_B.sf_ZerooutZ1.P[1]) /
      controlMCUSlugsMKIINewNav_B.sf_negprotect_d.zpVal;
    controlMCUSlugsMKIINewNav_B.Switch2 = (controlMCUSlugsMKIINewNav_B.Merge[0] *
      controlMCUSlugsMKIINewNav_B.sf_ZerooutZ1.P[0] +
      controlMCUSlugsMKIINewNav_B.Merge[1] *
      controlMCUSlugsMKIINewNav_B.sf_ZerooutZ1.P[1]) * (1.0F /
      controlMCUSlugsMKIINewNav_B.sf_negprotect_d.zpVal);
  }

  /* End of Switch: '<S217>/Switch' */
  /* S-Function "dsPIC_C_function_Call" Block: <S221>/C Function Call */
  controlMCUSlugsMKIINewNav_B.CFunctionCall_k = myAtan2
    (controlMCUSlugsMKIINewNav_B.Switch,controlMCUSlugsMKIINewNav_B.Switch2);

  /* MATLAB Function: '<S128>/myMux Fun1' incorporates:
   *  DataTypeConversion: '<S128>/Data Type Conversion5'
   *  DataTypeConversion: '<S128>/Data Type Conversion6'
   *  Delay: '<S128>/Integer Delay'
   *  Delay: '<S128>/Integer Delay1'
   *  Gain: '<S132>/Rad2Deg'
   */
  /* MATLAB Function 'Navigation/Navigation/myMux Fun1': '<S140>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S140>:1:4' */
  controlMCUSlugsMKIINewNav_B.y_k[0] = 57.2957802F *
    controlMCUSlugsMKIINewNav_B.CFunctionCall_k;
  controlMCUSlugsMKIINewNav_B.y_k[1] = controlMCUSlugsMKIINewNav_B.Merge2;
  controlMCUSlugsMKIINewNav_B.y_k[2] = (real32_T)
    controlMCUSlugsMKIINewNav_DWork.IntegerDelay_DSTATE_d;
  controlMCUSlugsMKIINewNav_B.y_k[3] = (real32_T)
    controlMCUSlugsMKIINewNav_DWork.IntegerDelay1_DSTATE_j2;

  /* S-Function "dsPIC_C_function_Call" Block: <S128>/Get XYZ [navSupport.c] */
  setNavNav(controlMCUSlugsMKIINewNav_B.y_k);

  /* MATLAB Function: '<S128>/myMux Fun5' */
  /* MATLAB Function 'Navigation/Navigation/myMux Fun5': '<S144>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S144>:1:5' */
  controlMCUSlugsMKIINewNav_B.y_e[0] = controlMCUSlugsMKIINewNav_B.IC4[0];
  controlMCUSlugsMKIINewNav_B.y_e[1] = controlMCUSlugsMKIINewNav_B.IC4[1];
  controlMCUSlugsMKIINewNav_B.y_e[2] = (real32_T)0.0;

  /* S-Function "dsPIC_C_function_Call" Block: <S128>/Log 1 [navSupport.c] */
  setLogFloat1(controlMCUSlugsMKIINewNav_B.y_e);

  /* Gain: '<S205>/Deg2R' */
  rtb_Deg2R_m_idx_0 = 0.0174532924F * controlMCUSlugsMKIINewNav_B.IC4[0];
  rtb_Deg2R_m_idx = 0.0174532924F * controlMCUSlugsMKIINewNav_B.IC4[1];

  /* Gain: '<S206>/Deg2R' */
  rtb_Ze = 0.0174532924F *
    controlMCUSlugsMKIINewNav_B.GetISRLocationnavSupportc[0];

  /* Trigonometry: '<S206>/sin(phi)' */
  rtb_cosphi = (real32_T)sin(rtb_Ze);

  /* Sum: '<S206>/Sum1' incorporates:
   *  Constant: '<S206>/const'
   *  Product: '<S206>/Product1'
   *  Product: '<S206>/sin(phi)^2'
   */
  rtb_Merge_idx = 1.0F - rtb_cosphi * rtb_cosphi *
    controlMCUSlugsMKIINewNa_ConstB.Sum5;

  /* Fcn: '<S206>/f' */
  if (rtb_Merge_idx < 0.0F) {
    rtb_Product3_p4_idx = -(real32_T)sqrt(-rtb_Merge_idx);
  } else {
    rtb_Product3_p4_idx = (real32_T)sqrt(rtb_Merge_idx);
  }

  /* End of Fcn: '<S206>/f' */

  /* Product: '<S206>/Rh' incorporates:
   *  Constant: '<S206>/Re=equatorial radius'
   */
  rtb_Merge_idx = 6.378137E+6F / rtb_Product3_p4_idx;

  /* Sum: '<S206>/Sum2' */
  rtb_RhhcosphisinlambYe =
    controlMCUSlugsMKIINewNav_B.GetISRLocationnavSupportc[2] + rtb_Merge_idx;

  /* Trigonometry: '<S206>/cos(phi)' */
  rtb_Ze = (real32_T)cos(rtb_Ze);

  /* Gain: '<S206>/Deg2R1' */
  rtb_Deg2R1 = 0.0174532924F *
    controlMCUSlugsMKIINewNav_B.GetISRLocationnavSupportc[1];

  /* Product: '<S206>/(Rh+h)cos(phi)*cos(lamb)=Xe' incorporates:
   *  Trigonometry: '<S206>/cos(lamb)'
   */
  rtb_RhhcosphicoslambXe = rtb_RhhcosphisinlambYe * rtb_Ze * (real32_T)cos
    (rtb_Deg2R1);

  /* Product: '<S206>/(Rh+h)cos(phi)*sin(lamb)=Ye' incorporates:
   *  Trigonometry: '<S206>/sin(lamb)'
   */
  rtb_RhhcosphisinlambYe = rtb_RhhcosphisinlambYe * rtb_Ze * (real32_T)sin
    (rtb_Deg2R1);

  /* Product: '<S206>/Ze' incorporates:
   *  Product: '<S206>/Rh(1-e^2)'
   *  Sum: '<S206>/Sum4'
   */
  rtb_cosphi *= controlMCUSlugsMKIINewNa_ConstB.e2 * rtb_Merge_idx +
    controlMCUSlugsMKIINewNav_B.GetISRLocationnavSupportc[2];

  /* SignalConversion: '<S205>/TmpSignal ConversionAtProduct1Inport1' incorporates:
   *  Fcn: '<S208>/11'
   *  Fcn: '<S208>/12'
   *  Fcn: '<S208>/13'
   *  Fcn: '<S208>/21'
   *  Fcn: '<S208>/22'
   *  Fcn: '<S208>/31'
   *  Fcn: '<S208>/32'
   *  Fcn: '<S208>/33'
   */
  rtb_VectorConcatenate_i[0L] = (real32_T)cos(rtb_Deg2R_m_idx_0) * (real32_T)cos
    (rtb_Deg2R_m_idx);
  rtb_VectorConcatenate_i[1L] = -(real32_T)sin(rtb_Deg2R_m_idx_0);
  rtb_VectorConcatenate_i[2L] = -(real32_T)sin(rtb_Deg2R_m_idx) * (real32_T)cos
    (rtb_Deg2R_m_idx_0);
  rtb_VectorConcatenate_i[3L] = (real32_T)sin(rtb_Deg2R_m_idx_0) * (real32_T)cos
    (rtb_Deg2R_m_idx);
  rtb_VectorConcatenate_i[4L] = (real32_T)cos(rtb_Deg2R_m_idx_0);
  rtb_VectorConcatenate_i[5L] = -(real32_T)sin(rtb_Deg2R_m_idx_0) * (real32_T)
    sin(rtb_Deg2R_m_idx);
  rtb_VectorConcatenate_i[6L] = (real32_T)sin(rtb_Deg2R_m_idx);
  rtb_VectorConcatenate_i[7L] = 0.0F;
  rtb_VectorConcatenate_i[8L] = (real32_T)cos(rtb_Deg2R_m_idx);

  /* Sum: '<S130>/Sum1' */
  rtb_Deg2R_m_idx = rtb_RhhcosphicoslambXe -
    controlMCUSlugsMKIINewNav_B.DataTypeConversion1[0];
  rtb_Deg2R_m_idx_0 = rtb_RhhcosphisinlambYe -
    controlMCUSlugsMKIINewNav_B.DataTypeConversion1[1];
  rtb_Product3_p4_idx = rtb_cosphi -
    controlMCUSlugsMKIINewNav_B.DataTypeConversion1[2];

  /* Product: '<S205>/Product1' incorporates:
   *  Gain: '<S130>/UEN 2 NEU'
   */
  for (i = 0; i < 3; i++) {
    tmp[i] = rtb_VectorConcatenate_i[i + 6] * rtb_Product3_p4_idx +
      (rtb_VectorConcatenate_i[i + 3] * rtb_Deg2R_m_idx_0 +
       rtb_VectorConcatenate_i[i] * rtb_Deg2R_m_idx);
  }

  /* End of Product: '<S205>/Product1' */

  /* Gain: '<S130>/UEN 2 NEU' */
  for (i = 0; i < 3; i++) {
    rtb_Product6[i] = controlMCUSlugsMKIINewNa_ConstP.pooled45[i + 6] * tmp[2] +
      (controlMCUSlugsMKIINewNa_ConstP.pooled45[i + 3] * tmp[1] +
       controlMCUSlugsMKIINewNa_ConstP.pooled45[i] * tmp[0]);
  }

  /* MATLAB Function: '<S218>/negprotect' */
  controlMCUSlugsMKII_negprotect3
    (controlMCUSlugsMKIINewNav_B.GetRangeofValuesnavSupportc_k[0],
     &controlMCUSlugsMKIINewNav_B.sf_negprotect_n);

  /* Gain: '<S218>/Gain' incorporates:
   *  Product: '<S218>/Divide'
   */
  rtb_Ze = controlMCUSlugsMKIINewNav_B.Switch *
    controlMCUSlugsMKIINewNav_B.CFunctionCall_a /
    controlMCUSlugsMKIINewNav_B.sf_negprotect_n.zpVal * 2.0F;

  /* S-Function "dsPIC_C_function_Call" Block: <S220>/C Function Call */
  controlMCUSlugsMKIINewNav_B.CFunctionCall_i = myAbs
    (controlMCUSlugsMKIINewNav_B.CFunctionCall_k);

  /* Switch: '<S132>/Switch' incorporates:
   *  Constant: '<S132>/Constant10'
   *  RelationalOperator: '<S132>/Relational Operator'
   */
  if (!(controlMCUSlugsMKIINewNav_B.CFunctionCall_i <= 1.57079637F)) {
    /* Switch: '<S222>/Switch1' incorporates:
     *  Constant: '<S222>/Constant1'
     *  Constant: '<S222>/Constant9'
     *  Constant: '<S241>/Constant'
     *  RelationalOperator: '<S241>/Compare'
     */
    if ((controlMCUSlugsMKIINewNav_B.Switch < 0.0F) > 0) {
      rtb_Ze = -4.57681F;
    } else {
      rtb_Ze = 4.57681F;
    }

    /* End of Switch: '<S222>/Switch1' */
  }

  /* End of Switch: '<S132>/Switch' */

  /* Product: '<S219>/Product' incorporates:
   *  Constant: '<S219>/Constant'
   */
  controlMCUSlugsMKIINewNav_B.Product = rtb_Ze / 9.815F;

  /* S-Function "dsPIC_C_function_Call" Block: <S237>/C Function Call */
  controlMCUSlugsMKIINewNav_B.CFunctionCall_pf = myAtan
    (controlMCUSlugsMKIINewNav_B.Product);

  /* Saturate: '<S219>/Bank  Limit Command' */
  rtb_BankLimitCommand = controlMCUSlugsMKIINewNav_B.CFunctionCall_pf >=
    0.436332315F ? 0.436332315F : controlMCUSlugsMKIINewNav_B.CFunctionCall_pf <=
    -0.436332315F ? -0.436332315F : controlMCUSlugsMKIINewNav_B.CFunctionCall_pf;

  /* MATLAB Function: '<S128>/myMux Fun3' */
  /* MATLAB Function 'Navigation/Navigation/myMux Fun3': '<S142>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S142>:1:5' */
  rtb_Product5[0] = rtb_Product6[0];
  rtb_Product5[1] = rtb_Product6[1];
  rtb_Product5[2] = controlMCUSlugsMKIINewNav_B.IC3;

  /* DataTypeConversion: '<S128>/Data Type Conversion9' incorporates:
   *  MATLAB Function: '<S128>/myMux Fun6'
   */
  /* MATLAB Function 'Navigation/Navigation/myMux Fun6': '<S145>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S145>:1:5' */
  controlMCUSlugsMKIINewNav_B.DataTypeConversion9[0] = (real32_T)0.0;
  controlMCUSlugsMKIINewNav_B.DataTypeConversion9[1] = (real32_T)0.0;
  controlMCUSlugsMKIINewNav_B.DataTypeConversion9[2] = (real32_T)0.0;

  /* S-Function "dsPIC_C_function_Call" Block: <S128>/Log 2 [navSupport.c] */
  setLogFloat2(controlMCUSlugsMKIINewNav_B.DataTypeConversion9);

  /* Update for Delay: '<S412>/Delay' */
  controlMCUSlugsMKIINewNav_DWork.Delay_DSTATE = rtb_IC1;

  /* Update for Delay: '<S128>/Integer Delay' */
  controlMCUSlugsMKIINewNav_DWork.IntegerDelay_DSTATE_d =
    controlMCUSlugsMKIINewNav_B.WP0;

  /* Update for Delay: '<S128>/Integer Delay1' */
  controlMCUSlugsMKIINewNav_DWork.IntegerDelay1_DSTATE_j2 =
    controlMCUSlugsMKIINewNav_B.WP1;

  /* End of Outputs for SubSystem: '<S3>/Navigation Encaps' */

  /* Switch: '<S3>/Switch' */
  if (controlMCUSlugsMKIINewNav_B.isitinMidLevelPtorSPTnavSupport > 0) {
    rtb_RhhcosphisinlambYe =
      controlMCUSlugsMKIINewNav_B.GetMidLevelCommandsnavSupportc[1];
  } else {
    rtb_RhhcosphisinlambYe = controlMCUSlugsMKIINewNav_B.Merge1;
  }

  /* End of Switch: '<S3>/Switch' */

  /* Switch: '<S19>/Switch3' incorporates:
   *  Delay: '<S19>/Integer Delay3'
   *  RelationalOperator: '<S19>/Relational Operator2'
   */
  if ((controlMCUSlugsMKIINewNav_B.GetXYZnavSupportc[2] ==
       controlMCUSlugsMKIINewNav_B.GetXYZnavSupportc[2]) > 0) {
    Switch = controlMCUSlugsMKIINewNav_B.GetXYZnavSupportc[2];
  } else {
    Switch = controlMCUSlugsMKIINewNav_DWork.IntegerDelay3_DSTATE;
  }

  /* End of Switch: '<S19>/Switch3' */
  /* S-Function "dsPIC_C_function_Call" Block: <S4>/Get Range of Values [navSupport.c]  */
  getRangeOfParams(((uint8_T)9U),((uint8_T)10U),
                   &controlMCUSlugsMKIINewNav_B.GetRangeofValuesnavSupportc_g[0]);

  /* S-Function "dsPIC_C_function_Call" Block: <S4>/Get a single Param [navSupport.c] */
  controlMCUSlugsMKIINewNav_B.GetasingleParamnavSupportc = getParamIdx(((uint8_T)
    11U));

  /* S-Function "dsPIC_C_function_Call" Block: <S10>/Get Range of Values [navSupport.c] */
  getRangeOfParams(((uint8_T)16U),((uint8_T)18U),
                   &controlMCUSlugsMKIINewNav_B.GetRangeofValuesnavSupportc_m[0]);

  /* Gain: '<S512>/Unit Conversion' */
  rtb_Product6[0] = 0.0174532924F *
    controlMCUSlugsMKIINewNav_B.GetRangeofValuesnavSupportc_m[0];
  rtb_Product6[1] = 0.0174532924F *
    controlMCUSlugsMKIINewNav_B.GetRangeofValuesnavSupportc_m[1];
  rtb_Product6[2] = 0.0174532924F *
    controlMCUSlugsMKIINewNav_B.GetRangeofValuesnavSupportc_m[2];

  /* Trigonometry: '<S515>/SinCos' */
  rtb_Product3_p4_idx = (real32_T)sin(rtb_Product6[0]);
  rtb_Product2_ep[0] = (real32_T)cos(rtb_Product6[0]);
  rtb_Deg2R_m_idx_0 = (real32_T)sin(rtb_Product6[1]);
  rtb_Product2_ep[1] = (real32_T)cos(rtb_Product6[1]);
  rtb_Deg2R_m_idx = (real32_T)sin(rtb_Product6[2]);
  rtb_Product2_ep[2] = (real32_T)cos(rtb_Product6[2]);

  /* Product: '<S522>/u(5)*u(6)' */
  rtb_VectorConcatenate_i[0] = rtb_Product2_ep[1] * rtb_Product2_ep[2];

  /* Sum: '<S525>/Sum' incorporates:
   *  Product: '<S525>/u(3)*u(4)'
   *  Product: '<S525>/u(6)*u(1)*u(2)'
   */
  rtb_VectorConcatenate_i[1] = rtb_Product2_ep[2] * rtb_Product3_p4_idx *
    rtb_Deg2R_m_idx_0 - rtb_Deg2R_m_idx * rtb_Product2_ep[0];

  /* Sum: '<S528>/Sum' incorporates:
   *  Product: '<S528>/u(1)*u(3)'
   *  Product: '<S528>/u(2)*u(4)*u(6)'
   */
  rtb_VectorConcatenate_i[2] = rtb_Deg2R_m_idx_0 * rtb_Product2_ep[0] *
    rtb_Product2_ep[2] + rtb_Product3_p4_idx * rtb_Deg2R_m_idx;

  /* Product: '<S523>/u(3)*u(5)' */
  rtb_VectorConcatenate_i[3] = rtb_Deg2R_m_idx * rtb_Product2_ep[1];

  /* Sum: '<S526>/Sum' incorporates:
   *  Product: '<S526>/u(1)*u(2)*u(3)'
   *  Product: '<S526>/u(4)*u(6)'
   */
  rtb_VectorConcatenate_i[4] = rtb_Product3_p4_idx * rtb_Deg2R_m_idx_0 *
    rtb_Deg2R_m_idx + rtb_Product2_ep[0] * rtb_Product2_ep[2];

  /* Sum: '<S529>/Sum' incorporates:
   *  Product: '<S529>/u(1)*u(6)'
   *  Product: '<S529>/u(2)*u(3)*u(4)'
   */
  rtb_VectorConcatenate_i[5] = rtb_Deg2R_m_idx_0 * rtb_Deg2R_m_idx *
    rtb_Product2_ep[0] - rtb_Product2_ep[2] * rtb_Product3_p4_idx;

  /* Gain: '<S524>/Gain2' */
  rtb_VectorConcatenate_i[6] = -rtb_Deg2R_m_idx_0;

  /* Product: '<S527>/u(1)*u(3)' */
  rtb_VectorConcatenate_i[7] = rtb_Product3_p4_idx * rtb_Product2_ep[1];

  /* Product: '<S530>/u(4)*u(5)' */
  rtb_VectorConcatenate_i[8] = rtb_Product2_ep[0] * rtb_Product2_ep[1];

  /* Math: '<S10>/Math Function' */
  for (i = 0; i < 3; i++) {
    rtb_MathFunction[3 * i] = rtb_VectorConcatenate_i[i];
    rtb_MathFunction[1 + 3 * i] = rtb_VectorConcatenate_i[i + 3];
    rtb_MathFunction[2 + 3 * i] = rtb_VectorConcatenate_i[i + 6];
  }

  /* End of Math: '<S10>/Math Function' */
  /* S-Function "dsPIC_C_function_Call" Block: <S10>/Get Attitude [navSupport.c] */
  getAttitude(&controlMCUSlugsMKIINewNav_B.GetAttitudenavSupportc[0]);

  /* Trigonometry: '<S516>/SinCos' */
  rtb_Product2_ep[0] = (real32_T)sin
    (controlMCUSlugsMKIINewNav_B.GetAttitudenavSupportc[0]);
  rtb_Product3_p4_idx = (real32_T)cos
    (controlMCUSlugsMKIINewNav_B.GetAttitudenavSupportc[0]);
  rtb_Product2_ep[1] = (real32_T)sin
    (controlMCUSlugsMKIINewNav_B.GetAttitudenavSupportc[1]);
  rtb_Deg2R_m_idx_0 = (real32_T)cos
    (controlMCUSlugsMKIINewNav_B.GetAttitudenavSupportc[1]);
  rtb_Product2_ep[2] = (real32_T)sin
    (controlMCUSlugsMKIINewNav_B.GetAttitudenavSupportc[2]);
  rtb_Deg2R_m_idx = (real32_T)cos
    (controlMCUSlugsMKIINewNav_B.GetAttitudenavSupportc[2]);

  /* Product: '<S532>/u(5)*u(6)' */
  rtb_VectorConcatenate_i[0] = rtb_Deg2R_m_idx_0 * rtb_Deg2R_m_idx;

  /* Sum: '<S535>/Sum' incorporates:
   *  Product: '<S535>/u(3)*u(4)'
   *  Product: '<S535>/u(6)*u(1)*u(2)'
   */
  rtb_VectorConcatenate_i[1] = rtb_Deg2R_m_idx * rtb_Product2_ep[0] *
    rtb_Product2_ep[1] - rtb_Product2_ep[2] * rtb_Product3_p4_idx;

  /* Sum: '<S538>/Sum' incorporates:
   *  Product: '<S538>/u(1)*u(3)'
   *  Product: '<S538>/u(2)*u(4)*u(6)'
   */
  rtb_VectorConcatenate_i[2] = rtb_Product2_ep[1] * rtb_Product3_p4_idx *
    rtb_Deg2R_m_idx + rtb_Product2_ep[0] * rtb_Product2_ep[2];

  /* Product: '<S533>/u(3)*u(5)' */
  rtb_VectorConcatenate_i[3] = rtb_Product2_ep[2] * rtb_Deg2R_m_idx_0;

  /* Sum: '<S536>/Sum' incorporates:
   *  Product: '<S536>/u(1)*u(2)*u(3)'
   *  Product: '<S536>/u(4)*u(6)'
   */
  rtb_VectorConcatenate_i[4] = rtb_Product2_ep[0] * rtb_Product2_ep[1] *
    rtb_Product2_ep[2] + rtb_Product3_p4_idx * rtb_Deg2R_m_idx;

  /* Sum: '<S539>/Sum' incorporates:
   *  Product: '<S539>/u(1)*u(6)'
   *  Product: '<S539>/u(2)*u(3)*u(4)'
   */
  rtb_VectorConcatenate_i[5] = rtb_Product2_ep[1] * rtb_Product2_ep[2] *
    rtb_Product3_p4_idx - rtb_Deg2R_m_idx * rtb_Product2_ep[0];

  /* Gain: '<S534>/Gain2' */
  rtb_VectorConcatenate_i[6] = -rtb_Product2_ep[1];

  /* Product: '<S537>/u(1)*u(3)' */
  rtb_VectorConcatenate_i[7] = rtb_Product2_ep[0] * rtb_Deg2R_m_idx_0;

  /* Product: '<S540>/u(4)*u(5)' */
  rtb_VectorConcatenate_i[8] = rtb_Product3_p4_idx * rtb_Deg2R_m_idx_0;

  /* Product: '<S10>/Product' */
  for (i = 0; i < 3; i++) {
    for (i_0 = 0; i_0 < 3; i_0++) {
      rtb_Product_ib[i + 3 * i_0] = 0.0F;
      rtb_Product_ib[i + 3 * i_0] = rtb_Product_ib[3 * i_0 + i] +
        rtb_VectorConcatenate_i[3 * i_0] * rtb_MathFunction[i];
      rtb_Product_ib[i + 3 * i_0] = rtb_VectorConcatenate_i[3 * i_0 + 1] *
        rtb_MathFunction[i + 3] + rtb_Product_ib[3 * i_0 + i];
      rtb_Product_ib[i + 3 * i_0] = rtb_VectorConcatenate_i[3 * i_0 + 2] *
        rtb_MathFunction[i + 6] + rtb_Product_ib[3 * i_0 + i];
    }
  }

  /* End of Product: '<S10>/Product' */

  /* If: '<S513>/If' incorporates:
   *  Gain: '<S521>/Gain1'
   *  Selector: '<S521>/Selector1'
   */
  if ((-rtb_Product_ib[6] >= 1.0F) || (-rtb_Product_ib[6] <= -1.0F)) {
    /* Outputs for IfAction SubSystem: '<S513>/AxisRotZeroR3' incorporates:
     *  ActionPort: '<S520>/Action Port'
     */
    /* Fcn: '<S520>/Fcn1' incorporates:
     *  Gain: '<S521>/Gain3'
     *  Selector: '<S521>/Selector3'
     */
    rtb_Merge_idx = rt_atan2f_snf(-rtb_Product_ib[1], rtb_Product_ib[4]);

    /* Fcn: '<S520>/Fcn2' */
    rtb_Product3_p4_idx = (real32_T)asin(-rtb_Product_ib[6] >= 1.0F ? 1.0F :
      -rtb_Product_ib[6] <= -1.0F ? -1.0F : -rtb_Product_ib[6]);

    /* Fcn: '<S520>/Fcn3' */
    rtb_Deg2R_m_idx_0 = 0.0F;

    /* End of Outputs for SubSystem: '<S513>/AxisRotZeroR3' */
  } else {
    /* Outputs for IfAction SubSystem: '<S513>/AxisRotDefault' incorporates:
     *  ActionPort: '<S519>/Action Port'
     */
    /* Fcn: '<S519>/Fcn1' */
    rtb_Merge_idx = rt_atan2f_snf(rtb_Product_ib[3], rtb_Product_ib[0]);

    /* Fcn: '<S519>/Fcn2' */
    rtb_Product3_p4_idx = (real32_T)asin(-rtb_Product_ib[6] >= 1.0F ? 1.0F :
      -rtb_Product_ib[6] <= -1.0F ? -1.0F : -rtb_Product_ib[6]);

    /* Fcn: '<S519>/Fcn3' incorporates:
     *  Gain: '<S521>/Gain2'
     *  Selector: '<S521>/Selector2'
     */
    rtb_Deg2R_m_idx_0 = rt_atan2f_snf(rtb_Product_ib[7], rtb_Product_ib[8]);

    /* End of Outputs for SubSystem: '<S513>/AxisRotDefault' */
  }

  /* End of If: '<S513>/If' */

  /* MATLAB Function: '<S10>/Embedded MATLAB Function1' */
  /* MATLAB Function 'get Nav Vars/Embedded MATLAB Function1': '<S514>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  if (rtb_Merge_idx < 0.0F) {
    /* '<S514>:1:5' */
    /* '<S514>:1:6' */
    rtb_Merge_idx += 6.28318548F;
  } else {
    /* '<S514>:1:8' */
  }

  /* End of MATLAB Function: '<S10>/Embedded MATLAB Function1' */

  /* MATLAB Function: '<S10>/myMux Fun1' */
  /* MATLAB Function 'get Nav Vars/myMux Fun1': '<S517>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S517>:1:5' */
  controlMCUSlugsMKIINewNav_B.y_i[0] = rtb_Deg2R_m_idx_0;
  controlMCUSlugsMKIINewNav_B.y_i[1] = rtb_Product3_p4_idx;
  controlMCUSlugsMKIINewNav_B.y_i[2] = rtb_Merge_idx;

  /* Product: '<S10>/Product1' */
  for (i = 0; i < 3; i++) {
    controlMCUSlugsMKIINewNav_B.Product1[i] = 0.0F;
    controlMCUSlugsMKIINewNav_B.Product1[i] = rtb_MathFunction[i] *
      controlMCUSlugsMKIINewNav_B.GetAttitudenavSupportc[3] +
      controlMCUSlugsMKIINewNav_B.Product1[i];
    controlMCUSlugsMKIINewNav_B.Product1[i] = rtb_MathFunction[i + 3] *
      controlMCUSlugsMKIINewNav_B.GetAttitudenavSupportc[4] +
      controlMCUSlugsMKIINewNav_B.Product1[i];
    controlMCUSlugsMKIINewNav_B.Product1[i] = rtb_MathFunction[i + 6] *
      controlMCUSlugsMKIINewNav_B.GetAttitudenavSupportc[5] +
      controlMCUSlugsMKIINewNav_B.Product1[i];
  }

  /* End of Product: '<S10>/Product1' */

  /* MATLAB Function: '<S10>/myMux Fun2' */
  /* MATLAB Function 'get Nav Vars/myMux Fun2': '<S518>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S518>:1:5' */
  controlMCUSlugsMKIINewNav_B.y[0] = controlMCUSlugsMKIINewNav_B.y_i[0];
  controlMCUSlugsMKIINewNav_B.y[1] = controlMCUSlugsMKIINewNav_B.y_i[1];
  controlMCUSlugsMKIINewNav_B.y[2] = controlMCUSlugsMKIINewNav_B.y_i[2];
  controlMCUSlugsMKIINewNav_B.y[3] = controlMCUSlugsMKIINewNav_B.Product1[0];
  controlMCUSlugsMKIINewNav_B.y[4] = controlMCUSlugsMKIINewNav_B.Product1[1];
  controlMCUSlugsMKIINewNav_B.y[5] = controlMCUSlugsMKIINewNav_B.Product1[2];

  /* S-Function "dsPIC_C_function_Call" Block: <S4>/Get Range of Values [navSupport.c]    */
  getRangeOfParams(((uint8_T)3U),((uint8_T)4U),
                   &controlMCUSlugsMKIINewNav_B.GetRangeofValuesnavSupportc_md[0]);

  /* S-Function "dsPIC_C_function_Call" Block: <S4>/Get a single Param [navSupport.c]1 */
  controlMCUSlugsMKIINewNav_B.GetasingleParamnavSupportc1 = getParamIdx
    (((uint8_T)15U));

  /* Outputs for Atomic SubSystem: '<S3>/Longitudinal Channel Encaps' */
  /* Sum: '<S114>/Add' incorporates:
   *  Constant: '<S114>/Constant'
   *  Constant: '<S114>/Constant from Model'
   *  Gain: '<S119>/Unit Conversion'
   *  Product: '<S114>/Divide'
   *  Saturate: '<S114>/[0 1000]'
   */
  controlMCUSlugsMKIINewNav_B.Add_m = 1.0F - (Switch >= 1000.0F ? 1000.0F :
    Switch <= 0.0F ? 0.0F : Switch) * 3.28084F / 145442.0F;

  /* S-Function "dsPIC_C_function_Call" Block: <S120>/C Function Call */
  controlMCUSlugsMKIINewNav_B.CFunctionCall_nt = myPow
    (controlMCUSlugsMKIINewNav_B.Add_m,4.25587606F);

  /* Product: '<S114>/Divide1' incorporates:
   *  Constant: '<S114>/Rho_0 (Kg//m^3)'
   */
  rtb_Product3_p4_idx = controlMCUSlugsMKIINewNav_B.CFunctionCall_nt * 1.225F;

  /* Product: '<S115>/Divide2' incorporates:
   *  Constant: '<S115>/a'
   *  Saturate: '<S115>/[ 0.01 50000]'
   */
  rtb_Divide2_m = 2.0F * controlMCUSlugsMKIINewNav_B.GetDynamicPnavSupportc /
    (rtb_Product3_p4_idx >= 50000.0F ? 50000.0F : rtb_Product3_p4_idx <= 0.01F ?
     0.01F : rtb_Product3_p4_idx);

  /* MATLAB Function: '<S122>/negprotect' */
  controlMCUSlugsMKIIN_negprotect(rtb_Divide2_m,
    &controlMCUSlugsMKIINewNav_B.sf_negprotect_e);

  /* S-Function "dsPIC_C_function_Call" Block: <S122>/C Function Call */
  controlMCUSlugsMKIINewNav_B.CFunctionCall_lc = mySqrt
    (controlMCUSlugsMKIINewNav_B.sf_negprotect_e.zpVal);

  /* Switch: '<S116>/Switch3' incorporates:
   *  Delay: '<S116>/Integer Delay3'
   *  RelationalOperator: '<S116>/Relational Operator2'
   */
  if ((controlMCUSlugsMKIINewNav_B.CFunctionCall_lc ==
       controlMCUSlugsMKIINewNav_B.CFunctionCall_lc) > 0) {
    controlMCUSlugsMKIINewNav_B.Switch3_o =
      controlMCUSlugsMKIINewNav_B.CFunctionCall_lc;
  } else {
    controlMCUSlugsMKIINewNav_B.Switch3_o =
      controlMCUSlugsMKIINewNav_DWork.IntegerDelay3_DSTATE_g;
  }

  /* End of Switch: '<S116>/Switch3' */

  /* RelationalOperator: '<S117>/Compare' incorporates:
   *  Constant: '<S117>/Constant'
   */
  rtb_IC2 = (controlMCUSlugsMKIINewNav_B.Switch3_o <= 5.0F);

  /* MATLAB Function: '<S113>/Embedded MATLAB Function' */
  controlM_EmbeddedMATLABFunction(controlMCUSlugsMKIINewNav_B.Switch3_o, 0.01,
    1.0, &controlMCUSlugsMKIINewNav_B.sf_EmbeddedMATLABFunction_d,
    &controlMCUSlugsMKIINewNav_DWork.sf_EmbeddedMATLABFunction_d);

  /* Switch: '<S93>/Schedule LPF' */
  if (rtb_IC2 > 0) {
    rtb_ScheduleLPF = controlMCUSlugsMKIINewNav_B.Switch3_o;
  } else {
    rtb_ScheduleLPF = controlMCUSlugsMKIINewNav_B.sf_EmbeddedMATLABFunction_d.y;
  }

  /* End of Switch: '<S93>/Schedule LPF' */

  /* Product: '<S98>/delta rise limit' */
  rtb_Product3_p4_idx = 0.02F;

  /* Switch: '<S126>/Init' incorporates:
   *  Constant: '<S17>/Constant3'
   *  UnitDelay: '<S126>/FixPt Unit Delay1'
   *  UnitDelay: '<S126>/FixPt Unit Delay2'
   */
  if (controlMCUSlugsMKIINewNav_DWork.FixPtUnitDelay2_DSTATE != 0) {
    rtb_Add4_l = 100.0F;
  } else {
    rtb_Add4_l = controlMCUSlugsMKIINewNav_DWork.FixPtUnitDelay1_DSTATE;
  }

  /* End of Switch: '<S126>/Init' */

  /* Sum: '<S98>/Difference Inputs1' */
  rtb_RhhcosphisinlambYe -= rtb_Add4_l;

  /* Switch: '<S125>/Switch2' incorporates:
   *  RelationalOperator: '<S125>/LowerRelop1'
   */
  if (!(rtb_RhhcosphisinlambYe > 0.02F)) {
    /* Switch: '<S125>/Switch' incorporates:
     *  RelationalOperator: '<S125>/UpperRelop'
     */
    if (rtb_RhhcosphisinlambYe < -0.03F) {
      rtb_Product3_p4_idx = -0.03F;
    } else {
      rtb_Product3_p4_idx = rtb_RhhcosphisinlambYe;
    }

    /* End of Switch: '<S125>/Switch' */
  }

  /* End of Switch: '<S125>/Switch2' */

  /* Sum: '<S98>/Difference Inputs2' */
  rtb_DifferenceInputs2 = rtb_Product3_p4_idx + rtb_Add4_l;

  /* Sum: '<S87>/Add2' */
  rtb_Add4_l = rtb_DifferenceInputs2 - Switch;

  /* Switch: '<S95>/Switch3' incorporates:
   *  Delay: '<S95>/Integer Delay3'
   *  RelationalOperator: '<S95>/Relational Operator2'
   */
  if ((rtb_Add4_l == rtb_Add4_l) > 0) {
    Switch3_m = rtb_Add4_l;
  } else {
    Switch3_m = controlMCUSlugsMKIINewNav_DWork.IntegerDelay3_DSTATE_c;
  }

  /* End of Switch: '<S95>/Switch3' */

  /* Product: '<S91>/Product1' */
  rtb_Merge_idx = Switch3_m *
    controlMCUSlugsMKIINewNav_B.GetRangeofValuesnavSupportc_g[0];

  /* Sum: '<S91>/Sum2' incorporates:
   *  Gain: '<S91>/Gain'
   *  Memory: '<S91>/Memory1'
   *  Product: '<S91>/Product4'
   */
  rtb_Add4_l = (0.01F * Switch3_m *
                controlMCUSlugsMKIINewNav_B.GetRangeofValuesnavSupportc_g[1] +
                controlMCUSlugsMKIINewNav_DWork.Memory1_PreviousInput) +
    rtb_Merge_idx;

  /* Switch: '<S91>/AntiWindup' incorporates:
   *  Constant: '<S91>/Constant5'
   *  Logic: '<S91>/Logical Operator'
   *  RelationalOperator: '<S91>/Relational Operator'
   *  RelationalOperator: '<S91>/Relational Operator1'
   */
  if ((rtb_Add4_l > controlMCUSlugsMKIINewNa_ConstB.Add4) && (rtb_Add4_l <
       controlMCUSlugsMKIINewNa_ConstB.Add3)) {
    rtb_Add4_l = Switch3_m;
  } else {
    rtb_Add4_l = 0.0F;
  }

  /* End of Switch: '<S91>/AntiWindup' */

  /* Switch: '<S108>/Switch3' incorporates:
   *  Delay: '<S108>/Integer Delay3'
   *  RelationalOperator: '<S108>/Relational Operator2'
   */
  if ((rtb_Add4_l == rtb_Add4_l) > 0) {
    Switch3_l = rtb_Add4_l;
  } else {
    Switch3_l = controlMCUSlugsMKIINewNav_DWork.IntegerDelay3_DSTATE_o;
  }

  /* End of Switch: '<S108>/Switch3' */

  /* Switch: '<S106>/Switch1' incorporates:
   *  Constant: '<S106>/Constant'
   *  Constant: '<S106>/Constant1'
   *  Constant: '<S106>/Constant2'
   *  Constant: '<S106>/Constant3'
   *  Constant: '<S106>/Constant5'
   *  Delay: '<S106>/Integer Delay'
   *  Delay: '<S106>/Integer Delay1'
   *  Delay: '<S106>/Integer Delay2'
   *  Product: '<S106>/Product'
   *  Product: '<S106>/Product1'
   *  Product: '<S106>/Product2'
   *  Product: '<S106>/Product3'
   *  Sum: '<S106>/Subtract'
   *  Sum: '<S106>/Subtract1'
   */
  if (controlMCUSlugsMKIINewNav_B.ManualorAutonavSupportc > 0) {
    rtb_Add4_l = 0.0F;
  } else {
    rtb_Add4_l = ((Switch3_l * 0.333333343F +
                   controlMCUSlugsMKIINewNav_DWork.IntegerDelay_DSTATE_c *
                   1.33333337F) +
                  controlMCUSlugsMKIINewNav_DWork.IntegerDelay1_DSTATE_j[0] *
                  0.333333343F) * 0.005F +
      controlMCUSlugsMKIINewNav_DWork.IntegerDelay2_DSTATE;
  }

  /* End of Switch: '<S106>/Switch1' */

  /* Switch: '<S107>/Switch3' incorporates:
   *  Delay: '<S107>/Integer Delay3'
   *  RelationalOperator: '<S107>/Relational Operator2'
   */
  if ((rtb_Add4_l == rtb_Add4_l) > 0) {
    Switch3_ae = rtb_Add4_l;
  } else {
    Switch3_ae = controlMCUSlugsMKIINewNav_DWork.IntegerDelay3_DSTATE_lo;
  }

  /* End of Switch: '<S107>/Switch3' */

  /* Switch: '<S91>/On//Off' incorporates:
   *  Constant: '<S91>/Constant1'
   *  Product: '<S91>/Product'
   *  Sum: '<S91>/Add2'
   */
  if (controlMCUSlugsMKIINewNav_B.ManualorAutonavSupportc > 0) {
    rtb_OnOff = 0.0F;
  } else {
    rtb_OnOff = Switch3_ae *
      controlMCUSlugsMKIINewNav_B.GetRangeofValuesnavSupportc_g[1] +
      rtb_Merge_idx;
  }

  /* End of Switch: '<S91>/On//Off' */

  /* Switch: '<S105>/Switch2' incorporates:
   *  RelationalOperator: '<S105>/LowerRelop1'
   *  RelationalOperator: '<S105>/UpperRelop'
   *  Switch: '<S105>/Switch'
   */
  if (rtb_OnOff > controlMCUSlugsMKIINewNa_ConstB.Add3) {
    rtb_Add4_l = controlMCUSlugsMKIINewNa_ConstB.Add3;
  } else if (rtb_OnOff < controlMCUSlugsMKIINewNa_ConstB.Add4) {
    /* Switch: '<S105>/Switch' */
    rtb_Add4_l = controlMCUSlugsMKIINewNa_ConstB.Add4;
  } else {
    rtb_Add4_l = rtb_OnOff;
  }

  /* End of Switch: '<S105>/Switch2' */

  /* Saturate: '<S87>/Theta_c Limit' */
  rtb_Theta_cLimit = rtb_Add4_l >= 0.261799395F ? 0.261799395F : rtb_Add4_l <=
    -0.261799395F ? -0.261799395F : rtb_Add4_l;

  /* MATLAB Function: '<S17>/myMux Fun1' */
  controlMCUSlugsMKIINe_myMuxFun1(rtb_ScheduleLPF, rtb_Theta_cLimit,
    rtb_DifferenceInputs2, &controlMCUSlugsMKIINewNav_B.sf_myMuxFun1_i);

  /* S-Function "dsPIC_C_function_Call" Block: <S17>/update Longitud Channel Commands [navSupport.c] */
  setNavLong(controlMCUSlugsMKIINewNav_B.sf_myMuxFun1_i.y);

  /* Switch: '<S97>/Switch3' incorporates:
   *  Delay: '<S97>/Integer Delay3'
   *  RelationalOperator: '<S97>/Relational Operator2'
   */
  if ((controlMCUSlugsMKIINewNav_B.y[0] == controlMCUSlugsMKIINewNav_B.y[0]) > 0)
  {
    controlMCUSlugsMKIINewNav_B.Switch3_bm = controlMCUSlugsMKIINewNav_B.y[0];
  } else {
    controlMCUSlugsMKIINewNav_B.Switch3_bm =
      controlMCUSlugsMKIINewNav_DWork.IntegerDelay3_DSTATE_i4;
  }

  /* End of Switch: '<S97>/Switch3' */

  /* MATLAB Function: '<S89>/Embedded MATLAB Function' */
  controlM_EmbeddedMATLABFunction(controlMCUSlugsMKIINewNav_B.Switch3_bm, 0.01,
    0.32, &controlMCUSlugsMKIINewNav_B.sf_EmbeddedMATLABFunction_a,
    &controlMCUSlugsMKIINewNav_DWork.sf_EmbeddedMATLABFunction_a);

  /* Sum: '<S87>/Add' */
  rtb_Add4_l = controlMCUSlugsMKIINewNav_B.GetMidLevelCommandsnavSupportc[0] -
    rtb_ScheduleLPF;

  /* Sum: '<S90>/Add3' incorporates:
   *  Constant: '<S90>/SaturationLimit'
   */
  rtb_Product3_p4_idx = 0.95F - controlMCUSlugsMKIINewNav_B.DataTypeConversion_j;

  /* Switch: '<S94>/Switch3' incorporates:
   *  Delay: '<S94>/Integer Delay3'
   *  RelationalOperator: '<S94>/Relational Operator2'
   */
  if ((rtb_Add4_l == rtb_Add4_l) > 0) {
    Switch3_lh = rtb_Add4_l;
  } else {
    Switch3_lh = controlMCUSlugsMKIINewNav_DWork.IntegerDelay3_DSTATE_gy;
  }

  /* End of Switch: '<S94>/Switch3' */

  /* Sum: '<S90>/Add1' incorporates:
   *  Constant: '<S90>/delayTime'
   *  Delay: '<S90>/NDelays'
   *  Product: '<S90>/Product1'
   *  Product: '<S90>/Product2'
   *  Product: '<S90>/Product3'
   *  Sum: '<S90>/Sum'
   */
  rtb_Merge_idx = (Switch3_lh - controlMCUSlugsMKIINewNav_DWork.NDelays_DSTATE[0])
    / 0.05F * controlMCUSlugsMKIINewNav_B.GetRangeofValuesnavSupportc[2] +
    Switch3_lh * controlMCUSlugsMKIINewNav_B.GetRangeofValuesnavSupportc[0];

  /* Sum: '<S90>/Sum2' incorporates:
   *  Gain: '<S90>/Gain'
   *  Memory: '<S90>/Memory1'
   *  Product: '<S90>/Product4'
   */
  rtb_Deg2R1 = (0.01F * Switch3_lh *
                controlMCUSlugsMKIINewNav_B.GetRangeofValuesnavSupportc[1] +
                controlMCUSlugsMKIINewNav_DWork.Memory1_PreviousInput_c) +
    rtb_Merge_idx;

  /* Switch: '<S90>/AntiWindup' incorporates:
   *  Constant: '<S90>/Constant5'
   *  Constant: '<S90>/SaturationLimit'
   *  Constant: '<S90>/SaturationLimit1'
   *  Logic: '<S90>/Logical Operator'
   *  RelationalOperator: '<S90>/Relational Operator'
   *  RelationalOperator: '<S90>/Relational Operator1'
   *  Sum: '<S90>/Add3'
   *  Sum: '<S90>/Add4'
   */
  if ((rtb_Deg2R1 > 0.0F - controlMCUSlugsMKIINewNav_B.DataTypeConversion_j) &&
      (rtb_Deg2R1 < 0.95F - controlMCUSlugsMKIINewNav_B.DataTypeConversion_j)) {
    rtb_Deg2R1 = Switch3_lh;
  } else {
    rtb_Deg2R1 = 0.0F;
  }

  /* End of Switch: '<S90>/AntiWindup' */

  /* Switch: '<S103>/Switch3' incorporates:
   *  Delay: '<S103>/Integer Delay3'
   *  RelationalOperator: '<S103>/Relational Operator2'
   */
  if ((rtb_Deg2R1 == rtb_Deg2R1) > 0) {
    rtb_Deg2R_m_idx_0 = rtb_Deg2R1;
  } else {
    rtb_Deg2R_m_idx_0 = controlMCUSlugsMKIINewNav_DWork.IntegerDelay3_DSTATE_oy;
  }

  /* End of Switch: '<S103>/Switch3' */

  /* Switch: '<S102>/Switch1' incorporates:
   *  Constant: '<S102>/Constant'
   *  Constant: '<S102>/Constant1'
   *  Constant: '<S102>/Constant2'
   *  Constant: '<S102>/Constant3'
   *  Constant: '<S102>/Constant5'
   *  Delay: '<S102>/Integer Delay'
   *  Delay: '<S102>/Integer Delay1'
   *  Delay: '<S102>/Integer Delay2'
   *  Product: '<S102>/Product'
   *  Product: '<S102>/Product1'
   *  Product: '<S102>/Product2'
   *  Product: '<S102>/Product3'
   *  Sum: '<S102>/Subtract'
   *  Sum: '<S102>/Subtract1'
   */
  if (controlMCUSlugsMKIINewNav_B.ManualorAutonavSupportc > 0) {
    rtb_Deg2R1 = 0.0F;
  } else {
    rtb_Deg2R1 = ((rtb_Deg2R_m_idx_0 * 0.333333343F +
                   controlMCUSlugsMKIINewNav_DWork.IntegerDelay_DSTATE_o *
                   1.33333337F) +
                  controlMCUSlugsMKIINewNav_DWork.IntegerDelay1_DSTATE_m[0] *
                  0.333333343F) * 0.005F +
      controlMCUSlugsMKIINewNav_DWork.IntegerDelay2_DSTATE_b;
  }

  /* End of Switch: '<S102>/Switch1' */

  /* Switch: '<S104>/Switch3' incorporates:
   *  Delay: '<S104>/Integer Delay3'
   *  RelationalOperator: '<S104>/Relational Operator2'
   */
  if ((rtb_Deg2R1 == rtb_Deg2R1) > 0) {
    rtb_Deg2R_m_idx = rtb_Deg2R1;
  } else {
    rtb_Deg2R_m_idx = controlMCUSlugsMKIINewNav_DWork.IntegerDelay3_DSTATE_d;
  }

  /* End of Switch: '<S104>/Switch3' */

  /* Switch: '<S90>/On//Off' incorporates:
   *  Constant: '<S90>/Constant1'
   *  Product: '<S90>/Product'
   *  Sum: '<S90>/Add2'
   */
  if (controlMCUSlugsMKIINewNav_B.ManualorAutonavSupportc > 0) {
    Product = 0.0F;
  } else {
    Product = rtb_Deg2R_m_idx *
      controlMCUSlugsMKIINewNav_B.GetRangeofValuesnavSupportc[1] + rtb_Merge_idx;
  }

  /* End of Switch: '<S90>/On//Off' */

  /* Switch: '<S101>/Switch2' incorporates:
   *  Constant: '<S90>/SaturationLimit'
   *  RelationalOperator: '<S101>/LowerRelop1'
   *  Sum: '<S90>/Add3'
   */
  if (!(Product > 0.95F - controlMCUSlugsMKIINewNav_B.DataTypeConversion_j)) {
    /* Switch: '<S101>/Switch' incorporates:
     *  Constant: '<S90>/SaturationLimit1'
     *  RelationalOperator: '<S101>/UpperRelop'
     *  Sum: '<S90>/Add4'
     */
    if (Product < 0.0F - controlMCUSlugsMKIINewNav_B.DataTypeConversion_j) {
      rtb_Product3_p4_idx = 0.0F -
        controlMCUSlugsMKIINewNav_B.DataTypeConversion_j;
    } else {
      rtb_Product3_p4_idx = Product;
    }

    /* End of Switch: '<S101>/Switch' */
  }

  /* End of Switch: '<S101>/Switch2' */

  /* Sum: '<S87>/Add1' incorporates:
   *  Product: '<S87>/Product2'
   */
  rtb_Product3_p4_idx = (rtb_Product3_p4_idx +
    controlMCUSlugsMKIINewNav_B.DataTypeConversion_j) +
    controlMCUSlugsMKIINewNav_B.GetasingleParamnavSupportc1 *
    controlMCUSlugsMKIINewNav_B.y[1];

  /* Sum: '<S87>/Add3' */
  rtb_Deg2R1 = rtb_Theta_cLimit - controlMCUSlugsMKIINewNav_B.y[1];

  /* Sum: '<S92>/Add3' incorporates:
   *  Constant: '<S92>/SaturationLimit'
   */
  rtb_Add4_l = 0.401425719F - controlMCUSlugsMKIINewNav_B.DataTypeConversion_h;

  /* Switch: '<S96>/Switch3' incorporates:
   *  Delay: '<S96>/Integer Delay3'
   *  RelationalOperator: '<S96>/Relational Operator2'
   */
  if ((rtb_Deg2R1 == rtb_Deg2R1) > 0) {
    rtb_cosphi = rtb_Deg2R1;
  } else {
    rtb_cosphi = controlMCUSlugsMKIINewNav_DWork.IntegerDelay3_DSTATE_cv;
  }

  /* End of Switch: '<S96>/Switch3' */

  /* Sum: '<S92>/Add1' incorporates:
   *  Constant: '<S92>/delayTime'
   *  Delay: '<S92>/NDelays'
   *  Product: '<S92>/Product1'
   *  Product: '<S92>/Product2'
   *  Product: '<S92>/Product3'
   *  Sum: '<S92>/Sum'
   */
  rtb_RhhcosphisinlambYe = (rtb_cosphi -
    controlMCUSlugsMKIINewNav_DWork.NDelays_DSTATE_n[0]) / 0.05F *
    controlMCUSlugsMKIINewNav_B.GetRangeofValuesnavSupportc_md[2] + rtb_cosphi *
    controlMCUSlugsMKIINewNav_B.GetRangeofValuesnavSupportc_md[0];

  /* Sum: '<S92>/Sum2' incorporates:
   *  Gain: '<S92>/Gain'
   *  Memory: '<S92>/Memory1'
   *  Product: '<S92>/Product4'
   */
  rtb_Merge_idx = (0.01F * rtb_cosphi *
                   controlMCUSlugsMKIINewNav_B.GetRangeofValuesnavSupportc_md[1]
                   + controlMCUSlugsMKIINewNav_DWork.Memory1_PreviousInput_b) +
    rtb_RhhcosphisinlambYe;

  /* Switch: '<S92>/AntiWindup' incorporates:
   *  Constant: '<S92>/Constant5'
   *  Constant: '<S92>/SaturationLimit'
   *  Constant: '<S92>/SaturationLimit1'
   *  Logic: '<S92>/Logical Operator'
   *  RelationalOperator: '<S92>/Relational Operator'
   *  RelationalOperator: '<S92>/Relational Operator1'
   *  Sum: '<S92>/Add3'
   *  Sum: '<S92>/Add4'
   */
  if ((rtb_Merge_idx > -0.401425719F -
       controlMCUSlugsMKIINewNav_B.DataTypeConversion_h) && (rtb_Merge_idx <
       0.401425719F - controlMCUSlugsMKIINewNav_B.DataTypeConversion_h)) {
    rtb_Merge_idx = rtb_cosphi;
  } else {
    rtb_Merge_idx = 0.0F;
  }

  /* End of Switch: '<S92>/AntiWindup' */

  /* Switch: '<S111>/Switch3' incorporates:
   *  Delay: '<S111>/Integer Delay3'
   *  RelationalOperator: '<S111>/Relational Operator2'
   */
  if ((rtb_Merge_idx == rtb_Merge_idx) > 0) {
    rtb_Ze = rtb_Merge_idx;
  } else {
    rtb_Ze = controlMCUSlugsMKIINewNav_DWork.IntegerDelay3_DSTATE_f;
  }

  /* End of Switch: '<S111>/Switch3' */

  /* Switch: '<S110>/Switch1' incorporates:
   *  Constant: '<S110>/Constant'
   *  Constant: '<S110>/Constant1'
   *  Constant: '<S110>/Constant2'
   *  Constant: '<S110>/Constant3'
   *  Constant: '<S110>/Constant5'
   *  Delay: '<S110>/Integer Delay'
   *  Delay: '<S110>/Integer Delay1'
   *  Delay: '<S110>/Integer Delay2'
   *  Product: '<S110>/Product'
   *  Product: '<S110>/Product1'
   *  Product: '<S110>/Product2'
   *  Product: '<S110>/Product3'
   *  Sum: '<S110>/Subtract'
   *  Sum: '<S110>/Subtract1'
   */
  if (controlMCUSlugsMKIINewNav_B.ManualorAutonavSupportc > 0) {
    rtb_Merge_idx = 0.0F;
  } else {
    rtb_Merge_idx = ((rtb_Ze * 0.333333343F +
                      controlMCUSlugsMKIINewNav_DWork.IntegerDelay_DSTATE_i *
                      1.33333337F) +
                     controlMCUSlugsMKIINewNav_DWork.IntegerDelay1_DSTATE_g[0] *
                     0.333333343F) * 0.005F +
      controlMCUSlugsMKIINewNav_DWork.IntegerDelay2_DSTATE_g;
  }

  /* End of Switch: '<S110>/Switch1' */

  /* Switch: '<S112>/Switch3' incorporates:
   *  Delay: '<S112>/Integer Delay3'
   *  RelationalOperator: '<S112>/Relational Operator2'
   */
  if ((rtb_Merge_idx == rtb_Merge_idx) > 0) {
    rtb_RhhcosphicoslambXe = rtb_Merge_idx;
  } else {
    rtb_RhhcosphicoslambXe =
      controlMCUSlugsMKIINewNav_DWork.IntegerDelay3_DSTATE_n;
  }

  /* End of Switch: '<S112>/Switch3' */

  /* Switch: '<S92>/On//Off' incorporates:
   *  Constant: '<S92>/Constant1'
   *  Product: '<S92>/Product'
   *  Sum: '<S92>/Add2'
   */
  if (controlMCUSlugsMKIINewNav_B.ManualorAutonavSupportc > 0) {
    rtb_Merge_idx = 0.0F;
  } else {
    rtb_Merge_idx = rtb_RhhcosphicoslambXe *
      controlMCUSlugsMKIINewNav_B.GetRangeofValuesnavSupportc_md[1] +
      rtb_RhhcosphisinlambYe;
  }

  /* End of Switch: '<S92>/On//Off' */

  /* Switch: '<S109>/Switch2' incorporates:
   *  Constant: '<S92>/SaturationLimit'
   *  RelationalOperator: '<S109>/LowerRelop1'
   *  Sum: '<S92>/Add3'
   */
  if (!(rtb_Merge_idx > 0.401425719F -
        controlMCUSlugsMKIINewNav_B.DataTypeConversion_h)) {
    /* Switch: '<S109>/Switch' incorporates:
     *  Constant: '<S92>/SaturationLimit1'
     *  RelationalOperator: '<S109>/UpperRelop'
     *  Sum: '<S92>/Add4'
     */
    if (rtb_Merge_idx < -0.401425719F -
        controlMCUSlugsMKIINewNav_B.DataTypeConversion_h) {
      rtb_Add4_l = -0.401425719F -
        controlMCUSlugsMKIINewNav_B.DataTypeConversion_h;
    } else {
      rtb_Add4_l = rtb_Merge_idx;
    }

    /* End of Switch: '<S109>/Switch' */
  }

  /* End of Switch: '<S109>/Switch2' */

  /* Saturate: '<S87>/[-60 60]' */
  controlMCUSlugsMKIINewNav_B.u060 =
    controlMCUSlugsMKIINewNav_B.sf_EmbeddedMATLABFunction_a.y >= 60.0F ? 60.0F :
    controlMCUSlugsMKIINewNav_B.sf_EmbeddedMATLABFunction_a.y <= -60.0F ? -60.0F
    : controlMCUSlugsMKIINewNav_B.sf_EmbeddedMATLABFunction_a.y;

  /* S-Function "dsPIC_C_function_Call" Block: <S99>/[apUtils.c] */
  controlMCUSlugsMKIINewNav_B.apUtilsc_b = myCos
    (controlMCUSlugsMKIINewNav_B.u060);

  /* Sum: '<S87>/Add4' incorporates:
   *  Constant: '<S87>/Constant2'
   *  Constant: '<S87>/Constant4'
   *  Product: '<S87>/Product'
   *  Product: '<S87>/Product1'
   *  Sum: '<S87>/Add5'
   */
  rtb_Add4_l = (1.0F / controlMCUSlugsMKIINewNav_B.apUtilsc_b - 1.0F) *
    controlMCUSlugsMKIINewNav_B.GetasingleParamnavSupportc + (rtb_Add4_l +
    controlMCUSlugsMKIINewNav_B.DataTypeConversion_h);

  /* Saturate: '<S87>/Elevator  Limit' */
  rtb_ElevatorLimit = rtb_Add4_l >= 0.401425719F ? 0.401425719F : rtb_Add4_l <=
    -0.401425719F ? -0.401425719F : rtb_Add4_l;

  /* Saturate: '<S87>/Throttle  Limit' */
  rtb_ThrottleLimit = rtb_Product3_p4_idx >= 0.95F ? 0.95F : rtb_Product3_p4_idx
    <= 0.0F ? 0.0F : rtb_Product3_p4_idx;

  /* Update for Delay: '<S116>/Integer Delay3' */
  controlMCUSlugsMKIINewNav_DWork.IntegerDelay3_DSTATE_g =
    controlMCUSlugsMKIINewNav_B.Switch3_o;

  /* Update for UnitDelay: '<S126>/FixPt Unit Delay2' incorporates:
   *  Constant: '<S126>/FixPt Constant'
   */
  controlMCUSlugsMKIINewNav_DWork.FixPtUnitDelay2_DSTATE = 0U;

  /* Update for UnitDelay: '<S126>/FixPt Unit Delay1' */
  controlMCUSlugsMKIINewNav_DWork.FixPtUnitDelay1_DSTATE = rtb_DifferenceInputs2;

  /* Update for Delay: '<S95>/Integer Delay3' */
  controlMCUSlugsMKIINewNav_DWork.IntegerDelay3_DSTATE_c = Switch3_m;

  /* Update for Delay: '<S106>/Integer Delay2' */
  controlMCUSlugsMKIINewNav_DWork.IntegerDelay2_DSTATE = Switch3_ae;

  /* Update for Memory: '<S91>/Memory1' */
  controlMCUSlugsMKIINewNav_DWork.Memory1_PreviousInput = rtb_OnOff;

  /* Update for Delay: '<S108>/Integer Delay3' */
  controlMCUSlugsMKIINewNav_DWork.IntegerDelay3_DSTATE_o = Switch3_l;

  /* Update for Delay: '<S106>/Integer Delay' */
  controlMCUSlugsMKIINewNav_DWork.IntegerDelay_DSTATE_c = Switch3_l;

  /* Update for Delay: '<S106>/Integer Delay1' */
  controlMCUSlugsMKIINewNav_DWork.IntegerDelay1_DSTATE_j[0] =
    controlMCUSlugsMKIINewNav_DWork.IntegerDelay1_DSTATE_j[1];
  controlMCUSlugsMKIINewNav_DWork.IntegerDelay1_DSTATE_j[1] = Switch3_l;

  /* Update for Delay: '<S107>/Integer Delay3' */
  controlMCUSlugsMKIINewNav_DWork.IntegerDelay3_DSTATE_lo = Switch3_ae;

  /* Update for Delay: '<S97>/Integer Delay3' */
  controlMCUSlugsMKIINewNav_DWork.IntegerDelay3_DSTATE_i4 =
    controlMCUSlugsMKIINewNav_B.Switch3_bm;

  /* Update for Delay: '<S94>/Integer Delay3' */
  controlMCUSlugsMKIINewNav_DWork.IntegerDelay3_DSTATE_gy = Switch3_lh;

  /* Update for Delay: '<S90>/NDelays' */
  controlMCUSlugsMKIINewNav_DWork.NDelays_DSTATE[0] =
    controlMCUSlugsMKIINewNav_DWork.NDelays_DSTATE[1];
  controlMCUSlugsMKIINewNav_DWork.NDelays_DSTATE[1] =
    controlMCUSlugsMKIINewNav_DWork.NDelays_DSTATE[2];
  controlMCUSlugsMKIINewNav_DWork.NDelays_DSTATE[2] =
    controlMCUSlugsMKIINewNav_DWork.NDelays_DSTATE[3];
  controlMCUSlugsMKIINewNav_DWork.NDelays_DSTATE[3] =
    controlMCUSlugsMKIINewNav_DWork.NDelays_DSTATE[4];
  controlMCUSlugsMKIINewNav_DWork.NDelays_DSTATE[4] = Switch3_lh;

  /* Update for Delay: '<S102>/Integer Delay2' */
  controlMCUSlugsMKIINewNav_DWork.IntegerDelay2_DSTATE_b = rtb_Deg2R_m_idx;

  /* Update for Memory: '<S90>/Memory1' */
  controlMCUSlugsMKIINewNav_DWork.Memory1_PreviousInput_c = Product;

  /* Update for Delay: '<S103>/Integer Delay3' */
  controlMCUSlugsMKIINewNav_DWork.IntegerDelay3_DSTATE_oy = rtb_Deg2R_m_idx_0;

  /* Update for Delay: '<S102>/Integer Delay' */
  controlMCUSlugsMKIINewNav_DWork.IntegerDelay_DSTATE_o = rtb_Deg2R_m_idx_0;

  /* Update for Delay: '<S102>/Integer Delay1' */
  controlMCUSlugsMKIINewNav_DWork.IntegerDelay1_DSTATE_m[0] =
    controlMCUSlugsMKIINewNav_DWork.IntegerDelay1_DSTATE_m[1];
  controlMCUSlugsMKIINewNav_DWork.IntegerDelay1_DSTATE_m[1] = rtb_Deg2R_m_idx_0;

  /* Update for Delay: '<S104>/Integer Delay3' */
  controlMCUSlugsMKIINewNav_DWork.IntegerDelay3_DSTATE_d = rtb_Deg2R_m_idx;

  /* Update for Delay: '<S96>/Integer Delay3' */
  controlMCUSlugsMKIINewNav_DWork.IntegerDelay3_DSTATE_cv = rtb_cosphi;

  /* Update for Delay: '<S92>/NDelays' */
  controlMCUSlugsMKIINewNav_DWork.NDelays_DSTATE_n[0] =
    controlMCUSlugsMKIINewNav_DWork.NDelays_DSTATE_n[1];
  controlMCUSlugsMKIINewNav_DWork.NDelays_DSTATE_n[1] =
    controlMCUSlugsMKIINewNav_DWork.NDelays_DSTATE_n[2];
  controlMCUSlugsMKIINewNav_DWork.NDelays_DSTATE_n[2] =
    controlMCUSlugsMKIINewNav_DWork.NDelays_DSTATE_n[3];
  controlMCUSlugsMKIINewNav_DWork.NDelays_DSTATE_n[3] =
    controlMCUSlugsMKIINewNav_DWork.NDelays_DSTATE_n[4];
  controlMCUSlugsMKIINewNav_DWork.NDelays_DSTATE_n[4] = rtb_cosphi;

  /* Update for Delay: '<S110>/Integer Delay2' */
  controlMCUSlugsMKIINewNav_DWork.IntegerDelay2_DSTATE_g =
    rtb_RhhcosphicoslambXe;

  /* Update for Memory: '<S92>/Memory1' */
  controlMCUSlugsMKIINewNav_DWork.Memory1_PreviousInput_b = rtb_Merge_idx;

  /* Update for Delay: '<S111>/Integer Delay3' */
  controlMCUSlugsMKIINewNav_DWork.IntegerDelay3_DSTATE_f = rtb_Ze;

  /* Update for Delay: '<S110>/Integer Delay' */
  controlMCUSlugsMKIINewNav_DWork.IntegerDelay_DSTATE_i = rtb_Ze;

  /* Update for Delay: '<S110>/Integer Delay1' */
  controlMCUSlugsMKIINewNav_DWork.IntegerDelay1_DSTATE_g[0] =
    controlMCUSlugsMKIINewNav_DWork.IntegerDelay1_DSTATE_g[1];
  controlMCUSlugsMKIINewNav_DWork.IntegerDelay1_DSTATE_g[1] = rtb_Ze;

  /* Update for Delay: '<S112>/Integer Delay3' */
  controlMCUSlugsMKIINewNav_DWork.IntegerDelay3_DSTATE_n =
    rtb_RhhcosphicoslambXe;

  /* End of Outputs for SubSystem: '<S3>/Longitudinal Channel Encaps' */
  /* S-Function "dsPIC_C_function_Call" Block: <S4>/Get Range of Values [navSupport.c]   */
  getRangeOfParams(((uint8_T)12U),((uint8_T)14U),
                   &controlMCUSlugsMKIINewNav_B.GetRangeofValuesnavSupportc_d[0]);

  /* S-Function "dsPIC_C_function_Call" Block: <S4>/Get Range of Values [navSupport.c]      */
  getRangeOfParams(((uint8_T)6U),((uint8_T)8U),
                   &controlMCUSlugsMKIINewNav_B.GetRangeofValuesnavSupportc_h[0]);

  /* S-Function "dsPIC_C_function_Call" Block: <S21>/[apUtils.c] */
  controlMCUSlugsMKIINewNav_B.apUtilsc = myCos(controlMCUSlugsMKIINewNav_B.y[0]);

  /* Product: '<S14>/Product' */
  rtb_Merge_idx = controlMCUSlugsMKIINewNav_B.y[5] *
    controlMCUSlugsMKIINewNav_B.apUtilsc;

  /* S-Function "dsPIC_C_function_Call" Block: <S22>/C Function Call */
  controlMCUSlugsMKIINewNav_B.CFunctionCall = mySin
    (controlMCUSlugsMKIINewNav_B.y[0]);

  /* Sum: '<S14>/Subtract' incorporates:
   *  Product: '<S14>/Product1'
   */
  rtb_Merge_idx += controlMCUSlugsMKIINewNav_B.y[4] *
    controlMCUSlugsMKIINewNav_B.CFunctionCall;

  /* Switch: '<S20>/Switch3' incorporates:
   *  Delay: '<S20>/Integer Delay3'
   *  RelationalOperator: '<S20>/Relational Operator2'
   */
  if ((rtb_Merge_idx == rtb_Merge_idx) > 0) {
    Switch3_a = rtb_Merge_idx;
  } else {
    Switch3_a = controlMCUSlugsMKIINewNav_DWork.IntegerDelay3_DSTATE_i;
  }

  /* End of Switch: '<S20>/Switch3' */
  /* S-Function "dsPIC_C_function_Call" Block: <Root>/Get Range of Values [navSupport.c]  */
  getRangeOfParams(((uint8_T)22U),((uint8_T)25U),
                   &controlMCUSlugsMKIINewNav_B.GetRangeofValuesnavSupportc_n[0]);

  /* Outputs for Enabled SubSystem: '<S3>/L1 Output Feedback Controller With  Projection Operator' incorporates:
   *  EnablePort: '<S15>/Enable'
   */
  if (controlMCUSlugsMKIINewNav_B.GetRangeofValuesnavSupportc_n[3] > 0.0F) {
    if (!controlMCUSlugsMKIINewNav_DWork.L1OutputFeedbackControllerWithP) {
      /* InitializeConditions for UnitDelay: '<S35>/UD' */
      controlMCUSlugsMKIINewNav_DWork.UD_DSTATE_b = 0.0F;

      /* InitializeConditions for Delay: '<S25>/Integer Delay' */
      controlMCUSlugsMKIINewNav_DWork.IntegerDelay_DSTATE_l = 0.0F;

      /* InitializeConditions for Delay: '<S25>/Integer Delay1' */
      controlMCUSlugsMKIINewNav_DWork.IntegerDelay1_DSTATE_k = 0.0F;

      /* InitializeConditions for Delay: '<S27>/Integer Delay3' */
      controlMCUSlugsMKIINewNav_DWork.IntegerDelay3_DSTATE_nd = 0.0F;

      /* InitializeConditions for UnitDelay: '<S45>/UD' */
      controlMCUSlugsMKIINewNav_DWork.UD_DSTATE_o = 0.0F;

      /* InitializeConditions for Delay: '<S29>/Integer Delay' */
      controlMCUSlugsMKIINewNav_DWork.IntegerDelay_DSTATE_f = 0.0F;

      /* InitializeConditions for Delay: '<S29>/Integer Delay1' */
      controlMCUSlugsMKIINewNav_DWork.IntegerDelay1_DSTATE_i = 0.0F;

      /* InitializeConditions for Delay: '<S15>/Integer Delay' */
      controlMCUSlugsMKIINewNav_DWork.IntegerDelay_DSTATE_cl = 0.0F;

      /* InitializeConditions for Delay: '<S15>/Integer Delay1' */
      controlMCUSlugsMKIINewNav_DWork.IntegerDelay1_DSTATE_c = 0.0F;

      /* InitializeConditions for Merge: '<S26>/Merge' */
      if (rtmIsFirstInitCond(controlMCUSlugsMKIINewNav_M)) {
        controlMCUSlugsMKIINewNav_B.Merge_e = 0.0F;
      }

      /* End of InitializeConditions for Merge: '<S26>/Merge' */

      /* InitializeConditions for Delay: '<S28>/Integer Delay3' */
      controlMCUSlugsMKIINewNav_DWork.IntegerDelay3_DSTATE_e = 0.0F;

      /* InitializeConditions for Delay: '<S30>/Integer Delay' */
      controlMCUSlugsMKIINewNav_DWork.IntegerDelay_DSTATE_fj = 0.0F;

      /* InitializeConditions for Delay: '<S30>/Integer Delay1' */
      controlMCUSlugsMKIINewNav_DWork.IntegerDelay1_DSTATE_bo[0] = 0.0F;
      controlMCUSlugsMKIINewNav_DWork.IntegerDelay1_DSTATE_bo[1] = 0.0F;

      /* InitializeConditions for Delay: '<S30>/Integer Delay2' */
      controlMCUSlugsMKIINewNav_DWork.IntegerDelay2_DSTATE_n = 0.0F;

      /* InitializeConditions for Delay: '<S46>/Integer Delay3' */
      controlMCUSlugsMKIINewNav_DWork.IntegerDelay3_DSTATE_o5 = 0.0F;

      /* InitializeConditions for Delay: '<S47>/Integer Delay3' */
      controlMCUSlugsMKIINewNav_DWork.IntegerDelay3_DSTATE_ns = 0.0F;
      controlMCUSlugsMKIINewNav_DWork.L1OutputFeedbackControllerWithP = TRUE;
    }

    /* InitialCondition: '<S25>/IC' */
    if (controlMCUSlugsMKIINewNav_DWork.IC_FirstOutputTime_a) {
      controlMCUSlugsMKIINewNav_DWork.IC_FirstOutputTime_a = FALSE;
      rtb_Merge_idx = 1.0F;
    } else {
      /* Abs: '<S32>/Abs1' incorporates:
       *  Sum: '<S35>/Diff'
       *  UnitDelay: '<S35>/UD'
       */
      rtb_Product3_p4_idx = (real32_T)fabs
        (controlMCUSlugsMKIINewNav_B.GetRangeofValuesnavSupportc_n[0] -
         controlMCUSlugsMKIINewNav_DWork.UD_DSTATE_b);

      /* Saturate: '<S32>/Saturation1' */
      rtb_Merge_idx = rtb_Product3_p4_idx >= 1.0F ? 1.0F : rtb_Product3_p4_idx <=
        0.0F ? 0.0F : rtb_Product3_p4_idx;
    }

    /* End of InitialCondition: '<S25>/IC' */

    /* Outputs for Enabled SubSystem: '<S25>/Compute Coef' incorporates:
     *  EnablePort: '<S31>/Enable'
     */
    if (rtb_Merge_idx > 0.0F) {
      /* Gain: '<S31>/-T' */
      controlMCUSlugsMKIINewNav_B.T_m = -0.01F *
        controlMCUSlugsMKIINewNav_B.GetRangeofValuesnavSupportc_n[0];

      /* S-Function "dsPIC_C_function_Call" Block: <S33>/C Function Call */
      controlMCUSlugsMKIINewNav_B.CFunctionCall_ai = myExp
        (controlMCUSlugsMKIINewNav_B.T_m);

      /* SignalConversion: '<S34>/Numerical Unity' */
      controlMCUSlugsMKIINewNav_B.NumericalUnity_p =
        controlMCUSlugsMKIINewNav_B.CFunctionCall_ai;

      /* Sum: '<S31>/1-c' incorporates:
       *  Constant: '<S31>/Constant'
       */
      controlMCUSlugsMKIINewNav_B.c_k = (real32_T)(1.0 - (real_T)
        controlMCUSlugsMKIINewNav_B.NumericalUnity_p);
    }

    /* End of Outputs for SubSystem: '<S25>/Compute Coef' */

    /* Sum: '<S25>/Subtract' incorporates:
     *  Delay: '<S25>/Integer Delay'
     *  Delay: '<S25>/Integer Delay1'
     *  Product: '<S25>/Divide'
     *  Product: '<S25>/Divide1'
     */
    controlMCUSlugsMKIINewNav_B.Subtract_g = controlMCUSlugsMKIINewNav_B.c_k *
      controlMCUSlugsMKIINewNav_DWork.IntegerDelay_DSTATE_l +
      controlMCUSlugsMKIINewNav_B.NumericalUnity_p *
      controlMCUSlugsMKIINewNav_DWork.IntegerDelay1_DSTATE_k;

    /* Switch: '<S27>/Switch3' incorporates:
     *  Delay: '<S27>/Integer Delay3'
     *  RelationalOperator: '<S27>/Relational Operator2'
     */
    if ((Switch3_a == Switch3_a) > 0) {
      controlMCUSlugsMKIINewNav_B.Switch3_ob = Switch3_a;
    } else {
      controlMCUSlugsMKIINewNav_B.Switch3_ob =
        controlMCUSlugsMKIINewNav_DWork.IntegerDelay3_DSTATE_nd;
    }

    /* End of Switch: '<S27>/Switch3' */

    /* InitialCondition: '<S29>/IC' */
    if (controlMCUSlugsMKIINewNav_DWork.IC_FirstOutputTime_m) {
      controlMCUSlugsMKIINewNav_DWork.IC_FirstOutputTime_m = FALSE;
      rtb_Merge_idx = 1.0F;
    } else {
      /* Abs: '<S42>/Abs1' incorporates:
       *  Sum: '<S45>/Diff'
       *  UnitDelay: '<S45>/UD'
       */
      rtb_Product3_p4_idx = (real32_T)fabs
        (controlMCUSlugsMKIINewNav_B.GetRangeofValuesnavSupportc_n[1] -
         controlMCUSlugsMKIINewNav_DWork.UD_DSTATE_o);

      /* Saturate: '<S42>/Saturation1' */
      rtb_Merge_idx = rtb_Product3_p4_idx >= 1.0F ? 1.0F : rtb_Product3_p4_idx <=
        0.0F ? 0.0F : rtb_Product3_p4_idx;
    }

    /* End of InitialCondition: '<S29>/IC' */

    /* Outputs for Enabled SubSystem: '<S29>/Compute Coef' incorporates:
     *  EnablePort: '<S41>/Enable'
     */
    if (rtb_Merge_idx > 0.0F) {
      /* Gain: '<S41>/-T' */
      controlMCUSlugsMKIINewNav_B.T = -0.01F *
        controlMCUSlugsMKIINewNav_B.GetRangeofValuesnavSupportc_n[1];

      /* S-Function "dsPIC_C_function_Call" Block: <S43>/C Function Call */
      controlMCUSlugsMKIINewNav_B.CFunctionCall_d = myExp
        (controlMCUSlugsMKIINewNav_B.T);

      /* SignalConversion: '<S44>/Numerical Unity' */
      controlMCUSlugsMKIINewNav_B.NumericalUnity_f =
        controlMCUSlugsMKIINewNav_B.CFunctionCall_d;

      /* Sum: '<S41>/1-c' incorporates:
       *  Constant: '<S41>/Constant'
       */
      controlMCUSlugsMKIINewNav_B.c_i = (real32_T)(1.0 - (real_T)
        controlMCUSlugsMKIINewNav_B.NumericalUnity_f);
    }

    /* End of Outputs for SubSystem: '<S29>/Compute Coef' */

    /* Sum: '<S29>/Subtract' incorporates:
     *  Delay: '<S29>/Integer Delay'
     *  Delay: '<S29>/Integer Delay1'
     *  Product: '<S29>/Divide'
     *  Product: '<S29>/Divide1'
     */
    controlMCUSlugsMKIINewNav_B.Subtract_f = controlMCUSlugsMKIINewNav_B.c_i *
      controlMCUSlugsMKIINewNav_DWork.IntegerDelay_DSTATE_f +
      controlMCUSlugsMKIINewNav_B.NumericalUnity_f *
      controlMCUSlugsMKIINewNav_DWork.IntegerDelay1_DSTATE_i;

    /* Gain: '<S15>/Gain' incorporates:
     *  Sum: '<S15>/Sum3'
     */
    rtb_Merge_idx = -(controlMCUSlugsMKIINewNav_B.Subtract_f -
                      controlMCUSlugsMKIINewNav_B.Switch3_ob);

    /* Product: '<S40>/Divide4' incorporates:
     *  Constant: '<S40>/Constant'
     *  Delay: '<S15>/Integer Delay1'
     */
    rtb_RhhcosphisinlambYe = 1.0F / controlMCUSlugsMKIINewNa_ConstB.Divide2 *
      controlMCUSlugsMKIINewNav_DWork.IntegerDelay1_DSTATE_c * 2.0F;

    /* Product: '<S40>/Divide3' incorporates:
     *  Delay: '<S15>/Integer Delay1'
     *  Product: '<S40>/Divide'
     *  Sum: '<S40>/Subtract'
     */
    rtb_Deg2R1 = (controlMCUSlugsMKIINewNav_DWork.IntegerDelay1_DSTATE_c *
                  controlMCUSlugsMKIINewNav_DWork.IntegerDelay1_DSTATE_c -
                  controlMCUSlugsMKIINewNa_ConstB.Divide1) /
      controlMCUSlugsMKIINewNa_ConstB.Divide2;

    /* Logic: '<S26>/Logical Operator2' incorporates:
     *  Constant: '<S36>/Constant'
     *  Constant: '<S37>/Constant'
     *  Product: '<S26>/Divide2'
     *  RelationalOperator: '<S36>/Compare'
     *  RelationalOperator: '<S37>/Compare'
     */
    rtb_IC2 = ((rtb_RhhcosphisinlambYe * rtb_Merge_idx > 0.0F) && (rtb_Deg2R1 >=
                0.0F));

    /* Outputs for Enabled SubSystem: '<S26>/Enabled Subsystem' incorporates:
     *  EnablePort: '<S38>/Enable'
     */
    /* Logic: '<S26>/Logical Operator' incorporates:
     *  Inport: '<S38>/In1'
     */
    if (!(rtb_IC2 != 0) > 0) {
      controlMCUSlugsMKIINewNav_B.Merge_e = rtb_Merge_idx;
    }

    /* End of Logic: '<S26>/Logical Operator' */
    /* End of Outputs for SubSystem: '<S26>/Enabled Subsystem' */

    /* Outputs for Enabled SubSystem: '<S26>/Enabled Subsystem1' incorporates:
     *  EnablePort: '<S39>/Enable'
     */
    if (rtb_IC2 > 0) {
      /* Product: '<S39>/Divide2' incorporates:
       *  Constant: '<S39>/Constant'
       *  Product: '<S39>/Divide1'
       *  Signum: '<S39>/Sign'
       *  Sum: '<S39>/Subtract'
       */
      controlMCUSlugsMKIINewNav_B.Merge_e = (1.0F - (rtb_RhhcosphisinlambYe <
        0.0F ? -1.0F : rtb_RhhcosphisinlambYe > 0.0F ? 1.0F :
        rtb_RhhcosphisinlambYe == 0.0F ? 0.0F : rtb_RhhcosphisinlambYe) *
        rtb_Deg2R1) * rtb_Merge_idx;
    }

    /* End of Outputs for SubSystem: '<S26>/Enabled Subsystem1' */

    /* Product: '<S15>/Projection' */
    rtb_Merge_idx = controlMCUSlugsMKIINewNav_B.GetRangeofValuesnavSupportc_n[2]
      * controlMCUSlugsMKIINewNav_B.Merge_e;

    /* Sum: '<S15>/Sum4' incorporates:
     *  Delay: '<S15>/Integer Delay'
     */
    rtb_Product3_p4_idx = controlMCUSlugsMKIINewNav_DWork.IntegerDelay_DSTATE_cl
      - controlMCUSlugsMKIINewNav_B.Subtract_g;

    /* Saturate: '<S15>/Psi Dot  Limit' */
    controlMCUSlugsMKIINewNav_B.PsiDotLimit = rtb_Product3_p4_idx >= 1.0F ? 1.0F
      : rtb_Product3_p4_idx <= -1.0F ? -1.0F : rtb_Product3_p4_idx;

    /* Sum: '<S15>/Sum2' incorporates:
     *  Delay: '<S15>/Integer Delay1'
     */
    rtb_RhhcosphisinlambYe = controlMCUSlugsMKIINewNav_B.PsiDotLimit +
      controlMCUSlugsMKIINewNav_DWork.IntegerDelay1_DSTATE_c;

    /* Switch: '<S28>/Switch3' incorporates:
     *  Delay: '<S28>/Integer Delay3'
     *  RelationalOperator: '<S28>/Relational Operator2'
     */
    if ((rtb_RhhcosphisinlambYe == rtb_RhhcosphisinlambYe) > 0) {
      controlMCUSlugsMKIINewNav_B.Switch3_a3 = rtb_RhhcosphisinlambYe;
    } else {
      controlMCUSlugsMKIINewNav_B.Switch3_a3 =
        controlMCUSlugsMKIINewNav_DWork.IntegerDelay3_DSTATE_e;
    }

    /* End of Switch: '<S28>/Switch3' */

    /* Switch: '<S46>/Switch3' incorporates:
     *  Delay: '<S46>/Integer Delay3'
     *  RelationalOperator: '<S46>/Relational Operator2'
     */
    if ((rtb_Merge_idx == rtb_Merge_idx) > 0) {
      controlMCUSlugsMKIINewNav_B.Switch3_fi = rtb_Merge_idx;
    } else {
      controlMCUSlugsMKIINewNav_B.Switch3_fi =
        controlMCUSlugsMKIINewNav_DWork.IntegerDelay3_DSTATE_o5;
    }

    /* End of Switch: '<S46>/Switch3' */

    /* Switch: '<S30>/Switch1' incorporates:
     *  Constant: '<S30>/Constant5'
     *  Logic: '<S15>/Logical Operator'
     */
    if (!(controlMCUSlugsMKIINewNav_B.GetRangeofValuesnavSupportc_n[3] != 0.0F) >
        0) {
      rtb_Merge_idx = 0.0F;
    } else {
      /* Sum: '<S30>/Subtract1' incorporates:
       *  Constant: '<S30>/Constant'
       *  Constant: '<S30>/Constant1'
       *  Constant: '<S30>/Constant2'
       *  Constant: '<S30>/Constant3'
       *  Delay: '<S30>/Integer Delay'
       *  Delay: '<S30>/Integer Delay1'
       *  Delay: '<S30>/Integer Delay2'
       *  Product: '<S30>/Product'
       *  Product: '<S30>/Product1'
       *  Product: '<S30>/Product2'
       *  Product: '<S30>/Product3'
       *  Sum: '<S30>/Subtract'
       */
      rtb_Product3_p4_idx = ((controlMCUSlugsMKIINewNav_B.Switch3_fi *
        0.333333343F + controlMCUSlugsMKIINewNav_DWork.IntegerDelay_DSTATE_fj *
        1.33333337F) + controlMCUSlugsMKIINewNav_DWork.IntegerDelay1_DSTATE_bo[0]
        * 0.333333343F) * 0.005F +
        controlMCUSlugsMKIINewNav_DWork.IntegerDelay2_DSTATE_n;

      /* Saturate: '<S30>/[min max]' */
      rtb_Merge_idx = rtb_Product3_p4_idx >= 2.0F ? 2.0F : rtb_Product3_p4_idx <=
        -2.0F ? -2.0F : rtb_Product3_p4_idx;
    }

    /* End of Switch: '<S30>/Switch1' */

    /* Switch: '<S47>/Switch3' incorporates:
     *  Delay: '<S47>/Integer Delay3'
     *  RelationalOperator: '<S47>/Relational Operator2'
     */
    if ((rtb_Merge_idx == rtb_Merge_idx) > 0) {
      controlMCUSlugsMKIINewNav_B.Switch3_ik = rtb_Merge_idx;
    } else {
      controlMCUSlugsMKIINewNav_B.Switch3_ik =
        controlMCUSlugsMKIINewNav_DWork.IntegerDelay3_DSTATE_ns;
    }

    /* End of Switch: '<S47>/Switch3' */
  } else {
    if (controlMCUSlugsMKIINewNav_DWork.L1OutputFeedbackControllerWithP) {
      controlMCUSlugsMKIINewNav_DWork.L1OutputFeedbackControllerWithP = FALSE;
    }
  }

  /* End of Outputs for SubSystem: '<S3>/L1 Output Feedback Controller With  Projection Operator' */
  /* S-Function "dsPIC_C_function_Call" Block: <S10>/Get the Accelerometers [navSupport.c] */
  getAccels(&controlMCUSlugsMKIINewNav_B.GettheAccelerometersnavSupportc[0]);

  /* Product: '<S10>/Product2' */
  for (i = 0; i < 3; i++) {
    rtb_Product2_ep[i] = rtb_MathFunction[i + 6] *
      controlMCUSlugsMKIINewNav_B.GettheAccelerometersnavSupportc[2] +
      (rtb_MathFunction[i + 3] *
       controlMCUSlugsMKIINewNav_B.GettheAccelerometersnavSupportc[1] +
       rtb_MathFunction[i] *
       controlMCUSlugsMKIINewNav_B.GettheAccelerometersnavSupportc[0]);
  }

  /* End of Product: '<S10>/Product2' */
  /* S-Function "dsPIC_C_function_Call" Block: <S10>/Get the Accel Bias [navSupport.c] */
  getAccBias(&controlMCUSlugsMKIINewNav_B.GettheAccelBiasnavSupportc[0]);

  /* Sum: '<S10>/Sum' incorporates:
   *  Product: '<S10>/Product3'
   */
  for (i = 0; i < 3; i++) {
    rtb_Product6[i] = ((rtb_MathFunction[i + 3] *
                        controlMCUSlugsMKIINewNav_B.GettheAccelBiasnavSupportc[1]
                        + rtb_MathFunction[i] *
                        controlMCUSlugsMKIINewNav_B.GettheAccelBiasnavSupportc[0])
                       + rtb_MathFunction[i + 6] *
                       controlMCUSlugsMKIINewNav_B.GettheAccelBiasnavSupportc[2])
      + rtb_Product2_ep[i];
  }

  /* End of Sum: '<S10>/Sum' */

  /* Outputs for Atomic SubSystem: '<S3>/Lateral Channel Encaps' */
  /* Switch: '<S83>/Switch3' incorporates:
   *  Delay: '<S83>/Integer Delay3'
   *  RelationalOperator: '<S83>/Relational Operator2'
   */
  if ((rtb_BankLimitCommand == rtb_BankLimitCommand) > 0) {
    Switch3_m = rtb_BankLimitCommand;
  } else {
    Switch3_m = controlMCUSlugsMKIINewNav_DWork.IntegerDelay3_DSTATE_m;
  }

  /* End of Switch: '<S83>/Switch3' */

  /* Saturate: '<S82>/bank Limit' */
  controlMCUSlugsMKIINewNav_B.bankLimit = Switch3_m >= 0.436332315F ?
    0.436332315F : Switch3_m <= -0.436332315F ? -0.436332315F : Switch3_m;

  /* S-Function "dsPIC_C_function_Call" Block: <S85>/C Function Call */
  controlMCUSlugsMKIINewNav_B.CFunctionCall_ao = myTan
    (controlMCUSlugsMKIINewNav_B.bankLimit);

  /* Switch: '<S84>/Switch3' incorporates:
   *  Delay: '<S84>/Integer Delay3'
   *  RelationalOperator: '<S84>/Relational Operator2'
   */
  if ((rtb_ScheduleLPF == rtb_ScheduleLPF) > 0) {
    Switch3_l = rtb_ScheduleLPF;
  } else {
    Switch3_l = controlMCUSlugsMKIINewNav_DWork.IntegerDelay3_DSTATE_k;
  }

  /* End of Switch: '<S84>/Switch3' */

  /* Switch: '<S50>/Switch1' incorporates:
   *  Constant: '<S82>/Constant1'
   *  Product: '<S82>/Divide'
   *  Product: '<S82>/Divide1'
   *  Saturate: '<S82>/[0 40]'
   */
  if (controlMCUSlugsMKIINewNav_B.isitinMidLevelPtorSPTnavSupport > 0) {
    Switch3_ae = controlMCUSlugsMKIINewNav_B.GetMidLevelCommandsnavSupportc[2];
  } else {
    Switch3_ae = 1.0F / (Switch3_l >= 40.0F ? 40.0F : Switch3_l <= 0.0F ? 0.0F :
                         Switch3_l) * 9.80665F *
      controlMCUSlugsMKIINewNav_B.CFunctionCall_ao;
  }

  /* End of Switch: '<S50>/Switch1' */

  /* Switch: '<S50>/Switch' */
  if (controlMCUSlugsMKIINewNav_B.GetRangeofValuesnavSupportc_n[3] > 0.3F) {
    rtb_Product3_p4_idx = controlMCUSlugsMKIINewNav_B.PsiDotLimit;
  } else {
    rtb_Product3_p4_idx = Switch3_ae;
  }

  /* End of Switch: '<S50>/Switch' */

  /* Saturate: '<S48>/Psi Dot  Limit' */
  rtb_PsiDotLimit = rtb_Product3_p4_idx >= 1.0F ? 1.0F : rtb_Product3_p4_idx <=
    -1.0F ? -1.0F : rtb_Product3_p4_idx;

  /* Outputs for Enabled SubSystem: '<S48>/Sideslip Compensation' incorporates:
   *  EnablePort: '<S56>/Enable'
   */
  if (controlMCUSlugsMKIINewNav_B.GetRangeofValuesnavSupportc_k[2] > 0.0F) {
    if (!controlMCUSlugsMKIINewNav_DWork.SideslipCompensation_MODE) {
      /* InitializeConditions for UnitDelay: '<S73>/UD' */
      controlMCUSlugsMKIINewNav_DWork.UD_DSTATE = 0.0F;

      /* InitializeConditions for Delay: '<S65>/Integer Delay' */
      controlMCUSlugsMKIINewNav_DWork.IntegerDelay_DSTATE_h = 0.0F;

      /* InitializeConditions for Delay: '<S65>/Integer Delay1' */
      controlMCUSlugsMKIINewNav_DWork.IntegerDelay1_DSTATE_e = 0.0F;

      /* InitializeConditions for Delay: '<S66>/Integer Delay3' */
      controlMCUSlugsMKIINewNav_DWork.IntegerDelay3_DSTATE_ij = 0.0F;
      controlMCUSlugsMKIINewNav_DWork.SideslipCompensation_MODE = TRUE;
    }

    /* MATLAB Function: '<S64>/negprotect' */
    /* MATLAB Function 'Control Blocks/Lateral Channel/Sideslip Compensation/Bank to Psi Dot/negprotect': '<S69>:1' */
    /*  This block supports an embeddable subset of the MATLAB language. */
    /*  See the help menu for details.  */
    if (rtb_ScheduleLPF >= 0.001F) {
      /* '<S69>:1:5' */
      /* '<S69>:1:6' */
      rtb_Merge_idx = rtb_ScheduleLPF;
    } else {
      /* '<S69>:1:8' */
      rtb_Merge_idx = 0.001F;
    }

    /* End of MATLAB Function: '<S64>/negprotect' */

    /* Product: '<S64>/Divide' incorporates:
     *  Constant: '<S64>/Constant1'
     */
    rtb_RhhcosphisinlambYe = 1.0F / rtb_Merge_idx * 9.80665F;

    /* Saturate: '<S64>/bank Limit' */
    controlMCUSlugsMKIINewNav_B.bankLimit_o = controlMCUSlugsMKIINewNav_B.y[0] >=
      0.436332315F ? 0.436332315F : controlMCUSlugsMKIINewNav_B.y[0] <=
      -0.436332315F ? -0.436332315F : controlMCUSlugsMKIINewNav_B.y[0];

    /* S-Function "dsPIC_C_function_Call" Block: <S68>/C Function Call */
    controlMCUSlugsMKIINewNav_B.CFunctionCall_mlv = myTan
      (controlMCUSlugsMKIINewNav_B.bankLimit_o);

    /* InitialCondition: '<S65>/IC' */
    if (controlMCUSlugsMKIINewNav_DWork.IC_FirstOutputTime_h) {
      controlMCUSlugsMKIINewNav_DWork.IC_FirstOutputTime_h = FALSE;
      rtb_Merge_idx = 1.0F;
    } else {
      /* Abs: '<S72>/Abs1' incorporates:
       *  Constant: '<S56>/Constant'
       *  Sum: '<S73>/Diff'
       *  UnitDelay: '<S73>/UD'
       */
      rtb_Product3_p4_idx = (real32_T)fabs(0.3F -
        controlMCUSlugsMKIINewNav_DWork.UD_DSTATE);

      /* Saturate: '<S72>/Saturation1' */
      rtb_Merge_idx = rtb_Product3_p4_idx >= 1.0F ? 1.0F : rtb_Product3_p4_idx <=
        0.0F ? 0.0F : rtb_Product3_p4_idx;
    }

    /* End of InitialCondition: '<S65>/IC' */

    /* Outputs for Enabled SubSystem: '<S65>/Compute Coef' incorporates:
     *  EnablePort: '<S71>/Enable'
     */
    if (rtb_Merge_idx > 0.0F) {
      /* Math: '<S71>/Math Function'
       *
       * About '<S71>/Math Function':
       *  Operator: exp
       */
      controlMCUSlugsMKIINewNav_B.c = 0.997004509F;

      /* Sum: '<S71>/1-c' incorporates:
       *  Constant: '<S71>/Constant'
       */
      controlMCUSlugsMKIINewNav_B.c_d = (real32_T)(1.0 - (real_T)
        controlMCUSlugsMKIINewNav_B.c);
    }

    /* End of Outputs for SubSystem: '<S65>/Compute Coef' */

    /* Sum: '<S65>/Subtract' incorporates:
     *  Delay: '<S65>/Integer Delay'
     *  Delay: '<S65>/Integer Delay1'
     *  Product: '<S65>/Divide'
     *  Product: '<S65>/Divide1'
     */
    controlMCUSlugsMKIINewNav_B.Subtract = controlMCUSlugsMKIINewNav_B.c_d *
      controlMCUSlugsMKIINewNav_DWork.IntegerDelay_DSTATE_h +
      controlMCUSlugsMKIINewNav_B.c *
      controlMCUSlugsMKIINewNav_DWork.IntegerDelay1_DSTATE_e;

    /* Delay: '<S66>/Integer Delay3' */
    rtb_Merge_idx = controlMCUSlugsMKIINewNav_DWork.IntegerDelay3_DSTATE_ij;

    /* Product: '<S67>/Divide' incorporates:
     *  Constant: '<S67>/Constant1'
     *  Product: '<S64>/Divide1'
     *  Sum: '<S56>/Subtract'
     */
    controlMCUSlugsMKIINewNav_B.Divide_b =
      (controlMCUSlugsMKIINewNav_B.CFunctionCall_mlv * rtb_RhhcosphisinlambYe -
       controlMCUSlugsMKIINewNav_B.y[5]) * rtb_ScheduleLPF / 9.80665F;

    /* S-Function "dsPIC_C_function_Call" Block: <S74>/C Function Call */
    controlMCUSlugsMKIINewNav_B.CFunctionCall_h = myAtan
      (controlMCUSlugsMKIINewNav_B.Divide_b);

    /* Switch: '<S66>/Switch3' incorporates:
     *  RelationalOperator: '<S66>/Relational Operator2'
     */
    if ((controlMCUSlugsMKIINewNav_B.CFunctionCall_h ==
         controlMCUSlugsMKIINewNav_B.CFunctionCall_h) > 0) {
      rtb_Merge_idx = controlMCUSlugsMKIINewNav_B.CFunctionCall_h;
    }

    /* End of Switch: '<S66>/Switch3' */

    /* Update for UnitDelay: '<S73>/UD' incorporates:
     *  Constant: '<S56>/Constant'
     */
    controlMCUSlugsMKIINewNav_DWork.UD_DSTATE = 0.3F;

    /* Update for Delay: '<S65>/Integer Delay' */
    controlMCUSlugsMKIINewNav_DWork.IntegerDelay_DSTATE_h = rtb_Merge_idx;

    /* Update for Delay: '<S65>/Integer Delay1' */
    controlMCUSlugsMKIINewNav_DWork.IntegerDelay1_DSTATE_e =
      controlMCUSlugsMKIINewNav_B.Subtract;

    /* Update for Delay: '<S66>/Integer Delay3' */
    controlMCUSlugsMKIINewNav_DWork.IntegerDelay3_DSTATE_ij = rtb_Merge_idx;
  } else {
    if (controlMCUSlugsMKIINewNav_DWork.SideslipCompensation_MODE) {
      /* Disable for Outport: '<S56>/bankComp' */
      controlMCUSlugsMKIINewNav_B.Subtract = 0.0F;
      controlMCUSlugsMKIINewNav_DWork.SideslipCompensation_MODE = FALSE;
    }
  }

  /* End of Outputs for SubSystem: '<S48>/Sideslip Compensation' */

  /* Product: '<S58>/Divide' incorporates:
   *  Constant: '<S58>/Constant1'
   */
  controlMCUSlugsMKIINewNav_B.Divide = rtb_PsiDotLimit * rtb_ScheduleLPF /
    9.80665F;

  /* S-Function "dsPIC_C_function_Call" Block: <S80>/C Function Call */
  controlMCUSlugsMKIINewNav_B.CFunctionCall_kw = myAtan
    (controlMCUSlugsMKIINewNav_B.Divide);

  /* Sum: '<S48>/Add2' */
  rtb_Product3_p4_idx = controlMCUSlugsMKIINewNav_B.Subtract +
    controlMCUSlugsMKIINewNav_B.CFunctionCall_kw;

  /* Saturate: '<S48>/Bank  Limit Command' */
  rtb_BankLimitCommand_i = rtb_Product3_p4_idx >= 0.436332315F ? 0.436332315F :
    rtb_Product3_p4_idx <= -0.436332315F ? -0.436332315F : rtb_Product3_p4_idx;

  /* Switch: '<S54>/Switch3' incorporates:
   *  Delay: '<S54>/Integer Delay3'
   *  RelationalOperator: '<S54>/Relational Operator2'
   */
  if ((rtb_Product6[1] == rtb_Product6[1]) > 0) {
    rtb_OnOff = rtb_Product6[1];
  } else {
    rtb_OnOff = controlMCUSlugsMKIINewNav_DWork.IntegerDelay3_DSTATE_h;
  }

  /* End of Switch: '<S54>/Switch3' */

  /* Saturate: '<S48>/[-20 20]' */
  rtb_u020 = rtb_OnOff >= 20.0F ? 20.0F : rtb_OnOff <= -20.0F ? -20.0F :
    rtb_OnOff;

  /* MATLAB Function: '<S51>/Embedded MATLAB Function' */
  controlM_EmbeddedMATLABFunction(rtb_u020, 0.01, 10.0,
    &controlMCUSlugsMKIINewNav_B.sf_EmbeddedMATLABFunction_b,
    &controlMCUSlugsMKIINewNav_DWork.sf_EmbeddedMATLABFunction_b);

  /* Gain: '<S48>/Neg Feedback ' */
  controlMCUSlugsMKIINewNav_B.NegFeedback =
    -controlMCUSlugsMKIINewNav_B.sf_EmbeddedMATLABFunction_b.y;

  /* MATLAB Function: '<S16>/myMux Fun1' */
  controlMCUSlugsMKIINe_myMuxFun1(rtb_PsiDotLimit, rtb_BankLimitCommand_i,
    controlMCUSlugsMKIINewNav_B.NegFeedback,
    &controlMCUSlugsMKIINewNav_B.sf_myMuxFun1_f);

  /* S-Function "dsPIC_C_function_Call" Block: <S16>/setNavLat [navSupport.c] */
  setNavLat(controlMCUSlugsMKIINewNav_B.sf_myMuxFun1_f.y);

  /* Sum: '<S57>/Add3' incorporates:
   *  Constant: '<S57>/SaturationLimit'
   */
  Switch3_lh = 0.17453292F - controlMCUSlugsMKIINewNav_B.DataTypeConversion_k;

  /* Sum: '<S57>/Add1' incorporates:
   *  Constant: '<S57>/delayTime'
   *  Delay: '<S57>/NDelays'
   *  Product: '<S57>/Product1'
   *  Product: '<S57>/Product2'
   *  Product: '<S57>/Product3'
   *  Sum: '<S57>/Sum'
   */
  rtb_Merge_idx = (controlMCUSlugsMKIINewNav_B.NegFeedback -
                   controlMCUSlugsMKIINewNav_DWork.NDelays_DSTATE_j[0]) / 0.05F *
    controlMCUSlugsMKIINewNav_B.GetRangeofValuesnavSupportc_d[2] +
    controlMCUSlugsMKIINewNav_B.NegFeedback *
    controlMCUSlugsMKIINewNav_B.GetRangeofValuesnavSupportc_d[0];

  /* Sum: '<S57>/Sum2' incorporates:
   *  Gain: '<S57>/Gain'
   *  Memory: '<S57>/Memory1'
   *  Product: '<S57>/Product4'
   */
  Product = (0.01F * controlMCUSlugsMKIINewNav_B.NegFeedback *
             controlMCUSlugsMKIINewNav_B.GetRangeofValuesnavSupportc_d[1] +
             controlMCUSlugsMKIINewNav_DWork.Memory1_PreviousInput_n) +
    rtb_Merge_idx;

  /* Switch: '<S57>/AntiWindup' incorporates:
   *  Constant: '<S57>/Constant5'
   *  Constant: '<S57>/SaturationLimit'
   *  Constant: '<S57>/SaturationLimit1'
   *  Logic: '<S57>/Logical Operator'
   *  RelationalOperator: '<S57>/Relational Operator'
   *  RelationalOperator: '<S57>/Relational Operator1'
   *  Sum: '<S57>/Add3'
   *  Sum: '<S57>/Add4'
   */
  if ((Product > -0.17453292F - controlMCUSlugsMKIINewNav_B.DataTypeConversion_k)
      && (Product < 0.17453292F -
          controlMCUSlugsMKIINewNav_B.DataTypeConversion_k)) {
    Product = controlMCUSlugsMKIINewNav_B.NegFeedback;
  } else {
    Product = 0.0F;
  }

  /* End of Switch: '<S57>/AntiWindup' */

  /* Switch: '<S78>/Switch3' incorporates:
   *  Delay: '<S78>/Integer Delay3'
   *  RelationalOperator: '<S78>/Relational Operator2'
   */
  if ((Product == Product) > 0) {
    rtb_Deg2R_m_idx_0 = Product;
  } else {
    rtb_Deg2R_m_idx_0 = controlMCUSlugsMKIINewNav_DWork.IntegerDelay3_DSTATE_mx;
  }

  /* End of Switch: '<S78>/Switch3' */

  /* Switch: '<S77>/Switch1' incorporates:
   *  Constant: '<S77>/Constant'
   *  Constant: '<S77>/Constant1'
   *  Constant: '<S77>/Constant2'
   *  Constant: '<S77>/Constant3'
   *  Constant: '<S77>/Constant5'
   *  Delay: '<S77>/Integer Delay'
   *  Delay: '<S77>/Integer Delay1'
   *  Delay: '<S77>/Integer Delay2'
   *  Product: '<S77>/Product'
   *  Product: '<S77>/Product1'
   *  Product: '<S77>/Product2'
   *  Product: '<S77>/Product3'
   *  Sum: '<S77>/Subtract'
   *  Sum: '<S77>/Subtract1'
   */
  if (controlMCUSlugsMKIINewNav_B.ManualorAutonavSupportc > 0) {
    rtb_Deg2R_m_idx = 0.0F;
  } else {
    rtb_Deg2R_m_idx = ((rtb_Deg2R_m_idx_0 * 0.333333343F +
                        controlMCUSlugsMKIINewNav_DWork.IntegerDelay_DSTATE_ie *
                        1.33333337F) +
                       controlMCUSlugsMKIINewNav_DWork.IntegerDelay1_DSTATE_jy[0]
                       * 0.333333343F) * 0.005F +
      controlMCUSlugsMKIINewNav_DWork.IntegerDelay2_DSTATE_o;
  }

  /* End of Switch: '<S77>/Switch1' */

  /* Switch: '<S79>/Switch3' incorporates:
   *  Delay: '<S79>/Integer Delay3'
   *  RelationalOperator: '<S79>/Relational Operator2'
   */
  if ((rtb_Deg2R_m_idx == rtb_Deg2R_m_idx) > 0) {
    rtb_Product3_p4_idx = rtb_Deg2R_m_idx;
  } else {
    rtb_Product3_p4_idx =
      controlMCUSlugsMKIINewNav_DWork.IntegerDelay3_DSTATE_gf;
  }

  /* End of Switch: '<S79>/Switch3' */

  /* Switch: '<S57>/On//Off' incorporates:
   *  Constant: '<S57>/Constant1'
   *  Product: '<S57>/Product'
   *  Sum: '<S57>/Add2'
   */
  if (controlMCUSlugsMKIINewNav_B.ManualorAutonavSupportc > 0) {
    rtb_Add4_l = 0.0F;
  } else {
    rtb_Add4_l = rtb_Product3_p4_idx *
      controlMCUSlugsMKIINewNav_B.GetRangeofValuesnavSupportc_d[1] +
      rtb_Merge_idx;
  }

  /* End of Switch: '<S57>/On//Off' */

  /* Switch: '<S76>/Switch2' incorporates:
   *  Constant: '<S57>/SaturationLimit'
   *  RelationalOperator: '<S76>/LowerRelop1'
   *  Sum: '<S57>/Add3'
   */
  if (!(rtb_Add4_l > 0.17453292F -
        controlMCUSlugsMKIINewNav_B.DataTypeConversion_k)) {
    /* Switch: '<S76>/Switch' incorporates:
     *  Constant: '<S57>/SaturationLimit1'
     *  RelationalOperator: '<S76>/UpperRelop'
     *  Sum: '<S57>/Add4'
     */
    if (rtb_Add4_l < -0.17453292F -
        controlMCUSlugsMKIINewNav_B.DataTypeConversion_k) {
      Switch3_lh = -0.17453292F -
        controlMCUSlugsMKIINewNav_B.DataTypeConversion_k;
    } else {
      Switch3_lh = rtb_Add4_l;
    }

    /* End of Switch: '<S76>/Switch' */
  }

  /* End of Switch: '<S76>/Switch2' */

  /* Sum: '<S48>/Add1' */
  rtb_Deg2R_m_idx = rtb_BankLimitCommand_i - controlMCUSlugsMKIINewNav_B.y[0];

  /* Sum: '<S55>/Add3' incorporates:
   *  Constant: '<S55>/SaturationLimit'
   */
  Product = 0.383972436F - controlMCUSlugsMKIINewNav_B.DataTypeConversion_je;

  /* Switch: '<S53>/Switch3' incorporates:
   *  Delay: '<S53>/Integer Delay3'
   *  RelationalOperator: '<S53>/Relational Operator2'
   */
  if ((rtb_Deg2R_m_idx == rtb_Deg2R_m_idx) > 0) {
    rtb_RhhcosphicoslambXe = rtb_Deg2R_m_idx;
  } else {
    rtb_RhhcosphicoslambXe =
      controlMCUSlugsMKIINewNav_DWork.IntegerDelay3_DSTATE_p;
  }

  /* End of Switch: '<S53>/Switch3' */

  /* Sum: '<S55>/Add1' incorporates:
   *  Constant: '<S55>/delayTime'
   *  Delay: '<S55>/NDelays'
   *  Product: '<S55>/Product1'
   *  Product: '<S55>/Product2'
   *  Product: '<S55>/Product3'
   *  Sum: '<S55>/Sum'
   */
  rtb_Merge_idx = (rtb_RhhcosphicoslambXe -
                   controlMCUSlugsMKIINewNav_DWork.NDelays_DSTATE_a[0]) / 0.05F *
    controlMCUSlugsMKIINewNav_B.GetRangeofValuesnavSupportc_h[2] +
    rtb_RhhcosphicoslambXe *
    controlMCUSlugsMKIINewNav_B.GetRangeofValuesnavSupportc_h[0];

  /* Sum: '<S55>/Sum2' incorporates:
   *  Gain: '<S55>/Gain'
   *  Memory: '<S55>/Memory1'
   *  Product: '<S55>/Product4'
   */
  rtb_Deg2R_m_idx = (0.01F * rtb_RhhcosphicoslambXe *
                     controlMCUSlugsMKIINewNav_B.GetRangeofValuesnavSupportc_h[1]
                     + controlMCUSlugsMKIINewNav_DWork.Memory1_PreviousInput_g)
    + rtb_Merge_idx;

  /* Switch: '<S55>/AntiWindup' incorporates:
   *  Constant: '<S55>/Constant5'
   *  Constant: '<S55>/SaturationLimit'
   *  Constant: '<S55>/SaturationLimit1'
   *  Logic: '<S55>/Logical Operator'
   *  RelationalOperator: '<S55>/Relational Operator'
   *  RelationalOperator: '<S55>/Relational Operator1'
   *  Sum: '<S55>/Add3'
   *  Sum: '<S55>/Add4'
   */
  if ((rtb_Deg2R_m_idx > -0.383972436F -
       controlMCUSlugsMKIINewNav_B.DataTypeConversion_je) && (rtb_Deg2R_m_idx <
       0.383972436F - controlMCUSlugsMKIINewNav_B.DataTypeConversion_je)) {
    rtb_Deg2R_m_idx = rtb_RhhcosphicoslambXe;
  } else {
    rtb_Deg2R_m_idx = 0.0F;
  }

  /* End of Switch: '<S55>/AntiWindup' */

  /* Switch: '<S62>/Switch3' incorporates:
   *  Delay: '<S62>/Integer Delay3'
   *  RelationalOperator: '<S62>/Relational Operator2'
   */
  if ((rtb_Deg2R_m_idx == rtb_Deg2R_m_idx) > 0) {
    rtb_cosphi = rtb_Deg2R_m_idx;
  } else {
    rtb_cosphi = controlMCUSlugsMKIINewNav_DWork.IntegerDelay3_DSTATE_c5;
  }

  /* End of Switch: '<S62>/Switch3' */

  /* Switch: '<S61>/Switch1' incorporates:
   *  Constant: '<S61>/Constant'
   *  Constant: '<S61>/Constant1'
   *  Constant: '<S61>/Constant2'
   *  Constant: '<S61>/Constant3'
   *  Constant: '<S61>/Constant5'
   *  Delay: '<S61>/Integer Delay'
   *  Delay: '<S61>/Integer Delay1'
   *  Delay: '<S61>/Integer Delay2'
   *  Product: '<S61>/Product'
   *  Product: '<S61>/Product1'
   *  Product: '<S61>/Product2'
   *  Product: '<S61>/Product3'
   *  Sum: '<S61>/Subtract'
   *  Sum: '<S61>/Subtract1'
   */
  if (controlMCUSlugsMKIINewNav_B.ManualorAutonavSupportc > 0) {
    rtb_Deg2R_m_idx = 0.0F;
  } else {
    rtb_Deg2R_m_idx = ((rtb_cosphi * 0.333333343F +
                        controlMCUSlugsMKIINewNav_DWork.IntegerDelay_DSTATE_k *
                        1.33333337F) +
                       controlMCUSlugsMKIINewNav_DWork.IntegerDelay1_DSTATE_b[0]
                       * 0.333333343F) * 0.005F +
      controlMCUSlugsMKIINewNav_DWork.IntegerDelay2_DSTATE_g4;
  }

  /* End of Switch: '<S61>/Switch1' */

  /* Switch: '<S63>/Switch3' incorporates:
   *  Delay: '<S63>/Integer Delay3'
   *  RelationalOperator: '<S63>/Relational Operator2'
   */
  if ((rtb_Deg2R_m_idx == rtb_Deg2R_m_idx) > 0) {
    rtb_Ze = rtb_Deg2R_m_idx;
  } else {
    rtb_Ze = controlMCUSlugsMKIINewNav_DWork.IntegerDelay3_DSTATE_le;
  }

  /* End of Switch: '<S63>/Switch3' */

  /* Switch: '<S55>/On//Off' incorporates:
   *  Constant: '<S55>/Constant1'
   *  Product: '<S55>/Product'
   *  Sum: '<S55>/Add2'
   */
  if (controlMCUSlugsMKIINewNav_B.ManualorAutonavSupportc > 0) {
    rtb_Deg2R1 = 0.0F;
  } else {
    rtb_Deg2R1 = rtb_Ze *
      controlMCUSlugsMKIINewNav_B.GetRangeofValuesnavSupportc_h[1] +
      rtb_Merge_idx;
  }

  /* End of Switch: '<S55>/On//Off' */

  /* Switch: '<S60>/Switch2' incorporates:
   *  Constant: '<S55>/SaturationLimit'
   *  RelationalOperator: '<S60>/LowerRelop1'
   *  Sum: '<S55>/Add3'
   */
  if (!(rtb_Deg2R1 > 0.383972436F -
        controlMCUSlugsMKIINewNav_B.DataTypeConversion_je)) {
    /* Switch: '<S60>/Switch' incorporates:
     *  Constant: '<S55>/SaturationLimit1'
     *  RelationalOperator: '<S60>/UpperRelop'
     *  Sum: '<S55>/Add4'
     */
    if (rtb_Deg2R1 < -0.383972436F -
        controlMCUSlugsMKIINewNav_B.DataTypeConversion_je) {
      Product = -0.383972436F -
        controlMCUSlugsMKIINewNav_B.DataTypeConversion_je;
    } else {
      Product = rtb_Deg2R1;
    }

    /* End of Switch: '<S60>/Switch' */
  }

  /* End of Switch: '<S60>/Switch2' */

  /* Sum: '<S48>/Add3' */
  Product += controlMCUSlugsMKIINewNav_B.DataTypeConversion_je;

  /* Saturate: '<S48>/Aileron Limit' */
  rtb_Merge_idx = Product >= 0.383972436F ? 0.383972436F : Product <=
    -0.383972436F ? -0.383972436F : Product;

  /* Switch: '<S48>/Switch2' incorporates:
   *  Constant: '<S48>/Constant'
   *  Product: '<S48>/Product'
   */
  if (controlMCUSlugsMKIINewNav_B.GetRangeofValuesnavSupportc_c[1] > 0.3F) {
    rtb_Deg2R_m_idx = controlMCUSlugsMKIINewNav_B.GetRangeofValuesnavSupportc_c
      [2] * rtb_Merge_idx;
  } else {
    rtb_Deg2R_m_idx = 0.0F;
  }

  /* End of Switch: '<S48>/Switch2' */

  /* Sum: '<S48>/Add4' incorporates:
   *  Sum: '<S48>/Add'
   */
  rtb_Deg2R_m_idx += Switch3_lh +
    controlMCUSlugsMKIINewNav_B.DataTypeConversion_k;

  /* Update for Delay: '<S83>/Integer Delay3' */
  controlMCUSlugsMKIINewNav_DWork.IntegerDelay3_DSTATE_m = Switch3_m;

  /* Update for Delay: '<S84>/Integer Delay3' */
  controlMCUSlugsMKIINewNav_DWork.IntegerDelay3_DSTATE_k = Switch3_l;

  /* Update for Delay: '<S54>/Integer Delay3' */
  controlMCUSlugsMKIINewNav_DWork.IntegerDelay3_DSTATE_h = rtb_OnOff;

  /* Update for Delay: '<S57>/NDelays' */
  controlMCUSlugsMKIINewNav_DWork.NDelays_DSTATE_j[0] =
    controlMCUSlugsMKIINewNav_DWork.NDelays_DSTATE_j[1];
  controlMCUSlugsMKIINewNav_DWork.NDelays_DSTATE_j[1] =
    controlMCUSlugsMKIINewNav_DWork.NDelays_DSTATE_j[2];
  controlMCUSlugsMKIINewNav_DWork.NDelays_DSTATE_j[2] =
    controlMCUSlugsMKIINewNav_DWork.NDelays_DSTATE_j[3];
  controlMCUSlugsMKIINewNav_DWork.NDelays_DSTATE_j[3] =
    controlMCUSlugsMKIINewNav_DWork.NDelays_DSTATE_j[4];
  controlMCUSlugsMKIINewNav_DWork.NDelays_DSTATE_j[4] =
    controlMCUSlugsMKIINewNav_B.NegFeedback;

  /* Update for Delay: '<S77>/Integer Delay2' */
  controlMCUSlugsMKIINewNav_DWork.IntegerDelay2_DSTATE_o = rtb_Product3_p4_idx;

  /* Update for Memory: '<S57>/Memory1' */
  controlMCUSlugsMKIINewNav_DWork.Memory1_PreviousInput_n = rtb_Add4_l;

  /* Update for Delay: '<S78>/Integer Delay3' */
  controlMCUSlugsMKIINewNav_DWork.IntegerDelay3_DSTATE_mx = rtb_Deg2R_m_idx_0;

  /* Update for Delay: '<S77>/Integer Delay' */
  controlMCUSlugsMKIINewNav_DWork.IntegerDelay_DSTATE_ie = rtb_Deg2R_m_idx_0;

  /* Update for Delay: '<S77>/Integer Delay1' */
  controlMCUSlugsMKIINewNav_DWork.IntegerDelay1_DSTATE_jy[0] =
    controlMCUSlugsMKIINewNav_DWork.IntegerDelay1_DSTATE_jy[1];
  controlMCUSlugsMKIINewNav_DWork.IntegerDelay1_DSTATE_jy[1] = rtb_Deg2R_m_idx_0;

  /* Update for Delay: '<S79>/Integer Delay3' */
  controlMCUSlugsMKIINewNav_DWork.IntegerDelay3_DSTATE_gf = rtb_Product3_p4_idx;

  /* Update for Delay: '<S53>/Integer Delay3' */
  controlMCUSlugsMKIINewNav_DWork.IntegerDelay3_DSTATE_p =
    rtb_RhhcosphicoslambXe;

  /* Update for Delay: '<S55>/NDelays' */
  controlMCUSlugsMKIINewNav_DWork.NDelays_DSTATE_a[0] =
    controlMCUSlugsMKIINewNav_DWork.NDelays_DSTATE_a[1];
  controlMCUSlugsMKIINewNav_DWork.NDelays_DSTATE_a[1] =
    controlMCUSlugsMKIINewNav_DWork.NDelays_DSTATE_a[2];
  controlMCUSlugsMKIINewNav_DWork.NDelays_DSTATE_a[2] =
    controlMCUSlugsMKIINewNav_DWork.NDelays_DSTATE_a[3];
  controlMCUSlugsMKIINewNav_DWork.NDelays_DSTATE_a[3] =
    controlMCUSlugsMKIINewNav_DWork.NDelays_DSTATE_a[4];
  controlMCUSlugsMKIINewNav_DWork.NDelays_DSTATE_a[4] = rtb_RhhcosphicoslambXe;

  /* Update for Delay: '<S61>/Integer Delay2' */
  controlMCUSlugsMKIINewNav_DWork.IntegerDelay2_DSTATE_g4 = rtb_Ze;

  /* Update for Memory: '<S55>/Memory1' */
  controlMCUSlugsMKIINewNav_DWork.Memory1_PreviousInput_g = rtb_Deg2R1;

  /* Update for Delay: '<S62>/Integer Delay3' */
  controlMCUSlugsMKIINewNav_DWork.IntegerDelay3_DSTATE_c5 = rtb_cosphi;

  /* Update for Delay: '<S61>/Integer Delay' */
  controlMCUSlugsMKIINewNav_DWork.IntegerDelay_DSTATE_k = rtb_cosphi;

  /* Update for Delay: '<S61>/Integer Delay1' */
  controlMCUSlugsMKIINewNav_DWork.IntegerDelay1_DSTATE_b[0] =
    controlMCUSlugsMKIINewNav_DWork.IntegerDelay1_DSTATE_b[1];
  controlMCUSlugsMKIINewNav_DWork.IntegerDelay1_DSTATE_b[1] = rtb_cosphi;

  /* Update for Delay: '<S63>/Integer Delay3' */
  controlMCUSlugsMKIINewNav_DWork.IntegerDelay3_DSTATE_le = rtb_Ze;

  /* MATLAB Function: '<Root>/myMux Fun1' incorporates:
   *  Saturate: '<S48>/Rudder Limit'
   */
  /* MATLAB Function 'myMux Fun1': '<S11>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S11>:1:5' */
  rtb_Product3_p4_idx = rtb_Deg2R_m_idx >= 0.17453292F ? 0.17453292F :
    rtb_Deg2R_m_idx <= -0.17453292F ? -0.17453292F : rtb_Deg2R_m_idx;

  /* End of Outputs for SubSystem: '<S3>/Lateral Channel Encaps' */

  /* If: '<S442>/If' */
  if (controlMCUSlugsMKIINewNav_B.ManualorAutonavSupportc == 1) {
    /* Outputs for IfAction SubSystem: '<S442>/If  Control Type Is Manual' incorporates:
     *  ActionPort: '<S444>/Action Port'
     */
    /* Gain: '<S444>/Gain4' incorporates:
     *  SignalConversion: '<S444>/TmpSignal ConversionAtGain4Inport1'
     */
    controlMCUSlugsMKIINewNav_B.Merge_f[0] = rtb_y_c[0] >> 1;
    controlMCUSlugsMKIINewNav_B.Merge_f[1] = rtb_y_c[1] >> 1;
    controlMCUSlugsMKIINewNav_B.Merge_f[2] = rtb_y_c[3] >> 1;
    controlMCUSlugsMKIINewNav_B.Merge_f[3] = rtb_y_c[2] >> 1;

    /* End of Outputs for SubSystem: '<S442>/If  Control Type Is Manual' */
  } else if ((controlMCUSlugsMKIINewNav_B.NavigationModenavSupportc == 3) ||
             (controlMCUSlugsMKIINewNav_B.NavigationModenavSupportc == 4) ||
             (controlMCUSlugsMKIINewNav_B.NavigationModenavSupportc == 9) ||
             (controlMCUSlugsMKIINewNav_B.NavigationModenavSupportc == 10)) {
    /* Outputs for IfAction SubSystem: '<S442>/If  Control Type Is Wp, ISR, LP or  Mid Lvl Cmds' incorporates:
     *  ActionPort: '<S447>/Action Port'
     */
    /* DataTypeConversion: '<S453>/Data Type Conversion' incorporates:
     *  Constant: '<S453>/Constant1'
     *  Constant: '<S453>/Constant2'
     *  MATLAB Function: '<Root>/myMux Fun1'
     *  Product: '<S453>/Divide'
     *  Sum: '<S453>/Add'
     */
    tmp_1 = floor((real_T)rtb_ElevatorLimit * 6016.0568488736444 +
                  3766.6666666666679);
    if (rtIsNaN(tmp_1) || rtIsInf(tmp_1)) {
      tmp_1 = 0.0;
    } else {
      tmp_1 = fmod(tmp_1, 65536.0);
    }

    rtb_DataTypeConversion_pv = (uint16_T)tmp_1;

    /* End of DataTypeConversion: '<S453>/Data Type Conversion' */

    /* DataTypeConversion: '<S454>/Data Type Conversion' incorporates:
     *  Constant: '<S454>/Constant1'
     *  Constant: '<S454>/Constant2'
     *  MATLAB Function: '<Root>/myMux Fun1'
     *  Product: '<S454>/Divide'
     *  Sum: '<S454>/Add'
     */
    tmp_1 = floor((real_T)rtb_ThrottleLimit * 1274.0 + 3176.0);
    if (rtIsNaN(tmp_1) || rtIsInf(tmp_1)) {
      tmp_1 = 0.0;
    } else {
      tmp_1 = fmod(tmp_1, 65536.0);
    }

    rtb_DataTypeConversion_lc = (uint16_T)tmp_1;

    /* End of DataTypeConversion: '<S454>/Data Type Conversion' */

    /* DataTypeConversion: '<S455>/Data Type Conversion' incorporates:
     *  Constant: '<S455>/Constant1'
     *  Constant: '<S455>/Constant2'
     *  MATLAB Function: '<Root>/myMux Fun1'
     *  Product: '<S455>/Divide'
     *  Sum: '<S455>/Add'
     */
    tmp_1 = floor((real_T)rtb_Merge_idx * -4679.1553269017213 +
                  3683.3333333333344);
    if (rtIsNaN(tmp_1) || rtIsInf(tmp_1)) {
      tmp_1 = 0.0;
    } else {
      tmp_1 = fmod(tmp_1, 65536.0);
    }

    rtb_DataTypeConversion_i3 = (uint16_T)tmp_1;

    /* End of DataTypeConversion: '<S455>/Data Type Conversion' */

    /* DataTypeConversion: '<S456>/Data Type Conversion' incorporates:
     *  Constant: '<S456>/Constant1'
     *  Constant: '<S456>/Constant2'
     *  Product: '<S456>/Divide'
     *  Sum: '<S456>/Add'
     */
    tmp_1 = floor((real_T)rtb_Product3_p4_idx * -2900.5988378497932 +
                  3725.0000000000009);
    if (rtIsNaN(tmp_1) || rtIsInf(tmp_1)) {
      tmp_1 = 0.0;
    } else {
      tmp_1 = fmod(tmp_1, 65536.0);
    }

    rtb_DataTypeConversion_b = (uint16_T)tmp_1;

    /* End of DataTypeConversion: '<S456>/Data Type Conversion' */

    /* MATLAB Function: '<S447>/myMux Fun1' */
    controlMCUSlugsMKII_myMuxFun1_e(rtb_DataTypeConversion_lc,
      rtb_DataTypeConversion_i3, rtb_DataTypeConversion_b,
      rtb_DataTypeConversion_pv, controlMCUSlugsMKIINewNav_B.Merge_f);

    /* End of Outputs for SubSystem: '<S442>/If  Control Type Is Wp, ISR, LP or  Mid Lvl Cmds' */
  } else if (controlMCUSlugsMKIINewNav_B.NavigationModenavSupportc == 2) {
    /* Outputs for IfAction SubSystem: '<S442>/If  Control Type Is Passthrough' incorporates:
     *  ActionPort: '<S445>/Action Port'
     */
    /* Gain: '<S445>/Gain4' incorporates:
     *  SignalConversion: '<S445>/TmpSignal ConversionAtGain4Inport1'
     */
    controlMCUSlugsMKIINewNav_B.Merge_f[0] = rtb_y_c[0] >> 1;
    controlMCUSlugsMKIINewNav_B.Merge_f[1] = rtb_y_c[1] >> 1;
    controlMCUSlugsMKIINewNav_B.Merge_f[2] = rtb_y_c[3] >> 1;
    controlMCUSlugsMKIINewNav_B.Merge_f[3] = rtb_y_c[2] >> 1;

    /* End of Outputs for SubSystem: '<S442>/If  Control Type Is Passthrough' */
  } else {
    if (controlMCUSlugsMKIINewNav_B.NavigationModenavSupportc == 8) {
      /* Outputs for IfAction SubSystem: '<S442>/If  Control Type Is Selective Passthrough' incorporates:
       *  ActionPort: '<S446>/Action Port'
       */
      /* S-Function "dsPIC_C_function_Call" Block: <S446>/Get Pass Switches [navSupport.c] */
      getPassValues(&controlMCUSlugsMKIINewNav_B.GetPassSwitchesnavSupportc[0]);

      /* Switch: '<S446>/Switch' incorporates:
       *  DataTypeConversion: '<S449>/Data Type Conversion'
       *  Gain: '<S446>/Gain'
       */
      if (controlMCUSlugsMKIINewNav_B.GetPassSwitchesnavSupportc[3] >= 1) {
        rtb_Switch_it = rtb_y_c[0] >> 1;
      } else {
        /* DataTypeConversion: '<S449>/Data Type Conversion' incorporates:
         *  Constant: '<S449>/Constant1'
         *  Constant: '<S449>/Constant2'
         *  MATLAB Function: '<Root>/myMux Fun1'
         *  Product: '<S449>/Divide'
         *  Sum: '<S449>/Add'
         */
        tmp_1 = floor((real_T)rtb_ThrottleLimit * 1274.0 + 3176.0);
        if (rtIsNaN(tmp_1) || rtIsInf(tmp_1)) {
          tmp_1 = 0.0;
        } else {
          tmp_1 = fmod(tmp_1, 65536.0);
        }

        rtb_Switch_it = (uint16_T)tmp_1;
      }

      /* End of Switch: '<S446>/Switch' */

      /* Switch: '<S446>/Switch1' incorporates:
       *  DataTypeConversion: '<S450>/Data Type Conversion'
       *  Gain: '<S446>/Gain1'
       */
      if (controlMCUSlugsMKIINewNav_B.GetPassSwitchesnavSupportc[2] >= 1) {
        rtb_Switch1_h = rtb_y_c[1] >> 1;
      } else {
        /* DataTypeConversion: '<S450>/Data Type Conversion' incorporates:
         *  Constant: '<S450>/Constant1'
         *  Constant: '<S450>/Constant2'
         *  MATLAB Function: '<Root>/myMux Fun1'
         *  Product: '<S450>/Divide'
         *  Sum: '<S450>/Add'
         */
        tmp_1 = floor((real_T)rtb_Merge_idx * -4679.1553269017213 +
                      3683.3333333333344);
        if (rtIsNaN(tmp_1) || rtIsInf(tmp_1)) {
          tmp_1 = 0.0;
        } else {
          tmp_1 = fmod(tmp_1, 65536.0);
        }

        rtb_Switch1_h = (uint16_T)tmp_1;
      }

      /* End of Switch: '<S446>/Switch1' */

      /* Switch: '<S446>/Switch2' incorporates:
       *  DataTypeConversion: '<S451>/Data Type Conversion'
       *  Gain: '<S446>/Gain2'
       */
      if (controlMCUSlugsMKIINewNav_B.GetPassSwitchesnavSupportc[1] >= 1) {
        rtb_Switch2 = rtb_y_c[3] >> 1;
      } else {
        /* DataTypeConversion: '<S451>/Data Type Conversion' incorporates:
         *  Constant: '<S451>/Constant1'
         *  Constant: '<S451>/Constant2'
         *  Product: '<S451>/Divide'
         *  Sum: '<S451>/Add'
         */
        tmp_1 = floor((real_T)rtb_Product3_p4_idx * -2900.5988378497932 +
                      3725.0000000000009);
        if (rtIsNaN(tmp_1) || rtIsInf(tmp_1)) {
          tmp_1 = 0.0;
        } else {
          tmp_1 = fmod(tmp_1, 65536.0);
        }

        rtb_Switch2 = (uint16_T)tmp_1;
      }

      /* End of Switch: '<S446>/Switch2' */

      /* Switch: '<S446>/Switch3' incorporates:
       *  DataTypeConversion: '<S448>/Data Type Conversion'
       *  Gain: '<S446>/Gain3'
       */
      if (controlMCUSlugsMKIINewNav_B.GetPassSwitchesnavSupportc[0] >= 1) {
        rtb_Switch3 = rtb_y_c[2] >> 1;
      } else {
        /* DataTypeConversion: '<S448>/Data Type Conversion' incorporates:
         *  Constant: '<S448>/Constant1'
         *  Constant: '<S448>/Constant2'
         *  MATLAB Function: '<Root>/myMux Fun1'
         *  Product: '<S448>/Divide'
         *  Sum: '<S448>/Add'
         */
        tmp_1 = floor((real_T)rtb_ElevatorLimit * 6016.0568488736444 +
                      3766.6666666666679);
        if (rtIsNaN(tmp_1) || rtIsInf(tmp_1)) {
          tmp_1 = 0.0;
        } else {
          tmp_1 = fmod(tmp_1, 65536.0);
        }

        rtb_Switch3 = (uint16_T)tmp_1;
      }

      /* End of Switch: '<S446>/Switch3' */

      /* MATLAB Function: '<S446>/myMux Fun1' */
      controlMCUSlugsMKII_myMuxFun1_e(rtb_Switch_it, rtb_Switch1_h, rtb_Switch2,
        rtb_Switch3, controlMCUSlugsMKIINewNav_B.Merge_f);

      /* End of Outputs for SubSystem: '<S442>/If  Control Type Is Selective Passthrough' */
    }
  }

  /* End of If: '<S442>/If' */
  /* S-Function "dsPIC_PWM_motor" Block: <S9>/PWM Servo Output [dR dE dA dT] */
  PDC1 = controlMCUSlugsMKIINewNav_B.Merge_f[2] ;
  PDC2 = controlMCUSlugsMKIINewNav_B.Merge_f[3] ;
  PDC3 = controlMCUSlugsMKIINewNav_B.Merge_f[1] ;
  PDC4 = controlMCUSlugsMKIINewNav_B.Merge_f[0] ;

  /* DataTypeConversion: '<S416>/Data Type Conversion2' */
  rtb_DataTypeConversion2 = (real32_T)
    controlMCUSlugsMKIINewNav_B.InputCapture_o6;

  /* MATLAB Function: '<S425>/Embedded MATLAB Function' */
  controlM_EmbeddedMATLABFunction(rtb_DataTypeConversion2, 0.01, 0.1,
    &controlMCUSlugsMKIINewNav_B.sf_EmbeddedMATLABFunction,
    &controlMCUSlugsMKIINewNav_DWork.sf_EmbeddedMATLABFunction);

  /* DataTypeConversion: '<S416>/Data Type Conversion1' */
  rtb_Product3_p4_idx = (real32_T)floor
    (controlMCUSlugsMKIINewNav_B.sf_EmbeddedMATLABFunction.y);
  if (rtIsNaNF(rtb_Product3_p4_idx) || rtIsInfF(rtb_Product3_p4_idx)) {
    rtb_Product3_p4_idx = 0.0F;
  } else {
    rtb_Product3_p4_idx = (real32_T)fmod(rtb_Product3_p4_idx, 65536.0F);
  }

  /* DataTypeConversion: '<S416>/Data Type Conversion3' */
  rtb_DataTypeConversion3 = (real32_T)
    controlMCUSlugsMKIINewNav_B.InputCapture_o1;

  /* MATLAB Function: '<S426>/Embedded MATLAB Function' */
  controlM_EmbeddedMATLABFunction(rtb_DataTypeConversion3, 0.01, 0.1,
    &controlMCUSlugsMKIINewNav_B.sf_EmbeddedMATLABFunction_i,
    &controlMCUSlugsMKIINewNav_DWork.sf_EmbeddedMATLABFunction_i);

  /* MATLAB Function: '<S416>/myMux Fun1' incorporates:
   *  DataTypeConversion: '<S416>/Data Type Conversion1'
   */
  /* MATLAB Function 'Trim Vals/Control Surface Input/myMux Fun1': '<S427>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S427>:1:5' */
  controlMCUSlugsMKIINewNav_B.y_l[0] = rtb_Product3_p4_idx < 0.0F ? (uint16_T)
    -(int16_T)(uint16_T)-rtb_Product3_p4_idx : (uint16_T)rtb_Product3_p4_idx;

  /* DataTypeConversion: '<S416>/Data Type Conversion4' */
  rtb_Product3_p4_idx = (real32_T)floor
    (controlMCUSlugsMKIINewNav_B.sf_EmbeddedMATLABFunction_i.y);
  if (rtIsNaNF(rtb_Product3_p4_idx) || rtIsInfF(rtb_Product3_p4_idx)) {
    rtb_Product3_p4_idx = 0.0F;
  } else {
    rtb_Product3_p4_idx = (real32_T)fmod(rtb_Product3_p4_idx, 65536.0F);
  }

  /* MATLAB Function: '<S416>/myMux Fun1' incorporates:
   *  DataTypeConversion: '<S416>/Data Type Conversion4'
   */
  controlMCUSlugsMKIINewNav_B.y_l[1] = rtb_Product3_p4_idx < 0.0F ? (uint16_T)
    -(int16_T)(uint16_T)-rtb_Product3_p4_idx : (uint16_T)rtb_Product3_p4_idx;

  /* DataTypeConversion: '<S441>/Data Type Conversion' incorporates:
   *  Gain: '<S441>/Convert to  Microseconds'
   */
  controlMCUSlugsMKIINewNav_B.DataTypeConversion_jet[0] = (uint16_T)(52429UL *
    (uint32_T)controlMCUSlugsMKIINewNav_B.Merge_f[0] >> 17);
  controlMCUSlugsMKIINewNav_B.DataTypeConversion_jet[1] = (uint16_T)(52429UL *
    (uint32_T)controlMCUSlugsMKIINewNav_B.Merge_f[1] >> 17);
  controlMCUSlugsMKIINewNav_B.DataTypeConversion_jet[2] = (uint16_T)(52429UL *
    (uint32_T)controlMCUSlugsMKIINewNav_B.Merge_f[2] >> 17);
  controlMCUSlugsMKIINewNav_B.DataTypeConversion_jet[3] = (uint16_T)(52429UL *
    (uint32_T)controlMCUSlugsMKIINewNav_B.Merge_f[3] >> 17);

  /* S-Function "dsPIC_C_function_Call" Block: <S9>/Update PWM  Commands [updateControlMcu.c] */
  updatePWM(controlMCUSlugsMKIINewNav_B.DataTypeConversion_jet);

  /* S-Function "dsPIC_C_function_Call" Block: <S9>/Update the Euler Angles [updateControlMcu.c]  */
  updateEuler(controlMCUSlugsMKIINewNav_B.y_i);

  /* S-Function "dsPIC_C_function_Call" Block: <S9>/Update PQR [updateControlMcu.c]  */
  updatePQR(controlMCUSlugsMKIINewNav_B.Product1);

  /* S-Function "dsPIC_C_function_Call" Block: <S9>/Update Voltage Current Sensor  [updateControlMcu.c] */
  updateVISensor(controlMCUSlugsMKIINewNav_B.y_l);
  if (controlMCUSlugsMKIINewNav_M->Timing.TaskCounters.TID[1] == 0) {
    /* S-Function "dsPIC_C_function_Call" Block: <S9>/Create Telemetry  Sentence [mavlinkCommsControlMcu.c] */
    prepareTelemetryMavlink
      (&controlMCUSlugsMKIINewNav_B.CreateTelemetrySentencemavlinkC[0]);

    /* S-Function "dsPIC_C_function_Call" Block: <S9>/Send Data to Radio Modem [mavlinkCommsControlMcu.c] */
    send2GS(controlMCUSlugsMKIINewNav_B.CreateTelemetrySentencemavlinkC);
  }

  /* S-Function "dsPIC_C_function_Call" Block: <S1>/Get Range of Values [navSupport.c]      */
  getRangeOfParams(((uint8_T)29U),((uint8_T)30U),
                   &controlMCUSlugsMKIINewNav_B.GetRangeofValuesnavSupportc_d3[0]);

  /* MATLAB Function: '<S1>/myMux Fun4' incorporates:
   *  Constant: '<S1>/Constant'
   */
  /* MATLAB Function 'Camera Location/myMux Fun4': '<S12>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S12>:1:5' */
  rtb_Product6[0] = controlMCUSlugsMKIINewNav_B.GetRangeofValuesnavSupportc_d3[0];
  rtb_Product6[1] = 0.0F;
  rtb_Product6[2] = controlMCUSlugsMKIINewNav_B.GetRangeofValuesnavSupportc_d3[1];

  /* S-Function "dsPIC_C_function_Call" Block: <S9>/ISR Option 1 for Camera Tracking [navSupport.c] */
  controlMCUSlugsMKIINewNav_B.ISROption1forCameraTrackingnavS =
    getISRCameraOption1();

  /* RelationalOperator: '<S458>/Compare' incorporates:
   *  Constant: '<S458>/Constant'
   */
  rtb_IC2 = (controlMCUSlugsMKIINewNav_B.ISROption1forCameraTrackingnavS == 1);

  /* Outputs for Enabled SubSystem: '<S443>/Camera Track' incorporates:
   *  EnablePort: '<S460>/Enable'
   */
  if (rtb_IC2 > 0) {
    /* Trigonometry: '<S482>/SinCos' */
    rtb_Product3_p4_idx = (real32_T)sin(controlMCUSlugsMKIINewNav_B.y_i[0]);
    rtb_Deg2R_m_idx_0 = (real32_T)cos(controlMCUSlugsMKIINewNav_B.y_i[0]);
    rtb_Deg2R_m_idx = (real32_T)sin(controlMCUSlugsMKIINewNav_B.y_i[1]);
    rtb_Add4_l = (real32_T)cos(controlMCUSlugsMKIINewNav_B.y_i[1]);
    rtb_OnOff = (real32_T)sin(controlMCUSlugsMKIINewNav_B.y_i[2]);
    Switch3_lh = (real32_T)cos(controlMCUSlugsMKIINewNav_B.y_i[2]);

    /* Product: '<S487>/u(5)*u(6)' */
    rtb_VectorConcatenate_i[0] = rtb_Add4_l * Switch3_lh;

    /* Sum: '<S490>/Sum' incorporates:
     *  Product: '<S490>/u(3)*u(4)'
     *  Product: '<S490>/u(6)*u(1)*u(2)'
     */
    rtb_VectorConcatenate_i[1] = Switch3_lh * rtb_Product3_p4_idx *
      rtb_Deg2R_m_idx - rtb_OnOff * rtb_Deg2R_m_idx_0;

    /* Sum: '<S493>/Sum' incorporates:
     *  Product: '<S493>/u(1)*u(3)'
     *  Product: '<S493>/u(2)*u(4)*u(6)'
     */
    rtb_VectorConcatenate_i[2] = rtb_Deg2R_m_idx * rtb_Deg2R_m_idx_0 *
      Switch3_lh + rtb_Product3_p4_idx * rtb_OnOff;

    /* Product: '<S488>/u(3)*u(5)' */
    rtb_VectorConcatenate_i[3] = rtb_OnOff * rtb_Add4_l;

    /* Sum: '<S491>/Sum' incorporates:
     *  Product: '<S491>/u(1)*u(2)*u(3)'
     *  Product: '<S491>/u(4)*u(6)'
     */
    rtb_VectorConcatenate_i[4] = rtb_Product3_p4_idx * rtb_Deg2R_m_idx *
      rtb_OnOff + rtb_Deg2R_m_idx_0 * Switch3_lh;

    /* Sum: '<S494>/Sum' incorporates:
     *  Product: '<S494>/u(1)*u(6)'
     *  Product: '<S494>/u(2)*u(3)*u(4)'
     */
    rtb_VectorConcatenate_i[5] = rtb_Deg2R_m_idx * rtb_OnOff * rtb_Deg2R_m_idx_0
      - Switch3_lh * rtb_Product3_p4_idx;

    /* Gain: '<S489>/Gain2' */
    rtb_VectorConcatenate_i[6] = -rtb_Deg2R_m_idx;

    /* Product: '<S492>/u(1)*u(3)' */
    rtb_VectorConcatenate_i[7] = rtb_Product3_p4_idx * rtb_Add4_l;

    /* Product: '<S495>/u(4)*u(5)' */
    rtb_VectorConcatenate_i[8] = rtb_Deg2R_m_idx_0 * rtb_Add4_l;

    /* Sum: '<S460>/Subtract2' incorporates:
     *  Math: '<S478>/Math Function'
     *  Product: '<S460>/Product1'
     *  Sum: '<S460>/Subtract1'
     */
    for (i = 0; i < 3; i++) {
      rtb_Subtract2[i] = rtb_Product5[i] - (((rtb_VectorConcatenate_i[3 * i + 1]
        * 0.0F + rtb_VectorConcatenate_i[3 * i] * rtb_Product6[0]) +
        rtb_VectorConcatenate_i[3 * i + 2] * rtb_Product6[2]) + rtb_Puav[i]);
    }

    /* End of Sum: '<S460>/Subtract2' */

    /* MATLAB Function: '<S500>/Embedded MATLAB Function' */
    contro_EmbeddedMATLABFunction_o(rtb_Subtract2,
      &controlMCUSlugsMKIINewNav_B.sf_EmbeddedMATLABFunction_lb);

    /* MATLAB Function: '<S501>/negprotect' */
    controlMCUSlugsMKIIN_negprotect
      (controlMCUSlugsMKIINewNav_B.sf_EmbeddedMATLABFunction_lb.xDoty,
       &controlMCUSlugsMKIINewNav_B.sf_negprotect_j);

    /* S-Function "dsPIC_C_function_Call" Block: <S501>/C Function Call */
    controlMCUSlugsMKIINewNav_B.CFunctionCall_o = mySqrt
      (controlMCUSlugsMKIINewNav_B.sf_negprotect_j.zpVal);

    /* Product: '<S460>/Normalize' */
    rtb_Product3_p4_idx = rtb_Subtract2[0] /
      controlMCUSlugsMKIINewNav_B.CFunctionCall_o;
    rtb_Deg2R_m_idx_0 = rtb_Subtract2[1] /
      controlMCUSlugsMKIINewNav_B.CFunctionCall_o;
    rtb_Deg2R_m_idx = rtb_Subtract2[2] /
      controlMCUSlugsMKIINewNav_B.CFunctionCall_o;

    /* Product: '<S460>/Product2' */
    for (i = 0; i < 3; i++) {
      controlMCUSlugsMKIINewNav_B.Product2[i] = 0.0F;
      controlMCUSlugsMKIINewNav_B.Product2[i] = rtb_VectorConcatenate_i[i] *
        rtb_Product3_p4_idx + controlMCUSlugsMKIINewNav_B.Product2[i];
      controlMCUSlugsMKIINewNav_B.Product2[i] = rtb_VectorConcatenate_i[i + 3] *
        rtb_Deg2R_m_idx_0 + controlMCUSlugsMKIINewNav_B.Product2[i];
      controlMCUSlugsMKIINewNav_B.Product2[i] = rtb_VectorConcatenate_i[i + 6] *
        rtb_Deg2R_m_idx + controlMCUSlugsMKIINewNav_B.Product2[i];
    }

    /* End of Product: '<S460>/Product2' */

    /* Saturate: '<S480>/[-1,1]' */
    controlMCUSlugsMKIINewNav_B.u1 = controlMCUSlugsMKIINewNav_B.Product2[2] >=
      0.9999F ? 0.9999F : controlMCUSlugsMKIINewNav_B.Product2[2] <= -0.999F ?
      -0.999F : controlMCUSlugsMKIINewNav_B.Product2[2];

    /* S-Function "dsPIC_C_function_Call" Block: <S480>/C Function Call */
    controlMCUSlugsMKIINewNav_B.CFunctionCall_oo = myAsin
      (controlMCUSlugsMKIINewNav_B.u1);

    /* DataTypeConversion: '<S475>/Data Type Conversion' incorporates:
     *  Constant: '<S475>/Constant1'
     *  Constant: '<S475>/Constant2'
     *  Product: '<S475>/Divide'
     *  Sum: '<S475>/Add'
     */
    tmp_1 = floor((real_T)controlMCUSlugsMKIINewNav_B.CFunctionCall_oo *
                  357.89048655867009 + 1264.9359721509938);
    if (rtIsNaN(tmp_1) || rtIsInf(tmp_1)) {
      tmp_1 = 0.0;
    } else {
      tmp_1 = fmod(tmp_1, 65536.0);
    }

    /* S-Function "dsPIC_C_function_Call" Block: <S481>/C Function Call */
    controlMCUSlugsMKIINewNav_B.CFunctionCall_m = myAtan2
      (controlMCUSlugsMKIINewNav_B.Product2[1],
       controlMCUSlugsMKIINewNav_B.Product2[0]);

    /* Sum: '<S476>/Add' incorporates:
     *  Constant: '<S476>/Constant1'
     *  Constant: '<S476>/Constant2'
     *  Product: '<S476>/Divide'
     */
    rtb_Add_k = (real_T)controlMCUSlugsMKIINewNav_B.CFunctionCall_m *
      -386.71635603987778 + 977.68421052631561;

    /* S-Function "dsPIC_C_function_Call" Block: <S460>/Set Aides to Zero? [FCBEX1000.c] */
    controlMCUSlugsMKIINewNav_B.SetAidestoZeroFCBEX1000c = getGoHome();

    /* S-Function "dsPIC_C_function_Call" Block: <S460>/Get the Aide Pan [FCBEX1000c] */
    controlMCUSlugsMKIINewNav_B.GettheAidePanFCBEX1000c = getPanISRAide
      (controlMCUSlugsMKIINewNav_B.SetAidestoZeroFCBEX1000c);

    /* S-Function "dsPIC_C_function_Call" Block: <S460>/Get the Aide Tilt [FCBEX1000.c] */
    controlMCUSlugsMKIINewNav_B.GettheAideTiltFCBEX1000c = getTiltISRAide
      (controlMCUSlugsMKIINewNav_B.SetAidestoZeroFCBEX1000c);

    /* MATLAB Function: '<S483>/Embedded MATLAB Function' */
    controlM_EmbeddedMATLABFunction(controlMCUSlugsMKIINewNav_B.y_i[1], 0.01,
      75.0, &controlMCUSlugsMKIINewNav_B.sf_EmbeddedMATLABFunction_px,
      &controlMCUSlugsMKIINewNav_DWork.sf_EmbeddedMATLABFunction_px);

    /* MATLAB Function: '<S484>/Embedded MATLAB Function' */
    controlM_EmbeddedMATLABFunction(controlMCUSlugsMKIINewNav_B.y_i[0], 0.01,
      75.0, &controlMCUSlugsMKIINewNav_B.sf_EmbeddedMATLABFunction_e4,
      &controlMCUSlugsMKIINewNav_DWork.sf_EmbeddedMATLABFunction_e4);

    /* MATLAB Function: '<S485>/Embedded MATLAB Function' */
    controlM_EmbeddedMATLABFunction(controlMCUSlugsMKIINewNav_B.y_i[2], 0.01,
      75.0, &controlMCUSlugsMKIINewNav_B.sf_EmbeddedMATLABFunction_ab,
      &controlMCUSlugsMKIINewNav_DWork.sf_EmbeddedMATLABFunction_ab);

    /* MATLAB Function: '<S477>/myMux Fun5' */
    controlMCUSlugsMKIINe_myMuxFun1
      (controlMCUSlugsMKIINewNav_B.sf_EmbeddedMATLABFunction_e4.y,
       controlMCUSlugsMKIINewNav_B.sf_EmbeddedMATLABFunction_px.y,
       controlMCUSlugsMKIINewNav_B.sf_EmbeddedMATLABFunction_ab.y,
       &controlMCUSlugsMKIINewNav_B.sf_myMuxFun5_c);

    /* DataTypeConversion: '<S476>/Data Type Conversion' */
    rtb_Add_k = floor(rtb_Add_k);
    if (rtIsNaN(rtb_Add_k) || rtIsInf(rtb_Add_k)) {
      rtb_Add_k = 0.0;
    } else {
      rtb_Add_k = fmod(rtb_Add_k, 65536.0);
    }

    /* Sum: '<S460>/Sum' incorporates:
     *  DataTypeConversion: '<S476>/Data Type Conversion'
     */
    controlMCUSlugsMKIINewNav_B.Sum = (rtb_Add_k < 0.0 ? (uint16_T)-(int16_T)
      (uint16_T)-rtb_Add_k : (uint16_T)rtb_Add_k) + (uint16_T)
      controlMCUSlugsMKIINewNav_B.GettheAidePanFCBEX1000c;

    /* Sum: '<S460>/Sum1' incorporates:
     *  DataTypeConversion: '<S475>/Data Type Conversion'
     */
    controlMCUSlugsMKIINewNav_B.Sum1 = (tmp_1 < 0.0 ? (uint16_T)-(int16_T)
      (uint16_T)-tmp_1 : (uint16_T)tmp_1) + (uint16_T)
      controlMCUSlugsMKIINewNav_B.GettheAideTiltFCBEX1000c;

    /* Constant: '<S460>/Zero Roll' */
    controlMCUSlugsMKIINewNav_B.ZeroRoll = 977U;
  }

  /* End of Outputs for SubSystem: '<S443>/Camera Track' */

  /* Outputs for Enabled SubSystem: '<S443>/Camera Stab' incorporates:
   *  EnablePort: '<S459>/Enable'
   */
  /* Logic: '<S443>/Logical Operator1' */
  if (!(rtb_IC2 != 0)) {
    /* S-Function "dsPIC_C_function_Call" Block: <S459>/Set Camera to Home Position? [FCBEX1000.c]1 */
    controlMCUSlugsMKIINewNav_B.SetCameratoHomePositionFCBEX100 = getGoHome();

    /* S-Function "dsPIC_C_function_Call" Block: <S459>/Get the Pan [FCBEX1000.c]1 */
    controlMCUSlugsMKIINewNav_B.GetthePanFCBEX1000c1 = getTiltValue
      (controlMCUSlugsMKIINewNav_B.SetCameratoHomePositionFCBEX100);

    /* Sum: '<S459>/Add' */
    controlMCUSlugsMKIINewNav_B.Add = (real_T)
      controlMCUSlugsMKIINewNav_B.GetthePanFCBEX1000c1;

    /* S-Function "dsPIC_C_function_Call" Block: <S459>/Get the Pan [FCBEX1000.c] */
    controlMCUSlugsMKIINewNav_B.GetthePanFCBEX1000c = getPanValue
      (controlMCUSlugsMKIINewNav_B.SetCameratoHomePositionFCBEX100);

    /* Switch: '<S459>/Switch1' */
    controlMCUSlugsMKIINewNav_B.Switch1_e = 977U;

    /* MATLAB Function: '<S466>/Embedded MATLAB Function' */
    controlM_EmbeddedMATLABFunction(controlMCUSlugsMKIINewNav_B.y_i[1], 0.01,
      60.0, &controlMCUSlugsMKIINewNav_B.sf_EmbeddedMATLABFunction_dk,
      &controlMCUSlugsMKIINewNav_DWork.sf_EmbeddedMATLABFunction_dk);
  }

  /* End of Logic: '<S443>/Logical Operator1' */
  /* End of Outputs for SubSystem: '<S443>/Camera Stab' */

  /* Switch: '<S443>/Switch2' */
  if (rtb_IC2 > 0) {
    controlMCUSlugsMKIINewNav_B.Switch2_b = controlMCUSlugsMKIINewNav_B.Sum;
  } else {
    controlMCUSlugsMKIINewNav_B.Switch2_b =
      controlMCUSlugsMKIINewNav_B.GetthePanFCBEX1000c;
  }

  /* End of Switch: '<S443>/Switch2' */
  /* S-Function "dsPIC_C_function_Call" Block: <S9>/Set the Pan [FCBEX1000.c] */
  updatePan(controlMCUSlugsMKIINewNav_B.Switch2_b);

  /* Switch: '<S443>/Switch3' */
  if (rtb_IC2 > 0) {
    controlMCUSlugsMKIINewNav_B.Switch3_nv5 =
      controlMCUSlugsMKIINewNav_B.ZeroRoll;
  } else {
    controlMCUSlugsMKIINewNav_B.Switch3_nv5 =
      controlMCUSlugsMKIINewNav_B.Switch1_e;
  }

  /* End of Switch: '<S443>/Switch3' */
  /* S-Function "dsPIC_C_function_Call" Block: <S9>/Set the Roll [FCBEX1000.c] */
  updateRoll(controlMCUSlugsMKIINewNav_B.Switch3_nv5);

  /* Switch: '<S443>/Switch1' incorporates:
   *  DataTypeConversion: '<S443>/Data Type Conversion4'
   */
  if (rtb_IC2 > 0) {
    controlMCUSlugsMKIINewNav_B.Switch1_l = controlMCUSlugsMKIINewNav_B.Sum1;
  } else {
    /* DataTypeConversion: '<S443>/Data Type Conversion4' */
    tmp_1 = floor(controlMCUSlugsMKIINewNav_B.Add);
    if (rtIsNaN(tmp_1) || rtIsInf(tmp_1)) {
      tmp_1 = 0.0;
    } else {
      tmp_1 = fmod(tmp_1, 65536.0);
    }

    controlMCUSlugsMKIINewNav_B.Switch1_l = tmp_1 < 0.0 ? (uint16_T)-(int16_T)
      (uint16_T)-tmp_1 : (uint16_T)tmp_1;
  }

  /* End of Switch: '<S443>/Switch1' */
  /* S-Function "dsPIC_C_function_Call" Block: <S9>/Set the Tilt [FCBEX1000.c] */
  updateTilt(controlMCUSlugsMKIINewNav_B.Switch1_l);

  /* S-Function "dsPIC_C_function_Call" Block: <S443>/Get the Zoom [FCBEX1000.c] */
  controlMCUSlugsMKIINewNav_B.GettheZoomFCBEX1000c = getCurrentZoom();

  /* S-Function "dsPIC_C_function_Call" Block: <S443>/Set the Zoom [FCBEX1000.c] */
  changeZoom(controlMCUSlugsMKIINewNav_B.GettheZoomFCBEX1000c);

  /* S-Function "dsPIC_C_function_Call" Block: <S443>/Configure ICR, Brightness, etcc [FCBEX1000.c] */
  setCameraConfig();

  /* DataTypeConversion: '<S461>/Data Type Conversion1' incorporates:
   *  Constant: '<S509>/Constant1'
   *  Constant: '<S509>/Constant2'
   *  DataTypeConversion: '<S509>/Data Type Conversion'
   *  Gain: '<S461>/Gain'
   *  Gain: '<S508>/Unit Conversion'
   *  Product: '<S509>/Divide'
   *  Sum: '<S509>/Add'
   */
  /* MATLAB Function 'Update PWM Commands and Send Telemetry/PTZ Unit/myMux Fun1': '<S463>:1' */
  /*  This block supports an embeddable subset of the MATLAB language. */
  /*  See the help menu for details.  */
  /* '<S463>:1:5' */
  rtb_Product3_p4_idx = (real32_T)fmod((real32_T)floor((real32_T)((real_T)
    controlMCUSlugsMKIINewNav_B.Switch2_b * -0.0025855075867988581 +
    2.5278099438092423) * 57.2957802F * 10.0F), 65536.0F);

  /* MATLAB Function: '<S443>/myMux Fun1' incorporates:
   *  DataTypeConversion: '<S461>/Data Type Conversion1'
   */
  controlMCUSlugsMKIINewNav_B.y_o[0] = rtb_Product3_p4_idx < 0.0F ? -(int16_T)
    (uint16_T)-rtb_Product3_p4_idx : (int16_T)(uint16_T)rtb_Product3_p4_idx;

  /* DataTypeConversion: '<S462>/Data Type Conversion1' incorporates:
   *  Constant: '<S511>/Constant1'
   *  Constant: '<S511>/Constant2'
   *  DataTypeConversion: '<S511>/Data Type Conversion'
   *  Gain: '<S462>/Gain'
   *  Gain: '<S510>/Unit Conversion'
   *  Product: '<S511>/Divide'
   *  Sum: '<S511>/Add'
   */
  rtb_Product3_p4_idx = (real32_T)fmod((real32_T)floor((real32_T)((real_T)
    controlMCUSlugsMKIINewNav_B.Switch1_l * 0.0027935206789596215 +
    -3.5337328712985774) * 57.2957802F * 10.0F), 65536.0F);

  /* MATLAB Function: '<S443>/myMux Fun1' incorporates:
   *  DataTypeConversion: '<S443>/Data Type Conversion1'
   *  DataTypeConversion: '<S462>/Data Type Conversion1'
   */
  controlMCUSlugsMKIINewNav_B.y_o[1] = rtb_Product3_p4_idx < 0.0F ? -(int16_T)
    (uint16_T)-rtb_Product3_p4_idx : (int16_T)(uint16_T)rtb_Product3_p4_idx;
  controlMCUSlugsMKIINewNav_B.y_o[2] =
    controlMCUSlugsMKIINewNav_B.GettheZoomFCBEX1000c;

  /* S-Function "dsPIC_C_function_Call" Block: <S443>/Update the Pan Til Zoom [updateControMcuState.c] */
  updatePTZ(controlMCUSlugsMKIINewNav_B.y_o);

  /* S-Function "dsPIC_C_function_Call" Block: <S9>/Turn on Lights? [updateControlMCUState.c] */
  controlMCUSlugsMKIINewNav_B.TurnonLightsupdateControlMCUSta = getLightsOnOff();

  /* DataTypeConversion: '<S9>/Data Type Conversion' */
  controlMCUSlugsMKIINewNav_B.DataTypeConversion_l =
    (controlMCUSlugsMKIINewNav_B.TurnonLightsupdateControlMCUSta != 0);

  /* S-Function "dsPIC_Digital_OutputWrite" Block: <S9>/Update the  Light Status */
  LATGbits.LATG15 = controlMCUSlugsMKIINewNav_B.DataTypeConversion_l;

  /* S-Function "dsPIC_C_function_Call" Block: <S9>/Visible or IR? [updateControlMCUState.c] */
  controlMCUSlugsMKIINewNav_B.VisibleorIRupdateControlMCUStat =
    getLightsDayNight();

  /* DataTypeConversion: '<S9>/Data Type Conversion1' */
  controlMCUSlugsMKIINewNav_B.DataTypeConversion1_f =
    (controlMCUSlugsMKIINewNav_B.VisibleorIRupdateControlMCUStat != 0);

  /* S-Function "dsPIC_Digital_OutputWrite" Block: <S9>/Update the  Light Status1 */
  LATGbits.LATG7 = controlMCUSlugsMKIINewNav_B.DataTypeConversion1_f;

  /* S-Function "dsPIC_C_function_Call" Block: <S6>/get if in HIL or not [updateControlMcuStatec] */
  controlMCUSlugsMKIINewNav_B.getifinHILornotupdateControlMcu = getHilOnOff();

  /* DataTypeConversion: '<S6>/Data Type Conversion1' */
  controlMCUSlugsMKIINewNav_B.DataTypeConversion1_a =
    (controlMCUSlugsMKIINewNav_B.getifinHILornotupdateControlMcu != 0);

  /* S-Function "dsPIC_Digital_OutputWrite" Block: <S6>/HIL Attitude  On//Off Switch */
  LATBbits.LATB6 = controlMCUSlugsMKIINewNav_B.DataTypeConversion1_a;

  /* S-Function "dsPIC_Digital_OutputWrite" Block: <S6>/HIL On//Off Switch */
  LATEbits.LATE9 = controlMCUSlugsMKIINewNav_B.DataTypeConversion1_a;

  /* S-Function "dsPIC_C_function_Call" Block: <Root>/is it in Mid Level, Pt or SPT? [navSupport.c] */
  controlMCUSlugsMKIINewNav_B.isitinMidLevelPtorSPTnavSuppo_n = isPassthrough();

  /* S-Function "dsPIC_C_function_Call" Block: <Root>/C Function Call */

  /* Update for Delay: '<S19>/Integer Delay3' */
  controlMCUSlugsMKIINewNav_DWork.IntegerDelay3_DSTATE = Switch;

  /* Update for Delay: '<S20>/Integer Delay3' */
  controlMCUSlugsMKIINewNav_DWork.IntegerDelay3_DSTATE_i = Switch3_a;

  /* Update for Enabled SubSystem: '<S3>/L1 Output Feedback Controller With  Projection Operator' incorporates:
   *  Update for EnablePort: '<S15>/Enable'
   */
  if (controlMCUSlugsMKIINewNav_DWork.L1OutputFeedbackControllerWithP) {
    /* Update for UnitDelay: '<S35>/UD' */
    controlMCUSlugsMKIINewNav_DWork.UD_DSTATE_b =
      controlMCUSlugsMKIINewNav_B.GetRangeofValuesnavSupportc_n[0];

    /* Update for Delay: '<S25>/Integer Delay' */
    controlMCUSlugsMKIINewNav_DWork.IntegerDelay_DSTATE_l =
      controlMCUSlugsMKIINewNav_B.Switch3_ik;

    /* Update for Delay: '<S25>/Integer Delay1' */
    controlMCUSlugsMKIINewNav_DWork.IntegerDelay1_DSTATE_k =
      controlMCUSlugsMKIINewNav_B.Subtract_g;

    /* Update for Delay: '<S27>/Integer Delay3' */
    controlMCUSlugsMKIINewNav_DWork.IntegerDelay3_DSTATE_nd =
      controlMCUSlugsMKIINewNav_B.Switch3_ob;

    /* Update for UnitDelay: '<S45>/UD' */
    controlMCUSlugsMKIINewNav_DWork.UD_DSTATE_o =
      controlMCUSlugsMKIINewNav_B.GetRangeofValuesnavSupportc_n[1];

    /* Update for Delay: '<S29>/Integer Delay' */
    controlMCUSlugsMKIINewNav_DWork.IntegerDelay_DSTATE_f =
      controlMCUSlugsMKIINewNav_B.Switch3_a3;

    /* Update for Delay: '<S29>/Integer Delay1' */
    controlMCUSlugsMKIINewNav_DWork.IntegerDelay1_DSTATE_i =
      controlMCUSlugsMKIINewNav_B.Subtract_f;

    /* Update for Delay: '<S15>/Integer Delay' */
    controlMCUSlugsMKIINewNav_DWork.IntegerDelay_DSTATE_cl = Switch3_ae;

    /* Update for Delay: '<S15>/Integer Delay1' */
    controlMCUSlugsMKIINewNav_DWork.IntegerDelay1_DSTATE_c =
      controlMCUSlugsMKIINewNav_B.Switch3_ik;

    /* Update for Delay: '<S28>/Integer Delay3' */
    controlMCUSlugsMKIINewNav_DWork.IntegerDelay3_DSTATE_e =
      controlMCUSlugsMKIINewNav_B.Switch3_a3;

    /* Update for Delay: '<S30>/Integer Delay' */
    controlMCUSlugsMKIINewNav_DWork.IntegerDelay_DSTATE_fj =
      controlMCUSlugsMKIINewNav_B.Switch3_fi;

    /* Update for Delay: '<S30>/Integer Delay1' */
    controlMCUSlugsMKIINewNav_DWork.IntegerDelay1_DSTATE_bo[0] =
      controlMCUSlugsMKIINewNav_DWork.IntegerDelay1_DSTATE_bo[1];
    controlMCUSlugsMKIINewNav_DWork.IntegerDelay1_DSTATE_bo[1] =
      controlMCUSlugsMKIINewNav_B.Switch3_fi;

    /* Update for Delay: '<S30>/Integer Delay2' */
    controlMCUSlugsMKIINewNav_DWork.IntegerDelay2_DSTATE_n =
      controlMCUSlugsMKIINewNav_B.Switch3_ik;

    /* Update for Delay: '<S46>/Integer Delay3' */
    controlMCUSlugsMKIINewNav_DWork.IntegerDelay3_DSTATE_o5 =
      controlMCUSlugsMKIINewNav_B.Switch3_fi;

    /* Update for Delay: '<S47>/Integer Delay3' */
    controlMCUSlugsMKIINewNav_DWork.IntegerDelay3_DSTATE_ns =
      controlMCUSlugsMKIINewNav_B.Switch3_ik;
  }

  /* End of Update for SubSystem: '<S3>/L1 Output Feedback Controller With  Projection Operator' */
  rate_scheduler();

  /* tid is required for a uniform function interface.
   * Argument tid is not used in the function. */
  UNUSED_PARAMETER(tid);
}

/* Model initialize function */
void controlMCUSlugsMKIINewNav_initialize(boolean_T firstTime)
{
  (void)firstTime;

  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* initialize real-time model */
  (void) memset((void *)controlMCUSlugsMKIINewNav_M, 0,
                sizeof(RT_MODEL_controlMCUSlugsMKIINew));
  rtmSetFirstInitCond(controlMCUSlugsMKIINewNav_M, 1);

  /* block I/O */
  (void) memset(((void *) &controlMCUSlugsMKIINewNav_B), 0,
                sizeof(BlockIO_controlMCUSlugsMKIINewN));

  /* states (dwork) */
  (void) memset((void *)&controlMCUSlugsMKIINewNav_DWork, 0,
                sizeof(D_Work_controlMCUSlugsMKIINewNa));

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
  TRISB = 65471;
  TRISE = 65023;
  TRISG = 31871;

  /* Configuration ADCHS */
  AD1PCFGL = (68U);
  AD2PCFGL = (68U);
  AD1PCFGH = ((32U) & 65535);

  /* S-Function "dsPIC_PWM_IC" initialization Block: <S416>/Input Capture */
  /* Initialise Input Capture */
  /* config capture */
  OpenCapture1(IC_IDLE_CON & IC_TIMER2_SRC & IC_INT_1CAPTURE &
               IC_EVERY_RISE_EDGE);
  ConfigIntCapture1(IC_INT_ON & IC_INT_PRIOR_6);

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
  OpenCapture7(IC_IDLE_CON & IC_TIMER2_SRC & IC_INT_1CAPTURE &
               IC_EVERY_RISE_EDGE);
  ConfigIntCapture7(IC_INT_ON & IC_INT_PRIOR_6);

  /* config capture */
  OpenCapture8(IC_IDLE_CON & IC_TIMER2_SRC & IC_INT_1CAPTURE &
               IC_EVERY_RISE_EDGE);
  ConfigIntCapture8(IC_INT_ON & IC_INT_PRIOR_6);

  /* Start for InitialCondition: '<S418>/IC1' */
  controlMCUSlugsMKIINewNav_DWork.IC1_FirstOutputTime = TRUE;

  /* Start for Atomic SubSystem: '<S3>/Navigation Encaps' */
  /* Start for InitialCondition: '<S137>/IC1' */
  controlMCUSlugsMKIINewNav_DWork.IC1_FirstOutputTime_p = TRUE;

  /* Start for InitialCondition: '<S137>/IC2' */
  controlMCUSlugsMKIINewNav_DWork.IC2_FirstOutputTime = TRUE;

  /* Start for InitialCondition: '<S137>/IC3' */
  controlMCUSlugsMKIINewNav_B.IC3 = 0.0F;
  controlMCUSlugsMKIINewNav_DWork.IC3_FirstOutputTime = TRUE;

  /* Start for InitialCondition: '<S137>/IC4' */
  controlMCUSlugsMKIINewNav_B.IC4[0] = 36.9885063F;
  controlMCUSlugsMKIINewNav_B.IC4[1] = -122.055305F;
  controlMCUSlugsMKIINewNav_DWork.IC4_FirstOutputTime = TRUE;

  /* Start for InitialCondition: '<S134>/IC' */
  controlMCUSlugsMKIINewNav_DWork.IC_FirstOutputTime = TRUE;

  /* Start for IfAction SubSystem: '<S128>/RTB//Follow Mobile Navigation' */
  /* InitializeConditions for DiscreteIntegrator: '<S391>/Discrete-Time Integrator' */
  controlMCUSlugsMKIINewNav_DWork.DiscreteTimeIntegrator_IC_LOADI = 1U;

  /* End of Start for SubSystem: '<S128>/RTB//Follow Mobile Navigation' */

  /* Start for IfAction SubSystem: '<S128>/Normal WP  Navigation' */

  /* Start for InitialCondition: '<S135>/IC' */
  controlMCUSlugsMKIINewNav_B.IC = 0U;
  controlMCUSlugsMKIINewNav_DWork.IC_FirstOutputTime_l = TRUE;

  /* Start for Enabled SubSystem: '<S135>/Get Frenet' */
  /* InitializeConditions for Delay: '<S251>/Integer Delay1' */
  controlMCUSlugsMKIINewNav_DWork.IntegerDelay1_DSTATE_p = 1.0F;

  /* End of Start for SubSystem: '<S135>/Get Frenet' */
  /* InitializeConditions for Delay: '<S135>/Integer Delay' */
  controlMCUSlugsMKIINewNav_DWork.IntegerDelay_DSTATE_p = 1U;

  /* InitializeConditions for MATLAB Function: '<S135>/Embedded MATLAB Function' */
  controlMCUSlugsMKIINewNav_DWork.persistentDidReachIP = 0U;

  /* InitializeConditions for MATLAB Function: '<S135>/computeCurrentWP' */
  controlMCUSlugsMKIINewNav_DWork.fromWp = 1U;
  controlMCUSlugsMKIINewNav_DWork.toWp = 2U;

  /* InitializeConditions for Delay: '<S263>/Integer Delay3' */
  controlMCUSlugsMKIINewNav_DWork.IntegerDelay3_DSTATE_l = 0.0F;

  /* VirtualOutportStart for Outport: '<S135>/FromWP' */
  controlMCUSlugsMKIINewNav_B.WP0 = 1U;

  /* VirtualOutportStart for Outport: '<S135>/ToWP' */
  controlMCUSlugsMKIINewNav_B.WP1 = 2U;

  /* End of Start for SubSystem: '<S128>/Normal WP  Navigation' */

  /* Start for IfAction SubSystem: '<S128>/Circle Navigation' */

  /* InitializeConditions for Delay: '<S146>/Integer Delay1' */
  controlMCUSlugsMKIINewNav_DWork.IntegerDelay1_DSTATE = 0.0F;

  /* InitializeConditions for Delay: '<S129>/Integer Delay' */
  controlMCUSlugsMKIINewNav_DWork.IntegerDelay_DSTATE = 0.0F;

  /* End of Start for SubSystem: '<S128>/Circle Navigation' */
  /* End of Start for SubSystem: '<S3>/Navigation Encaps' */

  /* Start for Enabled SubSystem: '<S3>/L1 Output Feedback Controller With  Projection Operator' */
  /* Start for InitialCondition: '<S25>/IC' */
  controlMCUSlugsMKIINewNav_DWork.IC_FirstOutputTime_a = TRUE;

  /* Start for InitialCondition: '<S29>/IC' */
  controlMCUSlugsMKIINewNav_DWork.IC_FirstOutputTime_m = TRUE;

  /* InitializeConditions for UnitDelay: '<S35>/UD' */
  controlMCUSlugsMKIINewNav_DWork.UD_DSTATE_b = 0.0F;

  /* InitializeConditions for Delay: '<S25>/Integer Delay' */
  controlMCUSlugsMKIINewNav_DWork.IntegerDelay_DSTATE_l = 0.0F;

  /* InitializeConditions for Delay: '<S25>/Integer Delay1' */
  controlMCUSlugsMKIINewNav_DWork.IntegerDelay1_DSTATE_k = 0.0F;

  /* InitializeConditions for Delay: '<S27>/Integer Delay3' */
  controlMCUSlugsMKIINewNav_DWork.IntegerDelay3_DSTATE_nd = 0.0F;

  /* InitializeConditions for UnitDelay: '<S45>/UD' */
  controlMCUSlugsMKIINewNav_DWork.UD_DSTATE_o = 0.0F;

  /* InitializeConditions for Delay: '<S29>/Integer Delay' */
  controlMCUSlugsMKIINewNav_DWork.IntegerDelay_DSTATE_f = 0.0F;

  /* InitializeConditions for Delay: '<S29>/Integer Delay1' */
  controlMCUSlugsMKIINewNav_DWork.IntegerDelay1_DSTATE_i = 0.0F;

  /* InitializeConditions for Delay: '<S15>/Integer Delay' */
  controlMCUSlugsMKIINewNav_DWork.IntegerDelay_DSTATE_cl = 0.0F;

  /* InitializeConditions for Delay: '<S15>/Integer Delay1' */
  controlMCUSlugsMKIINewNav_DWork.IntegerDelay1_DSTATE_c = 0.0F;

  /* InitializeConditions for Merge: '<S26>/Merge' */
  if (rtmIsFirstInitCond(controlMCUSlugsMKIINewNav_M)) {
    controlMCUSlugsMKIINewNav_B.Merge_e = 0.0F;
  }

  /* End of InitializeConditions for Merge: '<S26>/Merge' */

  /* InitializeConditions for Delay: '<S28>/Integer Delay3' */
  controlMCUSlugsMKIINewNav_DWork.IntegerDelay3_DSTATE_e = 0.0F;

  /* InitializeConditions for Delay: '<S30>/Integer Delay' */
  controlMCUSlugsMKIINewNav_DWork.IntegerDelay_DSTATE_fj = 0.0F;

  /* InitializeConditions for Delay: '<S30>/Integer Delay1' */
  controlMCUSlugsMKIINewNav_DWork.IntegerDelay1_DSTATE_bo[0] = 0.0F;
  controlMCUSlugsMKIINewNav_DWork.IntegerDelay1_DSTATE_bo[1] = 0.0F;

  /* InitializeConditions for Delay: '<S30>/Integer Delay2' */
  controlMCUSlugsMKIINewNav_DWork.IntegerDelay2_DSTATE_n = 0.0F;

  /* InitializeConditions for Delay: '<S46>/Integer Delay3' */
  controlMCUSlugsMKIINewNav_DWork.IntegerDelay3_DSTATE_o5 = 0.0F;

  /* InitializeConditions for Delay: '<S47>/Integer Delay3' */
  controlMCUSlugsMKIINewNav_DWork.IntegerDelay3_DSTATE_ns = 0.0F;

  /* End of Start for SubSystem: '<S3>/L1 Output Feedback Controller With  Projection Operator' */
  /* Start for Atomic SubSystem: '<S3>/Lateral Channel Encaps' */
  /* Start for Enabled SubSystem: '<S48>/Sideslip Compensation' */
  /* Start for InitialCondition: '<S65>/IC' */
  controlMCUSlugsMKIINewNav_DWork.IC_FirstOutputTime_h = TRUE;

  /* InitializeConditions for UnitDelay: '<S73>/UD' */
  controlMCUSlugsMKIINewNav_DWork.UD_DSTATE = 0.0F;

  /* InitializeConditions for Delay: '<S65>/Integer Delay' */
  controlMCUSlugsMKIINewNav_DWork.IntegerDelay_DSTATE_h = 0.0F;

  /* InitializeConditions for Delay: '<S65>/Integer Delay1' */
  controlMCUSlugsMKIINewNav_DWork.IntegerDelay1_DSTATE_e = 0.0F;

  /* InitializeConditions for Delay: '<S66>/Integer Delay3' */
  controlMCUSlugsMKIINewNav_DWork.IntegerDelay3_DSTATE_ij = 0.0F;

  /* End of Start for SubSystem: '<S48>/Sideslip Compensation' */
  /* End of Start for SubSystem: '<S3>/Lateral Channel Encaps' */

  /* S-Function "dsPIC_PWM_motor" initialization Block: <S9>/PWM Servo Output [dR dE dA dT] */
  /* Initialise Output Capture */
  DTCON1 = 255;                        /* Configure Dead Time */
  SetDCMCPWM(1,0,0);
  SetDCMCPWM(2,0,0);
  SetDCMCPWM(3,0,0);
  SetDCMCPWM(4,0,0);

#define config                         (PWM_INT_DIS & PWM_FLTA_DIS_INT & PWM_INT_PR1 & PWM_FLTA_INT_PR0)

  ConfigIntMCPWM( config );

#define config1                        (PWM_EN & 0xDFFF & PWM_OP_SCALE1 & PWM_IPCLK_SCALE16 & PWM_MOD_UPDN)
#define config2                        (0xffff & PWM_PEN1L & PWM_MOD1_IND & PWM_PDIS1H & PWM_PEN2L & PWM_MOD2_IND & PWM_PDIS2H & PWM_PEN3L & PWM_MOD3_IND & PWM_PDIS3H & PWM_PEN4L & PWM_MOD4_IND & PWM_PDIS4H)
#define config3                        (PWM_SEVOPS1 & PWM_OSYNC_TCY & PWM_UEN)

  OpenMCPWM(25000,0,config1,config2,config3);

  /* Start for Enabled SubSystem: '<S443>/Camera Track' */
  /* InitializeConditions for MATLAB Function: '<S483>/Embedded MATLAB Function' */
  con_EmbeddedMATLABFunction_Init
    (&controlMCUSlugsMKIINewNav_DWork.sf_EmbeddedMATLABFunction_px);

  /* InitializeConditions for MATLAB Function: '<S484>/Embedded MATLAB Function' */
  con_EmbeddedMATLABFunction_Init
    (&controlMCUSlugsMKIINewNav_DWork.sf_EmbeddedMATLABFunction_e4);

  /* InitializeConditions for MATLAB Function: '<S485>/Embedded MATLAB Function' */
  con_EmbeddedMATLABFunction_Init
    (&controlMCUSlugsMKIINewNav_DWork.sf_EmbeddedMATLABFunction_ab);

  /* VirtualOutportStart for Outport: '<S460>/Roll_c(abs)' */
  controlMCUSlugsMKIINewNav_B.ZeroRoll = 977U;

  /* End of Start for SubSystem: '<S443>/Camera Track' */

  /* Start for Enabled SubSystem: '<S443>/Camera Stab' */
  /* Start for Enabled SubSystem: '<S459>/Roll Compensation' */
  /* InitializeConditions for MATLAB Function: '<S470>/Embedded MATLAB Function' */
  con_EmbeddedMATLABFunction_Init
    (&controlMCUSlugsMKIINewNav_DWork.sf_EmbeddedMATLABFunction_lo);

  /* VirtualOutportStart for Outport: '<S465>/Roll Correction' */
  controlMCUSlugsMKIINewNav_B.Switch_n = 977U;

  /* End of Start for SubSystem: '<S459>/Roll Compensation' */

  /* InitializeConditions for MATLAB Function: '<S466>/Embedded MATLAB Function' */
  con_EmbeddedMATLABFunction_Init
    (&controlMCUSlugsMKIINewNav_DWork.sf_EmbeddedMATLABFunction_dk);

  /* End of Start for SubSystem: '<S443>/Camera Stab' */
  /* Start for S-Function (dsPIC_C_function_Call): '<Root>/C Function Call' */
  controlMCUInit();

  /* S-Function "dsPIC_Digital_OutputWrite" Block: <S6>/VI Sensor DO1 */
  LATGbits.LATG8 = FALSE;

  /* S-Function "dsPIC_Digital_OutputWrite" Block: <S6>/VI Sensor DO2 */
  LATGbits.LATG9 = FALSE;

  {
    int16_T i;

    /* InitializeConditions for MATLAB Function: '<S420>/Buffer Failsafe Channel' */
    controlMCU_BufferICChannel_Init
      (&controlMCUSlugsMKIINewNav_DWork.sf_BufferFailsafeChannel);

    /* InitializeConditions for MATLAB Function: '<S415>/Buffer IC Channel' */
    controlMCU_BufferICChannel_Init
      (&controlMCUSlugsMKIINewNav_DWork.sf_BufferICChannel);

    /* InitializeConditions for MATLAB Function: '<S415>/Buffer IC Channel1' */
    controlMCU_BufferICChannel_Init
      (&controlMCUSlugsMKIINewNav_DWork.sf_BufferICChannel1);

    /* InitializeConditions for MATLAB Function: '<S415>/Buffer IC Channel2' */
    controlMCU_BufferICChannel_Init
      (&controlMCUSlugsMKIINewNav_DWork.sf_BufferICChannel2);

    /* InitializeConditions for MATLAB Function: '<S415>/Buffer IC Channel3' */
    controlMCU_BufferICChannel_Init
      (&controlMCUSlugsMKIINewNav_DWork.sf_BufferICChannel3);

    /* InitializeConditions for Atomic SubSystem: '<S3>/Navigation Encaps' */
    /* InitializeConditions for Delay: '<S412>/Delay' */
    controlMCUSlugsMKIINewNav_DWork.Delay_DSTATE = FALSE;

    /* InitializeConditions for Delay: '<S128>/Integer Delay' */
    controlMCUSlugsMKIINewNav_DWork.IntegerDelay_DSTATE_d = 0U;

    /* InitializeConditions for Delay: '<S128>/Integer Delay1' */
    controlMCUSlugsMKIINewNav_DWork.IntegerDelay1_DSTATE_j2 = 0U;

    /* End of InitializeConditions for SubSystem: '<S3>/Navigation Encaps' */

    /* InitializeConditions for Delay: '<S19>/Integer Delay3' */
    controlMCUSlugsMKIINewNav_DWork.IntegerDelay3_DSTATE = 0.0F;

    /* InitializeConditions for Atomic SubSystem: '<S3>/Longitudinal Channel Encaps' */
    /* InitializeConditions for Delay: '<S116>/Integer Delay3' */
    controlMCUSlugsMKIINewNav_DWork.IntegerDelay3_DSTATE_g = 0.0F;

    /* InitializeConditions for MATLAB Function: '<S113>/Embedded MATLAB Function' */
    con_EmbeddedMATLABFunction_Init
      (&controlMCUSlugsMKIINewNav_DWork.sf_EmbeddedMATLABFunction_d);

    /* InitializeConditions for UnitDelay: '<S126>/FixPt Unit Delay2' */
    controlMCUSlugsMKIINewNav_DWork.FixPtUnitDelay2_DSTATE = 1U;

    /* InitializeConditions for Delay: '<S95>/Integer Delay3' */
    controlMCUSlugsMKIINewNav_DWork.IntegerDelay3_DSTATE_c = 0.0F;

    /* InitializeConditions for Delay: '<S106>/Integer Delay2' */
    controlMCUSlugsMKIINewNav_DWork.IntegerDelay2_DSTATE = 0.0F;

    /* InitializeConditions for Delay: '<S108>/Integer Delay3' */
    controlMCUSlugsMKIINewNav_DWork.IntegerDelay3_DSTATE_o = 0.0F;

    /* InitializeConditions for Delay: '<S106>/Integer Delay' */
    controlMCUSlugsMKIINewNav_DWork.IntegerDelay_DSTATE_c = 0.0F;

    /* InitializeConditions for Delay: '<S106>/Integer Delay1' */
    controlMCUSlugsMKIINewNav_DWork.IntegerDelay1_DSTATE_j[0] = 0.0F;
    controlMCUSlugsMKIINewNav_DWork.IntegerDelay1_DSTATE_j[1] = 0.0F;

    /* InitializeConditions for Delay: '<S107>/Integer Delay3' */
    controlMCUSlugsMKIINewNav_DWork.IntegerDelay3_DSTATE_lo = 0.0F;

    /* InitializeConditions for Delay: '<S97>/Integer Delay3' */
    controlMCUSlugsMKIINewNav_DWork.IntegerDelay3_DSTATE_i4 = 0.0F;

    /* InitializeConditions for MATLAB Function: '<S89>/Embedded MATLAB Function' */
    con_EmbeddedMATLABFunction_Init
      (&controlMCUSlugsMKIINewNav_DWork.sf_EmbeddedMATLABFunction_a);

    /* InitializeConditions for Delay: '<S94>/Integer Delay3' */
    controlMCUSlugsMKIINewNav_DWork.IntegerDelay3_DSTATE_gy = 0.0F;

    /* InitializeConditions for Delay: '<S102>/Integer Delay2' */
    controlMCUSlugsMKIINewNav_DWork.IntegerDelay2_DSTATE_b = 0.0F;

    /* InitializeConditions for Delay: '<S103>/Integer Delay3' */
    controlMCUSlugsMKIINewNav_DWork.IntegerDelay3_DSTATE_oy = 0.0F;

    /* InitializeConditions for Delay: '<S102>/Integer Delay' */
    controlMCUSlugsMKIINewNav_DWork.IntegerDelay_DSTATE_o = 0.0F;

    /* InitializeConditions for Delay: '<S102>/Integer Delay1' */
    controlMCUSlugsMKIINewNav_DWork.IntegerDelay1_DSTATE_m[0] = 0.0F;
    controlMCUSlugsMKIINewNav_DWork.IntegerDelay1_DSTATE_m[1] = 0.0F;

    /* InitializeConditions for Delay: '<S104>/Integer Delay3' */
    controlMCUSlugsMKIINewNav_DWork.IntegerDelay3_DSTATE_d = 0.0F;

    /* InitializeConditions for Delay: '<S96>/Integer Delay3' */
    controlMCUSlugsMKIINewNav_DWork.IntegerDelay3_DSTATE_cv = 0.0F;
    for (i = 0; i < 5; i++) {
      /* InitializeConditions for Delay: '<S90>/NDelays' */
      controlMCUSlugsMKIINewNav_DWork.NDelays_DSTATE[i] = 0.0F;

      /* InitializeConditions for Delay: '<S92>/NDelays' */
      controlMCUSlugsMKIINewNav_DWork.NDelays_DSTATE_n[i] = 0.0F;
    }

    /* InitializeConditions for Delay: '<S110>/Integer Delay2' */
    controlMCUSlugsMKIINewNav_DWork.IntegerDelay2_DSTATE_g = 0.0F;

    /* InitializeConditions for Delay: '<S111>/Integer Delay3' */
    controlMCUSlugsMKIINewNav_DWork.IntegerDelay3_DSTATE_f = 0.0F;

    /* InitializeConditions for Delay: '<S110>/Integer Delay' */
    controlMCUSlugsMKIINewNav_DWork.IntegerDelay_DSTATE_i = 0.0F;

    /* InitializeConditions for Delay: '<S110>/Integer Delay1' */
    controlMCUSlugsMKIINewNav_DWork.IntegerDelay1_DSTATE_g[0] = 0.0F;
    controlMCUSlugsMKIINewNav_DWork.IntegerDelay1_DSTATE_g[1] = 0.0F;

    /* InitializeConditions for Delay: '<S112>/Integer Delay3' */
    controlMCUSlugsMKIINewNav_DWork.IntegerDelay3_DSTATE_n = 0.0F;

    /* End of InitializeConditions for SubSystem: '<S3>/Longitudinal Channel Encaps' */

    /* InitializeConditions for Delay: '<S20>/Integer Delay3' */
    controlMCUSlugsMKIINewNav_DWork.IntegerDelay3_DSTATE_i = 0.0F;

    /* InitializeConditions for Atomic SubSystem: '<S3>/Lateral Channel Encaps' */
    /* InitializeConditions for Delay: '<S83>/Integer Delay3' */
    controlMCUSlugsMKIINewNav_DWork.IntegerDelay3_DSTATE_m = 0.0F;

    /* InitializeConditions for Delay: '<S84>/Integer Delay3' */
    controlMCUSlugsMKIINewNav_DWork.IntegerDelay3_DSTATE_k = 0.0F;

    /* InitializeConditions for Delay: '<S54>/Integer Delay3' */
    controlMCUSlugsMKIINewNav_DWork.IntegerDelay3_DSTATE_h = 0.0F;

    /* InitializeConditions for MATLAB Function: '<S51>/Embedded MATLAB Function' */
    con_EmbeddedMATLABFunction_Init
      (&controlMCUSlugsMKIINewNav_DWork.sf_EmbeddedMATLABFunction_b);

    /* InitializeConditions for Delay: '<S77>/Integer Delay2' */
    controlMCUSlugsMKIINewNav_DWork.IntegerDelay2_DSTATE_o = 0.0F;

    /* InitializeConditions for Delay: '<S78>/Integer Delay3' */
    controlMCUSlugsMKIINewNav_DWork.IntegerDelay3_DSTATE_mx = 0.0F;

    /* InitializeConditions for Delay: '<S77>/Integer Delay' */
    controlMCUSlugsMKIINewNav_DWork.IntegerDelay_DSTATE_ie = 0.0F;

    /* InitializeConditions for Delay: '<S77>/Integer Delay1' */
    controlMCUSlugsMKIINewNav_DWork.IntegerDelay1_DSTATE_jy[0] = 0.0F;
    controlMCUSlugsMKIINewNav_DWork.IntegerDelay1_DSTATE_jy[1] = 0.0F;

    /* InitializeConditions for Delay: '<S79>/Integer Delay3' */
    controlMCUSlugsMKIINewNav_DWork.IntegerDelay3_DSTATE_gf = 0.0F;

    /* InitializeConditions for Delay: '<S53>/Integer Delay3' */
    controlMCUSlugsMKIINewNav_DWork.IntegerDelay3_DSTATE_p = 0.0F;
    for (i = 0; i < 5; i++) {
      /* InitializeConditions for Delay: '<S57>/NDelays' */
      controlMCUSlugsMKIINewNav_DWork.NDelays_DSTATE_j[i] = 0.0F;

      /* InitializeConditions for Delay: '<S55>/NDelays' */
      controlMCUSlugsMKIINewNav_DWork.NDelays_DSTATE_a[i] = 0.0F;
    }

    /* InitializeConditions for Delay: '<S61>/Integer Delay2' */
    controlMCUSlugsMKIINewNav_DWork.IntegerDelay2_DSTATE_g4 = 0.0F;

    /* InitializeConditions for Delay: '<S62>/Integer Delay3' */
    controlMCUSlugsMKIINewNav_DWork.IntegerDelay3_DSTATE_c5 = 0.0F;

    /* InitializeConditions for Delay: '<S61>/Integer Delay' */
    controlMCUSlugsMKIINewNav_DWork.IntegerDelay_DSTATE_k = 0.0F;

    /* InitializeConditions for Delay: '<S61>/Integer Delay1' */
    controlMCUSlugsMKIINewNav_DWork.IntegerDelay1_DSTATE_b[0] = 0.0F;
    controlMCUSlugsMKIINewNav_DWork.IntegerDelay1_DSTATE_b[1] = 0.0F;

    /* InitializeConditions for Delay: '<S63>/Integer Delay3' */
    controlMCUSlugsMKIINewNav_DWork.IntegerDelay3_DSTATE_le = 0.0F;

    /* End of InitializeConditions for SubSystem: '<S3>/Lateral Channel Encaps' */

    /* InitializeConditions for MATLAB Function: '<S425>/Embedded MATLAB Function' */
    con_EmbeddedMATLABFunction_Init
      (&controlMCUSlugsMKIINewNav_DWork.sf_EmbeddedMATLABFunction);

    /* InitializeConditions for MATLAB Function: '<S426>/Embedded MATLAB Function' */
    con_EmbeddedMATLABFunction_Init
      (&controlMCUSlugsMKIINewNav_DWork.sf_EmbeddedMATLABFunction_i);

    /* set "at time zero" to false */
    if (rtmIsFirstInitCond(controlMCUSlugsMKIINewNav_M)) {
      rtmSetFirstInitCond(controlMCUSlugsMKIINewNav_M, 0);
    }
  }
}

/* File trailer for Real-Time Workshop generated code.
 *
 * [EOF]
 */
