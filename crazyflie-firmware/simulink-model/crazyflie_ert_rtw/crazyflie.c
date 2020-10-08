/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: crazyflie.c
 *
 * Code generated for Simulink model 'crazyflie'.
 *
 * Model version                  : 1.392
 * Simulink Coder version         : 9.0 (R2018b) 24-May-2018
 * C/C++ source code generated on : Wed May 15 13:56:26 2019
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#include "crazyflie.h"

/* Block signals and states (default storage) */
DW rtDW;

/* External inputs (root inport signals with default storage) */
ExtU rtU;

/* External outputs (root outports fed by signals with default storage) */
ExtY rtY;

/* Model step function */
void crazyflie_step(void)
{
  real_T y_n;
  real_T z_n;
  static const real_T a[20] = { -0.0130516411025351, -0.0130516411029438,
    0.0130516411030164, 0.0130516411026696, -0.0134352368017384,
    0.01343523680194, 0.0134352368022339, -0.0134352368020933,
    -0.010787136896059, -0.0107871368966255, 0.0107871368964568,
    0.0107871368958907, -0.0111041774433157, 0.0111041774440371,
    0.0111041774437252, -0.011104177443004, -0.00499998563150369,
    0.00499998563150644, -0.00499998563150448, 0.00499998563150769 };

  real_T rtb_motor_thrust[4];
  real_T rtb_u_optimal[4];
  real_T tmp[5];
  real_T a_0[4];
  int32_T i;
  int32_T i_0;
  real_T rtb_acc_rp_idx_0;
  real_T y_n_tmp;

  /* MATLAB Function: '<Root>/MATLAB Function3' incorporates:
   *  Inport: '<Root>/Acc_x'
   *  Inport: '<Root>/Acc_y'
   *  Inport: '<Root>/Acc_z'
   */
  /* MATLAB Function 'MATLAB Function3': '<S2>:1' */
  /* '<S2>:1:2' acc_x=accelerometer(1); */
  /* '<S2>:1:3' acc_y=accelerometer(2); */
  /* '<S2>:1:4' acc_z=accelerometer(3); */
  /* '<S2>:1:6' if acc_x == 0 && acc_y == 0 && acc_z == 0 */
  if ((rtU.Acc_x == 0.0) && (rtU.Acc_y == 0.0) && (rtU.Acc_z == 0.0)) {
    /* '<S2>:1:7' roll = 0; */
    /* '<S2>:1:8' pitch = 0; */
    /* '<S2>:1:9' acc_rp=[roll;pitch]; */
    rtb_acc_rp_idx_0 = 0.0;
    z_n = 0.0;
  } else {
    /* '<S2>:1:13' y_n=acc_y/sqrt(acc_x^2+acc_y^2+acc_z^2); */
    y_n_tmp = sqrt((rtU.Acc_x * rtU.Acc_x + rtU.Acc_y * rtU.Acc_y) + rtU.Acc_z *
                   rtU.Acc_z);
    y_n = rtU.Acc_y / y_n_tmp;

    /* '<S2>:1:14' z_n=acc_z/sqrt(acc_x^2+acc_y^2+acc_z^2); */
    z_n = rtU.Acc_z / y_n_tmp;

    /* '<S2>:1:15' x_n=acc_x/sqrt(acc_x^2+acc_y^2+acc_z^2); */
    /* '<S2>:1:17' roll=atan2d(y_n,z_n); */
    /* '<S2>:1:18' pitch = atan2d(-x_n,sqrt(y_n^2+z_n^2)); */
    /* '<S2>:1:20' acc_rp=[roll;pitch]; */
    rtb_acc_rp_idx_0 = 57.295779513082323 * atan2(y_n, z_n);
    z_n = atan2(-(rtU.Acc_x / y_n_tmp), sqrt(y_n * y_n + z_n * z_n)) *
      57.295779513082323;
  }

  /* End of MATLAB Function: '<Root>/MATLAB Function3' */

  /* MATLAB Function: '<Root>/Complementary filter' incorporates:
   *  Constant: '<Root>/dt'
   *  Constant: '<Root>/gamma'
   *  Delay: '<Root>/Delay'
   *  Delay: '<Root>/Delay1'
   *  Inport: '<Root>/Gyro_x'
   *  Inport: '<Root>/Gyro_y'
   */
  /* MATLAB Function 'Complementary filter': '<S1>:1' */
  /* '<S1>:1:2' if nargin < 5 */
  /* '<S1>:1:6' measured_roll = (gamma)*(roll+gyro(1)*dt)+(1-gamma)*acc(1); */
  y_n = (rtU.Gyro_x * 0.01 + rtDW.Delay1_DSTATE) * 0.98 + 0.020000000000000018 *
    rtb_acc_rp_idx_0;

  /* '<S1>:1:7' measured_pitch = (gamma)*(pitch+gyro(2)*dt)+(1-gamma)*acc(2); */
  z_n = (rtU.Gyro_y * 0.01 + rtDW.Delay_DSTATE) * 0.98 + 0.020000000000000018 *
    z_n;

  /* MATLAB Function: '<Root>/optimal output' incorporates:
   *  Inport: '<Root>/Gyro_x'
   *  Inport: '<Root>/Gyro_y'
   *  Inport: '<Root>/Gyro_z'
   *  Inport: '<Root>/Ref_Pitch'
   *  Inport: '<Root>/Ref_Roll'
   */
  /* MATLAB Function 'optimal output': '<S4>:1' */
  /* '<S4>:1:3' u = [ */
  /* '<S4>:1:4' 	ref_roll-measured_roll; */
  /* '<S4>:1:5' 	ref_pitch-measured_pitch; */
  /* '<S4>:1:6' 	0-gyro(1); */
  /* '<S4>:1:7' 	0-gyro(2); */
  /* '<S4>:1:8' 	0-gyro(3) */
  /* '<S4>:1:9' 	]; */
  /* '<S4>:1:10' thrust_conversion= 1/(0.06/4*9.81/65536); */
  /* '<S4>:1:11' angle_conversion = pi/180; */
  /*  K = LQR_gain; */
  /* '<S4>:1:13' K = [-0.0130516411025351 -0.0134352368017384 -0.0107871368960590 -0.0111041774433157 -0.00499998563150369;-0.0130516411029438 0.0134352368019400 -0.0107871368966255 0.0111041774440371 0.00499998563150644;0.0130516411030164 0.0134352368022339 0.0107871368964568 0.0111041774437252 -0.00499998563150448;0.0130516411026696 -0.0134352368020933 0.0107871368958907 -0.0111041774430040 0.00499998563150769] */
  /* '<S4>:1:14' u_optimal = K*u.*thrust_conversion*angle_conversion; */
  tmp[0] = rtU.Ref_Roll - y_n;
  tmp[1] = rtU.Ref_Pitch - z_n;
  tmp[2] = 0.0 - rtU.Gyro_x;
  tmp[3] = 0.0 - rtU.Gyro_y;
  tmp[4] = 0.0 - rtU.Gyro_z;

  /* MATLAB Function 'gain_thrust -> motor_thrust': '<S3>:1' */
  /* '<S3>:1:2' motor_thrust = u_optimal+base_thrust; */
  /* '<S3>:1:4' for i = 1:length(motor_thrust) */
  for (i = 0; i < 4; i++) {
    a_0[i] = 0.0;
    for (i_0 = 0; i_0 < 5; i_0++) {
      a_0[i] += a[(i_0 << 2) + i] * tmp[i_0];
    }

    rtb_acc_rp_idx_0 = a_0[i] * 445368.67142371729 * 0.017453292519943295;

    /* MATLAB Function: '<Root>/gain_thrust -> motor_thrust' incorporates:
     *  Constant: '<Root>/Physical'
     */
    /* '<S3>:1:5' motor_thrust(i) = max([0,motor_thrust(i)]); */
    if (0.0 < rtb_acc_rp_idx_0 + 33000.0) {
      y_n_tmp = rtb_acc_rp_idx_0 + 33000.0;
    } else {
      y_n_tmp = 0.0;
    }

    /* '<S3>:1:6' motor_thrust(i) = min([65536,motor_thrust(i)]); */
    if (65536.0 > y_n_tmp) {
      rtb_motor_thrust[i] = y_n_tmp;
    } else {
      rtb_motor_thrust[i] = 65536.0;
    }

    /* End of MATLAB Function: '<Root>/gain_thrust -> motor_thrust' */
    rtb_u_optimal[i] = rtb_acc_rp_idx_0;
  }

  /* End of MATLAB Function: '<Root>/optimal output' */

  /* DataTypeConversion: '<Root>/ToUint16' */
  if (rtb_motor_thrust[0] < 65536.0) {
    /* Outport: '<Root>/Motor_1' */
    rtY.Motor_1 = (uint16_T)rtb_motor_thrust[0];
  } else {
    /* Outport: '<Root>/Motor_1' */
    rtY.Motor_1 = MAX_uint16_T;
  }

  /* End of DataTypeConversion: '<Root>/ToUint16' */

  /* DataTypeConversion: '<Root>/ToUint16_1' */
  if (rtb_motor_thrust[1] < 65536.0) {
    /* Outport: '<Root>/Motor_2' */
    rtY.Motor_2 = (uint16_T)rtb_motor_thrust[1];
  } else {
    /* Outport: '<Root>/Motor_2' */
    rtY.Motor_2 = MAX_uint16_T;
  }

  /* End of DataTypeConversion: '<Root>/ToUint16_1' */

  /* DataTypeConversion: '<Root>/ToUint16_2' */
  if (rtb_motor_thrust[2] < 65536.0) {
    /* Outport: '<Root>/Motor_3' */
    rtY.Motor_3 = (uint16_T)rtb_motor_thrust[2];
  } else {
    /* Outport: '<Root>/Motor_3' */
    rtY.Motor_3 = MAX_uint16_T;
  }

  /* End of DataTypeConversion: '<Root>/ToUint16_2' */

  /* DataTypeConversion: '<Root>/ToUint16_3' */
  if (rtb_motor_thrust[3] < 65536.0) {
    /* Outport: '<Root>/Motor_4' */
    rtY.Motor_4 = (uint16_T)rtb_motor_thrust[3];
  } else {
    /* Outport: '<Root>/Motor_4' */
    rtY.Motor_4 = MAX_uint16_T;
  }

  /* End of DataTypeConversion: '<Root>/ToUint16_3' */

  /* Outport: '<Root>/Log1' */
  rtY.Log1 = rtb_u_optimal[0];

  /* Outport: '<Root>/Log2' */
  rtY.Log2 = rtb_u_optimal[1];

  /* Outport: '<Root>/Log3' */
  rtY.Log3 = rtb_u_optimal[2];

  /* Outport: '<Root>/Log4' */
  rtY.Log4 = rtb_u_optimal[3];

  /* Outport: '<Root>/Log5' */
  rtY.Log5 = y_n;

  /* Outport: '<Root>/Log6' */
  rtY.Log6 = z_n;

  /* Update for Delay: '<Root>/Delay1' */
  rtDW.Delay1_DSTATE = y_n;

  /* Update for Delay: '<Root>/Delay' */
  rtDW.Delay_DSTATE = z_n;
}

/* Model initialize function */
void crazyflie_initialize(void)
{
  /* (no initialization code required) */
}

/* Model terminate function */
void crazyflie_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
