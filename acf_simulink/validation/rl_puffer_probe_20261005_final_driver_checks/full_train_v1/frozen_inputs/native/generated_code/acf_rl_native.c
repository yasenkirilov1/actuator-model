/*
 * acf_rl_native.c
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "acf_rl_native".
 *
 * Model version              : 1.48
 * Simulink Coder version : 26.2 (R2026b) 22-May-2026
 * C source code generated on : Mon Oct  5 15:29:23 2026
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "acf_rl_native.h"
#include "rtwtypes.h"
#include <math.h>
#include "rt_nonfinite.h"
#include "acf_rl_native_private.h"
#include <string.h>
#include <float.h>

/* Block signals (default storage) */
B_acf_rl_native_T acf_rl_native_B;

/* Continuous states */
X_acf_rl_native_T acf_rl_native_X;

/* Disabled State Vector */
XDis_acf_rl_native_T acf_rl_native_XDis;

/* Block states (default storage) */
DW_acf_rl_native_T acf_rl_native_DW;

/* External inputs (root inport signals with default storage) */
ExtU_acf_rl_native_T acf_rl_native_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_acf_rl_native_T acf_rl_native_Y;

/* Real-time model */
static RT_MODEL_acf_rl_native_T acf_rl_native_M_;
RT_MODEL_acf_rl_native_T *const acf_rl_native_M = &acf_rl_native_M_;
static void rate_scheduler(void);
real_T rt_TDelayInterpolate(real_T tMinusDelay, real_T tStart, real_T *uBuf,
  int32_T bufSz, int32_T *lastIdx, int32_T oldestIdx, int32_T newIdx, real_T
  initOutput, boolean_T discrete, boolean_T minorStepAndTAtLastMajorOutput)
{
  real_T yout;
  if ((newIdx == 0) && (oldestIdx == 0) && (tMinusDelay > tStart)) {
    yout = initOutput;
  } else if (tMinusDelay <= tStart) {
    yout = initOutput;
  } else if (tMinusDelay <= uBuf[bufSz + oldestIdx]) {
    if (discrete) {
      yout = uBuf[oldestIdx];
    } else {
      real_T t1;
      real_T t2;
      real_T u2;
      int32_T i;
      i = oldestIdx + 1;
      if (bufSz - 1 == oldestIdx) {
        i = 0;
      }

      t1 = uBuf[bufSz + oldestIdx];
      t2 = uBuf[bufSz + i];
      yout = uBuf[oldestIdx];
      u2 = uBuf[i];
      if (t2 == t1) {
        if (tMinusDelay >= t2) {
          yout = u2;
        }
      } else {
        t1 = (t2 - tMinusDelay) / (t2 - t1);
        yout = (1.0 - t1) * u2 + t1 * yout;
      }
    }
  } else {
    int32_T i;
    if (minorStepAndTAtLastMajorOutput) {
      if (newIdx != 0) {
        if (*lastIdx == newIdx) {
          (*lastIdx)--;
        }

        newIdx--;
      } else {
        if (*lastIdx == newIdx) {
          *lastIdx = bufSz - 1;
        }

        newIdx = bufSz - 1;
      }
    }

    i = *lastIdx;
    if (uBuf[bufSz + i] < tMinusDelay) {
      while ((uBuf[bufSz + i] < tMinusDelay) && !(i == newIdx)) {
        if (i < bufSz - 1) {
          i++;
        } else {
          i = 0;
        }
      }
    } else {
      while (uBuf[bufSz + i] >= tMinusDelay) {
        if (i > 0) {
          i--;
        } else {
          i = bufSz - 1;
        }
      }

      if (i < bufSz - 1) {
        i++;
      } else {
        i = 0;
      }
    }

    *lastIdx = i;
    if (discrete) {
      real_T t1;
      t1 = 2.842170943040401E-14 * fabs(uBuf[bufSz + i]);
      t1 = fmax(t1, 2.842170943040401E-14) / 2.0;
      if (tMinusDelay >= uBuf[bufSz + i] - t1) {
        yout = uBuf[i];
      } else if (i == 0) {
        yout = uBuf[bufSz - 1];
      } else {
        yout = uBuf[i - 1];
      }
    } else {
      real_T t1;
      real_T t2;
      real_T u2;
      if (i == 0) {
        t1 = uBuf[bufSz + (bufSz - 1)];
        yout = uBuf[bufSz - 1];
      } else {
        t1 = uBuf[bufSz + (i - 1)];
        yout = uBuf[i - 1];
      }

      t2 = uBuf[bufSz + i];
      u2 = uBuf[i];
      if (t2 == t1) {
        if (tMinusDelay >= t2) {
          yout = u2;
        }
      } else {
        t1 = (t2 - tMinusDelay) / (t2 - t1);
        yout = (1.0 - t1) * u2 + t1 * yout;
      }
    }
  }

  return yout;
}

/*
 *         This function updates active task flag for each subrate.
 *         The function is called at model base rate, hence the
 *         generated code self-manages all its subrates.
 */
static void rate_scheduler(void)
{
  /* Compute which subrates run during the next base time step.  Subrates
   * are an integer multiple of the base rate counter.  Therefore, the subtask
   * counter is reset when it reaches its limit (zero means run).
   */
  (acf_rl_native_M->Timing.TaskCounters.TID[2])++;
  if ((acf_rl_native_M->Timing.TaskCounters.TID[2]) > 9) {/* Sample time: [3.33333e-05s, 0.0s] */
    acf_rl_native_M->Timing.TaskCounters.TID[2] = 0;
  }

  (acf_rl_native_M->Timing.TaskCounters.TID[3])++;
  if ((acf_rl_native_M->Timing.TaskCounters.TID[3]) > 29) {/* Sample time: [0.0001s, 0.0s] */
    acf_rl_native_M->Timing.TaskCounters.TID[3] = 0;
  }

  (acf_rl_native_M->Timing.TaskCounters.TID[4])++;
  if ((acf_rl_native_M->Timing.TaskCounters.TID[4]) > 2999) {/* Sample time: [0.01s, 0.0s] */
    acf_rl_native_M->Timing.TaskCounters.TID[4] = 0;
  }
}

/*
 * This function updates continuous states using the ODE4 fixed-step
 * solver algorithm
 */
static void rt_ertODEUpdateContinuousStates(RTWSolverInfo *si )
{
  time_T t = rtsiGetT(si);
  time_T tnew = rtsiGetSolverStopTime(si);
  time_T h = rtsiGetStepSize(si);
  real_T *x = rtsiGetContStates(si);
  ODE4_IntgData *id = (ODE4_IntgData *)rtsiGetSolverData(si);
  real_T *y = id->y;
  real_T *f0 = id->f[0];
  real_T *f1 = id->f[1];
  real_T *f2 = id->f[2];
  real_T *f3 = id->f[3];
  real_T temp;
  int_T i;
  int_T nXc = 30;
  rtsiSetSimTimeStep(si,MINOR_TIME_STEP);

  /* Save the state values at time t in y, we'll use x as ynew. */
  (void) memcpy(y, x,
                (uint_T)nXc*sizeof(real_T));

  /* Assumes that rtsiSetT and ModelOutputs are up-to-date */
  /* f0 = f(t,y) */
  rtsiSetdX(si, f0);
  acf_rl_native_derivatives();

  /* f1 = f(t + (h/2), y + (h/2)*f0) */
  temp = 0.5 * h;
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (temp*f0[i]);
  }

  rtsiSetT(si, t + temp);
  rtsiSetdX(si, f1);
  acf_rl_native_step();
  acf_rl_native_derivatives();

  /* f2 = f(t + (h/2), y + (h/2)*f1) */
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (temp*f1[i]);
  }

  rtsiSetdX(si, f2);
  acf_rl_native_step();
  acf_rl_native_derivatives();

  /* f3 = f(t + h, y + h*f2) */
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (h*f2[i]);
  }

  rtsiSetT(si, tnew);
  rtsiSetdX(si, f3);
  acf_rl_native_step();
  acf_rl_native_derivatives();

  /* tnew = t + h
     ynew = y + (h/6)*(f0 + 2*f1 + 2*f2 + 2*f3) */
  temp = h / 6.0;
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + temp*(f0[i] + 2.0*f1[i] + 2.0*f2[i] + f3[i]);
  }

  rtsiSetSimTimeStep(si,MAJOR_TIME_STEP);
}

real_T rt_urand_Upu32_Yd_f_pw_snf(uint32_T *u)
{
  uint32_T hi;
  uint32_T lo;

  /* Uniform random number generator (random number between 0 and 1)

     #define IA      16807                      magic multiplier = 7^5
     #define IM      2147483647                 modulus = 2^31-1
     #define IQ      127773                     IM div IA
     #define IR      2836                       IM modulo IA
     #define S       4.656612875245797e-10      reciprocal of 2^31-1
     test = IA * (seed % IQ) - IR * (seed/IQ)
     seed = test < 0 ? (test + IM) : test
     return (seed*S)
   */
  lo = *u % 127773U * 16807U;
  hi = *u / 127773U * 2836U;
  if (lo < hi) {
    *u = 2147483647U - (hi - lo);
  } else {
    *u = lo - hi;
  }

  return (real_T)*u * 4.656612875245797E-10;
}

real_T rt_nrand_Upu32_Yd_f_pw_snf(uint32_T *u)
{
  real_T si;
  real_T sr;
  real_T y;

  /* Normal (Gaussian) random number generator */
  do {
    sr = 2.0 * rt_urand_Upu32_Yd_f_pw_snf(u) - 1.0;
    si = 2.0 * rt_urand_Upu32_Yd_f_pw_snf(u) - 1.0;
    si = sr * sr + si * si;
  } while (si > 1.0);

  y = sqrt(-2.0 * log(si) / si) * sr;
  return y;
}

real_T rt_modd_snf(real_T u0, real_T u1)
{
  real_T y;
  y = u0;
  if (u1 == 0.0) {
    if (u0 == 0.0) {
      y = u1;
    }
  } else if (rtIsNaN(u0) || rtIsNaN(u1) || rtIsInf(u0)) {
    y = (rtNaN);
  } else if (u0 == 0.0) {
    y = 0.0 / u1;
  } else if (rtIsInf(u1)) {
    if ((u1 < 0.0) != (u0 < 0.0)) {
      y = u1;
    }
  } else {
    boolean_T yEq;
    y = fmod(u0, u1);
    yEq = (y == 0.0);
    if (!yEq && (u1 > floor(u1))) {
      real_T q;
      q = fabs(u0 / u1);
      yEq = !(fabs(q - floor(q + 0.5)) > DBL_EPSILON * q);
    }

    if (yEq) {
      y = u1 * 0.0;
    } else if ((u0 < 0.0) != (u1 < 0.0)) {
      y += u1;
    }
  }

  return y;
}

real_T rt_roundd_snf(real_T u)
{
  real_T y;
  if (fabs(u) < 4.503599627370496E+15) {
    if (u >= 0.5) {
      y = floor(u + 0.5);
    } else if (u > -0.5) {
      y = u * 0.0;
    } else {
      y = ceil(u - 0.5);
    }
  } else {
    y = u;
  }

  return y;
}

/* Model step function */
void acf_rl_native_step(void)
{
  /* local block i/o variables */
  real_T rtb_Lowpassfilter;
  real_T rtb_Sensorlowpass;
  real_T rtb_Independentlateralnoise;
  real_T rtb_Zfresh10kHzaccelerationfram;
  real_T rtb_Errortonormaldeviation;
  real_T rtb_Normalerrorfilteredderivati;
  real_T rtb_Filteredmeasureddragratio;
  real_T rtb_Capfaststablecount;
  real_T rtb_Normalerrorandantiwindup;
  real_T rtb_Headgapandvelocityobserver[2];
  real_T rtb_Forcetrendfilter;
  real_T rtb_Capstablecount;
  if (rtmIsMajorTimeStep(acf_rl_native_M)) {
    /* set solver stop time */
    if (!(acf_rl_native_M->Timing.clockTick0+1)) {
      rtsiSetSolverStopTime(&acf_rl_native_M->solverInfo,
                            ((acf_rl_native_M->Timing.clockTickH0 + 1) *
        acf_rl_native_M->Timing.stepSize0 * 4294967296.0));
    } else {
      rtsiSetSolverStopTime(&acf_rl_native_M->solverInfo,
                            ((acf_rl_native_M->Timing.clockTick0 + 1) *
        acf_rl_native_M->Timing.stepSize0 + acf_rl_native_M->Timing.clockTickH0 *
        acf_rl_native_M->Timing.stepSize0 * 4294967296.0));
    }
  }                                    /* end MajorTimeStep */

  /* Update absolute time of base rate at minor time step */
  if (rtmIsMinorTimeStep(acf_rl_native_M)) {
    acf_rl_native_M->Timing.t[0] = rtsiGetT(&acf_rl_native_M->solverInfo);
  }

  {
    real_T rtb_Actualactuatortravel;
    real_T rtb_Causalaccelerationforcurren;
    real_T rtb_Causalsurfacedirection;
    real_T rtb_Compensatedaxialreaction;
    real_T rtb_Compensatedlateralreaction;
    real_T rtb_Currentlimits;
    real_T rtb_Filtermemory;
    real_T rtb_Filtermemory_j;
    real_T rtb_Forcetransmittedtothebodies;
    real_T rtb_Lateralprojection;
    real_T rtb_Measurednormalforce;
    real_T rtb_Normalizer;
    real_T rtb_PIPIDandfeedforwardcurrent;
    real_T rtb_PIandPIDcorrection;
    real_T rtb_Positionwithinevent;
    real_T rtb_Previousfaststablecount;
    real_T rtb_Resetonanyinstability;
    real_T rtb_Sendcorrectiononlywhensettl;
    real_T rtb_Springanddamperforce;
    real_T rtb_Taredaxialcellforce;
    real_T rtb_Upperrigidreaction;
    real_T rtb_Voltagelimitflag;
    real_T rtb_Xfresh10kHzaccelerationfram;
    int32_T i;
    uint32_T delayLen;
    boolean_T rtb_Selectcontinuousfastdwell;
    boolean_T rtb_Travelstopreached;
    boolean_T tmp;
    boolean_T tmp_0;
    boolean_T tmp_1;

    /* TransportDelay: '<S5>/Sensor sample delay' incorporates:
     *  Clock: '<S17>/Time'
     *  TransportDelay: '<S16>/X known channel delay'
     *  TransportDelay: '<S16>/Z known channel delay'
     *  TransportDelay: '<S27>/Position ADC delivery pipeline'
     *  TransportDelay: '<S27>/Robot acceleration frame matching delay'
     *  TransportDelay: '<S27>/Robot velocity frame matching delay'
     *  TransportDelay: '<S8>/Sensor sample delay'
     */
    rtb_Positionwithinevent = acf_rl_native_M->Timing.t[0];

    /* TransportDelay: '<S5>/Sensor sample delay' */
    acf_rl_native_B.Sensorsampledelay = rt_TDelayInterpolate
      (rtb_Positionwithinevent - acf_rl_native_P.Sensorsampledelay_Delay,
       acf_rl_native_DW.Sensorsampledelay_RWORK[0], (real_T *)
       acf_rl_native_DW.Sensorsampledelay_PWORK[0],
       acf_rl_native_DW.Sensorsampledelay_IWORK[3],
       &acf_rl_native_DW.Sensorsampledelay_IWORK[2],
       acf_rl_native_DW.Sensorsampledelay_IWORK[0],
       acf_rl_native_DW.Sensorsampledelay_IWORK[1],
       acf_rl_native_P.Sensorsampledelay_InitOutput, true, rtmIsMinorTimeStep
       (acf_rl_native_M) && (((real_T *)
         acf_rl_native_DW.Sensorsampledelay_PWORK[0])
        [acf_rl_native_DW.Sensorsampledelay_IWORK[1] +
        acf_rl_native_DW.Sensorsampledelay_IWORK[3]] == rtb_Positionwithinevent));

    /* ZeroOrderHold: '<S27>/Fresh 10 kHz robot velocity frame' incorporates:
     *  ZeroOrderHold: '<Root>/Robot prediction sampling'
     *  ZeroOrderHold: '<S12>/RL fresh 100 Hz robot request'
     *  ZeroOrderHold: '<S27>/Fresh 10 kHz position frame'
     *  ZeroOrderHold: '<S27>/Fresh 10 kHz robot acceleration frame'
     */
    tmp = (rtmIsMajorTimeStep(acf_rl_native_M) &&
           acf_rl_native_M->Timing.TaskCounters.TID[3] == 0);
    if (tmp) {
      /* ZeroOrderHold: '<S5>/Fresh 10 kHz force frame' */
      rtb_Sendcorrectiononlywhensettl = acf_rl_native_B.Sensorsampledelay;

      /* Bias: '<S26>/Tared axial cell force' incorporates:
       *  Constant: '<Root>/Requested force'
       *  Gain: '<S26>/Cell error to deviation'
       *  Sum: '<Root>/Force error'
       *  ZeroOrderHold: '<S5>/Fresh 10 kHz force frame'
       */
      rtb_Taredaxialcellforce = (acf_rl_native_P.Requestedforce_Value -
        acf_rl_native_B.Sensorsampledelay) *
        acf_rl_native_P.Cellerrortodeviation_Gain +
        acf_rl_native_P.Taredaxialcellforce_Bias;
    }

    /* TransportDelay: '<S16>/Z known channel delay' */
    acf_rl_native_B.Zknownchanneldelay = rt_TDelayInterpolate
      (rtb_Positionwithinevent - acf_rl_native_P.Zknownchanneldelay_Delay,
       acf_rl_native_DW.Zknownchanneldelay_RWORK[0], (real_T *)
       acf_rl_native_DW.Zknownchanneldelay_PWORK[0],
       acf_rl_native_DW.Zknownchanneldelay_IWORK[3],
       &acf_rl_native_DW.Zknownchanneldelay_IWORK[2],
       acf_rl_native_DW.Zknownchanneldelay_IWORK[0],
       acf_rl_native_DW.Zknownchanneldelay_IWORK[1],
       acf_rl_native_P.Zknownchanneldelay_InitOutput, false, rtmIsMinorTimeStep
       (acf_rl_native_M) && (((real_T *)
         acf_rl_native_DW.Zknownchanneldelay_PWORK[0])
        [acf_rl_native_DW.Zknownchanneldelay_IWORK[1] +
        acf_rl_native_DW.Zknownchanneldelay_IWORK[3]] == rtb_Positionwithinevent));
    if (tmp) {
      /* ZeroOrderHold: '<S16>/Z fresh 10 kHz acceleration frame' */
      rtb_Zfresh10kHzaccelerationfram = acf_rl_native_B.Zknownchanneldelay;

      /* Sum: '<S26>/Compensated axial reaction' incorporates:
       *  Gain: '<S26>/Downstream Z inertia'
       */
      rtb_Compensatedaxialreaction = acf_rl_native_P.DownstreamZinertia_Gain *
        rtb_Zfresh10kHzaccelerationfram + rtb_Taredaxialcellforce;
    }

    /* TransportDelay: '<S8>/Sensor sample delay' */
    acf_rl_native_B.Sensorsampledelay_h = rt_TDelayInterpolate
      (rtb_Positionwithinevent - acf_rl_native_P.Sensorsampledelay_Delay_j,
       acf_rl_native_DW.Sensorsampledelay_RWORK_k[0], (real_T *)
       acf_rl_native_DW.Sensorsampledelay_PWORK_f[0],
       acf_rl_native_DW.Sensorsampledelay_IWORK_i[3],
       &acf_rl_native_DW.Sensorsampledelay_IWORK_i[2],
       acf_rl_native_DW.Sensorsampledelay_IWORK_i[0],
       acf_rl_native_DW.Sensorsampledelay_IWORK_i[1],
       acf_rl_native_P.Sensorsampledelay_InitOutput_h, true, rtmIsMinorTimeStep
       (acf_rl_native_M) && (((real_T *)
         acf_rl_native_DW.Sensorsampledelay_PWORK_f[0])
        [acf_rl_native_DW.Sensorsampledelay_IWORK_i[1] +
        acf_rl_native_DW.Sensorsampledelay_IWORK_i[3]] ==
        rtb_Positionwithinevent));
    if (tmp) {
      /* ZeroOrderHold: '<S8>/Fresh 10 kHz force frame' */
      rtb_Sendcorrectiononlywhensettl = acf_rl_native_B.Sensorsampledelay_h;
    }

    /* TransportDelay: '<S16>/X known channel delay' */
    acf_rl_native_B.Xknownchanneldelay = rt_TDelayInterpolate
      (rtb_Positionwithinevent - acf_rl_native_P.Xknownchanneldelay_Delay,
       acf_rl_native_DW.Xknownchanneldelay_RWORK[0], (real_T *)
       acf_rl_native_DW.Xknownchanneldelay_PWORK[0],
       acf_rl_native_DW.Xknownchanneldelay_IWORK[3],
       &acf_rl_native_DW.Xknownchanneldelay_IWORK[2],
       acf_rl_native_DW.Xknownchanneldelay_IWORK[0],
       acf_rl_native_DW.Xknownchanneldelay_IWORK[1],
       acf_rl_native_P.Xknownchanneldelay_InitOutput, false, rtmIsMinorTimeStep
       (acf_rl_native_M) && (((real_T *)
         acf_rl_native_DW.Xknownchanneldelay_PWORK[0])
        [acf_rl_native_DW.Xknownchanneldelay_IWORK[1] +
        acf_rl_native_DW.Xknownchanneldelay_IWORK[3]] == rtb_Positionwithinevent));
    if (tmp) {
      /* ZeroOrderHold: '<S16>/X fresh 10 kHz acceleration frame' */
      rtb_Xfresh10kHzaccelerationfram = acf_rl_native_B.Xknownchanneldelay;

      /* Sum: '<S26>/Compensated lateral reaction' incorporates:
       *  Gain: '<S26>/Downstream X inertia'
       *  ZeroOrderHold: '<S16>/X fresh 10 kHz acceleration frame'
       */
      rtb_Compensatedlateralreaction = acf_rl_native_P.DownstreamXinertia_Gain *
        acf_rl_native_B.Xknownchanneldelay + rtb_Sendcorrectiononlywhensettl;
    }

    /* TransportDelay: '<S27>/Robot velocity frame matching delay' */
    acf_rl_native_B.Robotvelocityframematchingdelay = rt_TDelayInterpolate
      (rtb_Positionwithinevent - acf_rl_native_P.Robotvelocityframematchingdelay,
       acf_rl_native_DW.Robotvelocityframematchingdelay[0], (real_T *)
       acf_rl_native_DW.Robotvelocityframematchingdel_l[0],
       acf_rl_native_DW.Robotvelocityframematchingdel_c[3],
       &acf_rl_native_DW.Robotvelocityframematchingdel_c[2],
       acf_rl_native_DW.Robotvelocityframematchingdel_c[0],
       acf_rl_native_DW.Robotvelocityframematchingdel_c[1],
       acf_rl_native_P.Robotvelocityframematchingdel_j, true, rtmIsMinorTimeStep
       (acf_rl_native_M) && (((real_T *)
         acf_rl_native_DW.Robotvelocityframematchingdel_l[0])
        [acf_rl_native_DW.Robotvelocityframematchingdel_c[1] +
        acf_rl_native_DW.Robotvelocityframematchingdel_c[3]] ==
        rtb_Positionwithinevent));

    /* ZeroOrderHold: '<S27>/Fresh 10 kHz robot velocity frame' */
    if (tmp) {
      /* ZeroOrderHold: '<S27>/Fresh 10 kHz robot velocity frame' */
      acf_rl_native_B.Fresh10kHzrobotvelocityframe =
        acf_rl_native_B.Robotvelocityframematchingdelay;
    }

    /* TransportDelay: '<S27>/Position ADC delivery pipeline' */
    acf_rl_native_B.PositionADCdeliverypipeline = rt_TDelayInterpolate
      (rtb_Positionwithinevent - acf_rl_native_P.PositionADCdeliverypipeline_Del,
       acf_rl_native_DW.PositionADCdeliverypipeline_RWO[0], (real_T *)
       acf_rl_native_DW.PositionADCdeliverypipeline_PWO[0],
       acf_rl_native_DW.PositionADCdeliverypipeline_IWO[3],
       &acf_rl_native_DW.PositionADCdeliverypipeline_IWO[2],
       acf_rl_native_DW.PositionADCdeliverypipeline_IWO[0],
       acf_rl_native_DW.PositionADCdeliverypipeline_IWO[1],
       acf_rl_native_P.PositionADCdeliverypipeline_Ini, true, rtmIsMinorTimeStep
       (acf_rl_native_M) && (((real_T *)
         acf_rl_native_DW.PositionADCdeliverypipeline_PWO[0])
        [acf_rl_native_DW.PositionADCdeliverypipeline_IWO[1] +
        acf_rl_native_DW.PositionADCdeliverypipeline_IWO[3]] ==
        rtb_Positionwithinevent));

    /* ZeroOrderHold: '<S27>/Fresh 10 kHz position frame' */
    if (tmp) {
      /* ZeroOrderHold: '<S27>/Fresh 10 kHz position frame' */
      acf_rl_native_B.Fresh10kHzpositionframe =
        acf_rl_native_B.PositionADCdeliverypipeline;
    }

    /* TransportDelay: '<S27>/Robot acceleration frame matching delay' */
    acf_rl_native_B.Robotaccelerationframematchingd = rt_TDelayInterpolate
      (rtb_Positionwithinevent - acf_rl_native_P.Robotaccelerationframematchingd,
       acf_rl_native_DW.Robotaccelerationframematchingd[0], (real_T *)
       acf_rl_native_DW.Robotaccelerationframematchin_h[0],
       acf_rl_native_DW.Robotaccelerationframematchin_n[3],
       &acf_rl_native_DW.Robotaccelerationframematchin_n[2],
       acf_rl_native_DW.Robotaccelerationframematchin_n[0],
       acf_rl_native_DW.Robotaccelerationframematchin_n[1],
       acf_rl_native_P.Robotaccelerationframematchin_f, true, rtmIsMinorTimeStep
       (acf_rl_native_M) && (((real_T *)
         acf_rl_native_DW.Robotaccelerationframematchin_h[0])
        [acf_rl_native_DW.Robotaccelerationframematchin_n[1] +
        acf_rl_native_DW.Robotaccelerationframematchin_n[3]] ==
        rtb_Positionwithinevent));

    /* ZeroOrderHold: '<S27>/Fresh 10 kHz robot acceleration frame' */
    if (tmp) {
      /* ZeroOrderHold: '<S27>/Fresh 10 kHz robot acceleration frame' */
      acf_rl_native_B.Fresh10kHzrobotaccelerationfram =
        acf_rl_native_B.Robotaccelerationframematchingd;

      /* DiscreteStateSpace: '<S27>/Head gap and velocity observer' */
      {
        rtb_Headgapandvelocityobserver[0] =
          acf_rl_native_P.Headgapandvelocityobserver_C[0]*
          acf_rl_native_DW.Headgapandvelocityobserver_DSTA[0];
        rtb_Headgapandvelocityobserver[0] +=
          acf_rl_native_P.Headgapandvelocityobserver_D[0]*
          acf_rl_native_B.Fresh10kHzpositionframe;
        rtb_Headgapandvelocityobserver[1] =
          acf_rl_native_P.Headgapandvelocityobserver_C[1]*
          acf_rl_native_DW.Headgapandvelocityobserver_DSTA[0] +
          acf_rl_native_P.Headgapandvelocityobserver_C[2]*
          acf_rl_native_DW.Headgapandvelocityobserver_DSTA[1];
        rtb_Headgapandvelocityobserver[1] +=
          acf_rl_native_P.Headgapandvelocityobserver_D[1]*
          acf_rl_native_B.Fresh10kHzpositionframe;
      }

      /* Gain: '<S28>/Causal surface direction' incorporates:
       *  Sum: '<S28>/Estimated S velocity'
       */
      rtb_Causalsurfacedirection = (acf_rl_native_B.Fresh10kHzrobotvelocityframe
        - rtb_Headgapandvelocityobserver[1]) *
        acf_rl_native_P.Causalsurfacedirection_Gain;

      /* Product: '<S28>/Lateral projection' */
      rtb_Lateralprojection = rtb_Compensatedlateralreaction *
        rtb_Causalsurfacedirection;

      /* Bias: '<S28>/Squared slope plus one' incorporates:
       *  Math: '<S28>/Squared slope'
       */
      rtb_Sendcorrectiononlywhensettl = rtb_Causalsurfacedirection *
        rtb_Causalsurfacedirection + acf_rl_native_P.Squaredslopeplusone_Bias;

      /* Math: '<S28>/Normalizer'
       *
       * About '<S28>/Normalizer':
       *  Operator: sqrt
       */
      if (rtb_Sendcorrectiononlywhensettl < 0.0) {
        rtb_Normalizer = -sqrt(fabs(rtb_Sendcorrectiononlywhensettl));
      } else {
        rtb_Normalizer = sqrt(rtb_Sendcorrectiononlywhensettl);
      }

      /* End of Math: '<S28>/Normalizer' */

      /* Product: '<S28>/Measured normal force' incorporates:
       *  Sum: '<S28>/Normal projection numerator'
       */
      rtb_Measurednormalforce = (rtb_Compensatedaxialreaction -
        rtb_Lateralprojection) / rtb_Normalizer;

      /* Bias: '<S27>/Estimated head travel' */
      acf_rl_native_B.Estimatedheadtravel = rtb_Headgapandvelocityobserver[0] +
        acf_rl_native_P.Estimatedheadtravel_Bias;
    }

    /* Integrator: '<S7>/Predicted velocity' */
    acf_rl_native_B.Predictedvelocity = acf_rl_native_X.Predictedvelocity_CSTATE;
    if (tmp) {
      /* ZeroOrderHold: '<Root>/Predicted velocity sampling' */
      acf_rl_native_B.Predictedvelocitysampling =
        acf_rl_native_B.Predictedvelocity;
    }

    /* ZeroOrderHold: '<S12>/Robot feedback sampling' incorporates:
     *  ZeroOrderHold: '<S12>/RL fresh 100 Hz robot request'
     *  ZeroOrderHold: '<S39>/Fast ready at robot rate'
     *  ZeroOrderHold: '<S39>/Robot-rate force sample'
     */
    tmp_0 = (rtmIsMajorTimeStep(acf_rl_native_M) &&
             acf_rl_native_M->Timing.TaskCounters.TID[4] == 0);
    if (tmp_0) {
      /* Delay: '<S7>/Expected command delay' incorporates:
       *  Constant: '<S7>/RL reset-fixed queue length'
       */
      if ((acf_rl_native_P.RLresetfixedqueuelength_Value < 1.0) || rtIsNaN
          (acf_rl_native_P.RLresetfixedqueuelength_Value)) {
        delayLen = 1U;
      } else if (acf_rl_native_P.RLresetfixedqueuelength_Value > 12.0) {
        delayLen = 12U;
      } else {
        rtb_Upperrigidreaction = trunc
          (acf_rl_native_P.RLresetfixedqueuelength_Value);
        if (rtIsNaN(rtb_Upperrigidreaction) || rtIsInf(rtb_Upperrigidreaction))
        {
          rtb_Upperrigidreaction = 0.0;
        } else {
          rtb_Upperrigidreaction = fmod(rtb_Upperrigidreaction, 4.294967296E+9);
        }

        if (rtb_Upperrigidreaction < 0.0) {
          delayLen = (uint32_T)-(int32_T)(uint32_T)-rtb_Upperrigidreaction;
        } else {
          delayLen = (uint32_T)rtb_Upperrigidreaction;
        }
      }

      if (delayLen <= acf_rl_native_DW.CircBufIdx) {
        delayLen = acf_rl_native_DW.CircBufIdx - delayLen;
      } else {
        delayLen = (acf_rl_native_DW.CircBufIdx - delayLen) + 12U;
      }

      rtb_Resetonanyinstability =
        acf_rl_native_DW.Expectedcommanddelay_DSTATE[delayLen];

      /* End of Delay: '<S7>/Expected command delay' */

      /* Saturate: '<S7>/Expected speed limit' */
      if (rtb_Resetonanyinstability >
          acf_rl_native_P.Expectedspeedlimit_UpperSat) {
        /* Saturate: '<S7>/Expected speed limit' */
        acf_rl_native_B.Expectedspeedlimit =
          acf_rl_native_P.Expectedspeedlimit_UpperSat;
      } else if (rtb_Resetonanyinstability <
                 acf_rl_native_P.Expectedspeedlimit_LowerSat) {
        /* Saturate: '<S7>/Expected speed limit' */
        acf_rl_native_B.Expectedspeedlimit =
          acf_rl_native_P.Expectedspeedlimit_LowerSat;
      } else {
        /* Saturate: '<S7>/Expected speed limit' */
        acf_rl_native_B.Expectedspeedlimit = rtb_Resetonanyinstability;
      }

      /* End of Saturate: '<S7>/Expected speed limit' */
    }

    /* Gain: '<S7>/Expected velocity response' incorporates:
     *  Sum: '<S7>/Predicted velocity error'
     */
    rtb_Upperrigidreaction = (acf_rl_native_B.Expectedspeedlimit -
      acf_rl_native_B.Predictedvelocity) *
      acf_rl_native_P.Expectedvelocityresponse_Gain;

    /* Saturate: '<S7>/Expected acceleration limit' */
    if (rtb_Upperrigidreaction > acf_rl_native_P.Expectedaccelerationlimit_Upper)
    {
      /* Saturate: '<S7>/Expected acceleration limit' */
      acf_rl_native_B.Expectedaccelerationlimit =
        acf_rl_native_P.Expectedaccelerationlimit_Upper;
    } else if (rtb_Upperrigidreaction <
               acf_rl_native_P.Expectedaccelerationlimit_Lower) {
      /* Saturate: '<S7>/Expected acceleration limit' */
      acf_rl_native_B.Expectedaccelerationlimit =
        acf_rl_native_P.Expectedaccelerationlimit_Lower;
    } else {
      /* Saturate: '<S7>/Expected acceleration limit' */
      acf_rl_native_B.Expectedaccelerationlimit = rtb_Upperrigidreaction;
    }

    /* End of Saturate: '<S7>/Expected acceleration limit' */

    /* ZeroOrderHold: '<Root>/Robot prediction sampling' */
    if (tmp) {
      /* ZeroOrderHold: '<Root>/Robot prediction sampling' */
      acf_rl_native_B.Robotpredictionsampling =
        acf_rl_native_B.Expectedaccelerationlimit;

      /* Gain: '<Root>/Enable robot acceleration prediction' */
      acf_rl_native_B.Enablerobotaccelerationpredicti =
        acf_rl_native_P.Enablerobotaccelerationpredicti *
        acf_rl_native_B.Robotpredictionsampling;

      /* Gain: '<S3>/Normal force error sign' incorporates:
       *  Bias: '<S28>/Measured normal force deviation'
       */
      rtb_Sendcorrectiononlywhensettl = (rtb_Measurednormalforce +
        acf_rl_native_P.Measurednormalforcedeviation_Bi) *
        acf_rl_native_P.Normalforceerrorsign_Gain;

      /* Gain: '<S36>/Error to normal deviation' */
      rtb_Errortonormaldeviation = acf_rl_native_P.Errortonormaldeviation_Gain *
        rtb_Sendcorrectiononlywhensettl;

      /* Sum: '<S36>/Normal error filtered derivative' incorporates:
       *  Gain: '<S36>/Normal error new derivative weight'
       *  Gain: '<S36>/Normal error old derivative weight'
       *  Sum: '<S36>/Normal error sample increment'
       *  UnitDelay: '<S36>/Normal error derivative memory'
       *  UnitDelay: '<S36>/Normal error previous sample'
       */
      rtb_Normalerrorfilteredderivati = (rtb_Errortonormaldeviation -
        acf_rl_native_DW.Normalerrorprevioussample_DSTAT) *
        acf_rl_native_P.Normalerrornewderivativeweight_ +
        acf_rl_native_P.Normalerroroldderivativeweight_ *
        acf_rl_native_DW.Normalerrorderivativememory_DST;

      /* DiscreteTransferFcn: '<S36>/Optional feedback rolloff' incorporates:
       *  DiscreteIntegrator: '<S36>/Normal force error integral'
       *  Gain: '<S36>/Normal force derivative current'
       *  Gain: '<S36>/Normal force integral current'
       *  Gain: '<S36>/Normal force proportional current'
       *  Sum: '<S36>/PI and PID correction'
       */
      acf_rl_native_DW.Optionalfeedbackrolloff_tmp =
        ((((acf_rl_native_P.Normalforceproportionalcurrent_ *
            rtb_Errortonormaldeviation +
            acf_rl_native_P.Normalforceintegralcurrent_Gain *
            acf_rl_native_DW.Normalforceerrorintegral_DSTATE) +
           acf_rl_native_P.Normalforcederivativecurrent_Ga *
           rtb_Normalerrorfilteredderivati) -
          acf_rl_native_DW.Optionalfeedbackrolloff_states[0] *
          acf_rl_native_P.Optionalfeedbackrolloff_DenCoef[1]) -
         acf_rl_native_DW.Optionalfeedbackrolloff_states[1] *
         acf_rl_native_P.Optionalfeedbackrolloff_DenCoef[2]) /
        acf_rl_native_P.Optionalfeedbackrolloff_DenCoef[0];

      /* DiscreteTransferFcn: '<S35>/Filtered measured acceleration rate' */
      acf_rl_native_DW.Filteredmeasuredaccelerationr_f =
        (rtb_Zfresh10kHzaccelerationfram -
         acf_rl_native_P.Filteredmeasuredaccelerationr_f[1] *
         acf_rl_native_DW.Filteredmeasuredaccelerationrat) /
        acf_rl_native_P.Filteredmeasuredaccelerationr_f[0];

      /* Switch: '<S32>/Hold drag estimate below force guard' incorporates:
       *  Constant: '<S32>/Minimum observable normal force'
       *  RelationalOperator: '<S32>/Force sufficient for drag estimate'
       *  UnitDelay: '<S32>/Previous filtered drag ratio'
       */
      if (rtb_Measurednormalforce >
          acf_rl_native_P.Minimumobservablenormalforce_Va) {
        /* Product: '<S32>/Normal projection denominator' */
        rtb_Upperrigidreaction = rtb_Measurednormalforce * rtb_Normalizer;

        /* Saturate: '<S32>/Guard projection denominator' */
        if (rtb_Upperrigidreaction >
            acf_rl_native_P.Guardprojectiondenominator_Uppe) {
          rtb_Upperrigidreaction =
            acf_rl_native_P.Guardprojectiondenominator_Uppe;
        } else if (rtb_Upperrigidreaction <
                   acf_rl_native_P.Guardprojectiondenominator_Lowe) {
          rtb_Upperrigidreaction =
            acf_rl_native_P.Guardprojectiondenominator_Lowe;
        }

        /* Product: '<S32>/Measured drag ratio' incorporates:
         *  Product: '<S32>/Slope times axial reaction'
         *  Saturate: '<S32>/Guard projection denominator'
         *  Sum: '<S32>/Tangential force numerator'
         */
        rtb_Currentlimits = ((0.0 - rtb_Compensatedaxialreaction *
                              rtb_Causalsurfacedirection) -
                             rtb_Compensatedlateralreaction) /
          rtb_Upperrigidreaction;

        /* Saturate: '<S32>/Bound signed drag ratio' */
        if (rtb_Currentlimits > acf_rl_native_P.Boundsigneddragratio_UpperSat) {
          rtb_Currentlimits = acf_rl_native_P.Boundsigneddragratio_UpperSat;
        } else if (rtb_Currentlimits <
                   acf_rl_native_P.Boundsigneddragratio_LowerSat) {
          rtb_Currentlimits = acf_rl_native_P.Boundsigneddragratio_LowerSat;
        }

        /* End of Saturate: '<S32>/Bound signed drag ratio' */
      } else {
        rtb_Currentlimits = acf_rl_native_DW.Previousfiltereddragratio_DSTAT;
      }

      /* End of Switch: '<S32>/Hold drag estimate below force guard' */

      /* Sum: '<S32>/Filtered measured drag ratio' incorporates:
       *  Gain: '<S32>/New drag estimate weight'
       *  Gain: '<S32>/Previous drag estimate weight'
       *  UnitDelay: '<S32>/Previous filtered drag ratio'
       */
      rtb_Filteredmeasureddragratio = acf_rl_native_P.Newdragestimateweight_Gain
        * rtb_Currentlimits + acf_rl_native_P.Previousdragestimateweight_Gain *
        acf_rl_native_DW.Previousfiltereddragratio_DSTAT;

      /* Switch: '<S28>/Select axial force geometry request' incorporates:
       *  Bias: '<S32>/Axial normal-force geometry factor'
       *  Constant: '<S28>/Measured drag geometry selected'
       *  Gain: '<S28>/Normal reference projection'
       *  Gain: '<S32>/Drag correction sign'
       *  Gain: '<S32>/Normal force reference'
       *  Product: '<S32>/Estimated drag slope product'
       *  Product: '<S32>/Requested axial force from measured drag'
       *  Sum: '<S28>/Requested axial force'
       */
      if (acf_rl_native_P.Measureddraggeometryselected_Va >=
          acf_rl_native_P.Selectaxialforcegeometryrequest) {
        rtb_Currentlimits = (rtb_Filteredmeasureddragratio *
                             rtb_Causalsurfacedirection *
                             acf_rl_native_P.Dragcorrectionsign_Gain +
                             acf_rl_native_P.Axialnormalforcegeometryfactor_) *
          acf_rl_native_P.Normalforcereference_Gain / rtb_Normalizer;
      } else {
        rtb_Currentlimits = acf_rl_native_P.Normalreferenceprojection_Gain *
          rtb_Normalizer + rtb_Lateralprojection;
      }

      /* End of Switch: '<S28>/Select axial force geometry request' */

      /* Bias: '<S28>/Axial reference deviation' */
      rtb_Currentlimits += acf_rl_native_P.Axialreferencedeviation_Bias;

      /* Sum: '<S35>/Predicted guide velocity' incorporates:
       *  Gain: '<S35>/Identified guide prediction time'
       *  Sum: '<S35>/Measured head gap acceleration'
       */
      rtb_PIandPIDcorrection = (acf_rl_native_B.Fresh10kHzrobotaccelerationfram
        - rtb_Zfresh10kHzaccelerationfram) *
        acf_rl_native_P.Identifiedguidepredictiontime_G +
        rtb_Headgapandvelocityobserver[1];

      /* DiscreteTransferFcn: '<S34>/Proper identified lower-link inverse' incorporates:
       *  Gain: '<S34>/Known downstream S inertia'
       *  Sum: '<S34>/Desired lower load deviation'
       */
      acf_rl_native_DW.Properidentifiedlowerlinkinve_k = ((((rtb_Currentlimits -
        acf_rl_native_P.KnowndownstreamSinertia_Gain *
        rtb_Zfresh10kHzaccelerationfram) -
        acf_rl_native_DW.Properidentifiedlowerlinkinvers[0] *
        acf_rl_native_P.Properidentifiedlowerlinkinve_p[1]) -
        acf_rl_native_DW.Properidentifiedlowerlinkinvers[1] *
        acf_rl_native_P.Properidentifiedlowerlinkinve_p[2]) -
        acf_rl_native_DW.Properidentifiedlowerlinkinvers[2] *
        acf_rl_native_P.Properidentifiedlowerlinkinve_p[3]) /
        acf_rl_native_P.Properidentifiedlowerlinkinve_p[0];

      /* Sum: '<S33>/PI PID and feedforward current' incorporates:
       *  Constant: '<S33>/Flat holding current'
       *  DiscreteTransferFcn: '<S34>/Proper identified lower-link inverse'
       *  DiscreteTransferFcn: '<S35>/Filtered measured acceleration rate'
       *  DiscreteTransferFcn: '<S36>/Optional feedback rolloff'
       *  Gain: '<S34>/Optional inverse feedforward scale'
       *  Gain: '<S35>/Acceleration FF prediction time'
       *  Gain: '<S35>/Geometry current feedforward'
       *  Gain: '<S35>/Guide Coulomb friction current'
       *  Gain: '<S35>/Guide friction velocity scale'
       *  Gain: '<S35>/Guide viscous friction current'
       *  Gain: '<S35>/Measured acceleration current feedforward'
       *  Sum: '<S35>/Causal acceleration for current FF'
       *  Sum: '<S35>/Guide friction feedforward'
       *  Trigonometry: '<S35>/Guide friction direction'
       */
      rtb_PIPIDandfeedforwardcurrent =
        (((((acf_rl_native_P.Filteredmeasuredaccelerationrat[0] *
             acf_rl_native_DW.Filteredmeasuredaccelerationr_f +
             acf_rl_native_P.Filteredmeasuredaccelerationrat[1] *
             acf_rl_native_DW.Filteredmeasuredaccelerationrat) *
            acf_rl_native_P.AccelerationFFpredictiontime_Ga +
            rtb_Zfresh10kHzaccelerationfram) *
           acf_rl_native_P.Measuredaccelerationcurrentfeed +
           (((acf_rl_native_P.Optionalfeedbackrolloff_NumCoef[0] *
              acf_rl_native_DW.Optionalfeedbackrolloff_tmp +
              acf_rl_native_DW.Optionalfeedbackrolloff_states[0] *
              acf_rl_native_P.Optionalfeedbackrolloff_NumCoef[1]) +
             acf_rl_native_DW.Optionalfeedbackrolloff_states[1] *
             acf_rl_native_P.Optionalfeedbackrolloff_NumCoef[2]) +
            acf_rl_native_P.Flatholdingcurrent_Value)) +
          acf_rl_native_P.Geometrycurrentfeedforward_Gain * rtb_Currentlimits) +
         (tanh(acf_rl_native_P.Guidefrictionvelocityscale_Gain *
               rtb_PIandPIDcorrection) *
          acf_rl_native_P.GuideCoulombfrictioncurrent_Gai +
          acf_rl_native_P.Guideviscousfrictioncurrent_Gai *
          rtb_PIandPIDcorrection)) +
        (((acf_rl_native_P.Properidentifiedlowerlinkinvers[0] *
           acf_rl_native_DW.Properidentifiedlowerlinkinve_k +
           acf_rl_native_DW.Properidentifiedlowerlinkinvers[0] *
           acf_rl_native_P.Properidentifiedlowerlinkinvers[1]) +
          acf_rl_native_DW.Properidentifiedlowerlinkinvers[1] *
          acf_rl_native_P.Properidentifiedlowerlinkinvers[2]) +
         acf_rl_native_DW.Properidentifiedlowerlinkinvers[2] *
         acf_rl_native_P.Properidentifiedlowerlinkinvers[3]) *
        acf_rl_native_P.Optionalinversefeedforwardscale;

      /* Switch: '<S33>/Baseline or external current' incorporates:
       *  Inport: '<Root>/rl_baseline_mode'
       *  Inport: '<Root>/rl_current_request_A'
       */
      if (acf_rl_native_U.rl_baseline_mode >
          acf_rl_native_P.Baselineorexternalcurrent_Thres) {
        rtb_Upperrigidreaction = rtb_PIPIDandfeedforwardcurrent;
      } else {
        rtb_Upperrigidreaction = acf_rl_native_U.rl_current_request_A;
      }

      /* DiscreteTransferFcn: '<S33>/Optional structural-mode current notch' incorporates:
       *  Switch: '<S33>/Baseline or external current'
       */
      acf_rl_native_DW.Optionalstructuralmodecurrent_n =
        ((rtb_Upperrigidreaction -
          acf_rl_native_DW.Optionalstructuralmodecurrentno[0] *
          acf_rl_native_P.Optionalstructuralmodecurrent_c[1]) -
         acf_rl_native_DW.Optionalstructuralmodecurrentno[1] *
         acf_rl_native_P.Optionalstructuralmodecurrent_c[2]) /
        acf_rl_native_P.Optionalstructuralmodecurrent_c[0];
      rtb_PIandPIDcorrection = (acf_rl_native_P.Optionalstructuralmodecurrentno
        [0] * acf_rl_native_DW.Optionalstructuralmodecurrent_n +
        acf_rl_native_DW.Optionalstructuralmodecurrentno[0] *
        acf_rl_native_P.Optionalstructuralmodecurrentno[1]) +
        acf_rl_native_DW.Optionalstructuralmodecurrentno[1] *
        acf_rl_native_P.Optionalstructuralmodecurrentno[2];

      /* Saturate: '<S33>/Current limits' */
      if (rtb_PIandPIDcorrection > acf_rl_native_P.Currentlimits_UpperSat) {
        rtb_Currentlimits = acf_rl_native_P.Currentlimits_UpperSat;
      } else if (rtb_PIandPIDcorrection < acf_rl_native_P.Currentlimits_LowerSat)
      {
        rtb_Currentlimits = acf_rl_native_P.Currentlimits_LowerSat;
      } else {
        rtb_Currentlimits = rtb_PIandPIDcorrection;
      }

      /* End of Saturate: '<S33>/Current limits' */
    }

    if (tmp_0) {
      /* Switch: '<S17>/Bounded or constant normal program' incorporates:
       *  Constant: '<S17>/Bounded program selected'
       */
      if (acf_rl_native_P.Boundedprogramselected_Value >
          acf_rl_native_P.Boundedorconstantnormalprogram_) {
        /* Step: '<S17>/Normal command change 1' incorporates:
         *  Step: '<S17>/Normal command change 2'
         *  Step: '<S17>/Normal command change 3'
         */
        rtb_Taredaxialcellforce = (((acf_rl_native_M->Timing.clockTick4+
          acf_rl_native_M->Timing.clockTickH4* 4294967296.0)) * 0.01);
        if (rtb_Taredaxialcellforce < acf_rl_native_P.Normalcommandchange1_Time)
        {
          rtb_Upperrigidreaction = acf_rl_native_P.Normalcommandchange1_Y0;
        } else {
          rtb_Upperrigidreaction = acf_rl_native_P.Normalcommandchange1_YFinal;
        }

        /* Step: '<S17>/Normal command change 2' */
        if (rtb_Taredaxialcellforce < acf_rl_native_P.Normalcommandchange2_Time)
        {
          rtb_Lateralprojection = acf_rl_native_P.Normalcommandchange2_Y0;
        } else {
          rtb_Lateralprojection = acf_rl_native_P.Normalcommandchange2_YFinal;
        }

        /* Step: '<S17>/Normal command change 3' */
        if (rtb_Taredaxialcellforce < acf_rl_native_P.Normalcommandchange3_Time)
        {
          rtb_Taredaxialcellforce = acf_rl_native_P.Normalcommandchange3_Y0;
        } else {
          rtb_Taredaxialcellforce = acf_rl_native_P.Normalcommandchange3_YFinal;
        }

        /* Switch: '<S17>/Bounded or constant normal program' incorporates:
         *  Constant: '<S17>/Nominal robot velocity'
         *  Product: '<S17>/Normal velocity times bounded program'
         *  Step: '<S17>/Normal command change 1'
         *  Step: '<S17>/Normal command change 2'
         *  Step: '<S17>/Normal command change 3'
         *  Sum: '<S17>/Bounded normal command factor'
         */
        acf_rl_native_B.Boundedorconstantnormalprogram =
          ((rtb_Upperrigidreaction + rtb_Lateralprojection) +
           rtb_Taredaxialcellforce) * acf_rl_native_P.Nominalrobotvelocity_Value;
      } else {
        /* Switch: '<S17>/Bounded or constant normal program' incorporates:
         *  Constant: '<S17>/Nominal robot velocity'
         */
        acf_rl_native_B.Boundedorconstantnormalprogram =
          acf_rl_native_P.Nominalrobotvelocity_Value;
      }

      /* End of Switch: '<S17>/Bounded or constant normal program' */
    }

    /* Sum: '<S17>/Time since event' incorporates:
     *  Constant: '<S17>/Event start'
     */
    rtb_Lateralprojection = rtb_Positionwithinevent -
      acf_rl_native_P.Eventstart_Value;

    /* Gain: '<S17>/Robot error time scale' */
    rtb_Normalizer = acf_rl_native_P.Roboterrortimescale_Gain *
      rtb_Lateralprojection;

    /* Saturate: '<S17>/Robot error transition' */
    if (rtb_Normalizer > acf_rl_native_P.Roboterrortransition_UpperSat) {
      rtb_Normalizer = acf_rl_native_P.Roboterrortransition_UpperSat;
    } else if (rtb_Normalizer < acf_rl_native_P.Roboterrortransition_LowerSat) {
      rtb_Normalizer = acf_rl_native_P.Roboterrortransition_LowerSat;
    }

    /* End of Saturate: '<S17>/Robot error transition' */

    /* Sum: '<S17>/Nominal path velocity' incorporates:
     *  Gain: '<S17>/Robot positioning error velocity'
     *  Polyval: '<S17>/Smooth robot error slope'
     */
    acf_rl_native_B.Nominalpathvelocity =
      ((((acf_rl_native_P.Smoothroboterrorslope_Coefs[0] * rtb_Normalizer +
          acf_rl_native_P.Smoothroboterrorslope_Coefs[1]) * rtb_Normalizer +
         acf_rl_native_P.Smoothroboterrorslope_Coefs[2]) * rtb_Normalizer +
        acf_rl_native_P.Smoothroboterrorslope_Coefs[3]) * rtb_Normalizer +
       acf_rl_native_P.Smoothroboterrorslope_Coefs[4]) *
      acf_rl_native_P.Robotpositioningerrorvelocity_G +
      acf_rl_native_B.Boundedorconstantnormalprogram;
    if (tmp_0) {
      /* ZeroOrderHold: '<S12>/Nominal path sampling' */
      acf_rl_native_B.Nominalpathsampling = acf_rl_native_B.Nominalpathvelocity;
    }

    /* Switch: '<S17>/Single or repeated' incorporates:
     *  Constant: '<S17>/Repeat selected'
     */
    if (acf_rl_native_P.Repeatselected_Value != 0.0) {
      /* Math: '<S17>/Repeated bump time' incorporates:
       *  Constant: '<S17>/Repeat interval'
       */
      rtb_Taredaxialcellforce = rt_modd_snf(rtb_Lateralprojection,
        acf_rl_native_P.Repeatinterval_Value);
    } else {
      rtb_Taredaxialcellforce = rtb_Lateralprojection;
    }

    /* End of Switch: '<S17>/Single or repeated' */

    /* Gain: '<S17>/Scan speed over width' */
    rtb_Positionwithinevent = acf_rl_native_P.Scanspeedoverwidth_Gain *
      rtb_Taredaxialcellforce;

    /* Saturate: '<S17>/Position within event' */
    if (rtb_Positionwithinevent > acf_rl_native_P.Positionwithinevent_UpperSat)
    {
      rtb_Positionwithinevent = acf_rl_native_P.Positionwithinevent_UpperSat;
    } else if (rtb_Positionwithinevent <
               acf_rl_native_P.Positionwithinevent_LowerSat) {
      rtb_Positionwithinevent = acf_rl_native_P.Positionwithinevent_LowerSat;
    }

    /* End of Saturate: '<S17>/Position within event' */

    /* Switch: '<S17>/Surface shape 1' incorporates:
     *  Constant: '<S17>/Ramp selected'
     */
    if (acf_rl_native_P.Rampselected_Value != 0.0) {
      /* Polyval: '<S17>/Ramp component 1' */
      rtb_Taredaxialcellforce = acf_rl_native_P.Rampcomponent1_Coefs[0];
      for (i = 0; i < 5; i++) {
        rtb_Taredaxialcellforce = rtb_Taredaxialcellforce *
          rtb_Positionwithinevent + acf_rl_native_P.Rampcomponent1_Coefs[i + 1];
      }

      /* End of Polyval: '<S17>/Ramp component 1' */
    } else {
      /* Polyval: '<S17>/Bump component 1' */
      rtb_Taredaxialcellforce = acf_rl_native_P.Bumpcomponent1_Coefs[0];
      for (i = 0; i < 6; i++) {
        rtb_Taredaxialcellforce = rtb_Taredaxialcellforce *
          rtb_Positionwithinevent + acf_rl_native_P.Bumpcomponent1_Coefs[i + 1];
      }

      /* End of Polyval: '<S17>/Bump component 1' */
    }

    /* End of Switch: '<S17>/Surface shape 1' */

    /* Switch: '<S17>/Enable surface event' incorporates:
     *  Constant: '<S17>/Disabled'
     *  Constant: '<S17>/Surface displacement enabled'
     *  Constant: '<S17>/Zero'
     *  RelationalOperator: '<S17>/Event has started'
     */
    if (rtb_Lateralprojection >= acf_rl_native_P.Zero_Value) {
      rtb_Normalizer = acf_rl_native_P.Surfacedisplacementenabled_Valu;
    } else {
      rtb_Normalizer = acf_rl_native_P.Disabled_Value;
    }

    /* End of Switch: '<S17>/Enable surface event' */

    /* Gain: '<S17>/Surface SI scale 1' incorporates:
     *  Product: '<S17>/Enable motion 1'
     */
    rtb_Causalaccelerationforcurren = rtb_Taredaxialcellforce * rtb_Normalizer *
      acf_rl_native_P.SurfaceSIscale1_Gain;

    /* Sum: '<S23>/Actual actuator travel' incorporates:
     *  Constant: '<S23>/Mid travel'
     *  Integrator: '<S21>/Position'
     *  Integrator: '<S22>/Position'
     *  Sum: '<S23>/Gap displacement'
     */
    rtb_Actualactuatortravel = (acf_rl_native_X.Position_CSTATE -
      acf_rl_native_X.Position_CSTATE_p) + acf_rl_native_P.Midtravel_Value;

    /* Switch: '<S13>/Select position instrument' incorporates:
     *  Constant: '<S13>/Coordinate reference'
     *  Constant: '<S13>/Use toolhead position'
     *  Gain: '<S13>/Tool S relative to A'
     *  Integrator: '<S21>/Position'
     *  SignalConversion generated from: '<S13>/Tool S relative to A'
     *  Sum: '<S13>/Tool measurement coordinate'
     */
    if (acf_rl_native_P.Usetoolheadposition_Value != 0.0) {
      rtb_Lateralprojection = (acf_rl_native_P.ToolSrelativetoA_Gain[0] *
        acf_rl_native_X.Position_CSTATE + acf_rl_native_P.ToolSrelativetoA_Gain
        [1] * rtb_Causalaccelerationforcurren) +
        acf_rl_native_P.Coordinatereference_Value;
    } else {
      rtb_Lateralprojection = rtb_Actualactuatortravel;
    }

    /* End of Switch: '<S13>/Select position instrument' */
    tmp_1 = (rtmIsMajorTimeStep(acf_rl_native_M) &&
             acf_rl_native_M->Timing.TaskCounters.TID[1] == 0);
    if (tmp_1) {
      /* UniformRandomNumber: '<S8>/Independent lateral noise' incorporates:
       *  UniformRandomNumber: '<S13>/Position noise'
       */
      rtb_Independentlateralnoise = acf_rl_native_DW.Positionnoise_NextOutput;

      /* Gain: '<S13>/Position noise RMS' */
      acf_rl_native_B.PositionnoiseRMS = acf_rl_native_P.PositionnoiseRMS_Gain *
        rtb_Independentlateralnoise;
    }

    /* Gain: '<S13>/Measurement delay' incorporates:
     *  Bias: '<S13>/Position calibrated origin'
     *  Bias: '<S13>/Position calibrated zero'
     *  Gain: '<S13>/Residual position calibration'
     *  Sum: '<S13>/Sensed position'
     */
    rtb_Lateralprojection = (((rtb_Lateralprojection +
      acf_rl_native_P.Positioncalibratedzero_Bias) *
      acf_rl_native_P.Residualpositioncalibration_Gai +
      acf_rl_native_P.Positioncalibratedorigin_Bias) +
      acf_rl_native_B.PositionnoiseRMS) * acf_rl_native_P.Measurementdelay_Gain;

    /* Switch: '<S12>/Measured or estimated travel' incorporates:
     *  Constant: '<S12>/Tool sensor needs gap estimate'
     */
    if (acf_rl_native_P.Toolsensorneedsgapestimate_Valu != 0.0) {
      /* Switch: '<S12>/Measured or estimated travel' */
      acf_rl_native_B.Measuredorestimatedtravel =
        acf_rl_native_B.Estimatedheadtravel;
    } else {
      /* Switch: '<S12>/Measured or estimated travel' */
      acf_rl_native_B.Measuredorestimatedtravel = rtb_Lateralprojection;
    }

    /* End of Switch: '<S12>/Measured or estimated travel' */

    /* ZeroOrderHold: '<S12>/Robot feedback sampling' */
    if (tmp_0) {
      /* ZeroOrderHold: '<S12>/Robot feedback sampling' */
      acf_rl_native_B.Robotfeedbacksampling =
        acf_rl_native_B.Measuredorestimatedtravel;

      /* Sum: '<S12>/Distance from mid stroke' incorporates:
       *  Constant: '<S12>/Mid stroke'
       */
      acf_rl_native_B.Distancefrommidstroke = acf_rl_native_P.Midstroke_Value -
        acf_rl_native_B.Robotfeedbacksampling;

      /* DeadZone: '<S12>/Recenter deadband' */
      if (acf_rl_native_B.Distancefrommidstroke >
          acf_rl_native_P.Recenterdeadband_End) {
        rtb_Upperrigidreaction = acf_rl_native_B.Distancefrommidstroke -
          acf_rl_native_P.Recenterdeadband_End;
      } else if (acf_rl_native_B.Distancefrommidstroke >=
                 acf_rl_native_P.Recenterdeadband_Start) {
        rtb_Upperrigidreaction = 0.0;
      } else {
        rtb_Upperrigidreaction = acf_rl_native_B.Distancefrommidstroke -
          acf_rl_native_P.Recenterdeadband_Start;
      }

      /* Gain: '<S12>/Recenter gain' incorporates:
       *  DeadZone: '<S12>/Recenter deadband'
       */
      acf_rl_native_B.Recentergain = acf_rl_native_P.Recentergain_Gain *
        rtb_Upperrigidreaction;

      /* MinMax: '<S12>/RL positive recenter envelope' incorporates:
       *  Constant: '<S12>/Hold recentering'
       */
      rtb_Resetonanyinstability = fmax(acf_rl_native_B.Recentergain,
        acf_rl_native_P.Holdrecentering_Value);
    }

    /* ZeroOrderHold: '<S12>/RL fresh 100 Hz robot request' */
    if (tmp && tmp_0) {
      /* ZeroOrderHold: '<S12>/RL fresh 100 Hz robot request' incorporates:
       *  Inport: '<Root>/rl_robot_correction_m_s'
       */
      acf_rl_native_B.RLfresh100Hzrobotrequest =
        acf_rl_native_U.rl_robot_correction_m_s;
    }

    if (tmp_0) {
      /* Switch: '<S40>/Switch2' incorporates:
       *  RelationalOperator: '<S40>/LowerRelop1'
       */
      if (acf_rl_native_B.RLfresh100Hzrobotrequest > rtb_Resetonanyinstability)
      {
        /* Switch: '<S40>/Switch2' */
        acf_rl_native_B.Switch2 = rtb_Resetonanyinstability;
      } else {
        /* MinMax: '<S12>/RL negative recenter envelope' incorporates:
         *  Constant: '<S12>/Hold recentering'
         */
        rtb_Taredaxialcellforce = fmin(acf_rl_native_B.Recentergain,
          acf_rl_native_P.Holdrecentering_Value);

        /* Switch: '<S40>/Switch' incorporates:
         *  RelationalOperator: '<S40>/UpperRelop'
         */
        if (acf_rl_native_B.RLfresh100Hzrobotrequest < rtb_Taredaxialcellforce)
        {
          /* Switch: '<S40>/Switch2' */
          acf_rl_native_B.Switch2 = rtb_Taredaxialcellforce;
        } else {
          /* Switch: '<S40>/Switch2' */
          acf_rl_native_B.Switch2 = acf_rl_native_B.RLfresh100Hzrobotrequest;
        }

        /* End of Switch: '<S40>/Switch' */
      }

      /* End of Switch: '<S40>/Switch2' */
    }

    if (tmp) {
      /* UnitDelay: '<S41>/Previous fast stable count' */
      rtb_Previousfaststablecount =
        acf_rl_native_DW.Previousfaststablecount_DSTATE;

      /* Switch: '<S12>/Gate contact or cell error' */
      acf_rl_native_B.Gatecontactorcellerror = rtb_Sendcorrectiononlywhensettl;

      /* ZeroOrderHold: '<S39>/Robot-rate force sample' */
      if (tmp_0) {
        /* ZeroOrderHold: '<S39>/Robot-rate force sample' */
        acf_rl_native_B.Robotrateforcesample =
          acf_rl_native_B.Gatecontactorcellerror;
      }
    }

    if (tmp_0) {
      /* DiscreteTransferFcn: '<S39>/Force trend filter' */
      rtb_Forcetrendfilter = acf_rl_native_P.Forcetrendfilter_NumCoef *
        acf_rl_native_DW.Forcetrendfilter_states;

      /* Logic: '<S39>/Force stable now' incorporates:
       *  Abs: '<S39>/Absolute force rate'
       *  Abs: '<S39>/Absolute sampled error'
       *  Constant: '<S39>/Force band'
       *  Constant: '<S39>/Force rate limit'
       *  Gain: '<S39>/Change per second'
       *  RelationalOperator: '<S39>/Changing slowly'
       *  RelationalOperator: '<S39>/Near target'
       *  Sum: '<S39>/Force change'
       *  UnitDelay: '<S39>/Previous force trend'
       */
      acf_rl_native_B.Forcestablenow = ((fabs
        (acf_rl_native_B.Robotrateforcesample) <=
        acf_rl_native_P.Forceband_Value) && (fabs((rtb_Forcetrendfilter -
        acf_rl_native_DW.Previousforcetrend_DSTATE) *
        acf_rl_native_P.Changepersecond_Gain) <=
        acf_rl_native_P.Forceratelimit_Value));
    }

    if (tmp) {
      /* Gain: '<S41>/Normal force above target' */
      rtb_Resetonanyinstability = acf_rl_native_P.Normalforceabovetarget_Gain *
        acf_rl_native_B.Gatecontactorcellerror;

      /* DiscreteTransferFcn: '<S41>/Fast normal force mean' */
      acf_rl_native_DW.Fastnormalforcemean_tmp = (rtb_Resetonanyinstability -
        acf_rl_native_P.Fastnormalforcemean_DenCoef[1] *
        acf_rl_native_DW.Fastnormalforcemean_states) /
        acf_rl_native_P.Fastnormalforcemean_DenCoef[0];

      /* Sum: '<S41>/Normal force deviation' incorporates:
       *  DiscreteTransferFcn: '<S41>/Fast normal force mean'
       */
      rtb_Sendcorrectiononlywhensettl = rtb_Resetonanyinstability -
        (acf_rl_native_P.Fastnormalforcemean_NumCoef[0] *
         acf_rl_native_DW.Fastnormalforcemean_tmp +
         acf_rl_native_P.Fastnormalforcemean_NumCoef[1] *
         acf_rl_native_DW.Fastnormalforcemean_states);

      /* DiscreteTransferFcn: '<S41>/Fast force variance' incorporates:
       *  Math: '<S41>/Squared normal force deviation'
       */
      acf_rl_native_DW.Fastforcevariance_tmp = (rtb_Sendcorrectiononlywhensettl *
        rtb_Sendcorrectiononlywhensettl -
        acf_rl_native_P.Fastforcevariance_DenCoef[1] *
        acf_rl_native_DW.Fastforcevariance_states) /
        acf_rl_native_P.Fastforcevariance_DenCoef[0];

      /* Switch: '<S41>/Reset dwell at every fast instability' incorporates:
       *  Bias: '<S41>/One more fast stable sample'
       *  Constant: '<S41>/Allowed force variance'
       *  Constant: '<S41>/Reset fast dwell'
       *  DiscreteTransferFcn: '<S41>/Fast force variance'
       *  Logic: '<S41>/All force stability conditions'
       *  RelationalOperator: '<S41>/Variation acceptable'
       *  ZeroOrderHold: '<S41>/Held slow stability at fast rate'
       */
      if (acf_rl_native_B.Forcestablenow &&
          (acf_rl_native_P.Fastforcevariance_NumCoef[0] *
           acf_rl_native_DW.Fastforcevariance_tmp +
           acf_rl_native_P.Fastforcevariance_NumCoef[1] *
           acf_rl_native_DW.Fastforcevariance_states <=
           acf_rl_native_P.Allowedforcevariance_Value)) {
        rtb_Sendcorrectiononlywhensettl = rtb_Previousfaststablecount +
          acf_rl_native_P.Onemorefaststablesample_Bias;
      } else {
        rtb_Sendcorrectiononlywhensettl = acf_rl_native_P.Resetfastdwell_Value;
      }

      /* End of Switch: '<S41>/Reset dwell at every fast instability' */

      /* Saturate: '<S41>/Cap fast stable count' */
      if (rtb_Sendcorrectiononlywhensettl >
          acf_rl_native_P.Capfaststablecount_UpperSat) {
        /* Saturate: '<S41>/Cap fast stable count' */
        rtb_Capfaststablecount = acf_rl_native_P.Capfaststablecount_UpperSat;
      } else if (rtb_Sendcorrectiononlywhensettl <
                 acf_rl_native_P.Capfaststablecount_LowerSat) {
        /* Saturate: '<S41>/Cap fast stable count' */
        rtb_Capfaststablecount = acf_rl_native_P.Capfaststablecount_LowerSat;
      } else {
        /* Saturate: '<S41>/Cap fast stable count' */
        rtb_Capfaststablecount = rtb_Sendcorrectiononlywhensettl;
      }

      /* End of Saturate: '<S41>/Cap fast stable count' */

      /* RelationalOperator: '<S41>/Fast continuous dwell completed' incorporates:
       *  Constant: '<S41>/Required fast dwell count'
       */
      acf_rl_native_B.Fastcontinuousdwellcompleted = (rtb_Capfaststablecount >=
        acf_rl_native_P.Requiredfastdwellcount_Value);

      /* ZeroOrderHold: '<S39>/Fast ready at robot rate' */
      if (tmp_0) {
        /* ZeroOrderHold: '<S39>/Fast ready at robot rate' */
        acf_rl_native_B.Fastreadyatrobotrate =
          acf_rl_native_B.Fastcontinuousdwellcompleted;
      }
    }

    if (tmp_0) {
      /* Switch: '<S39>/Reset on any instability' incorporates:
       *  Bias: '<S39>/One more stable sample'
       *  Constant: '<S39>/Reset count'
       *  UnitDelay: '<S39>/Previous stable count'
       */
      if (acf_rl_native_B.Forcestablenow) {
        rtb_Resetonanyinstability = acf_rl_native_DW.Previousstablecount_DSTATE
          + acf_rl_native_P.Onemorestablesample_Bias;
      } else {
        rtb_Resetonanyinstability = acf_rl_native_P.Resetcount_Value;
      }

      /* End of Switch: '<S39>/Reset on any instability' */

      /* Saturate: '<S39>/Cap stable count' */
      if (rtb_Resetonanyinstability > acf_rl_native_P.Capstablecount_UpperSat) {
        /* Saturate: '<S39>/Cap stable count' */
        rtb_Capstablecount = acf_rl_native_P.Capstablecount_UpperSat;
      } else if (rtb_Resetonanyinstability <
                 acf_rl_native_P.Capstablecount_LowerSat) {
        /* Saturate: '<S39>/Cap stable count' */
        rtb_Capstablecount = acf_rl_native_P.Capstablecount_LowerSat;
      } else {
        /* Saturate: '<S39>/Cap stable count' */
        rtb_Capstablecount = rtb_Resetonanyinstability;
      }

      /* End of Saturate: '<S39>/Cap stable count' */

      /* Switch: '<S39>/Select continuous fast dwell' incorporates:
       *  Constant: '<S39>/Dwell sample count'
       *  Constant: '<S39>/Fast variation gate enabled'
       *  RelationalOperator: '<S39>/Dwell completed'
       */
      if (acf_rl_native_P.Fastvariationgateenabled_Value >=
          acf_rl_native_P.Selectcontinuousfastdwell_Thres) {
        rtb_Selectcontinuousfastdwell = acf_rl_native_B.Fastreadyatrobotrate;
      } else {
        rtb_Selectcontinuousfastdwell = (rtb_Capstablecount >=
          acf_rl_native_P.Dwellsamplecount_Value);
      }

      /* Logic: '<S39>/Ready or bypassed' incorporates:
       *  Constant: '<S39>/Gate disabled'
       *  Switch: '<S39>/Select continuous fast dwell'
       */
      acf_rl_native_B.Readyorbypassed = ((acf_rl_native_P.Gatedisabled_Value !=
        0.0) || rtb_Selectcontinuousfastdwell);
    }

    if (tmp) {
      /* Switch: '<S12>/Send correction only when settled' incorporates:
       *  Constant: '<S12>/Hold recentering'
       *  Switch: '<S12>/Baseline or external robot'
       */
      if (acf_rl_native_B.Readyorbypassed) {
        /* Switch: '<S12>/Baseline or external robot' incorporates:
         *  Inport: '<Root>/rl_baseline_mode'
         */
        if (acf_rl_native_U.rl_baseline_mode >
            acf_rl_native_P.Baselineorexternalrobot_Thresho) {
          rtb_Sendcorrectiononlywhensettl = acf_rl_native_B.Recentergain;
        } else {
          rtb_Sendcorrectiononlywhensettl = acf_rl_native_B.Switch2;
        }
      } else {
        rtb_Sendcorrectiononlywhensettl = acf_rl_native_P.Holdrecentering_Value;
      }

      /* End of Switch: '<S12>/Send correction only when settled' */

      /* Sum: '<S12>/Robot velocity request' */
      rtb_Upperrigidreaction = acf_rl_native_B.Nominalpathsampling +
        rtb_Sendcorrectiononlywhensettl;

      /* Saturate: '<S12>/Robot speed limits' */
      if (rtb_Upperrigidreaction > acf_rl_native_P.Robotspeedlimits_UpperSat) {
        /* Saturate: '<S12>/Robot speed limits' */
        acf_rl_native_B.Robotspeedlimits =
          acf_rl_native_P.Robotspeedlimits_UpperSat;
      } else if (rtb_Upperrigidreaction <
                 acf_rl_native_P.Robotspeedlimits_LowerSat) {
        /* Saturate: '<S12>/Robot speed limits' */
        acf_rl_native_B.Robotspeedlimits =
          acf_rl_native_P.Robotspeedlimits_LowerSat;
      } else {
        /* Saturate: '<S12>/Robot speed limits' */
        acf_rl_native_B.Robotspeedlimits = rtb_Upperrigidreaction;
      }

      /* End of Saturate: '<S12>/Robot speed limits' */
    }

    if (tmp_0) {
      /* DataTypeConversion: '<S10>/Double 17' */
      acf_rl_native_B.Double17 = acf_rl_native_B.Readyorbypassed;
    }

    if (tmp) {
      /* Outport: '<Root>/rl_observation' incorporates:
       *  ZeroOrderHold: '<S10>/Fresh observation 17'
       *  ZeroOrderHold: '<S10>/Fresh observation 18'
       */
      acf_rl_native_Y.rl_observation[0] = rtb_Measurednormalforce;
      acf_rl_native_Y.rl_observation[1] = rtb_Compensatedaxialreaction;
      acf_rl_native_Y.rl_observation[2] = rtb_Compensatedlateralreaction;
      acf_rl_native_Y.rl_observation[3] = rtb_Zfresh10kHzaccelerationfram;
      acf_rl_native_Y.rl_observation[4] = rtb_Xfresh10kHzaccelerationfram;
      acf_rl_native_Y.rl_observation[5] =
        acf_rl_native_B.Fresh10kHzpositionframe;
      acf_rl_native_Y.rl_observation[6] = rtb_Headgapandvelocityobserver[1];
      acf_rl_native_Y.rl_observation[7] = acf_rl_native_B.Estimatedheadtravel;
      acf_rl_native_Y.rl_observation[8] = rtb_Causalsurfacedirection;
      acf_rl_native_Y.rl_observation[9] =
        acf_rl_native_B.Predictedvelocitysampling;
      acf_rl_native_Y.rl_observation[10] =
        acf_rl_native_B.Enablerobotaccelerationpredicti;
      acf_rl_native_Y.rl_observation[11] =
        acf_rl_native_B.Fresh10kHzrobotvelocityframe;
      acf_rl_native_Y.rl_observation[12] =
        acf_rl_native_B.Fresh10kHzrobotaccelerationfram;
      acf_rl_native_Y.rl_observation[13] = rtb_Currentlimits;
      acf_rl_native_Y.rl_observation[14] = acf_rl_native_B.Robotspeedlimits;
      acf_rl_native_Y.rl_observation[15] = rtb_Sendcorrectiononlywhensettl;
      acf_rl_native_Y.rl_observation[16] = acf_rl_native_B.Double17;
      acf_rl_native_Y.rl_observation[17] = acf_rl_native_B.Distancefrommidstroke;
    }

    /* Switch: '<S17>/Surface shape 2' incorporates:
     *  Constant: '<S17>/Ramp selected'
     *  Switch: '<S17>/Surface shape 3'
     */
    if (acf_rl_native_P.Rampselected_Value != 0.0) {
      /* Polyval: '<S17>/Ramp component 2' */
      rtb_Taredaxialcellforce = (((acf_rl_native_P.Rampcomponent2_Coefs[0] *
        rtb_Positionwithinevent + acf_rl_native_P.Rampcomponent2_Coefs[1]) *
        rtb_Positionwithinevent + acf_rl_native_P.Rampcomponent2_Coefs[2]) *
        rtb_Positionwithinevent + acf_rl_native_P.Rampcomponent2_Coefs[3]) *
        rtb_Positionwithinevent + acf_rl_native_P.Rampcomponent2_Coefs[4];

      /* Polyval: '<S17>/Ramp component 3' */
      rtb_Sendcorrectiononlywhensettl = ((acf_rl_native_P.Rampcomponent3_Coefs[0]
        * rtb_Positionwithinevent + acf_rl_native_P.Rampcomponent3_Coefs[1]) *
        rtb_Positionwithinevent + acf_rl_native_P.Rampcomponent3_Coefs[2]) *
        rtb_Positionwithinevent + acf_rl_native_P.Rampcomponent3_Coefs[3];
    } else {
      /* Polyval: '<S17>/Bump component 2' */
      rtb_Taredaxialcellforce = acf_rl_native_P.Bumpcomponent2_Coefs[0];
      for (i = 0; i < 5; i++) {
        rtb_Taredaxialcellforce = rtb_Taredaxialcellforce *
          rtb_Positionwithinevent + acf_rl_native_P.Bumpcomponent2_Coefs[i + 1];
      }

      /* End of Polyval: '<S17>/Bump component 2' */

      /* Polyval: '<S17>/Bump component 3' */
      rtb_Sendcorrectiononlywhensettl = (((acf_rl_native_P.Bumpcomponent3_Coefs
        [0] * rtb_Positionwithinevent + acf_rl_native_P.Bumpcomponent3_Coefs[1])
        * rtb_Positionwithinevent + acf_rl_native_P.Bumpcomponent3_Coefs[2]) *
        rtb_Positionwithinevent + acf_rl_native_P.Bumpcomponent3_Coefs[3]) *
        rtb_Positionwithinevent + acf_rl_native_P.Bumpcomponent3_Coefs[4];
    }

    /* End of Switch: '<S17>/Surface shape 2' */

    /* Gain: '<S17>/Surface SI scale 2' incorporates:
     *  Product: '<S17>/Enable motion 2'
     */
    rtb_Taredaxialcellforce = rtb_Taredaxialcellforce * rtb_Normalizer *
      acf_rl_native_P.SurfaceSIscale2_Gain;

    /* Integrator: '<S22>/Velocity' */
    acf_rl_native_B.Velocity = acf_rl_native_X.Velocity_CSTATE;

    /* Sum: '<S20>/Spring and damper force' incorporates:
     *  Constant: '<S20>/Preload'
     *  Gain: '<S20>/Damping'
     *  Gain: '<S20>/Stiffness'
     *  Integrator: '<S22>/Position'
     *  Sum: '<S20>/Endpoint difference'
     */
    rtb_Springanddamperforce = ((rtb_Causalaccelerationforcurren -
      acf_rl_native_X.Position_CSTATE_p) * acf_rl_native_P.Stiffness_Gain +
      acf_rl_native_P.Preload_Value) + (rtb_Taredaxialcellforce -
      acf_rl_native_B.Velocity) * acf_rl_native_P.Damping_Gain;

    /* Gain: '<S17>/Surface SI scale 3' incorporates:
     *  Product: '<S17>/Enable motion 3'
     */
    rtb_Sendcorrectiononlywhensettl = rtb_Sendcorrectiononlywhensettl *
      rtb_Normalizer * acf_rl_native_P.SurfaceSIscale3_Gain;

    /* Sum: '<S23>/Rigid tool contact reaction' incorporates:
     *  Constant: '<S23>/Gravity along flange axis'
     *  Gain: '<S23>/Tool mass'
     *  Sum: '<S23>/Surface acceleration and gravity'
     */
    rtb_Normalizer = (rtb_Sendcorrectiononlywhensettl +
                      acf_rl_native_P.Gravityalongflangeaxis_Value) *
      acf_rl_native_P.Toolmass_Gain + rtb_Springanddamperforce;

    /* Saturate: '<S23>/Unilateral contact force' */
    if (rtb_Normalizer > acf_rl_native_P.Unilateralcontactforce_UpperSat) {
      rtb_Compensatedaxialreaction =
        acf_rl_native_P.Unilateralcontactforce_UpperSat;
    } else if (rtb_Normalizer < acf_rl_native_P.Unilateralcontactforce_LowerSat)
    {
      rtb_Compensatedaxialreaction =
        acf_rl_native_P.Unilateralcontactforce_LowerSat;
    } else {
      rtb_Compensatedaxialreaction = rtb_Normalizer;
    }

    /* End of Saturate: '<S23>/Unilateral contact force' */

    /* Gain: '<S18>/Surface slope' */
    rtb_Xfresh10kHzaccelerationfram = acf_rl_native_P.Surfaceslope_Gain *
      rtb_Taredaxialcellforce;

    /* Bias: '<S18>/Squared slope plus one' incorporates:
     *  Math: '<S18>/Squared slope'
     */
    rtb_Compensatedlateralreaction = rtb_Xfresh10kHzaccelerationfram *
      rtb_Xfresh10kHzaccelerationfram +
      acf_rl_native_P.Squaredslopeplusone_Bias_i;

    /* Math: '<S18>/Normalizer'
     *
     * About '<S18>/Normalizer':
     *  Operator: sqrt
     */
    if (rtb_Compensatedlateralreaction < 0.0) {
      rtb_Compensatedlateralreaction = -sqrt(fabs(rtb_Compensatedlateralreaction));
    } else {
      rtb_Compensatedlateralreaction = sqrt(rtb_Compensatedlateralreaction);
    }

    /* End of Math: '<S18>/Normalizer' */

    /* Product: '<S18>/Normal reaction' incorporates:
     *  Constant: '<S18>/Unity'
     *  Gain: '<S18>/Friction times slope'
     *  Sum: '<S18>/Axial reaction factor'
     */
    rtb_Causalsurfacedirection = rtb_Compensatedaxialreaction *
      rtb_Compensatedlateralreaction / (acf_rl_native_P.Unity_Value -
      acf_rl_native_P.Frictiontimesslope_Gain * rtb_Xfresh10kHzaccelerationfram);

    /* Integrator: '<S21>/Velocity' */
    acf_rl_native_B.Velocity_j = acf_rl_native_X.Velocity_CSTATE_m;

    /* Sum: '<S23>/Gap speed' */
    rtb_Positionwithinevent = acf_rl_native_B.Velocity_j -
      acf_rl_native_B.Velocity;

    /* Sum: '<S25>/Force transmitted to the bodies' incorporates:
     *  Gain: '<S25>/Coil force constant'
     *  Gain: '<S25>/Coulomb friction'
     *  Gain: '<S25>/Friction speed scale'
     *  Gain: '<S25>/Viscous friction'
     *  Integrator: '<S25>/Winding current'
     *  Sum: '<S25>/Coulomb and viscous friction'
     *  Sum: '<S2>/A minus B position velocity'
     *  Trigonometry: '<S25>/Smooth friction'
     */
    rtb_Forcetransmittedtothebodies = acf_rl_native_P.Coilforceconstant_Gain *
      acf_rl_native_X.Windingcurrent_CSTATE - (tanh
      (acf_rl_native_P.Frictionspeedscale_Gain * rtb_Positionwithinevent) *
      acf_rl_native_P.Coulombfriction_Gain +
      acf_rl_native_P.Viscousfriction_Gain * rtb_Positionwithinevent);
    if (tmp) {
      /* Delay: '<S14>/Robot command samples' incorporates:
       *  Constant: '<S14>/RL reset-fixed queue length'
       */
      if ((acf_rl_native_P.RLresetfixedqueuelength_Value_n < 1.0) || rtIsNaN
          (acf_rl_native_P.RLresetfixedqueuelength_Value_n)) {
        delayLen = 1U;
      } else if (acf_rl_native_P.RLresetfixedqueuelength_Value_n > 1000.0) {
        delayLen = 1000U;
      } else {
        rtb_Upperrigidreaction = trunc
          (acf_rl_native_P.RLresetfixedqueuelength_Value_n);
        if (rtIsNaN(rtb_Upperrigidreaction) || rtIsInf(rtb_Upperrigidreaction))
        {
          rtb_Upperrigidreaction = 0.0;
        } else {
          rtb_Upperrigidreaction = fmod(rtb_Upperrigidreaction, 4.294967296E+9);
        }

        if (rtb_Upperrigidreaction < 0.0) {
          delayLen = (uint32_T)-(int32_T)(uint32_T)-rtb_Upperrigidreaction;
        } else {
          delayLen = (uint32_T)rtb_Upperrigidreaction;
        }
      }

      if (delayLen <= acf_rl_native_DW.CircBufIdx_e) {
        delayLen = acf_rl_native_DW.CircBufIdx_e - delayLen;
      } else {
        delayLen = (acf_rl_native_DW.CircBufIdx_e - delayLen) + 1000U;
      }

      rtb_Upperrigidreaction =
        acf_rl_native_DW.Robotcommandsamples_DSTATE[delayLen];

      /* End of Delay: '<S14>/Robot command samples' */

      /* Saturate: '<S15>/Requested robot speed' */
      if (rtb_Upperrigidreaction > acf_rl_native_P.Requestedrobotspeed_UpperSat)
      {
        /* Saturate: '<S15>/Requested robot speed' */
        acf_rl_native_B.Requestedrobotspeed =
          acf_rl_native_P.Requestedrobotspeed_UpperSat;
      } else if (rtb_Upperrigidreaction <
                 acf_rl_native_P.Requestedrobotspeed_LowerSat) {
        /* Saturate: '<S15>/Requested robot speed' */
        acf_rl_native_B.Requestedrobotspeed =
          acf_rl_native_P.Requestedrobotspeed_LowerSat;
      } else {
        /* Saturate: '<S15>/Requested robot speed' */
        acf_rl_native_B.Requestedrobotspeed = rtb_Upperrigidreaction;
      }

      /* End of Saturate: '<S15>/Requested robot speed' */
    }

    /* Integrator: '<S15>/Robot velocity' */
    acf_rl_native_B.Robotvelocity = acf_rl_native_X.Robotvelocity_CSTATE;

    /* Gain: '<S15>/Velocity response' incorporates:
     *  Sum: '<S15>/Robot speed error'
     */
    rtb_Upperrigidreaction = (acf_rl_native_B.Requestedrobotspeed -
      acf_rl_native_B.Robotvelocity) * acf_rl_native_P.Velocityresponse_Gain;

    /* Saturate: '<S15>/Robot acceleration limits' */
    if (rtb_Upperrigidreaction > acf_rl_native_P.Robotaccelerationlimits_UpperSa)
    {
      /* Saturate: '<S15>/Robot acceleration limits' */
      acf_rl_native_B.Robotaccelerationlimits =
        acf_rl_native_P.Robotaccelerationlimits_UpperSa;
    } else if (rtb_Upperrigidreaction <
               acf_rl_native_P.Robotaccelerationlimits_LowerSa) {
      /* Saturate: '<S15>/Robot acceleration limits' */
      acf_rl_native_B.Robotaccelerationlimits =
        acf_rl_native_P.Robotaccelerationlimits_LowerSa;
    } else {
      /* Saturate: '<S15>/Robot acceleration limits' */
      acf_rl_native_B.Robotaccelerationlimits = rtb_Upperrigidreaction;
    }

    /* End of Saturate: '<S15>/Robot acceleration limits' */

    /* Sum: '<S2>/Upper rigid reaction' incorporates:
     *  Constant: '<S2>/Gravity at upper rigid attachment'
     *  Gain: '<S2>/Upper attachment inertia'
     *  Sum: '<S2>/Upper imposed acceleration plus gravity'
     */
    rtb_Upperrigidreaction = rtb_Forcetransmittedtothebodies -
      (acf_rl_native_B.Robotaccelerationlimits +
       acf_rl_native_P.Gravityatupperrigidattachment_V) *
      acf_rl_native_P.Upperattachmentinertia_Gain;

    /* Sum: '<S21>/Newton acceleration' incorporates:
     *  Constant: '<S21>/Gravity'
     *  Gain: '<S21>/Inverse mass'
     *  Sum: '<S21>/Net applied force'
     */
    acf_rl_native_B.Newtonacceleration = (rtb_Forcetransmittedtothebodies -
      rtb_Upperrigidreaction) * acf_rl_native_P.Inversemass_Gain -
      acf_rl_native_P.Gravity_Value;

    /* Sum: '<S22>/Newton acceleration' incorporates:
     *  Constant: '<S22>/Gravity'
     *  Gain: '<S22>/Inverse mass'
     *  Sum: '<S22>/Net applied force'
     */
    acf_rl_native_B.Newtonacceleration_n = (rtb_Springanddamperforce -
      rtb_Forcetransmittedtothebodies) * acf_rl_native_P.Inversemass_Gain_h -
      acf_rl_native_P.Gravity_Value_i;

    /* Gain: '<S25>/Resistance' incorporates:
     *  Integrator: '<S25>/Winding current'
     */
    rtb_Measurednormalforce = acf_rl_native_P.Resistance_Gain *
      acf_rl_native_X.Windingcurrent_CSTATE;

    /* Gain: '<S25>/Back EMF' incorporates:
     *  Sum: '<S2>/A minus B position velocity'
     */
    rtb_Previousfaststablecount = acf_rl_native_P.BackEMF_Gain *
      rtb_Positionwithinevent;
    if (tmp) {
      /* Saturate: '<S25>/Current request limit' */
      if (rtb_Currentlimits > acf_rl_native_P.Currentrequestlimit_UpperSat) {
        /* Saturate: '<S25>/Current request limit' */
        acf_rl_native_B.Currentrequestlimit =
          acf_rl_native_P.Currentrequestlimit_UpperSat;
      } else if (rtb_Currentlimits <
                 acf_rl_native_P.Currentrequestlimit_LowerSat) {
        /* Saturate: '<S25>/Current request limit' */
        acf_rl_native_B.Currentrequestlimit =
          acf_rl_native_P.Currentrequestlimit_LowerSat;
      } else {
        /* Saturate: '<S25>/Current request limit' */
        acf_rl_native_B.Currentrequestlimit = rtb_Currentlimits;
      }

      /* End of Saturate: '<S25>/Current request limit' */
    }

    /* Sum: '<S25>/Requested voltage' incorporates:
     *  Gain: '<S25>/Current regulator'
     *  Integrator: '<S25>/Winding current'
     *  Sum: '<S25>/Current error'
     */
    rtb_Voltagelimitflag = (acf_rl_native_B.Currentrequestlimit -
      acf_rl_native_X.Windingcurrent_CSTATE) *
      acf_rl_native_P.Currentregulator_Gain + (rtb_Measurednormalforce +
      rtb_Previousfaststablecount);

    /* Saturate: '<S25>/Supply voltage limits' */
    if (rtb_Voltagelimitflag > acf_rl_native_P.Supplyvoltagelimits_UpperSat) {
      rtb_Resetonanyinstability = acf_rl_native_P.Supplyvoltagelimits_UpperSat;
    } else if (rtb_Voltagelimitflag <
               acf_rl_native_P.Supplyvoltagelimits_LowerSat) {
      rtb_Resetonanyinstability = acf_rl_native_P.Supplyvoltagelimits_LowerSat;
    } else {
      rtb_Resetonanyinstability = rtb_Voltagelimitflag;
    }

    /* End of Saturate: '<S25>/Supply voltage limits' */

    /* RelationalOperator: '<S23>/Contact maintained' incorporates:
     *  Constant: '<S23>/Zero'
     */
    rtb_Selectcontinuousfastdwell = (rtb_Normalizer >=
      acf_rl_native_P.Zero_Value_d);

    /* Logic: '<S23>/Travel stop reached' incorporates:
     *  Constant: '<S23>/Maximum travel'
     *  Constant: '<S23>/Minimum travel'
     *  RelationalOperator: '<S23>/Lower travel stop'
     *  RelationalOperator: '<S23>/Upper travel stop'
     */
    rtb_Travelstopreached = ((rtb_Actualactuatortravel <=
      acf_rl_native_P.Minimumtravel_Value) || (rtb_Actualactuatortravel >=
      acf_rl_native_P.Maximumtravel_Value));
    if (tmp) {
      /* DataTypeConversion: '<S25>/Current limit flag' incorporates:
       *  Abs: '<S25>/Command magnitude'
       *  Constant: '<S25>/Peak current threshold'
       *  RelationalOperator: '<S25>/Current at limit'
       */
      acf_rl_native_B.Currentlimitflag = (fabs(rtb_Currentlimits) >=
        acf_rl_native_P.Peakcurrentthreshold_Value);

      /* Outport: '<Root>/rl_comparator' incorporates:
       *  Constant: '<S33>/Flat holding current'
       */
      acf_rl_native_Y.rl_comparator[0] = rtb_PIPIDandfeedforwardcurrent;
      acf_rl_native_Y.rl_comparator[1] = acf_rl_native_B.Recentergain;
      acf_rl_native_Y.rl_comparator[2] =
        acf_rl_native_P.Flatholdingcurrent_Value;
    }

    /* Outport: '<Root>/rl_scoring' incorporates:
     *  Abs: '<S25>/Requested voltage magnitude'
     *  Constant: '<S23>/Zero'
     *  Constant: '<S25>/Supply voltage'
     *  Constant: '<S25>/Zero'
     *  DataTypeConversion: '<S11>/Double 2'
     *  DataTypeConversion: '<S23>/Contact flag'
     *  DataTypeConversion: '<S23>/Stroke stop flag'
     *  DataTypeConversion: '<S25>/Voltage limit flag'
     *  Gain: '<S23>/Equivalent screw angular speed'
     *  Gain: '<S25>/Inverse reduced mass'
     *  Gain: '<S2>/Lower spring load share'
     *  Gain: '<S2>/Upper spring load share'
     *  Integrator: '<S21>/Position'
     *  Integrator: '<S22>/Position'
     *  Integrator: '<S25>/Winding current'
     *  RelationalOperator: '<S25>/Voltage clipped'
     *  Sum: '<S25>/Net relative force'
     *  Sum: '<S2>/Spring load on actuator'
     */
    acf_rl_native_Y.rl_scoring[0] = rtb_Causalsurfacedirection;
    acf_rl_native_Y.rl_scoring[1] = rtb_Compensatedaxialreaction;
    acf_rl_native_Y.rl_scoring[2] = rtb_Actualactuatortravel;
    acf_rl_native_Y.rl_scoring[3] = rtb_Positionwithinevent;
    acf_rl_native_Y.rl_scoring[4] = (rtb_Forcetransmittedtothebodies -
      (acf_rl_native_P.Upperspringloadshare_Gain * rtb_Upperrigidreaction +
       acf_rl_native_P.Lowerspringloadshare_Gain * rtb_Springanddamperforce)) *
      acf_rl_native_P.Inversereducedmass_Gain;
    acf_rl_native_Y.rl_scoring[5] = acf_rl_native_X.Position_CSTATE;
    acf_rl_native_Y.rl_scoring[6] = acf_rl_native_X.Position_CSTATE_p;
    acf_rl_native_Y.rl_scoring[7] = rtb_Causalaccelerationforcurren;
    acf_rl_native_Y.rl_scoring[8] = acf_rl_native_B.Velocity_j;
    acf_rl_native_Y.rl_scoring[9] = acf_rl_native_B.Velocity;
    acf_rl_native_Y.rl_scoring[10] = rtb_Taredaxialcellforce;
    acf_rl_native_Y.rl_scoring[11] = acf_rl_native_B.Newtonacceleration;
    acf_rl_native_Y.rl_scoring[12] = acf_rl_native_B.Newtonacceleration_n;
    acf_rl_native_Y.rl_scoring[13] = rtb_Forcetransmittedtothebodies;
    acf_rl_native_Y.rl_scoring[14] = acf_rl_native_X.Windingcurrent_CSTATE;
    acf_rl_native_Y.rl_scoring[15] = rtb_Resetonanyinstability;
    acf_rl_native_Y.rl_scoring[16] = acf_rl_native_P.Zero_Value_i;
    acf_rl_native_Y.rl_scoring[17] =
      acf_rl_native_P.Equivalentscrewangularspeed_Gai * rtb_Positionwithinevent;
    acf_rl_native_Y.rl_scoring[18] = rtb_Selectcontinuousfastdwell;
    acf_rl_native_Y.rl_scoring[19] = acf_rl_native_P.Zero_Value_d;
    acf_rl_native_Y.rl_scoring[20] = acf_rl_native_P.Zero_Value_d;
    acf_rl_native_Y.rl_scoring[21] = rtb_Travelstopreached;
    acf_rl_native_Y.rl_scoring[22] = acf_rl_native_B.Currentlimitflag;
    acf_rl_native_Y.rl_scoring[23] = (fabs(rtb_Voltagelimitflag) >
      acf_rl_native_P.Supplyvoltage_Value);
    acf_rl_native_Y.rl_scoring[24] = rtb_Springanddamperforce;
    acf_rl_native_Y.rl_scoring[25] = rtb_Upperrigidreaction;
    acf_rl_native_Y.rl_scoring[26] = rtb_Xfresh10kHzaccelerationfram;
    if (tmp_1) {
      /* UniformRandomNumber: '<S8>/Independent lateral noise' incorporates:
       *  Constant: '<S16>/Physical S acceleration X'
       *  Constant: '<S16>/X gravity'
       *  Sum: '<S16>/X specific acceleration'
       */
      rtb_Independentlateralnoise = acf_rl_native_P.PhysicalSaccelerationX_Value
        + acf_rl_native_P.Xgravity_Value;

      /* Gain: '<S16>/Acceleration X frame diagonal' */
      acf_rl_native_B.AccelerationXframediagonal =
        acf_rl_native_P.AccelerationXframediagonal_Gain *
        rtb_Independentlateralnoise;

      /* Gain: '<S16>/Acceleration Z frame cross axis' */
      acf_rl_native_B.AccelerationZframecrossaxis =
        acf_rl_native_P.AccelerationZframecrossaxis_Gai *
        rtb_Independentlateralnoise;
    }

    /* Sum: '<S16>/Z specific acceleration' incorporates:
     *  Constant: '<S16>/Z gravity'
     */
    rtb_PIPIDandfeedforwardcurrent = rtb_Sendcorrectiononlywhensettl +
      acf_rl_native_P.Zgravity_Value;

    /* StateSpace: '<S42>/Continuous analog front end' */
    acf_rl_native_B.Continuousanalogfrontend = 0.0;

    /* StateSpace: '<S42>/Continuous analog front end' */
    for (delayLen = acf_rl_native_P.Continuousanalogfrontend_C_jc[0U]; delayLen <
         acf_rl_native_P.Continuousanalogfrontend_C_jc[1U]; delayLen++) {
      /* StateSpace: '<S42>/Continuous analog front end' */
      acf_rl_native_B.Continuousanalogfrontend +=
        acf_rl_native_P.Continuousanalogfrontend_C_pr *
        acf_rl_native_X.Continuousanalogfrontend_CSTATE[0U];
    }

    for (delayLen = acf_rl_native_P.Continuousanalogfrontend_C_jc[1U]; delayLen <
         acf_rl_native_P.Continuousanalogfrontend_C_jc[2U]; delayLen++) {
      /* StateSpace: '<S42>/Continuous analog front end' */
      acf_rl_native_B.Continuousanalogfrontend +=
        acf_rl_native_P.Continuousanalogfrontend_C_pr *
        acf_rl_native_X.Continuousanalogfrontend_CSTATE[1U];
    }

    /* StateSpace: '<S42>/ADC analog input filter' */
    acf_rl_native_B.ADCanaloginputfilter = 0.0;

    /* StateSpace: '<S42>/ADC analog input filter' */
    for (delayLen = acf_rl_native_P.ADCanaloginputfilter_D_jc[0U]; delayLen <
         acf_rl_native_P.ADCanaloginputfilter_D_jc[1U]; delayLen++) {
      /* StateSpace: '<S42>/ADC analog input filter' */
      acf_rl_native_B.ADCanaloginputfilter +=
        acf_rl_native_P.ADCanaloginputfilter_D_pr *
        acf_rl_native_B.Continuousanalogfrontend;
    }

    if (tmp_1) {
      /* DiscreteTransferFcn: '<S42>/Native converter impulse response' */
      rtb_Taredaxialcellforce = acf_rl_native_B.ADCanaloginputfilter;
      for (i = 0; i < 30; i++) {
        rtb_Taredaxialcellforce -=
          acf_rl_native_P.Nativeconverterimpulserespons_b[i + 1] *
          acf_rl_native_DW.Nativeconverterimpulseresponse_[i];
      }

      acf_rl_native_DW.Nativeconverterimpulserespons_o = rtb_Taredaxialcellforce
        / acf_rl_native_P.Nativeconverterimpulserespons_b[0];
      rtb_Taredaxialcellforce = acf_rl_native_P.Nativeconverterimpulseresponse_
        [0] * acf_rl_native_DW.Nativeconverterimpulserespons_o;
      for (i = 0; i < 30; i++) {
        rtb_Taredaxialcellforce +=
          acf_rl_native_P.Nativeconverterimpulseresponse_[i + 1] *
          acf_rl_native_DW.Nativeconverterimpulseresponse_[i];
      }

      /* DiscreteTransferFcn: '<S42>/Native converter impulse response' */
      acf_rl_native_B.Nativeconverterimpulseresponse = rtb_Taredaxialcellforce;

      /* ZeroOrderHold: '<S42>/Actual ADC output samples' */
      if (rtmIsMajorTimeStep(acf_rl_native_M) &&
          acf_rl_native_M->Timing.TaskCounters.TID[2] == 0) {
        /* ZeroOrderHold: '<S42>/Actual ADC output samples' */
        acf_rl_native_B.ActualADCoutputsamples =
          acf_rl_native_B.Nativeconverterimpulseresponse;
      }

      /* End of ZeroOrderHold: '<S42>/Actual ADC output samples' */
    }

    /* ZeroOrderHold: '<S43>/Actual ADC output samples' incorporates:
     *  ZeroOrderHold: '<S29>/Actual ADC output samples'
     *  ZeroOrderHold: '<S30>/Actual ADC output samples'
     *  ZeroOrderHold: '<S31>/Actual ADC output samples'
     *  ZeroOrderHold: '<S37>/Actual ADC output samples'
     *  ZeroOrderHold: '<S38>/Actual ADC output samples'
     */
    tmp_0 = (rtmIsMajorTimeStep(acf_rl_native_M) &&
             acf_rl_native_M->Timing.TaskCounters.TID[2] == 0);
    if (tmp_0) {
      /* DiscreteTransferFcn: '<S42>/ADC output noise correlation' incorporates:
       *  Gain: '<S42>/Converter output noise RMS'
       *  RandomNumber: '<S42>/Independent converter output noise'
       */
      acf_rl_native_DW.ADCoutputnoisecorrelation_tmp =
        (acf_rl_native_P.ConverteroutputnoiseRMS_Gain *
         acf_rl_native_DW.NextOutput -
         acf_rl_native_P.ADCoutputnoisecorrelation_DenCo[1] *
         acf_rl_native_DW.ADCoutputnoisecorrelation_state) /
        acf_rl_native_P.ADCoutputnoisecorrelation_DenCo[0];

      /* Bias: '<S42>/Restore physical static offset' incorporates:
       *  DiscreteTransferFcn: '<S42>/ADC output noise correlation'
       *  Sum: '<S42>/ADC input and output noise'
       */
      rtb_Upperrigidreaction =
        ((acf_rl_native_P.ADCoutputnoisecorrelation_NumCo[0] *
          acf_rl_native_DW.ADCoutputnoisecorrelation_tmp +
          acf_rl_native_P.ADCoutputnoisecorrelation_NumCo[1] *
          acf_rl_native_DW.ADCoutputnoisecorrelation_state) +
         acf_rl_native_B.ActualADCoutputsamples) +
        acf_rl_native_P.Restorephysicalstaticoffset_Bia;

      /* Saturate: '<S42>/Actual ADC input range' */
      if (rtb_Upperrigidreaction > acf_rl_native_P.ActualADCinputrange_UpperSat)
      {
        rtb_Upperrigidreaction = acf_rl_native_P.ActualADCinputrange_UpperSat;
      } else if (rtb_Upperrigidreaction <
                 acf_rl_native_P.ActualADCinputrange_LowerSat) {
        rtb_Upperrigidreaction = acf_rl_native_P.ActualADCinputrange_LowerSat;
      }

      /* DiscreteTransferFcn: '<S42>/Fresh per-conversion EMA' incorporates:
       *  Bias: '<S42>/Known quantized static offset'
       *  Quantizer: '<S42>/Actual converter quantization'
       *  Saturate: '<S42>/Actual ADC input range'
       */
      acf_rl_native_DW.FreshperconversionEMA_tmp = ((rt_roundd_snf
        (rtb_Upperrigidreaction /
         acf_rl_native_P.Actualconverterquantization_Int) *
        acf_rl_native_P.Actualconverterquantization_Int +
        acf_rl_native_P.Knownquantizedstaticoffset_Bias) -
        acf_rl_native_P.FreshperconversionEMA_DenCoef[1] *
        acf_rl_native_DW.FreshperconversionEMA_states) /
        acf_rl_native_P.FreshperconversionEMA_DenCoef[0];

      /* DiscreteTransferFcn: '<S42>/Fresh per-conversion EMA' */
      acf_rl_native_B.FreshperconversionEMA =
        acf_rl_native_P.FreshperconversionEMA_NumCoef[0] *
        acf_rl_native_DW.FreshperconversionEMA_tmp +
        acf_rl_native_P.FreshperconversionEMA_NumCoef[1] *
        acf_rl_native_DW.FreshperconversionEMA_states;
    }

    if (tmp_1) {
      /* UniformRandomNumber: '<S8>/Independent lateral noise' incorporates:
       *  RandomNumber: '<S16>/X raw noise'
       */
      rtb_Independentlateralnoise = acf_rl_native_DW.NextOutput_b;

      /* Gain: '<S16>/X raw noise RMS' */
      acf_rl_native_B.XrawnoiseRMS = acf_rl_native_P.XrawnoiseRMS_Gain *
        rtb_Independentlateralnoise;

      /* Gain: '<S16>/X old sample weight' incorporates:
       *  UnitDelay: '<S16>/X filter memory'
       */
      acf_rl_native_B.Xoldsampleweight = acf_rl_native_P.Xoldsampleweight_Gain *
        acf_rl_native_DW.Xfiltermemory_DSTATE;
    }

    /* Sin: '<S16>/X analog vibration probe' incorporates:
     *  Sin: '<S16>/Z analog vibration probe'
     *  Sin: '<S5>/Force analog vibration probe'
     *  Sin: '<S8>/Lateral analog vibration probe'
     */
    rtb_Normalizer = acf_rl_native_M->Timing.t[0];

    /* Sum: '<S16>/X noisy sensor' incorporates:
     *  Gain: '<S16>/Acceleration X frame cross axis'
     *  Gain: '<S16>/X sensor scale'
     *  Sin: '<S16>/X analog vibration probe'
     *  Sum: '<S16>/Acceleration X frame calibrated sum'
     */
    acf_rl_native_B.Xnoisysensor =
      ((acf_rl_native_P.AccelerationXframecrossaxis_Gai *
        rtb_PIPIDandfeedforwardcurrent +
        acf_rl_native_B.AccelerationXframediagonal) *
       acf_rl_native_P.Xsensorscale_Gain + acf_rl_native_B.XrawnoiseRMS) + (sin
      (acf_rl_native_P.Xanalogvibrationprobe_Freq * rtb_Normalizer +
       acf_rl_native_P.Xanalogvibrationprobe_Phase) *
      acf_rl_native_P.Xanalogvibrationprobe_Amp +
      acf_rl_native_P.Xanalogvibrationprobe_Bias);

    /* Sum: '<S16>/X identified sensor response' incorporates:
     *  Gain: '<S16>/X new sample weight'
     */
    acf_rl_native_B.Xidentifiedsensorresponse =
      acf_rl_native_P.Xnewsampleweight_Gain * acf_rl_native_B.Xnoisysensor +
      acf_rl_native_B.Xoldsampleweight;

    /* Switch: '<S16>/X select identified sensor response' incorporates:
     *  Constant: '<S16>/X use engineered acquisition'
     */
    if (acf_rl_native_P.Xuseengineeredacquisition_Value >=
        acf_rl_native_P.Xselectidentifiedsensorresponse) {
      rtb_Upperrigidreaction = acf_rl_native_B.FreshperconversionEMA;
    } else {
      rtb_Upperrigidreaction = acf_rl_native_B.Xidentifiedsensorresponse;
    }

    /* Saturate: '<S16>/X acceleration range' incorporates:
     *  Switch: '<S16>/X select identified sensor response'
     */
    if (rtb_Upperrigidreaction > acf_rl_native_P.Xaccelerationrange_UpperSat) {
      rtb_Upperrigidreaction = acf_rl_native_P.Xaccelerationrange_UpperSat;
    } else if (rtb_Upperrigidreaction <
               acf_rl_native_P.Xaccelerationrange_LowerSat) {
      rtb_Upperrigidreaction = acf_rl_native_P.Xaccelerationrange_LowerSat;
    }

    /* Sum: '<S16>/X unknown drift' incorporates:
     *  Constant: '<S16>/X drift after tare'
     *  Constant: '<S16>/X known static tare'
     *  Quantizer: '<S16>/X ADC quantization'
     *  Saturate: '<S16>/X acceleration range'
     *  Sum: '<S16>/X tared acceleration'
     */
    acf_rl_native_B.Xunknowndrift = (rt_roundd_snf(rtb_Upperrigidreaction /
      acf_rl_native_P.XADCquantization_Interval) *
      acf_rl_native_P.XADCquantization_Interval -
      acf_rl_native_P.Xknownstatictare_Value) +
      acf_rl_native_P.Xdriftaftertare_Value;

    /* StateSpace: '<S43>/Continuous analog front end' */
    acf_rl_native_B.Continuousanalogfrontend_j = 0.0;

    /* StateSpace: '<S43>/Continuous analog front end' */
    for (delayLen = acf_rl_native_P.Continuousanalogfrontend_C_jc_p[0U];
         delayLen < acf_rl_native_P.Continuousanalogfrontend_C_jc_p[1U];
         delayLen++) {
      /* StateSpace: '<S43>/Continuous analog front end' */
      acf_rl_native_B.Continuousanalogfrontend_j +=
        acf_rl_native_P.Continuousanalogfrontend_C_pr_m *
        acf_rl_native_X.Continuousanalogfrontend_CSTA_f[0U];
    }

    for (delayLen = acf_rl_native_P.Continuousanalogfrontend_C_jc_p[1U];
         delayLen < acf_rl_native_P.Continuousanalogfrontend_C_jc_p[2U];
         delayLen++) {
      /* StateSpace: '<S43>/Continuous analog front end' */
      acf_rl_native_B.Continuousanalogfrontend_j +=
        acf_rl_native_P.Continuousanalogfrontend_C_pr_m *
        acf_rl_native_X.Continuousanalogfrontend_CSTA_f[1U];
    }

    /* StateSpace: '<S43>/ADC analog input filter' */
    acf_rl_native_B.ADCanaloginputfilter_g = 0.0;

    /* StateSpace: '<S43>/ADC analog input filter' */
    for (delayLen = acf_rl_native_P.ADCanaloginputfilter_D_jc_k[0U]; delayLen <
         acf_rl_native_P.ADCanaloginputfilter_D_jc_k[1U]; delayLen++) {
      /* StateSpace: '<S43>/ADC analog input filter' */
      acf_rl_native_B.ADCanaloginputfilter_g +=
        acf_rl_native_P.ADCanaloginputfilter_D_pr_g *
        acf_rl_native_B.Continuousanalogfrontend_j;
    }

    if (tmp_1) {
      /* DiscreteTransferFcn: '<S43>/Native converter impulse response' */
      rtb_Taredaxialcellforce = acf_rl_native_B.ADCanaloginputfilter_g;
      for (i = 0; i < 30; i++) {
        rtb_Taredaxialcellforce -=
          acf_rl_native_P.Nativeconverterimpulserespons_d[i + 1] *
          acf_rl_native_DW.Nativeconverterimpulserespons_a[i];
      }

      acf_rl_native_DW.Nativeconverterimpulserespon_gz = rtb_Taredaxialcellforce
        / acf_rl_native_P.Nativeconverterimpulserespons_d[0];
      rtb_Taredaxialcellforce = acf_rl_native_P.Nativeconverterimpulserespons_k
        [0] * acf_rl_native_DW.Nativeconverterimpulserespon_gz;
      for (i = 0; i < 30; i++) {
        rtb_Taredaxialcellforce +=
          acf_rl_native_P.Nativeconverterimpulserespons_k[i + 1] *
          acf_rl_native_DW.Nativeconverterimpulserespons_a[i];
      }

      /* DiscreteTransferFcn: '<S43>/Native converter impulse response' */
      acf_rl_native_B.Nativeconverterimpulserespons_j = rtb_Taredaxialcellforce;

      /* ZeroOrderHold: '<S43>/Actual ADC output samples' */
      if (tmp_0) {
        /* ZeroOrderHold: '<S43>/Actual ADC output samples' */
        acf_rl_native_B.ActualADCoutputsamples_k =
          acf_rl_native_B.Nativeconverterimpulserespons_j;
      }

      /* UniformRandomNumber: '<S8>/Independent lateral noise' incorporates:
       *  RandomNumber: '<S16>/Z raw noise'
       */
      rtb_Independentlateralnoise = acf_rl_native_DW.NextOutput_bf;

      /* Gain: '<S16>/Z raw noise RMS' */
      acf_rl_native_B.ZrawnoiseRMS = acf_rl_native_P.ZrawnoiseRMS_Gain *
        rtb_Independentlateralnoise;
    }

    if (tmp_0) {
      /* DiscreteTransferFcn: '<S43>/ADC output noise correlation' incorporates:
       *  Gain: '<S43>/Converter output noise RMS'
       *  RandomNumber: '<S43>/Independent converter output noise'
       */
      acf_rl_native_DW.ADCoutputnoisecorrelation_tmp_e =
        (acf_rl_native_P.ConverteroutputnoiseRMS_Gain_f *
         acf_rl_native_DW.NextOutput_n -
         acf_rl_native_P.ADCoutputnoisecorrelation_Den_f[1] *
         acf_rl_native_DW.ADCoutputnoisecorrelation_sta_b) /
        acf_rl_native_P.ADCoutputnoisecorrelation_Den_f[0];

      /* Bias: '<S43>/Restore physical static offset' incorporates:
       *  DiscreteTransferFcn: '<S43>/ADC output noise correlation'
       *  Sum: '<S43>/ADC input and output noise'
       */
      rtb_Upperrigidreaction =
        ((acf_rl_native_P.ADCoutputnoisecorrelation_Num_p[0] *
          acf_rl_native_DW.ADCoutputnoisecorrelation_tmp_e +
          acf_rl_native_P.ADCoutputnoisecorrelation_Num_p[1] *
          acf_rl_native_DW.ADCoutputnoisecorrelation_sta_b) +
         acf_rl_native_B.ActualADCoutputsamples_k) +
        acf_rl_native_P.Restorephysicalstaticoffset_B_a;

      /* Saturate: '<S43>/Actual ADC input range' */
      if (rtb_Upperrigidreaction >
          acf_rl_native_P.ActualADCinputrange_UpperSat_e) {
        rtb_Upperrigidreaction = acf_rl_native_P.ActualADCinputrange_UpperSat_e;
      } else if (rtb_Upperrigidreaction <
                 acf_rl_native_P.ActualADCinputrange_LowerSat_i) {
        rtb_Upperrigidreaction = acf_rl_native_P.ActualADCinputrange_LowerSat_i;
      }

      /* DiscreteTransferFcn: '<S43>/Fresh per-conversion EMA' incorporates:
       *  Bias: '<S43>/Known quantized static offset'
       *  Quantizer: '<S43>/Actual converter quantization'
       *  Saturate: '<S43>/Actual ADC input range'
       */
      acf_rl_native_DW.FreshperconversionEMA_tmp_k = ((rt_roundd_snf
        (rtb_Upperrigidreaction /
         acf_rl_native_P.Actualconverterquantization_I_f) *
        acf_rl_native_P.Actualconverterquantization_I_f +
        acf_rl_native_P.Knownquantizedstaticoffset_Bi_g) -
        acf_rl_native_P.FreshperconversionEMA_DenCoef_h[1] *
        acf_rl_native_DW.FreshperconversionEMA_states_f) /
        acf_rl_native_P.FreshperconversionEMA_DenCoef_h[0];

      /* DiscreteTransferFcn: '<S43>/Fresh per-conversion EMA' */
      acf_rl_native_B.FreshperconversionEMA_l =
        acf_rl_native_P.FreshperconversionEMA_NumCoef_m[0] *
        acf_rl_native_DW.FreshperconversionEMA_tmp_k +
        acf_rl_native_P.FreshperconversionEMA_NumCoef_m[1] *
        acf_rl_native_DW.FreshperconversionEMA_states_f;
    }

    /* Sum: '<S16>/Z noisy sensor' incorporates:
     *  Gain: '<S16>/Acceleration Z frame diagonal'
     *  Gain: '<S16>/Z sensor scale'
     *  Sin: '<S16>/Z analog vibration probe'
     *  Sum: '<S16>/Acceleration Z frame calibrated sum'
     */
    acf_rl_native_B.Znoisysensor =
      ((acf_rl_native_P.AccelerationZframediagonal_Gain *
        rtb_PIPIDandfeedforwardcurrent +
        acf_rl_native_B.AccelerationZframecrossaxis) *
       acf_rl_native_P.Zsensorscale_Gain + acf_rl_native_B.ZrawnoiseRMS) + (sin
      (acf_rl_native_P.Zanalogvibrationprobe_Freq * rtb_Normalizer +
       acf_rl_native_P.Zanalogvibrationprobe_Phase) *
      acf_rl_native_P.Zanalogvibrationprobe_Amp +
      acf_rl_native_P.Zanalogvibrationprobe_Bias);
    if (tmp_1) {
      /* Gain: '<S16>/Z old sample weight' incorporates:
       *  UnitDelay: '<S16>/Z filter memory'
       */
      acf_rl_native_B.Zoldsampleweight = acf_rl_native_P.Zoldsampleweight_Gain *
        acf_rl_native_DW.Zfiltermemory_DSTATE;
    }

    /* Sum: '<S16>/Z identified sensor response' incorporates:
     *  Gain: '<S16>/Z new sample weight'
     */
    acf_rl_native_B.Zidentifiedsensorresponse =
      acf_rl_native_P.Znewsampleweight_Gain * acf_rl_native_B.Znoisysensor +
      acf_rl_native_B.Zoldsampleweight;

    /* Switch: '<S16>/Z select identified sensor response' incorporates:
     *  Constant: '<S16>/Z use engineered acquisition'
     */
    if (acf_rl_native_P.Zuseengineeredacquisition_Value >=
        acf_rl_native_P.Zselectidentifiedsensorresponse) {
      rtb_Upperrigidreaction = acf_rl_native_B.FreshperconversionEMA_l;
    } else {
      rtb_Upperrigidreaction = acf_rl_native_B.Zidentifiedsensorresponse;
    }

    /* Saturate: '<S16>/Z acceleration range' incorporates:
     *  Switch: '<S16>/Z select identified sensor response'
     */
    if (rtb_Upperrigidreaction > acf_rl_native_P.Zaccelerationrange_UpperSat) {
      rtb_Upperrigidreaction = acf_rl_native_P.Zaccelerationrange_UpperSat;
    } else if (rtb_Upperrigidreaction <
               acf_rl_native_P.Zaccelerationrange_LowerSat) {
      rtb_Upperrigidreaction = acf_rl_native_P.Zaccelerationrange_LowerSat;
    }

    /* Sum: '<S16>/Z unknown drift' incorporates:
     *  Constant: '<S16>/Z drift after tare'
     *  Constant: '<S16>/Z known static tare'
     *  Quantizer: '<S16>/Z ADC quantization'
     *  Saturate: '<S16>/Z acceleration range'
     *  Sum: '<S16>/Z tared acceleration'
     */
    acf_rl_native_B.Zunknowndrift = (rt_roundd_snf(rtb_Upperrigidreaction /
      acf_rl_native_P.ZADCquantization_Interval) *
      acf_rl_native_P.ZADCquantization_Interval -
      acf_rl_native_P.Zknownstatictare_Value) +
      acf_rl_native_P.Zdriftaftertare_Value;

    /* Gain: '<S18>/Lateral sign' incorporates:
     *  Bias: '<S18>/Slope plus friction'
     *  Product: '<S18>/Lateral reaction'
     */
    rtb_PIPIDandfeedforwardcurrent = (rtb_Xfresh10kHzaccelerationfram +
      acf_rl_native_P.Slopeplusfriction_Bias) * rtb_Causalsurfacedirection /
      rtb_Compensatedlateralreaction * acf_rl_native_P.Lateralsign_Gain;

    /* Gain: '<S25>/Inverse inductance' incorporates:
     *  Sum: '<S25>/Inductor voltage'
     */
    acf_rl_native_B.Inverseinductance = ((rtb_Resetonanyinstability -
      rtb_Measurednormalforce) - rtb_Previousfaststablecount) *
      acf_rl_native_P.Inverseinductance_Gain;

    /* Logic: '<S23>/Invalid sizing run' incorporates:
     *  Logic: '<S23>/Contact lost'
     */
    acf_rl_native_B.Invalidsizingrun = (!rtb_Selectcontinuousfastdwell ||
      rtb_Travelstopreached);
    if (tmp_1) {
      /* Stop: '<S23>/Stop failed sizing run' incorporates:
       *  ZeroOrderHold: '<S23>/Safety check sampling'
       */
      if (acf_rl_native_B.Invalidsizingrun) {
        rtmSetStopRequested(acf_rl_native_M, true);
      }

      /* End of Stop: '<S23>/Stop failed sizing run' */
    }

    /* Gain: '<S27>/Head gap coordinate sign' incorporates:
     *  Bias: '<S27>/Measured head coordinate deviation'
     */
    acf_rl_native_B.Headgapcoordinatesign = (rtb_Lateralprojection +
      acf_rl_native_P.Measuredheadcoordinatedeviation) *
      acf_rl_native_P.Headgapcoordinatesign_Gain;

    /* StateSpace: '<S29>/Continuous analog front end' */
    acf_rl_native_B.Continuousanalogfrontend_n = 0.0;

    /* StateSpace: '<S29>/Continuous analog front end' */
    for (delayLen = acf_rl_native_P.Continuousanalogfrontend_C_jc_o[0U];
         delayLen < acf_rl_native_P.Continuousanalogfrontend_C_jc_o[1U];
         delayLen++) {
      /* StateSpace: '<S29>/Continuous analog front end' */
      acf_rl_native_B.Continuousanalogfrontend_n +=
        acf_rl_native_P.Continuousanalogfrontend_C_pr_d *
        acf_rl_native_X.Continuousanalogfrontend_CSTA_m[0U];
    }

    for (delayLen = acf_rl_native_P.Continuousanalogfrontend_C_jc_o[1U];
         delayLen < acf_rl_native_P.Continuousanalogfrontend_C_jc_o[2U];
         delayLen++) {
      /* StateSpace: '<S29>/Continuous analog front end' */
      acf_rl_native_B.Continuousanalogfrontend_n +=
        acf_rl_native_P.Continuousanalogfrontend_C_pr_d *
        acf_rl_native_X.Continuousanalogfrontend_CSTA_m[1U];
    }

    /* StateSpace: '<S29>/ADC analog input filter' */
    acf_rl_native_B.ADCanaloginputfilter_o = 0.0;

    /* StateSpace: '<S29>/ADC analog input filter' */
    for (delayLen = acf_rl_native_P.ADCanaloginputfilter_D_jc_e[0U]; delayLen <
         acf_rl_native_P.ADCanaloginputfilter_D_jc_e[1U]; delayLen++) {
      /* StateSpace: '<S29>/ADC analog input filter' */
      acf_rl_native_B.ADCanaloginputfilter_o +=
        acf_rl_native_P.ADCanaloginputfilter_D_pr_f *
        acf_rl_native_B.Continuousanalogfrontend_n;
    }

    if (tmp_1) {
      /* DiscreteTransferFcn: '<S29>/Native converter impulse response' */
      rtb_Taredaxialcellforce = acf_rl_native_B.ADCanaloginputfilter_o;
      for (i = 0; i < 30; i++) {
        rtb_Taredaxialcellforce -=
          acf_rl_native_P.Nativeconverterimpulserespons_n[i + 1] *
          acf_rl_native_DW.Nativeconverterimpulserespons_e[i];
      }

      acf_rl_native_DW.Nativeconverterimpulserespon_au = rtb_Taredaxialcellforce
        / acf_rl_native_P.Nativeconverterimpulserespons_n[0];
      rtb_Taredaxialcellforce = acf_rl_native_P.Nativeconverterimpulserespon_kh
        [0] * acf_rl_native_DW.Nativeconverterimpulserespon_au;
      for (i = 0; i < 30; i++) {
        rtb_Taredaxialcellforce +=
          acf_rl_native_P.Nativeconverterimpulserespon_kh[i + 1] *
          acf_rl_native_DW.Nativeconverterimpulserespons_e[i];
      }

      /* DiscreteTransferFcn: '<S29>/Native converter impulse response' */
      acf_rl_native_B.Nativeconverterimpulserespons_g = rtb_Taredaxialcellforce;

      /* ZeroOrderHold: '<S29>/Actual ADC output samples' */
      if (tmp_0) {
        /* ZeroOrderHold: '<S29>/Actual ADC output samples' */
        acf_rl_native_B.ActualADCoutputsamples_g =
          acf_rl_native_B.Nativeconverterimpulserespons_g;
      }
    }

    if (tmp_0) {
      /* DiscreteTransferFcn: '<S29>/Fresh per-conversion EMA' */
      acf_rl_native_DW.FreshperconversionEMA_tmp_h =
        (acf_rl_native_B.ActualADCoutputsamples_g -
         acf_rl_native_P.FreshperconversionEMA_DenCoef_j[1] *
         acf_rl_native_DW.FreshperconversionEMA_states_j) /
        acf_rl_native_P.FreshperconversionEMA_DenCoef_j[0];

      /* DiscreteTransferFcn: '<S29>/Fresh per-conversion EMA' */
      acf_rl_native_B.FreshperconversionEMA_c =
        acf_rl_native_P.FreshperconversionEMA_NumCoef_l[0] *
        acf_rl_native_DW.FreshperconversionEMA_tmp_h +
        acf_rl_native_P.FreshperconversionEMA_NumCoef_l[1] *
        acf_rl_native_DW.FreshperconversionEMA_states_j;
    }

    if (tmp_1) {
      /* DiscreteStateSpace: '<S27>/Position frame matching' */
      {
        rtb_Independentlateralnoise = acf_rl_native_P.Positionframematching_C*
          acf_rl_native_DW.Positionframematching_DSTATE;
        rtb_Independentlateralnoise += acf_rl_native_P.Positionframematching_D*
          acf_rl_native_B.Headgapcoordinatesign;
      }

      /* Switch: '<S27>/Position acquisition frame' incorporates:
       *  Constant: '<S27>/Position use engineered acquisition'
       */
      if (acf_rl_native_P.Positionuseengineeredacquisitio >=
          acf_rl_native_P.Positionacquisitionframe_Thresh) {
        /* Switch: '<S27>/Position acquisition frame' */
        acf_rl_native_B.Positionacquisitionframe =
          acf_rl_native_B.FreshperconversionEMA_c;
      } else {
        /* Switch: '<S27>/Position acquisition frame' */
        acf_rl_native_B.Positionacquisitionframe = rtb_Independentlateralnoise;
      }

      /* End of Switch: '<S27>/Position acquisition frame' */
    }

    /* StateSpace: '<S30>/Continuous analog front end' */
    acf_rl_native_B.Continuousanalogfrontend_m = 0.0;

    /* StateSpace: '<S30>/Continuous analog front end' */
    for (delayLen = acf_rl_native_P.Continuousanalogfrontend_C_jc_l[0U];
         delayLen < acf_rl_native_P.Continuousanalogfrontend_C_jc_l[1U];
         delayLen++) {
      /* StateSpace: '<S30>/Continuous analog front end' */
      acf_rl_native_B.Continuousanalogfrontend_m +=
        acf_rl_native_P.Continuousanalogfrontend_C_pr_e *
        acf_rl_native_X.Continuousanalogfrontend_CSTA_p[0U];
    }

    for (delayLen = acf_rl_native_P.Continuousanalogfrontend_C_jc_l[1U];
         delayLen < acf_rl_native_P.Continuousanalogfrontend_C_jc_l[2U];
         delayLen++) {
      /* StateSpace: '<S30>/Continuous analog front end' */
      acf_rl_native_B.Continuousanalogfrontend_m +=
        acf_rl_native_P.Continuousanalogfrontend_C_pr_e *
        acf_rl_native_X.Continuousanalogfrontend_CSTA_p[1U];
    }

    /* StateSpace: '<S30>/ADC analog input filter' */
    acf_rl_native_B.ADCanaloginputfilter_c = 0.0;

    /* StateSpace: '<S30>/ADC analog input filter' */
    for (delayLen = acf_rl_native_P.ADCanaloginputfilter_D_jc_c[0U]; delayLen <
         acf_rl_native_P.ADCanaloginputfilter_D_jc_c[1U]; delayLen++) {
      /* StateSpace: '<S30>/ADC analog input filter' */
      acf_rl_native_B.ADCanaloginputfilter_c +=
        acf_rl_native_P.ADCanaloginputfilter_D_pr_j *
        acf_rl_native_B.Continuousanalogfrontend_m;
    }

    if (tmp_1) {
      /* DiscreteTransferFcn: '<S30>/Native converter impulse response' */
      rtb_Taredaxialcellforce = acf_rl_native_B.ADCanaloginputfilter_c;
      for (i = 0; i < 30; i++) {
        rtb_Taredaxialcellforce -=
          acf_rl_native_P.Nativeconverterimpulserespon_dv[i + 1] *
          acf_rl_native_DW.Nativeconverterimpulserespons_k[i];
      }

      acf_rl_native_DW.Nativeconverterimpulserespons_b = rtb_Taredaxialcellforce
        / acf_rl_native_P.Nativeconverterimpulserespon_dv[0];
      rtb_Taredaxialcellforce = acf_rl_native_P.Nativeconverterimpulserespon_dh
        [0] * acf_rl_native_DW.Nativeconverterimpulserespons_b;
      for (i = 0; i < 30; i++) {
        rtb_Taredaxialcellforce +=
          acf_rl_native_P.Nativeconverterimpulserespon_dh[i + 1] *
          acf_rl_native_DW.Nativeconverterimpulserespons_k[i];
      }

      /* DiscreteTransferFcn: '<S30>/Native converter impulse response' */
      acf_rl_native_B.Nativeconverterimpulserespons_n = rtb_Taredaxialcellforce;

      /* ZeroOrderHold: '<S30>/Actual ADC output samples' */
      if (tmp_0) {
        /* ZeroOrderHold: '<S30>/Actual ADC output samples' */
        acf_rl_native_B.ActualADCoutputsamples_m =
          acf_rl_native_B.Nativeconverterimpulserespons_n;
      }
    }

    if (tmp_0) {
      /* DiscreteTransferFcn: '<S30>/Fresh per-conversion EMA' */
      acf_rl_native_DW.FreshperconversionEMA_tmp_e =
        (acf_rl_native_B.ActualADCoutputsamples_m -
         acf_rl_native_P.FreshperconversionEMA_DenCoef_p[1] *
         acf_rl_native_DW.FreshperconversionEMA_states_h) /
        acf_rl_native_P.FreshperconversionEMA_DenCoef_p[0];

      /* DiscreteTransferFcn: '<S30>/Fresh per-conversion EMA' */
      acf_rl_native_B.FreshperconversionEMA_k =
        acf_rl_native_P.FreshperconversionEMA_NumCoef_o[0] *
        acf_rl_native_DW.FreshperconversionEMA_tmp_e +
        acf_rl_native_P.FreshperconversionEMA_NumCoef_o[1] *
        acf_rl_native_DW.FreshperconversionEMA_states_h;
    }

    if (tmp_1) {
      /* DiscreteStateSpace: '<S27>/Robot acceleration frame matching' */
      {
        rtb_Independentlateralnoise =
          acf_rl_native_P.Robotaccelerationframematchi_f0*
          acf_rl_native_DW.Robotaccelerationframematching_;
        rtb_Independentlateralnoise +=
          acf_rl_native_P.Robotaccelerationframematchin_d*
          acf_rl_native_B.Enablerobotaccelerationpredicti;
      }

      /* Switch: '<S27>/Robot acceleration acquisition frame' incorporates:
       *  Constant: '<S27>/Robot acceleration use engineered acquisition'
       */
      if (acf_rl_native_P.Robotaccelerationuseengineereda >=
          acf_rl_native_P.Robotaccelerationacquisitionfra) {
        /* Switch: '<S27>/Robot acceleration acquisition frame' */
        acf_rl_native_B.Robotaccelerationacquisitionfra =
          acf_rl_native_B.FreshperconversionEMA_k;
      } else {
        /* Switch: '<S27>/Robot acceleration acquisition frame' */
        acf_rl_native_B.Robotaccelerationacquisitionfra =
          rtb_Independentlateralnoise;
      }

      /* End of Switch: '<S27>/Robot acceleration acquisition frame' */
    }

    /* StateSpace: '<S31>/Continuous analog front end' */
    acf_rl_native_B.Continuousanalogfrontend_mn = 0.0;

    /* StateSpace: '<S31>/Continuous analog front end' */
    for (delayLen = acf_rl_native_P.Continuousanalogfrontend_C_jc_n[0U];
         delayLen < acf_rl_native_P.Continuousanalogfrontend_C_jc_n[1U];
         delayLen++) {
      /* StateSpace: '<S31>/Continuous analog front end' */
      acf_rl_native_B.Continuousanalogfrontend_mn +=
        acf_rl_native_P.Continuousanalogfrontend_C_pr_f *
        acf_rl_native_X.Continuousanalogfrontend_CSTA_c[0U];
    }

    for (delayLen = acf_rl_native_P.Continuousanalogfrontend_C_jc_n[1U];
         delayLen < acf_rl_native_P.Continuousanalogfrontend_C_jc_n[2U];
         delayLen++) {
      /* StateSpace: '<S31>/Continuous analog front end' */
      acf_rl_native_B.Continuousanalogfrontend_mn +=
        acf_rl_native_P.Continuousanalogfrontend_C_pr_f *
        acf_rl_native_X.Continuousanalogfrontend_CSTA_c[1U];
    }

    /* StateSpace: '<S31>/ADC analog input filter' */
    acf_rl_native_B.ADCanaloginputfilter_cf = 0.0;

    /* StateSpace: '<S31>/ADC analog input filter' */
    for (delayLen = acf_rl_native_P.ADCanaloginputfilter_D_jc_a[0U]; delayLen <
         acf_rl_native_P.ADCanaloginputfilter_D_jc_a[1U]; delayLen++) {
      /* StateSpace: '<S31>/ADC analog input filter' */
      acf_rl_native_B.ADCanaloginputfilter_cf +=
        acf_rl_native_P.ADCanaloginputfilter_D_pr_a *
        acf_rl_native_B.Continuousanalogfrontend_mn;
    }

    if (tmp_1) {
      /* DiscreteTransferFcn: '<S31>/Native converter impulse response' */
      rtb_Taredaxialcellforce = acf_rl_native_B.ADCanaloginputfilter_cf;
      for (i = 0; i < 30; i++) {
        rtb_Taredaxialcellforce -=
          acf_rl_native_P.Nativeconverterimpulserespons_a[i + 1] *
          acf_rl_native_DW.Nativeconverterimpulserespons_m[i];
      }

      acf_rl_native_DW.Nativeconverterimpulserespons_n = rtb_Taredaxialcellforce
        / acf_rl_native_P.Nativeconverterimpulserespons_a[0];
      rtb_Taredaxialcellforce = acf_rl_native_P.Nativeconverterimpulserespon_hz
        [0] * acf_rl_native_DW.Nativeconverterimpulserespons_n;
      for (i = 0; i < 30; i++) {
        rtb_Taredaxialcellforce +=
          acf_rl_native_P.Nativeconverterimpulserespon_hz[i + 1] *
          acf_rl_native_DW.Nativeconverterimpulserespons_m[i];
      }

      /* DiscreteTransferFcn: '<S31>/Native converter impulse response' */
      acf_rl_native_B.Nativeconverterimpulserespons_l = rtb_Taredaxialcellforce;

      /* ZeroOrderHold: '<S31>/Actual ADC output samples' */
      if (tmp_0) {
        /* ZeroOrderHold: '<S31>/Actual ADC output samples' */
        acf_rl_native_B.ActualADCoutputsamples_i =
          acf_rl_native_B.Nativeconverterimpulserespons_l;
      }
    }

    if (tmp_0) {
      /* DiscreteTransferFcn: '<S31>/Fresh per-conversion EMA' */
      acf_rl_native_DW.FreshperconversionEMA_tmp_p =
        (acf_rl_native_B.ActualADCoutputsamples_i -
         acf_rl_native_P.FreshperconversionEMA_DenCoe_pq[1] *
         acf_rl_native_DW.FreshperconversionEMA_states_l) /
        acf_rl_native_P.FreshperconversionEMA_DenCoe_pq[0];

      /* DiscreteTransferFcn: '<S31>/Fresh per-conversion EMA' */
      acf_rl_native_B.FreshperconversionEMA_g =
        acf_rl_native_P.FreshperconversionEMA_NumCoef_b[0] *
        acf_rl_native_DW.FreshperconversionEMA_tmp_p +
        acf_rl_native_P.FreshperconversionEMA_NumCoef_b[1] *
        acf_rl_native_DW.FreshperconversionEMA_states_l;
    }

    if (tmp_1) {
      /* DiscreteStateSpace: '<S27>/Robot velocity frame matching' */
      {
        rtb_Independentlateralnoise =
          acf_rl_native_P.Robotvelocityframematching_C*
          acf_rl_native_DW.Robotvelocityframematching_DSTA;
        rtb_Independentlateralnoise +=
          acf_rl_native_P.Robotvelocityframematching_D*
          acf_rl_native_B.Predictedvelocitysampling;
      }

      /* Switch: '<S27>/Robot velocity acquisition frame' incorporates:
       *  Constant: '<S27>/Robot velocity use engineered acquisition'
       */
      if (acf_rl_native_P.Robotvelocityuseengineeredacqui >=
          acf_rl_native_P.Robotvelocityacquisitionframe_T) {
        /* Switch: '<S27>/Robot velocity acquisition frame' */
        acf_rl_native_B.Robotvelocityacquisitionframe =
          acf_rl_native_B.FreshperconversionEMA_g;
      } else {
        /* Switch: '<S27>/Robot velocity acquisition frame' */
        acf_rl_native_B.Robotvelocityacquisitionframe =
          rtb_Independentlateralnoise;
      }

      /* End of Switch: '<S27>/Robot velocity acquisition frame' */
    }

    if (tmp) {
      /* Sum: '<S36>/Normal error and anti windup' incorporates:
       *  Gain: '<S33>/Integral anti windup'
       *  Sum: '<S33>/Current saturation error'
       */
      rtb_Normalerrorandantiwindup = (rtb_Currentlimits - rtb_PIandPIDcorrection)
        * acf_rl_native_P.Integralantiwindup_Gain + rtb_Errortonormaldeviation;
    }

    /* StateSpace: '<S37>/Continuous analog front end' */
    acf_rl_native_B.Continuousanalogfrontend_n1 = 0.0;

    /* StateSpace: '<S37>/Continuous analog front end' */
    for (delayLen = acf_rl_native_P.Continuousanalogfrontend_C_jc_d[0U];
         delayLen < acf_rl_native_P.Continuousanalogfrontend_C_jc_d[1U];
         delayLen++) {
      /* StateSpace: '<S37>/Continuous analog front end' */
      acf_rl_native_B.Continuousanalogfrontend_n1 +=
        acf_rl_native_P.Continuousanalogfrontend_C_p_ft *
        acf_rl_native_X.Continuousanalogfrontend_CSTA_e[0U];
    }

    for (delayLen = acf_rl_native_P.Continuousanalogfrontend_C_jc_d[1U];
         delayLen < acf_rl_native_P.Continuousanalogfrontend_C_jc_d[2U];
         delayLen++) {
      /* StateSpace: '<S37>/Continuous analog front end' */
      acf_rl_native_B.Continuousanalogfrontend_n1 +=
        acf_rl_native_P.Continuousanalogfrontend_C_p_ft *
        acf_rl_native_X.Continuousanalogfrontend_CSTA_e[1U];
    }

    /* StateSpace: '<S37>/ADC analog input filter' */
    acf_rl_native_B.ADCanaloginputfilter_j = 0.0;

    /* StateSpace: '<S37>/ADC analog input filter' */
    for (delayLen = acf_rl_native_P.ADCanaloginputfilter_D_jc_f[0U]; delayLen <
         acf_rl_native_P.ADCanaloginputfilter_D_jc_f[1U]; delayLen++) {
      /* StateSpace: '<S37>/ADC analog input filter' */
      acf_rl_native_B.ADCanaloginputfilter_j +=
        acf_rl_native_P.ADCanaloginputfilter_D_pr_p *
        acf_rl_native_B.Continuousanalogfrontend_n1;
    }

    if (tmp_1) {
      /* DiscreteTransferFcn: '<S37>/Native converter impulse response' */
      rtb_Taredaxialcellforce = acf_rl_native_B.ADCanaloginputfilter_j;
      for (i = 0; i < 30; i++) {
        rtb_Taredaxialcellforce -=
          acf_rl_native_P.Nativeconverterimpulserespons_l[i + 1] *
          acf_rl_native_DW.Nativeconverterimpulserespons_h[i];
      }

      acf_rl_native_DW.Nativeconverterimpulserespons_p = rtb_Taredaxialcellforce
        / acf_rl_native_P.Nativeconverterimpulserespons_l[0];
      rtb_Taredaxialcellforce = acf_rl_native_P.Nativeconverterimpulserespons_j
        [0] * acf_rl_native_DW.Nativeconverterimpulserespons_p;
      for (i = 0; i < 30; i++) {
        rtb_Taredaxialcellforce +=
          acf_rl_native_P.Nativeconverterimpulserespons_j[i + 1] *
          acf_rl_native_DW.Nativeconverterimpulserespons_h[i];
      }

      /* DiscreteTransferFcn: '<S37>/Native converter impulse response' */
      acf_rl_native_B.Nativeconverterimpulserespons_o = rtb_Taredaxialcellforce;

      /* ZeroOrderHold: '<S37>/Actual ADC output samples' */
      if (tmp_0) {
        /* ZeroOrderHold: '<S37>/Actual ADC output samples' */
        acf_rl_native_B.ActualADCoutputsamples_n =
          acf_rl_native_B.Nativeconverterimpulserespons_o;
      }

      /* Gain: '<S5>/Filter memory' incorporates:
       *  UnitDelay: '<S5>/Previous filtered force'
       */
      rtb_Filtermemory = acf_rl_native_P.Filtermemory_Gain *
        acf_rl_native_DW.Previousfilteredforce_DSTATE;
    }

    if (tmp_0) {
      /* DiscreteTransferFcn: '<S37>/ADC output noise correlation' incorporates:
       *  Gain: '<S37>/Converter output noise RMS'
       *  RandomNumber: '<S37>/Independent converter output noise'
       */
      acf_rl_native_DW.ADCoutputnoisecorrelation_tmp_p =
        (acf_rl_native_P.ConverteroutputnoiseRMS_Gain_l *
         acf_rl_native_DW.NextOutput_j -
         acf_rl_native_P.ADCoutputnoisecorrelation_Den_d[1] *
         acf_rl_native_DW.ADCoutputnoisecorrelation_sta_o) /
        acf_rl_native_P.ADCoutputnoisecorrelation_Den_d[0];

      /* Bias: '<S37>/Restore physical static offset' incorporates:
       *  DiscreteTransferFcn: '<S37>/ADC output noise correlation'
       *  Sum: '<S37>/ADC input and output noise'
       */
      rtb_Upperrigidreaction =
        ((acf_rl_native_P.ADCoutputnoisecorrelation_Nu_pq[0] *
          acf_rl_native_DW.ADCoutputnoisecorrelation_tmp_p +
          acf_rl_native_P.ADCoutputnoisecorrelation_Nu_pq[1] *
          acf_rl_native_DW.ADCoutputnoisecorrelation_sta_o) +
         acf_rl_native_B.ActualADCoutputsamples_n) +
        acf_rl_native_P.Restorephysicalstaticoffset_B_l;

      /* Saturate: '<S37>/Actual ADC input range' */
      if (rtb_Upperrigidreaction >
          acf_rl_native_P.ActualADCinputrange_UpperSat_i) {
        rtb_Upperrigidreaction = acf_rl_native_P.ActualADCinputrange_UpperSat_i;
      } else if (rtb_Upperrigidreaction <
                 acf_rl_native_P.ActualADCinputrange_LowerSat_c) {
        rtb_Upperrigidreaction = acf_rl_native_P.ActualADCinputrange_LowerSat_c;
      }

      /* DiscreteTransferFcn: '<S37>/Fresh per-conversion EMA' incorporates:
       *  Bias: '<S37>/Known quantized static offset'
       *  Quantizer: '<S37>/Actual converter quantization'
       *  Saturate: '<S37>/Actual ADC input range'
       */
      acf_rl_native_DW.FreshperconversionEMA_tmp_ep = ((rt_roundd_snf
        (rtb_Upperrigidreaction /
         acf_rl_native_P.Actualconverterquantization_I_i) *
        acf_rl_native_P.Actualconverterquantization_I_i +
        acf_rl_native_P.Knownquantizedstaticoffset_Bi_c) -
        acf_rl_native_P.FreshperconversionEMA_DenCoef_g[1] *
        acf_rl_native_DW.FreshperconversionEMA_states_m) /
        acf_rl_native_P.FreshperconversionEMA_DenCoef_g[0];

      /* Sum: '<S5>/Conditioned force with tare residual' incorporates:
       *  Constant: '<S5>/Sensor bias'
       *  DiscreteTransferFcn: '<S37>/Fresh per-conversion EMA'
       */
      acf_rl_native_B.Conditionedforcewithtareresidua =
        (acf_rl_native_P.FreshperconversionEMA_NumCoef_n[0] *
         acf_rl_native_DW.FreshperconversionEMA_tmp_ep +
         acf_rl_native_P.FreshperconversionEMA_NumCoef_n[1] *
         acf_rl_native_DW.FreshperconversionEMA_states_m) +
        acf_rl_native_P.Sensorbias_Value;
    }

    /* Sum: '<S5>/Tared cell force' incorporates:
     *  Constant: '<S5>/Axial gravity'
     *  Constant: '<S5>/Known weight tare'
     *  Gain: '<S5>/Known downstream tool mass'
     *  Sum: '<S5>/Physical B-S load cell'
     *  Sum: '<S5>/Tool specific acceleration'
     */
    rtb_Currentlimits = (rtb_Compensatedaxialreaction -
                         (rtb_Sendcorrectiononlywhensettl +
                          acf_rl_native_P.Axialgravity_Value) *
                         acf_rl_native_P.Knowndownstreamtoolmass_Gain) +
      acf_rl_native_P.Knownweighttare_Value;

    /* Gain: '<S5>/Residual axial calibration' incorporates:
     *  Gain: '<S5>/Force Z frame cross axis'
     *  Gain: '<S5>/Force Z frame diagonal'
     *  Sum: '<S5>/Force Z frame calibrated sum'
     */
    acf_rl_native_B.Residualaxialcalibration =
      (acf_rl_native_P.ForceZframediagonal_Gain * rtb_Currentlimits +
       acf_rl_native_P.ForceZframecrossaxis_Gain *
       rtb_PIPIDandfeedforwardcurrent) *
      acf_rl_native_P.Residualaxialcalibration_Gain;
    if (tmp_1) {
      /* UniformRandomNumber: '<S8>/Independent lateral noise' incorporates:
       *  ZeroOrderHold: '<S5>/Sensor sampling'
       */
      rtb_Independentlateralnoise = acf_rl_native_B.Residualaxialcalibration;

      /* Sum: '<S5>/Low pass filter' incorporates:
       *  Gain: '<S5>/New sample weight'
       */
      rtb_Lowpassfilter = acf_rl_native_P.Newsampleweight_Gain *
        rtb_Independentlateralnoise + rtb_Filtermemory;

      /* UniformRandomNumber: '<S8>/Independent lateral noise' incorporates:
       *  UniformRandomNumber: '<S5>/Uniform sensor noise'
       */
      rtb_Independentlateralnoise =
        acf_rl_native_DW.Uniformsensornoise_NextOutput;

      /* Gain: '<S5>/Sensor noise RMS' */
      rtb_Positionwithinevent = acf_rl_native_P.SensornoiseRMS_Gain *
        rtb_Independentlateralnoise;

      /* Gain: '<S5>/Raw output-noise normalization' */
      acf_rl_native_B.Rawoutputnoisenormalization =
        acf_rl_native_P.Rawoutputnoisenormalization_Gai *
        rtb_Positionwithinevent;

      /* Switch: '<S5>/Select identified sensor response' incorporates:
       *  Constant: '<S5>/Use engineered acquisition'
       */
      if (acf_rl_native_P.Useengineeredacquisition_Value >=
          acf_rl_native_P.Selectidentifiedsensorresponse_) {
        /* Switch: '<S5>/Select identified sensor response' */
        acf_rl_native_B.Selectidentifiedsensorresponse =
          acf_rl_native_B.Conditionedforcewithtareresidua;
      } else {
        /* Switch: '<S5>/Select identified sensor response' incorporates:
         *  Constant: '<S5>/Sensor bias'
         *  Sum: '<S5>/Measured force'
         */
        acf_rl_native_B.Selectidentifiedsensorresponse = (rtb_Lowpassfilter +
          acf_rl_native_P.Sensorbias_Value) + rtb_Positionwithinevent;
      }

      /* End of Switch: '<S5>/Select identified sensor response' */
    }

    /* Sum: '<S5>/Raw noisy force sample' incorporates:
     *  Sin: '<S5>/Force analog vibration probe'
     */
    acf_rl_native_B.Rawnoisyforcesample = (sin
      (acf_rl_native_P.Forceanalogvibrationprobe_Freq * rtb_Normalizer +
       acf_rl_native_P.Forceanalogvibrationprobe_Phase) *
      acf_rl_native_P.Forceanalogvibrationprobe_Amp +
      acf_rl_native_P.Forceanalogvibrationprobe_Bias) +
      (acf_rl_native_B.Residualaxialcalibration +
       acf_rl_native_B.Rawoutputnoisenormalization);

    /* StateSpace: '<S38>/Continuous analog front end' */
    acf_rl_native_B.Continuousanalogfrontend_jz = 0.0;

    /* StateSpace: '<S38>/Continuous analog front end' */
    for (delayLen = acf_rl_native_P.Continuousanalogfrontend_C_jc_k[0U];
         delayLen < acf_rl_native_P.Continuousanalogfrontend_C_jc_k[1U];
         delayLen++) {
      /* StateSpace: '<S38>/Continuous analog front end' */
      acf_rl_native_B.Continuousanalogfrontend_jz +=
        acf_rl_native_P.Continuousanalogfrontend_C_p_ms *
        acf_rl_native_X.Continuousanalogfrontend_CSTA_k[0U];
    }

    for (delayLen = acf_rl_native_P.Continuousanalogfrontend_C_jc_k[1U];
         delayLen < acf_rl_native_P.Continuousanalogfrontend_C_jc_k[2U];
         delayLen++) {
      /* StateSpace: '<S38>/Continuous analog front end' */
      acf_rl_native_B.Continuousanalogfrontend_jz +=
        acf_rl_native_P.Continuousanalogfrontend_C_p_ms *
        acf_rl_native_X.Continuousanalogfrontend_CSTA_k[1U];
    }

    /* StateSpace: '<S38>/ADC analog input filter' */
    acf_rl_native_B.ADCanaloginputfilter_f = 0.0;

    /* StateSpace: '<S38>/ADC analog input filter' */
    for (delayLen = acf_rl_native_P.ADCanaloginputfilter_D_jc_fu[0U]; delayLen <
         acf_rl_native_P.ADCanaloginputfilter_D_jc_fu[1U]; delayLen++) {
      /* StateSpace: '<S38>/ADC analog input filter' */
      acf_rl_native_B.ADCanaloginputfilter_f +=
        acf_rl_native_P.ADCanaloginputfilter_D_pr_p5 *
        acf_rl_native_B.Continuousanalogfrontend_jz;
    }

    if (tmp_1) {
      /* DiscreteTransferFcn: '<S38>/Native converter impulse response' */
      rtb_Taredaxialcellforce = acf_rl_native_B.ADCanaloginputfilter_f;
      for (i = 0; i < 30; i++) {
        rtb_Taredaxialcellforce -=
          acf_rl_native_P.Nativeconverterimpulserespon_al[i + 1] *
          acf_rl_native_DW.Nativeconverterimpulserespons_g[i];
      }

      acf_rl_native_DW.Nativeconverterimpulserespons_l = rtb_Taredaxialcellforce
        / acf_rl_native_P.Nativeconverterimpulserespon_al[0];
      rtb_Taredaxialcellforce = acf_rl_native_P.Nativeconverterimpulserespon_e5
        [0] * acf_rl_native_DW.Nativeconverterimpulserespons_l;
      for (i = 0; i < 30; i++) {
        rtb_Taredaxialcellforce +=
          acf_rl_native_P.Nativeconverterimpulserespon_e5[i + 1] *
          acf_rl_native_DW.Nativeconverterimpulserespons_g[i];
      }

      /* DiscreteTransferFcn: '<S38>/Native converter impulse response' */
      acf_rl_native_B.Nativeconverterimpulserespon_nt = rtb_Taredaxialcellforce;

      /* ZeroOrderHold: '<S38>/Actual ADC output samples' */
      if (tmp_0) {
        /* ZeroOrderHold: '<S38>/Actual ADC output samples' */
        acf_rl_native_B.ActualADCoutputsamples_b =
          acf_rl_native_B.Nativeconverterimpulserespon_nt;
      }

      /* UnitDelay: '<S8>/Filter memory' */
      rtb_Filtermemory_j = acf_rl_native_DW.Filtermemory_DSTATE;

      /* UniformRandomNumber: '<S8>/Independent lateral noise' */
      rtb_Independentlateralnoise =
        acf_rl_native_DW.Independentlateralnoise_NextOut;
    }

    if (tmp_0) {
      /* DiscreteTransferFcn: '<S38>/ADC output noise correlation' incorporates:
       *  Gain: '<S38>/Converter output noise RMS'
       *  RandomNumber: '<S38>/Independent converter output noise'
       */
      acf_rl_native_DW.ADCoutputnoisecorrelation_tmp_n =
        (acf_rl_native_P.ConverteroutputnoiseRMS_Gain_c *
         acf_rl_native_DW.NextOutput_m -
         acf_rl_native_P.ADCoutputnoisecorrelation_Den_j[1] *
         acf_rl_native_DW.ADCoutputnoisecorrelation_st_ov) /
        acf_rl_native_P.ADCoutputnoisecorrelation_Den_j[0];

      /* Bias: '<S38>/Restore physical static offset' incorporates:
       *  DiscreteTransferFcn: '<S38>/ADC output noise correlation'
       *  Sum: '<S38>/ADC input and output noise'
       */
      rtb_Upperrigidreaction =
        ((acf_rl_native_P.ADCoutputnoisecorrelation_Nu_pw[0] *
          acf_rl_native_DW.ADCoutputnoisecorrelation_tmp_n +
          acf_rl_native_P.ADCoutputnoisecorrelation_Nu_pw[1] *
          acf_rl_native_DW.ADCoutputnoisecorrelation_st_ov) +
         acf_rl_native_B.ActualADCoutputsamples_b) +
        acf_rl_native_P.Restorephysicalstaticoffset_B_b;

      /* Saturate: '<S38>/Actual ADC input range' */
      if (rtb_Upperrigidreaction >
          acf_rl_native_P.ActualADCinputrange_UpperSat_j) {
        rtb_Upperrigidreaction = acf_rl_native_P.ActualADCinputrange_UpperSat_j;
      } else if (rtb_Upperrigidreaction <
                 acf_rl_native_P.ActualADCinputrange_LowerSat_n) {
        rtb_Upperrigidreaction = acf_rl_native_P.ActualADCinputrange_LowerSat_n;
      }

      /* DiscreteTransferFcn: '<S38>/Fresh per-conversion EMA' incorporates:
       *  Bias: '<S38>/Known quantized static offset'
       *  Quantizer: '<S38>/Actual converter quantization'
       *  Saturate: '<S38>/Actual ADC input range'
       */
      acf_rl_native_DW.FreshperconversionEMA_tmp_j = ((rt_roundd_snf
        (rtb_Upperrigidreaction /
         acf_rl_native_P.Actualconverterquantization_I_b) *
        acf_rl_native_P.Actualconverterquantization_I_b +
        acf_rl_native_P.Knownquantizedstaticoffset_Bi_o) -
        acf_rl_native_P.FreshperconversionEMA_DenCoef_k[1] *
        acf_rl_native_DW.FreshperconversionEMA_states_a) /
        acf_rl_native_P.FreshperconversionEMA_DenCoef_k[0];

      /* Sum: '<S8>/Conditioned force with tare residual' incorporates:
       *  Constant: '<S8>/Known tare residual bias'
       *  DiscreteTransferFcn: '<S38>/Fresh per-conversion EMA'
       */
      acf_rl_native_B.Conditionedforcewithtareresid_m =
        (acf_rl_native_P.FreshperconversionEMA_NumCoe_b0[0] *
         acf_rl_native_DW.FreshperconversionEMA_tmp_j +
         acf_rl_native_P.FreshperconversionEMA_NumCoe_b0[1] *
         acf_rl_native_DW.FreshperconversionEMA_states_a) +
        acf_rl_native_P.Knowntareresidualbias_Value;
    }

    /* Gain: '<S8>/Residual lateral calibration' incorporates:
     *  Gain: '<S8>/Force X frame cross axis'
     *  Gain: '<S8>/Force X frame diagonal'
     *  Sum: '<S8>/Force X frame calibrated sum'
     */
    acf_rl_native_B.Residuallateralcalibration =
      (acf_rl_native_P.ForceXframediagonal_Gain * rtb_PIPIDandfeedforwardcurrent
       + acf_rl_native_P.ForceXframecrossaxis_Gain * rtb_Currentlimits) *
      acf_rl_native_P.Residuallateralcalibration_Gain;
    if (tmp_1) {
      /* Gain: '<S8>/Noise RMS' */
      rtb_Currentlimits = acf_rl_native_P.NoiseRMS_Gain *
        rtb_Independentlateralnoise;

      /* Sum: '<S8>/Sensor low pass' incorporates:
       *  Gain: '<S8>/New sample weight'
       *  Gain: '<S8>/Old sample weight'
       *  ZeroOrderHold: '<S8>/Sensor sampling'
       */
      rtb_Sensorlowpass = acf_rl_native_P.Newsampleweight_Gain_j *
        acf_rl_native_B.Residuallateralcalibration +
        acf_rl_native_P.Oldsampleweight_Gain * rtb_Filtermemory_j;

      /* Gain: '<S8>/Raw output-noise normalization' */
      acf_rl_native_B.Rawoutputnoisenormalization_g =
        acf_rl_native_P.Rawoutputnoisenormalization_G_a * rtb_Currentlimits;

      /* Switch: '<S8>/Select identified sensor response' incorporates:
       *  Constant: '<S8>/Use engineered acquisition'
       */
      if (acf_rl_native_P.Useengineeredacquisition_Valu_f >=
          acf_rl_native_P.Selectidentifiedsensorrespons_i) {
        /* Switch: '<S8>/Select identified sensor response' */
        acf_rl_native_B.Selectidentifiedsensorrespons_d =
          acf_rl_native_B.Conditionedforcewithtareresid_m;
      } else {
        /* Switch: '<S8>/Select identified sensor response' incorporates:
         *  Constant: '<S8>/Known tare residual bias'
         *  Sum: '<S8>/Noisy tared lateral force'
         */
        acf_rl_native_B.Selectidentifiedsensorrespons_d = (rtb_Sensorlowpass +
          rtb_Currentlimits) + acf_rl_native_P.Knowntareresidualbias_Value;
      }

      /* End of Switch: '<S8>/Select identified sensor response' */
    }

    /* Sum: '<S8>/Raw noisy force sample' incorporates:
     *  Sin: '<S8>/Lateral analog vibration probe'
     */
    acf_rl_native_B.Rawnoisyforcesample_l = (sin
      (acf_rl_native_P.Lateralanalogvibrationprobe_Fre * rtb_Normalizer +
       acf_rl_native_P.Lateralanalogvibrationprobe_Pha) *
      acf_rl_native_P.Lateralanalogvibrationprobe_Amp +
      acf_rl_native_P.Lateralanalogvibrationprobe_Bia) +
      (acf_rl_native_B.Residuallateralcalibration +
       acf_rl_native_B.Rawoutputnoisenormalization_g);
  }

  if (rtmIsMajorTimeStep(acf_rl_native_M)) {
    real_T tmp_2;
    int32_T j;
    boolean_T tmp;
    boolean_T tmp_0;
    boolean_T tmp_1;

    /* Update for TransportDelay: '<S5>/Sensor sample delay' incorporates:
     *  TransportDelay: '<S16>/X known channel delay'
     *  TransportDelay: '<S16>/Z known channel delay'
     *  TransportDelay: '<S27>/Position ADC delivery pipeline'
     *  TransportDelay: '<S27>/Robot acceleration frame matching delay'
     *  TransportDelay: '<S27>/Robot velocity frame matching delay'
     *  TransportDelay: '<S8>/Sensor sample delay'
     */
    if (acf_rl_native_DW.Sensorsampledelay_IWORK[1] <
        acf_rl_native_DW.Sensorsampledelay_IWORK[3] - 1) {
      acf_rl_native_DW.Sensorsampledelay_IWORK[1]++;
    } else {
      acf_rl_native_DW.Sensorsampledelay_IWORK[1] = 0;
    }

    if (acf_rl_native_DW.Sensorsampledelay_IWORK[0] ==
        acf_rl_native_DW.Sensorsampledelay_IWORK[1]) {
      if (acf_rl_native_DW.Sensorsampledelay_IWORK[0] <
          acf_rl_native_DW.Sensorsampledelay_IWORK[3] - 1) {
        acf_rl_native_DW.Sensorsampledelay_IWORK[0]++;
      } else {
        acf_rl_native_DW.Sensorsampledelay_IWORK[0] = 0;
      }
    }

    ((real_T *)acf_rl_native_DW.Sensorsampledelay_PWORK[0])
      [acf_rl_native_DW.Sensorsampledelay_IWORK[1]] =
      acf_rl_native_B.Selectidentifiedsensorresponse;
    tmp_2 = acf_rl_native_M->Timing.t[0];
    ((real_T *)acf_rl_native_DW.Sensorsampledelay_PWORK[0])
      [acf_rl_native_DW.Sensorsampledelay_IWORK[1] +
      acf_rl_native_DW.Sensorsampledelay_IWORK[3]] = tmp_2;

    /* End of Update for TransportDelay: '<S5>/Sensor sample delay' */

    /* Update for TransportDelay: '<S16>/Z known channel delay' */
    if (acf_rl_native_DW.Zknownchanneldelay_IWORK[1] <
        acf_rl_native_DW.Zknownchanneldelay_IWORK[3] - 1) {
      acf_rl_native_DW.Zknownchanneldelay_IWORK[1]++;
    } else {
      acf_rl_native_DW.Zknownchanneldelay_IWORK[1] = 0;
    }

    if (acf_rl_native_DW.Zknownchanneldelay_IWORK[0] ==
        acf_rl_native_DW.Zknownchanneldelay_IWORK[1]) {
      if (acf_rl_native_DW.Zknownchanneldelay_IWORK[0] <
          acf_rl_native_DW.Zknownchanneldelay_IWORK[3] - 1) {
        acf_rl_native_DW.Zknownchanneldelay_IWORK[0]++;
      } else {
        acf_rl_native_DW.Zknownchanneldelay_IWORK[0] = 0;
      }
    }

    ((real_T *)acf_rl_native_DW.Zknownchanneldelay_PWORK[0])
      [acf_rl_native_DW.Zknownchanneldelay_IWORK[1]] =
      acf_rl_native_B.Zunknowndrift;
    ((real_T *)acf_rl_native_DW.Zknownchanneldelay_PWORK[0])
      [acf_rl_native_DW.Zknownchanneldelay_IWORK[1] +
      acf_rl_native_DW.Zknownchanneldelay_IWORK[3]] = tmp_2;

    /* Update for TransportDelay: '<S8>/Sensor sample delay' */
    if (acf_rl_native_DW.Sensorsampledelay_IWORK_i[1] <
        acf_rl_native_DW.Sensorsampledelay_IWORK_i[3] - 1) {
      acf_rl_native_DW.Sensorsampledelay_IWORK_i[1]++;
    } else {
      acf_rl_native_DW.Sensorsampledelay_IWORK_i[1] = 0;
    }

    if (acf_rl_native_DW.Sensorsampledelay_IWORK_i[0] ==
        acf_rl_native_DW.Sensorsampledelay_IWORK_i[1]) {
      if (acf_rl_native_DW.Sensorsampledelay_IWORK_i[0] <
          acf_rl_native_DW.Sensorsampledelay_IWORK_i[3] - 1) {
        acf_rl_native_DW.Sensorsampledelay_IWORK_i[0]++;
      } else {
        acf_rl_native_DW.Sensorsampledelay_IWORK_i[0] = 0;
      }
    }

    ((real_T *)acf_rl_native_DW.Sensorsampledelay_PWORK_f[0])
      [acf_rl_native_DW.Sensorsampledelay_IWORK_i[1]] =
      acf_rl_native_B.Selectidentifiedsensorrespons_d;
    ((real_T *)acf_rl_native_DW.Sensorsampledelay_PWORK_f[0])
      [acf_rl_native_DW.Sensorsampledelay_IWORK_i[1] +
      acf_rl_native_DW.Sensorsampledelay_IWORK_i[3]] = tmp_2;

    /* Update for TransportDelay: '<S16>/X known channel delay' */
    if (acf_rl_native_DW.Xknownchanneldelay_IWORK[1] <
        acf_rl_native_DW.Xknownchanneldelay_IWORK[3] - 1) {
      acf_rl_native_DW.Xknownchanneldelay_IWORK[1]++;
    } else {
      acf_rl_native_DW.Xknownchanneldelay_IWORK[1] = 0;
    }

    if (acf_rl_native_DW.Xknownchanneldelay_IWORK[0] ==
        acf_rl_native_DW.Xknownchanneldelay_IWORK[1]) {
      if (acf_rl_native_DW.Xknownchanneldelay_IWORK[0] <
          acf_rl_native_DW.Xknownchanneldelay_IWORK[3] - 1) {
        acf_rl_native_DW.Xknownchanneldelay_IWORK[0]++;
      } else {
        acf_rl_native_DW.Xknownchanneldelay_IWORK[0] = 0;
      }
    }

    ((real_T *)acf_rl_native_DW.Xknownchanneldelay_PWORK[0])
      [acf_rl_native_DW.Xknownchanneldelay_IWORK[1]] =
      acf_rl_native_B.Xunknowndrift;
    ((real_T *)acf_rl_native_DW.Xknownchanneldelay_PWORK[0])
      [acf_rl_native_DW.Xknownchanneldelay_IWORK[1] +
      acf_rl_native_DW.Xknownchanneldelay_IWORK[3]] = tmp_2;

    /* Update for TransportDelay: '<S27>/Robot velocity frame matching delay' */
    if (acf_rl_native_DW.Robotvelocityframematchingdel_c[1] <
        acf_rl_native_DW.Robotvelocityframematchingdel_c[3] - 1) {
      acf_rl_native_DW.Robotvelocityframematchingdel_c[1]++;
    } else {
      acf_rl_native_DW.Robotvelocityframematchingdel_c[1] = 0;
    }

    if (acf_rl_native_DW.Robotvelocityframematchingdel_c[0] ==
        acf_rl_native_DW.Robotvelocityframematchingdel_c[1]) {
      if (acf_rl_native_DW.Robotvelocityframematchingdel_c[0] <
          acf_rl_native_DW.Robotvelocityframematchingdel_c[3] - 1) {
        acf_rl_native_DW.Robotvelocityframematchingdel_c[0]++;
      } else {
        acf_rl_native_DW.Robotvelocityframematchingdel_c[0] = 0;
      }
    }

    ((real_T *)acf_rl_native_DW.Robotvelocityframematchingdel_l[0])
      [acf_rl_native_DW.Robotvelocityframematchingdel_c[1]] =
      acf_rl_native_B.Robotvelocityacquisitionframe;
    ((real_T *)acf_rl_native_DW.Robotvelocityframematchingdel_l[0])
      [acf_rl_native_DW.Robotvelocityframematchingdel_c[1] +
      acf_rl_native_DW.Robotvelocityframematchingdel_c[3]] = tmp_2;

    /* Update for TransportDelay: '<S27>/Position ADC delivery pipeline' */
    if (acf_rl_native_DW.PositionADCdeliverypipeline_IWO[1] <
        acf_rl_native_DW.PositionADCdeliverypipeline_IWO[3] - 1) {
      acf_rl_native_DW.PositionADCdeliverypipeline_IWO[1]++;
    } else {
      acf_rl_native_DW.PositionADCdeliverypipeline_IWO[1] = 0;
    }

    if (acf_rl_native_DW.PositionADCdeliverypipeline_IWO[0] ==
        acf_rl_native_DW.PositionADCdeliverypipeline_IWO[1]) {
      if (acf_rl_native_DW.PositionADCdeliverypipeline_IWO[0] <
          acf_rl_native_DW.PositionADCdeliverypipeline_IWO[3] - 1) {
        acf_rl_native_DW.PositionADCdeliverypipeline_IWO[0]++;
      } else {
        acf_rl_native_DW.PositionADCdeliverypipeline_IWO[0] = 0;
      }
    }

    ((real_T *)acf_rl_native_DW.PositionADCdeliverypipeline_PWO[0])
      [acf_rl_native_DW.PositionADCdeliverypipeline_IWO[1]] =
      acf_rl_native_B.Positionacquisitionframe;
    ((real_T *)acf_rl_native_DW.PositionADCdeliverypipeline_PWO[0])
      [acf_rl_native_DW.PositionADCdeliverypipeline_IWO[1] +
      acf_rl_native_DW.PositionADCdeliverypipeline_IWO[3]] = tmp_2;

    /* Update for TransportDelay: '<S27>/Robot acceleration frame matching delay' */
    if (acf_rl_native_DW.Robotaccelerationframematchin_n[1] <
        acf_rl_native_DW.Robotaccelerationframematchin_n[3] - 1) {
      acf_rl_native_DW.Robotaccelerationframematchin_n[1]++;
    } else {
      acf_rl_native_DW.Robotaccelerationframematchin_n[1] = 0;
    }

    if (acf_rl_native_DW.Robotaccelerationframematchin_n[0] ==
        acf_rl_native_DW.Robotaccelerationframematchin_n[1]) {
      if (acf_rl_native_DW.Robotaccelerationframematchin_n[0] <
          acf_rl_native_DW.Robotaccelerationframematchin_n[3] - 1) {
        acf_rl_native_DW.Robotaccelerationframematchin_n[0]++;
      } else {
        acf_rl_native_DW.Robotaccelerationframematchin_n[0] = 0;
      }
    }

    ((real_T *)acf_rl_native_DW.Robotaccelerationframematchin_h[0])
      [acf_rl_native_DW.Robotaccelerationframematchin_n[1]] =
      acf_rl_native_B.Robotaccelerationacquisitionfra;
    ((real_T *)acf_rl_native_DW.Robotaccelerationframematchin_h[0])
      [acf_rl_native_DW.Robotaccelerationframematchin_n[1] +
      acf_rl_native_DW.Robotaccelerationframematchin_n[3]] = tmp_2;
    tmp = (rtmIsMajorTimeStep(acf_rl_native_M) &&
           acf_rl_native_M->Timing.TaskCounters.TID[3] == 0);
    if (tmp) {
      /* Update for DiscreteStateSpace: '<S27>/Head gap and velocity observer' */
      {
        real_T xnew[2];
        xnew[0] = acf_rl_native_P.Headgapandvelocityobserver_A[0]*
          acf_rl_native_DW.Headgapandvelocityobserver_DSTA[0] +
          acf_rl_native_P.Headgapandvelocityobserver_A[1]*
          acf_rl_native_DW.Headgapandvelocityobserver_DSTA[1];
        xnew[0] += acf_rl_native_P.Headgapandvelocityobserver_B[0]*
          acf_rl_native_B.Fresh10kHzpositionframe +
          acf_rl_native_P.Headgapandvelocityobserver_B[1]*
          acf_rl_native_B.Fresh10kHzrobotaccelerationfram
          + acf_rl_native_P.Headgapandvelocityobserver_B[2]*
          rtb_Zfresh10kHzaccelerationfram;
        xnew[1] = acf_rl_native_P.Headgapandvelocityobserver_A[2]*
          acf_rl_native_DW.Headgapandvelocityobserver_DSTA[0] +
          acf_rl_native_P.Headgapandvelocityobserver_A[3]*
          acf_rl_native_DW.Headgapandvelocityobserver_DSTA[1];
        xnew[1] += acf_rl_native_P.Headgapandvelocityobserver_B[3]*
          acf_rl_native_B.Fresh10kHzpositionframe +
          acf_rl_native_P.Headgapandvelocityobserver_B[4]*
          acf_rl_native_B.Fresh10kHzrobotaccelerationfram
          + acf_rl_native_P.Headgapandvelocityobserver_B[5]*
          rtb_Zfresh10kHzaccelerationfram;
        (void) memcpy(acf_rl_native_DW.Headgapandvelocityobserver_DSTA, xnew,
                      sizeof(real_T)*2);
      }
    }

    tmp_0 = (rtmIsMajorTimeStep(acf_rl_native_M) &&
             acf_rl_native_M->Timing.TaskCounters.TID[4] == 0);
    if (tmp_0) {
      /* Update for Delay: '<S7>/Expected command delay' */
      acf_rl_native_DW.Expectedcommanddelay_DSTATE[acf_rl_native_DW.CircBufIdx] =
        acf_rl_native_B.Robotspeedlimits;
      if (acf_rl_native_DW.CircBufIdx < 11U) {
        acf_rl_native_DW.CircBufIdx++;
      } else {
        acf_rl_native_DW.CircBufIdx = 0U;
      }

      /* End of Update for Delay: '<S7>/Expected command delay' */
    }

    if (tmp) {
      /* Update for DiscreteIntegrator: '<S36>/Normal force error integral' */
      acf_rl_native_DW.Normalforceerrorintegral_DSTATE +=
        acf_rl_native_P.Normalforceerrorintegral_gainva *
        rtb_Normalerrorandantiwindup;

      /* Update for UnitDelay: '<S36>/Normal error previous sample' */
      acf_rl_native_DW.Normalerrorprevioussample_DSTAT =
        rtb_Errortonormaldeviation;

      /* Update for UnitDelay: '<S36>/Normal error derivative memory' */
      acf_rl_native_DW.Normalerrorderivativememory_DST =
        rtb_Normalerrorfilteredderivati;

      /* Update for DiscreteTransferFcn: '<S36>/Optional feedback rolloff' */
      acf_rl_native_DW.Optionalfeedbackrolloff_states[1] =
        acf_rl_native_DW.Optionalfeedbackrolloff_states[0];
      acf_rl_native_DW.Optionalfeedbackrolloff_states[0] =
        acf_rl_native_DW.Optionalfeedbackrolloff_tmp;

      /* Update for DiscreteTransferFcn: '<S35>/Filtered measured acceleration rate' */
      acf_rl_native_DW.Filteredmeasuredaccelerationrat =
        acf_rl_native_DW.Filteredmeasuredaccelerationr_f;

      /* Update for UnitDelay: '<S32>/Previous filtered drag ratio' */
      acf_rl_native_DW.Previousfiltereddragratio_DSTAT =
        rtb_Filteredmeasureddragratio;

      /* Update for DiscreteTransferFcn: '<S34>/Proper identified lower-link inverse' */
      acf_rl_native_DW.Properidentifiedlowerlinkinvers[2] =
        acf_rl_native_DW.Properidentifiedlowerlinkinvers[1];
      acf_rl_native_DW.Properidentifiedlowerlinkinvers[1] =
        acf_rl_native_DW.Properidentifiedlowerlinkinvers[0];
      acf_rl_native_DW.Properidentifiedlowerlinkinvers[0] =
        acf_rl_native_DW.Properidentifiedlowerlinkinve_k;

      /* Update for DiscreteTransferFcn: '<S33>/Optional structural-mode current notch' */
      acf_rl_native_DW.Optionalstructuralmodecurrentno[1] =
        acf_rl_native_DW.Optionalstructuralmodecurrentno[0];
      acf_rl_native_DW.Optionalstructuralmodecurrentno[0] =
        acf_rl_native_DW.Optionalstructuralmodecurrent_n;
    }

    tmp_1 = (rtmIsMajorTimeStep(acf_rl_native_M) &&
             acf_rl_native_M->Timing.TaskCounters.TID[1] == 0);
    if (tmp_1) {
      /* Update for UniformRandomNumber: '<S13>/Position noise' */
      acf_rl_native_DW.Positionnoise_NextOutput =
        (acf_rl_native_P.Positionnoise_Maximum -
         acf_rl_native_P.Positionnoise_Minimum) * rt_urand_Upu32_Yd_f_pw_snf
        (&acf_rl_native_DW.RandSeed) + acf_rl_native_P.Positionnoise_Minimum;
    }

    if (tmp) {
      /* Update for UnitDelay: '<S41>/Previous fast stable count' */
      acf_rl_native_DW.Previousfaststablecount_DSTATE = rtb_Capfaststablecount;

      /* Update for DiscreteTransferFcn: '<S41>/Fast normal force mean' */
      acf_rl_native_DW.Fastnormalforcemean_states =
        acf_rl_native_DW.Fastnormalforcemean_tmp;

      /* Update for DiscreteTransferFcn: '<S41>/Fast force variance' */
      acf_rl_native_DW.Fastforcevariance_states =
        acf_rl_native_DW.Fastforcevariance_tmp;

      /* Update for Delay: '<S14>/Robot command samples' */
      acf_rl_native_DW.Robotcommandsamples_DSTATE[acf_rl_native_DW.CircBufIdx_e]
        = acf_rl_native_B.Robotspeedlimits;
      if (acf_rl_native_DW.CircBufIdx_e < 999U) {
        acf_rl_native_DW.CircBufIdx_e++;
      } else {
        acf_rl_native_DW.CircBufIdx_e = 0U;
      }

      /* End of Update for Delay: '<S14>/Robot command samples' */
    }

    if (tmp_0) {
      /* Update for DiscreteTransferFcn: '<S39>/Force trend filter' */
      acf_rl_native_DW.Forcetrendfilter_states =
        (acf_rl_native_B.Robotrateforcesample -
         acf_rl_native_P.Forcetrendfilter_DenCoef[1] *
         acf_rl_native_DW.Forcetrendfilter_states) /
        acf_rl_native_P.Forcetrendfilter_DenCoef[0];

      /* Update for UnitDelay: '<S39>/Previous force trend' */
      acf_rl_native_DW.Previousforcetrend_DSTATE = rtb_Forcetrendfilter;

      /* Update for UnitDelay: '<S39>/Previous stable count' */
      acf_rl_native_DW.Previousstablecount_DSTATE = rtb_Capstablecount;
    }

    if (tmp_1) {
      /* Update for DiscreteTransferFcn: '<S42>/Native converter impulse response' */
      for (j = 0; j < 29; j++) {
        acf_rl_native_DW.Nativeconverterimpulseresponse_[29 - j] =
          acf_rl_native_DW.Nativeconverterimpulseresponse_[28 - j];
      }

      acf_rl_native_DW.Nativeconverterimpulseresponse_[0] =
        acf_rl_native_DW.Nativeconverterimpulserespons_o;

      /* End of Update for DiscreteTransferFcn: '<S42>/Native converter impulse response' */
    }

    tmp = (rtmIsMajorTimeStep(acf_rl_native_M) &&
           acf_rl_native_M->Timing.TaskCounters.TID[2] == 0);
    if (tmp) {
      /* Update for RandomNumber: '<S42>/Independent converter output noise' */
      acf_rl_native_DW.NextOutput = rt_nrand_Upu32_Yd_f_pw_snf
        (&acf_rl_native_DW.RandSeed_f) *
        acf_rl_native_P.Independentconverteroutputnoi_o +
        acf_rl_native_P.Independentconverteroutputnoise;

      /* Update for DiscreteTransferFcn: '<S42>/ADC output noise correlation' */
      acf_rl_native_DW.ADCoutputnoisecorrelation_state =
        acf_rl_native_DW.ADCoutputnoisecorrelation_tmp;

      /* Update for DiscreteTransferFcn: '<S42>/Fresh per-conversion EMA' */
      acf_rl_native_DW.FreshperconversionEMA_states =
        acf_rl_native_DW.FreshperconversionEMA_tmp;
    }

    if (tmp_1) {
      /* Update for RandomNumber: '<S16>/X raw noise' */
      acf_rl_native_DW.NextOutput_b = rt_nrand_Upu32_Yd_f_pw_snf
        (&acf_rl_native_DW.RandSeed_c) * acf_rl_native_P.Xrawnoise_StdDev +
        acf_rl_native_P.Xrawnoise_Mean;

      /* Update for UnitDelay: '<S16>/X filter memory' */
      acf_rl_native_DW.Xfiltermemory_DSTATE =
        acf_rl_native_B.Xidentifiedsensorresponse;

      /* Update for DiscreteTransferFcn: '<S43>/Native converter impulse response' */
      for (j = 0; j < 29; j++) {
        acf_rl_native_DW.Nativeconverterimpulserespons_a[29 - j] =
          acf_rl_native_DW.Nativeconverterimpulserespons_a[28 - j];
      }

      acf_rl_native_DW.Nativeconverterimpulserespons_a[0] =
        acf_rl_native_DW.Nativeconverterimpulserespon_gz;

      /* End of Update for DiscreteTransferFcn: '<S43>/Native converter impulse response' */
    }

    if (tmp) {
      /* Update for RandomNumber: '<S43>/Independent converter output noise' */
      acf_rl_native_DW.NextOutput_n = rt_nrand_Upu32_Yd_f_pw_snf
        (&acf_rl_native_DW.RandSeed_l) *
        acf_rl_native_P.Independentconverteroutputnoi_p +
        acf_rl_native_P.Independentconverteroutputnoi_k;

      /* Update for DiscreteTransferFcn: '<S43>/ADC output noise correlation' */
      acf_rl_native_DW.ADCoutputnoisecorrelation_sta_b =
        acf_rl_native_DW.ADCoutputnoisecorrelation_tmp_e;

      /* Update for DiscreteTransferFcn: '<S43>/Fresh per-conversion EMA' */
      acf_rl_native_DW.FreshperconversionEMA_states_f =
        acf_rl_native_DW.FreshperconversionEMA_tmp_k;
    }

    if (tmp_1) {
      /* Update for RandomNumber: '<S16>/Z raw noise' */
      acf_rl_native_DW.NextOutput_bf = rt_nrand_Upu32_Yd_f_pw_snf
        (&acf_rl_native_DW.RandSeed_p) * acf_rl_native_P.Zrawnoise_StdDev +
        acf_rl_native_P.Zrawnoise_Mean;

      /* Update for UnitDelay: '<S16>/Z filter memory' */
      acf_rl_native_DW.Zfiltermemory_DSTATE =
        acf_rl_native_B.Zidentifiedsensorresponse;

      /* Update for DiscreteTransferFcn: '<S29>/Native converter impulse response' */
      for (j = 0; j < 29; j++) {
        acf_rl_native_DW.Nativeconverterimpulserespons_e[29 - j] =
          acf_rl_native_DW.Nativeconverterimpulserespons_e[28 - j];
      }

      acf_rl_native_DW.Nativeconverterimpulserespons_e[0] =
        acf_rl_native_DW.Nativeconverterimpulserespon_au;

      /* End of Update for DiscreteTransferFcn: '<S29>/Native converter impulse response' */
    }

    if (tmp) {
      /* Update for DiscreteTransferFcn: '<S29>/Fresh per-conversion EMA' */
      acf_rl_native_DW.FreshperconversionEMA_states_j =
        acf_rl_native_DW.FreshperconversionEMA_tmp_h;
    }

    if (tmp_1) {
      /* Update for DiscreteStateSpace: '<S27>/Position frame matching' */
      {
        real_T xnew[1];
        xnew[0] = acf_rl_native_P.Positionframematching_A*
          acf_rl_native_DW.Positionframematching_DSTATE;
        xnew[0] += acf_rl_native_P.Positionframematching_B*
          acf_rl_native_B.Headgapcoordinatesign;
        (void) memcpy(&acf_rl_native_DW.Positionframematching_DSTATE, xnew,
                      sizeof(real_T)*1);
      }

      /* Update for DiscreteTransferFcn: '<S30>/Native converter impulse response' */
      for (j = 0; j < 29; j++) {
        acf_rl_native_DW.Nativeconverterimpulserespons_k[29 - j] =
          acf_rl_native_DW.Nativeconverterimpulserespons_k[28 - j];
      }

      acf_rl_native_DW.Nativeconverterimpulserespons_k[0] =
        acf_rl_native_DW.Nativeconverterimpulserespons_b;

      /* End of Update for DiscreteTransferFcn: '<S30>/Native converter impulse response' */
    }

    if (tmp) {
      /* Update for DiscreteTransferFcn: '<S30>/Fresh per-conversion EMA' */
      acf_rl_native_DW.FreshperconversionEMA_states_h =
        acf_rl_native_DW.FreshperconversionEMA_tmp_e;
    }

    if (tmp_1) {
      /* Update for DiscreteStateSpace: '<S27>/Robot acceleration frame matching' */
      {
        real_T xnew[1];
        xnew[0] = acf_rl_native_P.Robotaccelerationframematching_*
          acf_rl_native_DW.Robotaccelerationframematching_;
        xnew[0] += acf_rl_native_P.Robotaccelerationframematchin_k*
          acf_rl_native_B.Enablerobotaccelerationpredicti;
        (void) memcpy(&acf_rl_native_DW.Robotaccelerationframematching_, xnew,
                      sizeof(real_T)*1);
      }

      /* Update for DiscreteTransferFcn: '<S31>/Native converter impulse response' */
      for (j = 0; j < 29; j++) {
        acf_rl_native_DW.Nativeconverterimpulserespons_m[29 - j] =
          acf_rl_native_DW.Nativeconverterimpulserespons_m[28 - j];
      }

      acf_rl_native_DW.Nativeconverterimpulserespons_m[0] =
        acf_rl_native_DW.Nativeconverterimpulserespons_n;

      /* End of Update for DiscreteTransferFcn: '<S31>/Native converter impulse response' */
    }

    if (tmp) {
      /* Update for DiscreteTransferFcn: '<S31>/Fresh per-conversion EMA' */
      acf_rl_native_DW.FreshperconversionEMA_states_l =
        acf_rl_native_DW.FreshperconversionEMA_tmp_p;
    }

    if (tmp_1) {
      /* Update for DiscreteStateSpace: '<S27>/Robot velocity frame matching' */
      {
        real_T xnew[1];
        xnew[0] = acf_rl_native_P.Robotvelocityframematching_A*
          acf_rl_native_DW.Robotvelocityframematching_DSTA;
        xnew[0] += acf_rl_native_P.Robotvelocityframematching_B*
          acf_rl_native_B.Predictedvelocitysampling;
        (void) memcpy(&acf_rl_native_DW.Robotvelocityframematching_DSTA, xnew,
                      sizeof(real_T)*1);
      }

      /* Update for DiscreteTransferFcn: '<S37>/Native converter impulse response' */
      for (j = 0; j < 29; j++) {
        acf_rl_native_DW.Nativeconverterimpulserespons_h[29 - j] =
          acf_rl_native_DW.Nativeconverterimpulserespons_h[28 - j];
      }

      acf_rl_native_DW.Nativeconverterimpulserespons_h[0] =
        acf_rl_native_DW.Nativeconverterimpulserespons_p;

      /* End of Update for DiscreteTransferFcn: '<S37>/Native converter impulse response' */
    }

    if (tmp) {
      /* Update for RandomNumber: '<S37>/Independent converter output noise' */
      acf_rl_native_DW.NextOutput_j = rt_nrand_Upu32_Yd_f_pw_snf
        (&acf_rl_native_DW.RandSeed_j) *
        acf_rl_native_P.Independentconverteroutputnoi_l +
        acf_rl_native_P.Independentconverteroutputno_pm;

      /* Update for DiscreteTransferFcn: '<S37>/ADC output noise correlation' */
      acf_rl_native_DW.ADCoutputnoisecorrelation_sta_o =
        acf_rl_native_DW.ADCoutputnoisecorrelation_tmp_p;

      /* Update for DiscreteTransferFcn: '<S37>/Fresh per-conversion EMA' */
      acf_rl_native_DW.FreshperconversionEMA_states_m =
        acf_rl_native_DW.FreshperconversionEMA_tmp_ep;
    }

    if (tmp_1) {
      /* Update for UnitDelay: '<S5>/Previous filtered force' */
      acf_rl_native_DW.Previousfilteredforce_DSTATE = rtb_Lowpassfilter;

      /* Update for UniformRandomNumber: '<S5>/Uniform sensor noise' */
      acf_rl_native_DW.Uniformsensornoise_NextOutput =
        (acf_rl_native_P.Uniformsensornoise_Maximum -
         acf_rl_native_P.Uniformsensornoise_Minimum) *
        rt_urand_Upu32_Yd_f_pw_snf(&acf_rl_native_DW.RandSeed_fw) +
        acf_rl_native_P.Uniformsensornoise_Minimum;

      /* Update for DiscreteTransferFcn: '<S38>/Native converter impulse response' */
      for (j = 0; j < 29; j++) {
        acf_rl_native_DW.Nativeconverterimpulserespons_g[29 - j] =
          acf_rl_native_DW.Nativeconverterimpulserespons_g[28 - j];
      }

      acf_rl_native_DW.Nativeconverterimpulserespons_g[0] =
        acf_rl_native_DW.Nativeconverterimpulserespons_l;

      /* End of Update for DiscreteTransferFcn: '<S38>/Native converter impulse response' */
    }

    if (tmp) {
      /* Update for RandomNumber: '<S38>/Independent converter output noise' */
      acf_rl_native_DW.NextOutput_m = rt_nrand_Upu32_Yd_f_pw_snf
        (&acf_rl_native_DW.RandSeed_n) *
        acf_rl_native_P.Independentconverteroutputnoi_f +
        acf_rl_native_P.Independentconverteroutputno_pu;

      /* Update for DiscreteTransferFcn: '<S38>/ADC output noise correlation' */
      acf_rl_native_DW.ADCoutputnoisecorrelation_st_ov =
        acf_rl_native_DW.ADCoutputnoisecorrelation_tmp_n;

      /* Update for DiscreteTransferFcn: '<S38>/Fresh per-conversion EMA' */
      acf_rl_native_DW.FreshperconversionEMA_states_a =
        acf_rl_native_DW.FreshperconversionEMA_tmp_j;
    }

    if (tmp_1) {
      /* Update for UnitDelay: '<S8>/Filter memory' */
      acf_rl_native_DW.Filtermemory_DSTATE = rtb_Sensorlowpass;

      /* Update for UniformRandomNumber: '<S8>/Independent lateral noise' */
      acf_rl_native_DW.Independentlateralnoise_NextOut =
        (acf_rl_native_P.Independentlateralnoise_Maximum -
         acf_rl_native_P.Independentlateralnoise_Minimum) *
        rt_urand_Upu32_Yd_f_pw_snf(&acf_rl_native_DW.RandSeed_h) +
        acf_rl_native_P.Independentlateralnoise_Minimum;
    }
  }                                    /* end MajorTimeStep */

  if (rtmIsMajorTimeStep(acf_rl_native_M)) {
    rt_ertODEUpdateContinuousStates(&acf_rl_native_M->solverInfo);

    /* Update absolute time for base rate */
    /* The "clockTick0" counts the number of times the code of this task has
     * been executed. The absolute time is the multiplication of "clockTick0"
     * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
     * overflow during the application lifespan selected.
     * Timer of this task consists of two 32 bit unsigned integers.
     * The two integers represent the low bits Timing.clockTick0 and the high bits
     * Timing.clockTickH0. When the low bit overflows to 0, the high bits increment.
     */
    if ((++acf_rl_native_M->Timing.clockTick0)==0) {
      ++acf_rl_native_M->Timing.clockTickH0;
    }

    acf_rl_native_M->Timing.t[0] = rtsiGetSolverStopTime
      (&acf_rl_native_M->solverInfo);

    {
      /* Update absolute timer for sample time: [3.3333333333333333E-6s, 0.0s] */
      /* The "clockTick1" counts the number of times the code of this task has
       * been executed. The resolution of this integer timer is 3.3333333333333333E-6, which is the step size
       * of the task. Size of "clockTick1" ensures timer will not overflow during the
       * application lifespan selected.
       * Timer of this task consists of two 32 bit unsigned integers.
       * The two integers represent the low bits Timing.clockTick1 and the high bits
       * Timing.clockTickH1. When the low bit overflows to 0, the high bits increment.
       */
      acf_rl_native_M->Timing.clockTick1++;
      if (acf_rl_native_M->Timing.clockTick1==0) {
        acf_rl_native_M->Timing.clockTickH1++;
      }
    }

    if (rtmIsMajorTimeStep(acf_rl_native_M) &&
        acf_rl_native_M->Timing.TaskCounters.TID[4] == 0) {
      /* Update absolute timer for sample time: [0.01s, 0.0s] */
      /* The "clockTick4" counts the number of times the code of this task has
       * been executed. The resolution of this integer timer is 0.01, which is the step size
       * of the task. Size of "clockTick4" ensures timer will not overflow during the
       * application lifespan selected.
       * Timer of this task consists of two 32 bit unsigned integers.
       * The two integers represent the low bits Timing.clockTick4 and the high bits
       * Timing.clockTickH4. When the low bit overflows to 0, the high bits increment.
       */
      acf_rl_native_M->Timing.clockTick4++;
      if (acf_rl_native_M->Timing.clockTick4==0) {
        acf_rl_native_M->Timing.clockTickH4++;
      }
    }

    rate_scheduler();
  }                                    /* end MajorTimeStep */
}

/* Derivatives for root system: '<Root>' */
void acf_rl_native_derivatives(void)
{
  XDot_acf_rl_native_T *_rtXdot;
  uint32_T ri;
  _rtXdot = ((XDot_acf_rl_native_T *) acf_rl_native_M->derivs);

  /* Derivatives for Integrator: '<S7>/Predicted velocity' */
  _rtXdot->Predictedvelocity_CSTATE = acf_rl_native_B.Expectedaccelerationlimit;

  /* Derivatives for Integrator: '<S21>/Position' */
  _rtXdot->Position_CSTATE = acf_rl_native_B.Velocity_j;

  /* Derivatives for Integrator: '<S22>/Position' */
  _rtXdot->Position_CSTATE_p = acf_rl_native_B.Velocity;

  /* Derivatives for Integrator: '<S22>/Velocity' */
  _rtXdot->Velocity_CSTATE = acf_rl_native_B.Newtonacceleration_n;

  /* Derivatives for Integrator: '<S21>/Velocity' */
  _rtXdot->Velocity_CSTATE_m = acf_rl_native_B.Newtonacceleration;

  /* Derivatives for Integrator: '<S25>/Winding current' */
  _rtXdot->Windingcurrent_CSTATE = acf_rl_native_B.Inverseinductance;

  /* Derivatives for Integrator: '<S15>/Robot velocity' */
  _rtXdot->Robotvelocity_CSTATE = acf_rl_native_B.Robotaccelerationlimits;

  /* Derivatives for StateSpace: '<S42>/Continuous analog front end' */
  _rtXdot->Continuousanalogfrontend_CSTATE[0] = 0.0;
  _rtXdot->Continuousanalogfrontend_CSTATE[1] = 0.0;
  for (ri = acf_rl_native_P.Continuousanalogfrontend_A_jc[0U]; ri <
       acf_rl_native_P.Continuousanalogfrontend_A_jc[1U]; ri++) {
    _rtXdot->
      Continuousanalogfrontend_CSTATE[acf_rl_native_P.Continuousanalogfrontend_A_ir
      [ri]] += acf_rl_native_P.Continuousanalogfrontend_A_pr[ri] *
      acf_rl_native_X.Continuousanalogfrontend_CSTATE[0U];
  }

  for (ri = acf_rl_native_P.Continuousanalogfrontend_A_jc[1U]; ri <
       acf_rl_native_P.Continuousanalogfrontend_A_jc[2U]; ri++) {
    _rtXdot->
      Continuousanalogfrontend_CSTATE[acf_rl_native_P.Continuousanalogfrontend_A_ir
      [ri]] += acf_rl_native_P.Continuousanalogfrontend_A_pr[ri] *
      acf_rl_native_X.Continuousanalogfrontend_CSTATE[1U];
  }

  for (ri = acf_rl_native_P.Continuousanalogfrontend_B_jc[0U]; ri <
       acf_rl_native_P.Continuousanalogfrontend_B_jc[1U]; ri++) {
    _rtXdot->
      Continuousanalogfrontend_CSTATE[acf_rl_native_P.Continuousanalogfrontend_B_ir]
      += acf_rl_native_P.Continuousanalogfrontend_B_pr *
      acf_rl_native_B.Xnoisysensor;
  }

  /* End of Derivatives for StateSpace: '<S42>/Continuous analog front end' */

  /* Derivatives for StateSpace: '<S42>/ADC analog input filter' */
  _rtXdot->ADCanaloginputfilter_CSTATE = 0.0;
  for (ri = acf_rl_native_P.ADCanaloginputfilter_A_jc[0U]; ri <
       acf_rl_native_P.ADCanaloginputfilter_A_jc[1U]; ri++) {
    _rtXdot->ADCanaloginputfilter_CSTATE +=
      acf_rl_native_P.ADCanaloginputfilter_A_pr *
      acf_rl_native_X.ADCanaloginputfilter_CSTATE;
  }

  /* End of Derivatives for StateSpace: '<S42>/ADC analog input filter' */

  /* Derivatives for StateSpace: '<S43>/Continuous analog front end' */
  _rtXdot->Continuousanalogfrontend_CSTA_f[0] = 0.0;
  _rtXdot->Continuousanalogfrontend_CSTA_f[1] = 0.0;
  for (ri = acf_rl_native_P.Continuousanalogfrontend_A_jc_n[0U]; ri <
       acf_rl_native_P.Continuousanalogfrontend_A_jc_n[1U]; ri++) {
    _rtXdot->
      Continuousanalogfrontend_CSTA_f[acf_rl_native_P.Continuousanalogfrontend_A_ir_l
      [ri]] += acf_rl_native_P.Continuousanalogfrontend_A_pr_g[ri] *
      acf_rl_native_X.Continuousanalogfrontend_CSTA_f[0U];
  }

  for (ri = acf_rl_native_P.Continuousanalogfrontend_A_jc_n[1U]; ri <
       acf_rl_native_P.Continuousanalogfrontend_A_jc_n[2U]; ri++) {
    _rtXdot->
      Continuousanalogfrontend_CSTA_f[acf_rl_native_P.Continuousanalogfrontend_A_ir_l
      [ri]] += acf_rl_native_P.Continuousanalogfrontend_A_pr_g[ri] *
      acf_rl_native_X.Continuousanalogfrontend_CSTA_f[1U];
  }

  for (ri = acf_rl_native_P.Continuousanalogfrontend_B_jc_m[0U]; ri <
       acf_rl_native_P.Continuousanalogfrontend_B_jc_m[1U]; ri++) {
    _rtXdot->
      Continuousanalogfrontend_CSTA_f[acf_rl_native_P.Continuousanalogfrontend_B_ir_n]
      += acf_rl_native_P.Continuousanalogfrontend_B_pr_a *
      acf_rl_native_B.Znoisysensor;
  }

  /* End of Derivatives for StateSpace: '<S43>/Continuous analog front end' */

  /* Derivatives for StateSpace: '<S43>/ADC analog input filter' */
  _rtXdot->ADCanaloginputfilter_CSTATE_o = 0.0;
  for (ri = acf_rl_native_P.ADCanaloginputfilter_A_jc_i[0U]; ri <
       acf_rl_native_P.ADCanaloginputfilter_A_jc_i[1U]; ri++) {
    _rtXdot->ADCanaloginputfilter_CSTATE_o +=
      acf_rl_native_P.ADCanaloginputfilter_A_pr_g *
      acf_rl_native_X.ADCanaloginputfilter_CSTATE_o;
  }

  /* End of Derivatives for StateSpace: '<S43>/ADC analog input filter' */

  /* Derivatives for StateSpace: '<S29>/Continuous analog front end' */
  _rtXdot->Continuousanalogfrontend_CSTA_m[0] = 0.0;
  _rtXdot->Continuousanalogfrontend_CSTA_m[1] = 0.0;
  for (ri = acf_rl_native_P.Continuousanalogfrontend_A_jc_f[0U]; ri <
       acf_rl_native_P.Continuousanalogfrontend_A_jc_f[1U]; ri++) {
    _rtXdot->
      Continuousanalogfrontend_CSTA_m[acf_rl_native_P.Continuousanalogfrontend_A_ir_a
      [ri]] += acf_rl_native_P.Continuousanalogfrontend_A_pr_a[ri] *
      acf_rl_native_X.Continuousanalogfrontend_CSTA_m[0U];
  }

  for (ri = acf_rl_native_P.Continuousanalogfrontend_A_jc_f[1U]; ri <
       acf_rl_native_P.Continuousanalogfrontend_A_jc_f[2U]; ri++) {
    _rtXdot->
      Continuousanalogfrontend_CSTA_m[acf_rl_native_P.Continuousanalogfrontend_A_ir_a
      [ri]] += acf_rl_native_P.Continuousanalogfrontend_A_pr_a[ri] *
      acf_rl_native_X.Continuousanalogfrontend_CSTA_m[1U];
  }

  for (ri = acf_rl_native_P.Continuousanalogfrontend_B_jc_c[0U]; ri <
       acf_rl_native_P.Continuousanalogfrontend_B_jc_c[1U]; ri++) {
    _rtXdot->
      Continuousanalogfrontend_CSTA_m[acf_rl_native_P.Continuousanalogfrontend_B_ir_o]
      += acf_rl_native_P.Continuousanalogfrontend_B_pr_d *
      acf_rl_native_B.Headgapcoordinatesign;
  }

  /* End of Derivatives for StateSpace: '<S29>/Continuous analog front end' */

  /* Derivatives for StateSpace: '<S29>/ADC analog input filter' */
  _rtXdot->ADCanaloginputfilter_CSTATE_e = 0.0;
  for (ri = acf_rl_native_P.ADCanaloginputfilter_A_jc_f[0U]; ri <
       acf_rl_native_P.ADCanaloginputfilter_A_jc_f[1U]; ri++) {
    _rtXdot->ADCanaloginputfilter_CSTATE_e +=
      acf_rl_native_P.ADCanaloginputfilter_A_pr_o *
      acf_rl_native_X.ADCanaloginputfilter_CSTATE_e;
  }

  /* End of Derivatives for StateSpace: '<S29>/ADC analog input filter' */

  /* Derivatives for StateSpace: '<S30>/Continuous analog front end' */
  _rtXdot->Continuousanalogfrontend_CSTA_p[0] = 0.0;
  _rtXdot->Continuousanalogfrontend_CSTA_p[1] = 0.0;
  for (ri = acf_rl_native_P.Continuousanalogfrontend_A_jc_p[0U]; ri <
       acf_rl_native_P.Continuousanalogfrontend_A_jc_p[1U]; ri++) {
    _rtXdot->
      Continuousanalogfrontend_CSTA_p[acf_rl_native_P.Continuousanalogfrontend_A_ir_d
      [ri]] += acf_rl_native_P.Continuousanalogfrontend_A_pr_i[ri] *
      acf_rl_native_X.Continuousanalogfrontend_CSTA_p[0U];
  }

  for (ri = acf_rl_native_P.Continuousanalogfrontend_A_jc_p[1U]; ri <
       acf_rl_native_P.Continuousanalogfrontend_A_jc_p[2U]; ri++) {
    _rtXdot->
      Continuousanalogfrontend_CSTA_p[acf_rl_native_P.Continuousanalogfrontend_A_ir_d
      [ri]] += acf_rl_native_P.Continuousanalogfrontend_A_pr_i[ri] *
      acf_rl_native_X.Continuousanalogfrontend_CSTA_p[1U];
  }

  for (ri = acf_rl_native_P.Continuousanalogfrontend_B_jc_a[0U]; ri <
       acf_rl_native_P.Continuousanalogfrontend_B_jc_a[1U]; ri++) {
    _rtXdot->
      Continuousanalogfrontend_CSTA_p[acf_rl_native_P.Continuousanalogfrontend_B_ir_a]
      += acf_rl_native_P.Continuousanalogfrontend_B_pr_j *
      acf_rl_native_B.Enablerobotaccelerationpredicti;
  }

  /* End of Derivatives for StateSpace: '<S30>/Continuous analog front end' */

  /* Derivatives for StateSpace: '<S30>/ADC analog input filter' */
  _rtXdot->ADCanaloginputfilter_CSTATE_g = 0.0;
  for (ri = acf_rl_native_P.ADCanaloginputfilter_A_jc_ir[0U]; ri <
       acf_rl_native_P.ADCanaloginputfilter_A_jc_ir[1U]; ri++) {
    _rtXdot->ADCanaloginputfilter_CSTATE_g +=
      acf_rl_native_P.ADCanaloginputfilter_A_pr_h *
      acf_rl_native_X.ADCanaloginputfilter_CSTATE_g;
  }

  /* End of Derivatives for StateSpace: '<S30>/ADC analog input filter' */

  /* Derivatives for StateSpace: '<S31>/Continuous analog front end' */
  _rtXdot->Continuousanalogfrontend_CSTA_c[0] = 0.0;
  _rtXdot->Continuousanalogfrontend_CSTA_c[1] = 0.0;
  for (ri = acf_rl_native_P.Continuousanalogfrontend_A_j_nh[0U]; ri <
       acf_rl_native_P.Continuousanalogfrontend_A_j_nh[1U]; ri++) {
    _rtXdot->
      Continuousanalogfrontend_CSTA_c[acf_rl_native_P.Continuousanalogfrontend_A_ir_i
      [ri]] += acf_rl_native_P.Continuousanalogfrontend_A_pr_j[ri] *
      acf_rl_native_X.Continuousanalogfrontend_CSTA_c[0U];
  }

  for (ri = acf_rl_native_P.Continuousanalogfrontend_A_j_nh[1U]; ri <
       acf_rl_native_P.Continuousanalogfrontend_A_j_nh[2U]; ri++) {
    _rtXdot->
      Continuousanalogfrontend_CSTA_c[acf_rl_native_P.Continuousanalogfrontend_A_ir_i
      [ri]] += acf_rl_native_P.Continuousanalogfrontend_A_pr_j[ri] *
      acf_rl_native_X.Continuousanalogfrontend_CSTA_c[1U];
  }

  for (ri = acf_rl_native_P.Continuousanalogfrontend_B_jc_f[0U]; ri <
       acf_rl_native_P.Continuousanalogfrontend_B_jc_f[1U]; ri++) {
    _rtXdot->
      Continuousanalogfrontend_CSTA_c[acf_rl_native_P.Continuousanalogfrontend_B_ir_p]
      += acf_rl_native_P.Continuousanalogfrontend_B_p_jg *
      acf_rl_native_B.Predictedvelocitysampling;
  }

  /* End of Derivatives for StateSpace: '<S31>/Continuous analog front end' */

  /* Derivatives for StateSpace: '<S31>/ADC analog input filter' */
  _rtXdot->ADCanaloginputfilter_CSTATE_ot = 0.0;
  for (ri = acf_rl_native_P.ADCanaloginputfilter_A_jc_o[0U]; ri <
       acf_rl_native_P.ADCanaloginputfilter_A_jc_o[1U]; ri++) {
    _rtXdot->ADCanaloginputfilter_CSTATE_ot +=
      acf_rl_native_P.ADCanaloginputfilter_A_pr_m *
      acf_rl_native_X.ADCanaloginputfilter_CSTATE_ot;
  }

  /* End of Derivatives for StateSpace: '<S31>/ADC analog input filter' */

  /* Derivatives for StateSpace: '<S37>/Continuous analog front end' */
  _rtXdot->Continuousanalogfrontend_CSTA_e[0] = 0.0;
  _rtXdot->Continuousanalogfrontend_CSTA_e[1] = 0.0;
  for (ri = acf_rl_native_P.Continuousanalogfrontend_A_jc_g[0U]; ri <
       acf_rl_native_P.Continuousanalogfrontend_A_jc_g[1U]; ri++) {
    _rtXdot->
      Continuousanalogfrontend_CSTA_e[acf_rl_native_P.Continuousanalogfrontend_A_ir_g
      [ri]] += acf_rl_native_P.Continuousanalogfrontend_A_pr_n[ri] *
      acf_rl_native_X.Continuousanalogfrontend_CSTA_e[0U];
  }

  for (ri = acf_rl_native_P.Continuousanalogfrontend_A_jc_g[1U]; ri <
       acf_rl_native_P.Continuousanalogfrontend_A_jc_g[2U]; ri++) {
    _rtXdot->
      Continuousanalogfrontend_CSTA_e[acf_rl_native_P.Continuousanalogfrontend_A_ir_g
      [ri]] += acf_rl_native_P.Continuousanalogfrontend_A_pr_n[ri] *
      acf_rl_native_X.Continuousanalogfrontend_CSTA_e[1U];
  }

  for (ri = acf_rl_native_P.Continuousanalogfrontend_B_j_co[0U]; ri <
       acf_rl_native_P.Continuousanalogfrontend_B_j_co[1U]; ri++) {
    _rtXdot->
      Continuousanalogfrontend_CSTA_e[acf_rl_native_P.Continuousanalogfrontend_B_ir_l]
      += acf_rl_native_P.Continuousanalogfrontend_B_pr_b *
      acf_rl_native_B.Rawnoisyforcesample;
  }

  /* End of Derivatives for StateSpace: '<S37>/Continuous analog front end' */

  /* Derivatives for StateSpace: '<S37>/ADC analog input filter' */
  _rtXdot->ADCanaloginputfilter_CSTATE_e3 = 0.0;
  for (ri = acf_rl_native_P.ADCanaloginputfilter_A_jc_e[0U]; ri <
       acf_rl_native_P.ADCanaloginputfilter_A_jc_e[1U]; ri++) {
    _rtXdot->ADCanaloginputfilter_CSTATE_e3 +=
      acf_rl_native_P.ADCanaloginputfilter_A_pr_f *
      acf_rl_native_X.ADCanaloginputfilter_CSTATE_e3;
  }

  /* End of Derivatives for StateSpace: '<S37>/ADC analog input filter' */

  /* Derivatives for Integrator: '<S7>/Predicted relative position' */
  _rtXdot->Predictedrelativeposition_CSTAT = acf_rl_native_B.Predictedvelocity;

  /* Derivatives for Integrator: '<S15>/Robot position' */
  _rtXdot->Robotposition_CSTATE = acf_rl_native_B.Robotvelocity;

  /* Derivatives for StateSpace: '<S38>/Continuous analog front end' */
  _rtXdot->Continuousanalogfrontend_CSTA_k[0] = 0.0;
  _rtXdot->Continuousanalogfrontend_CSTA_k[1] = 0.0;
  for (ri = acf_rl_native_P.Continuousanalogfrontend_A_jc_m[0U]; ri <
       acf_rl_native_P.Continuousanalogfrontend_A_jc_m[1U]; ri++) {
    _rtXdot->
      Continuousanalogfrontend_CSTA_k[acf_rl_native_P.Continuousanalogfrontend_A_ir_m
      [ri]] += acf_rl_native_P.Continuousanalogfrontend_A_pr_p[ri] *
      acf_rl_native_X.Continuousanalogfrontend_CSTA_k[0U];
  }

  for (ri = acf_rl_native_P.Continuousanalogfrontend_A_jc_m[1U]; ri <
       acf_rl_native_P.Continuousanalogfrontend_A_jc_m[2U]; ri++) {
    _rtXdot->
      Continuousanalogfrontend_CSTA_k[acf_rl_native_P.Continuousanalogfrontend_A_ir_m
      [ri]] += acf_rl_native_P.Continuousanalogfrontend_A_pr_p[ri] *
      acf_rl_native_X.Continuousanalogfrontend_CSTA_k[1U];
  }

  for (ri = acf_rl_native_P.Continuousanalogfrontend_B_jc_g[0U]; ri <
       acf_rl_native_P.Continuousanalogfrontend_B_jc_g[1U]; ri++) {
    _rtXdot->
      Continuousanalogfrontend_CSTA_k[acf_rl_native_P.Continuousanalogfrontend_B_i_ow]
      += acf_rl_native_P.Continuousanalogfrontend_B_pr_f *
      acf_rl_native_B.Rawnoisyforcesample_l;
  }

  /* End of Derivatives for StateSpace: '<S38>/Continuous analog front end' */

  /* Derivatives for StateSpace: '<S38>/ADC analog input filter' */
  _rtXdot->ADCanaloginputfilter_CSTATE_h = 0.0;
  for (ri = acf_rl_native_P.ADCanaloginputfilter_A_jc_j[0U]; ri <
       acf_rl_native_P.ADCanaloginputfilter_A_jc_j[1U]; ri++) {
    _rtXdot->ADCanaloginputfilter_CSTATE_h +=
      acf_rl_native_P.ADCanaloginputfilter_A_pr_n *
      acf_rl_native_X.ADCanaloginputfilter_CSTATE_h;
  }

  /* End of Derivatives for StateSpace: '<S38>/ADC analog input filter' */
}

/* Model initialize function */
void acf_rl_native_initialize(void)
{
  /* Registration code */

  /* initialize real-time model */
  (void) memset((void *)acf_rl_native_M, 0,
                sizeof(RT_MODEL_acf_rl_native_T));

  {
    /* Setup solver object */
    rtsiSetSimTimeStepPtr(&acf_rl_native_M->solverInfo,
                          &acf_rl_native_M->Timing.simTimeStep);
    rtsiSetTPtr(&acf_rl_native_M->solverInfo, &rtmGetTPtr(acf_rl_native_M));
    rtsiSetStepSizePtr(&acf_rl_native_M->solverInfo,
                       &acf_rl_native_M->Timing.stepSize0);
    rtsiSetdXPtr(&acf_rl_native_M->solverInfo, &acf_rl_native_M->derivs);
    rtsiSetContStatesPtr(&acf_rl_native_M->solverInfo, (real_T **)
                         &acf_rl_native_M->contStates);
    rtsiSetNumContStatesPtr(&acf_rl_native_M->solverInfo,
      &acf_rl_native_M->Sizes.numContStates);
    rtsiSetNumPeriodicContStatesPtr(&acf_rl_native_M->solverInfo,
      &acf_rl_native_M->Sizes.numPeriodicContStates);
    rtsiSetPeriodicContStateIndicesPtr(&acf_rl_native_M->solverInfo,
      &acf_rl_native_M->periodicContStateIndices);
    rtsiSetPeriodicContStateRangesPtr(&acf_rl_native_M->solverInfo,
      &acf_rl_native_M->periodicContStateRanges);
    rtsiSetContStateDisabledPtr(&acf_rl_native_M->solverInfo, (boolean_T**)
      &acf_rl_native_M->contStateDisabled);
    rtsiSetErrorStatusPtr(&acf_rl_native_M->solverInfo, (&rtmGetErrorStatus
      (acf_rl_native_M)));
    rtsiSetRTModelPtr(&acf_rl_native_M->solverInfo, acf_rl_native_M);
  }

  rtsiSetSimTimeStep(&acf_rl_native_M->solverInfo, MAJOR_TIME_STEP);
  rtsiSetIsMinorTimeStepWithModeChange(&acf_rl_native_M->solverInfo, false);
  rtsiSetIsContModeFrozen(&acf_rl_native_M->solverInfo, false);
  acf_rl_native_M->intgData.y = acf_rl_native_M->odeY;
  acf_rl_native_M->intgData.f[0] = acf_rl_native_M->odeF[0];
  acf_rl_native_M->intgData.f[1] = acf_rl_native_M->odeF[1];
  acf_rl_native_M->intgData.f[2] = acf_rl_native_M->odeF[2];
  acf_rl_native_M->intgData.f[3] = acf_rl_native_M->odeF[3];
  acf_rl_native_M->contStates = ((X_acf_rl_native_T *) &acf_rl_native_X);
  acf_rl_native_M->contStateDisabled = ((XDis_acf_rl_native_T *)
    &acf_rl_native_XDis);
  acf_rl_native_M->Timing.tStart = (0.0);
  rtsiSetSolverData(&acf_rl_native_M->solverInfo, (void *)
                    &acf_rl_native_M->intgData);
  rtsiSetSolverName(&acf_rl_native_M->solverInfo,"ode4");
  rtmSetTPtr(acf_rl_native_M, &acf_rl_native_M->Timing.tArray[0]);
  acf_rl_native_M->Timing.stepSize0 = 3.3333333333333333E-6;

  /* block I/O */
  (void) memset(((void *) &acf_rl_native_B), 0,
                sizeof(B_acf_rl_native_T));

  /* states (continuous) */
  {
    (void) memset((void *)&acf_rl_native_X, 0,
                  sizeof(X_acf_rl_native_T));
  }

  /* disabled states */
  {
    (void) memset((void *)&acf_rl_native_XDis, 0,
                  sizeof(XDis_acf_rl_native_T));
  }

  /* states (dwork) */
  (void) memset((void *)&acf_rl_native_DW, 0,
                sizeof(DW_acf_rl_native_T));

  /* external inputs */
  (void)memset(&acf_rl_native_U, 0, sizeof(ExtU_acf_rl_native_T));

  /* external outputs */
  (void)memset(&acf_rl_native_Y, 0, sizeof(ExtY_acf_rl_native_T));

  {
    real_T tmp;

    /* Start for TransportDelay: '<S5>/Sensor sample delay' incorporates:
     *  TransportDelay: '<S16>/X known channel delay'
     *  TransportDelay: '<S16>/Z known channel delay'
     *  TransportDelay: '<S27>/Position ADC delivery pipeline'
     *  TransportDelay: '<S27>/Robot acceleration frame matching delay'
     *  TransportDelay: '<S27>/Robot velocity frame matching delay'
     *  TransportDelay: '<S8>/Sensor sample delay'
     */
    acf_rl_native_DW.Sensorsampledelay_RWORK[0] = 0.0;
    acf_rl_native_DW.Sensorsampledelay_IWORK[0] = 0;
    acf_rl_native_DW.Sensorsampledelay_IWORK[1] = 0;
    acf_rl_native_DW.Sensorsampledelay_IWORK[2] = 0;
    acf_rl_native_DW.Sensorsampledelay_IWORK[3] = 1024;
    acf_rl_native_DW.Sensorsampledelay_PWORK[0] =
      &acf_rl_native_DW.Sensorsampledelay_RWORK[1];
    ((real_T *)acf_rl_native_DW.Sensorsampledelay_PWORK[0])[0] =
      acf_rl_native_P.Sensorsampledelay_InitOutput;
    tmp = acf_rl_native_M->Timing.t[0];
    ((real_T *)acf_rl_native_DW.Sensorsampledelay_PWORK[0])[1024] = tmp;

    /* Start for TransportDelay: '<S16>/Z known channel delay' */
    acf_rl_native_DW.Zknownchanneldelay_RWORK[0] = 0.0;
    acf_rl_native_DW.Zknownchanneldelay_IWORK[0] = 0;
    acf_rl_native_DW.Zknownchanneldelay_IWORK[1] = 0;
    acf_rl_native_DW.Zknownchanneldelay_IWORK[2] = 0;
    acf_rl_native_DW.Zknownchanneldelay_IWORK[3] = 1024;
    acf_rl_native_DW.Zknownchanneldelay_PWORK[0] =
      &acf_rl_native_DW.Zknownchanneldelay_RWORK[1];
    ((real_T *)acf_rl_native_DW.Zknownchanneldelay_PWORK[0])[0] =
      acf_rl_native_P.Zknownchanneldelay_InitOutput;
    ((real_T *)acf_rl_native_DW.Zknownchanneldelay_PWORK[0])[1024] = tmp;

    /* Start for TransportDelay: '<S8>/Sensor sample delay' */
    acf_rl_native_DW.Sensorsampledelay_RWORK_k[0] = 0.0;
    acf_rl_native_DW.Sensorsampledelay_IWORK_i[0] = 0;
    acf_rl_native_DW.Sensorsampledelay_IWORK_i[1] = 0;
    acf_rl_native_DW.Sensorsampledelay_IWORK_i[2] = 0;
    acf_rl_native_DW.Sensorsampledelay_IWORK_i[3] = 1024;
    acf_rl_native_DW.Sensorsampledelay_PWORK_f[0] =
      &acf_rl_native_DW.Sensorsampledelay_RWORK_k[1];
    ((real_T *)acf_rl_native_DW.Sensorsampledelay_PWORK_f[0])[0] =
      acf_rl_native_P.Sensorsampledelay_InitOutput_h;
    ((real_T *)acf_rl_native_DW.Sensorsampledelay_PWORK_f[0])[1024] = tmp;

    /* Start for TransportDelay: '<S16>/X known channel delay' */
    acf_rl_native_DW.Xknownchanneldelay_RWORK[0] = 0.0;
    acf_rl_native_DW.Xknownchanneldelay_IWORK[0] = 0;
    acf_rl_native_DW.Xknownchanneldelay_IWORK[1] = 0;
    acf_rl_native_DW.Xknownchanneldelay_IWORK[2] = 0;
    acf_rl_native_DW.Xknownchanneldelay_IWORK[3] = 1024;
    acf_rl_native_DW.Xknownchanneldelay_PWORK[0] =
      &acf_rl_native_DW.Xknownchanneldelay_RWORK[1];
    ((real_T *)acf_rl_native_DW.Xknownchanneldelay_PWORK[0])[0] =
      acf_rl_native_P.Xknownchanneldelay_InitOutput;
    ((real_T *)acf_rl_native_DW.Xknownchanneldelay_PWORK[0])[1024] = tmp;

    /* Start for TransportDelay: '<S27>/Robot velocity frame matching delay' */
    acf_rl_native_DW.Robotvelocityframematchingdelay[0] = 0.0;
    acf_rl_native_DW.Robotvelocityframematchingdel_c[0] = 0;
    acf_rl_native_DW.Robotvelocityframematchingdel_c[1] = 0;
    acf_rl_native_DW.Robotvelocityframematchingdel_c[2] = 0;
    acf_rl_native_DW.Robotvelocityframematchingdel_c[3] = 1024;
    acf_rl_native_DW.Robotvelocityframematchingdel_l[0] =
      &acf_rl_native_DW.Robotvelocityframematchingdelay[1];
    ((real_T *)acf_rl_native_DW.Robotvelocityframematchingdel_l[0])[0] =
      acf_rl_native_P.Robotvelocityframematchingdel_j;
    ((real_T *)acf_rl_native_DW.Robotvelocityframematchingdel_l[0])[1024] = tmp;

    /* Start for TransportDelay: '<S27>/Position ADC delivery pipeline' */
    acf_rl_native_DW.PositionADCdeliverypipeline_RWO[0] = 0.0;
    acf_rl_native_DW.PositionADCdeliverypipeline_IWO[0] = 0;
    acf_rl_native_DW.PositionADCdeliverypipeline_IWO[1] = 0;
    acf_rl_native_DW.PositionADCdeliverypipeline_IWO[2] = 0;
    acf_rl_native_DW.PositionADCdeliverypipeline_IWO[3] = 1024;
    acf_rl_native_DW.PositionADCdeliverypipeline_PWO[0] =
      &acf_rl_native_DW.PositionADCdeliverypipeline_RWO[1];
    ((real_T *)acf_rl_native_DW.PositionADCdeliverypipeline_PWO[0])[0] =
      acf_rl_native_P.PositionADCdeliverypipeline_Ini;
    ((real_T *)acf_rl_native_DW.PositionADCdeliverypipeline_PWO[0])[1024] = tmp;

    /* Start for TransportDelay: '<S27>/Robot acceleration frame matching delay' */
    acf_rl_native_DW.Robotaccelerationframematchingd[0] = 0.0;
    acf_rl_native_DW.Robotaccelerationframematchin_n[0] = 0;
    acf_rl_native_DW.Robotaccelerationframematchin_n[1] = 0;
    acf_rl_native_DW.Robotaccelerationframematchin_n[2] = 0;
    acf_rl_native_DW.Robotaccelerationframematchin_n[3] = 1024;
    acf_rl_native_DW.Robotaccelerationframematchin_h[0] =
      &acf_rl_native_DW.Robotaccelerationframematchingd[1];
    ((real_T *)acf_rl_native_DW.Robotaccelerationframematchin_h[0])[0] =
      acf_rl_native_P.Robotaccelerationframematchin_f;
    ((real_T *)acf_rl_native_DW.Robotaccelerationframematchin_h[0])[1024] = tmp;
  }

  {
    real_T tmp;
    int32_T i;
    uint32_T r;
    uint32_T t;
    uint32_T tseed;

    /* InitializeConditions for TransportDelay: '<S5>/Sensor sample delay' incorporates:
     *  TransportDelay: '<S16>/X known channel delay'
     *  TransportDelay: '<S16>/Z known channel delay'
     *  TransportDelay: '<S27>/Position ADC delivery pipeline'
     *  TransportDelay: '<S27>/Robot acceleration frame matching delay'
     *  TransportDelay: '<S27>/Robot velocity frame matching delay'
     *  TransportDelay: '<S8>/Sensor sample delay'
     */
    acf_rl_native_DW.Sensorsampledelay_IWORK[0] = 0;
    acf_rl_native_DW.Sensorsampledelay_IWORK[1] = 0;
    acf_rl_native_DW.Sensorsampledelay_IWORK[2] = 0;
    ((real_T *)acf_rl_native_DW.Sensorsampledelay_PWORK[0])[0] = 50.0;
    tmp = acf_rl_native_M->Timing.t[0];
    ((real_T *)acf_rl_native_DW.Sensorsampledelay_PWORK[0])
      [acf_rl_native_DW.Sensorsampledelay_IWORK[3]] = tmp;

    /* InitializeConditions for TransportDelay: '<S16>/Z known channel delay' */
    acf_rl_native_DW.Zknownchanneldelay_IWORK[0] = 0;
    acf_rl_native_DW.Zknownchanneldelay_IWORK[1] = 0;
    acf_rl_native_DW.Zknownchanneldelay_IWORK[2] = 0;
    ((real_T *)acf_rl_native_DW.Zknownchanneldelay_PWORK[0])[0] = 0.0;
    ((real_T *)acf_rl_native_DW.Zknownchanneldelay_PWORK[0])
      [acf_rl_native_DW.Zknownchanneldelay_IWORK[3]] = tmp;

    /* InitializeConditions for TransportDelay: '<S8>/Sensor sample delay' */
    acf_rl_native_DW.Sensorsampledelay_IWORK_i[0] = 0;
    acf_rl_native_DW.Sensorsampledelay_IWORK_i[1] = 0;
    acf_rl_native_DW.Sensorsampledelay_IWORK_i[2] = 0;
    ((real_T *)acf_rl_native_DW.Sensorsampledelay_PWORK_f[0])[0] = -15.0;
    ((real_T *)acf_rl_native_DW.Sensorsampledelay_PWORK_f[0])
      [acf_rl_native_DW.Sensorsampledelay_IWORK_i[3]] = tmp;

    /* InitializeConditions for TransportDelay: '<S16>/X known channel delay' */
    acf_rl_native_DW.Xknownchanneldelay_IWORK[0] = 0;
    acf_rl_native_DW.Xknownchanneldelay_IWORK[1] = 0;
    acf_rl_native_DW.Xknownchanneldelay_IWORK[2] = 0;
    ((real_T *)acf_rl_native_DW.Xknownchanneldelay_PWORK[0])[0] = 0.0;
    ((real_T *)acf_rl_native_DW.Xknownchanneldelay_PWORK[0])
      [acf_rl_native_DW.Xknownchanneldelay_IWORK[3]] = tmp;

    /* InitializeConditions for TransportDelay: '<S27>/Robot velocity frame matching delay' */
    acf_rl_native_DW.Robotvelocityframematchingdel_c[0] = 0;
    acf_rl_native_DW.Robotvelocityframematchingdel_c[1] = 0;
    acf_rl_native_DW.Robotvelocityframematchingdel_c[2] = 0;
    ((real_T *)acf_rl_native_DW.Robotvelocityframematchingdel_l[0])[0] = 0.0;
    ((real_T *)acf_rl_native_DW.Robotvelocityframematchingdel_l[0])
      [acf_rl_native_DW.Robotvelocityframematchingdel_c[3]] = tmp;

    /* InitializeConditions for TransportDelay: '<S27>/Position ADC delivery pipeline' */
    acf_rl_native_DW.PositionADCdeliverypipeline_IWO[0] = 0;
    acf_rl_native_DW.PositionADCdeliverypipeline_IWO[1] = 0;
    acf_rl_native_DW.PositionADCdeliverypipeline_IWO[2] = 0;
    ((real_T *)acf_rl_native_DW.PositionADCdeliverypipeline_PWO[0])[0] = 0.0;
    ((real_T *)acf_rl_native_DW.PositionADCdeliverypipeline_PWO[0])
      [acf_rl_native_DW.PositionADCdeliverypipeline_IWO[3]] = tmp;

    /* InitializeConditions for TransportDelay: '<S27>/Robot acceleration frame matching delay' */
    acf_rl_native_DW.Robotaccelerationframematchin_n[0] = 0;
    acf_rl_native_DW.Robotaccelerationframematchin_n[1] = 0;
    acf_rl_native_DW.Robotaccelerationframematchin_n[2] = 0;
    ((real_T *)acf_rl_native_DW.Robotaccelerationframematchin_h[0])[0] = 0.0;
    ((real_T *)acf_rl_native_DW.Robotaccelerationframematchin_h[0])
      [acf_rl_native_DW.Robotaccelerationframematchin_n[3]] = tmp;

    /* InitializeConditions for DiscreteStateSpace: '<S27>/Head gap and velocity observer' */
    acf_rl_native_DW.Headgapandvelocityobserver_DSTA[0] =
      acf_rl_native_P.Headgapandvelocityobserver_Init[0];
    acf_rl_native_DW.Headgapandvelocityobserver_DSTA[1] =
      acf_rl_native_P.Headgapandvelocityobserver_Init[1];

    /* InitializeConditions for Integrator: '<S7>/Predicted velocity' */
    acf_rl_native_X.Predictedvelocity_CSTATE =
      acf_rl_native_P.Predictedvelocity_IC;

    /* InitializeConditions for Delay: '<S7>/Expected command delay' */
    for (i = 0; i < 12; i++) {
      acf_rl_native_DW.Expectedcommanddelay_DSTATE[i] =
        acf_rl_native_P.Expectedcommanddelay_InitialCon;
    }

    acf_rl_native_DW.CircBufIdx = 0U;

    /* End of InitializeConditions for Delay: '<S7>/Expected command delay' */

    /* InitializeConditions for DiscreteIntegrator: '<S36>/Normal force error integral' */
    acf_rl_native_DW.Normalforceerrorintegral_DSTATE =
      acf_rl_native_P.Normalforceerrorintegral_IC;

    /* InitializeConditions for UnitDelay: '<S36>/Normal error previous sample' */
    acf_rl_native_DW.Normalerrorprevioussample_DSTAT =
      acf_rl_native_P.Normalerrorprevioussample_Initi;

    /* InitializeConditions for UnitDelay: '<S36>/Normal error derivative memory' */
    acf_rl_native_DW.Normalerrorderivativememory_DST =
      acf_rl_native_P.Normalerrorderivativememory_Ini;

    /* InitializeConditions for DiscreteTransferFcn: '<S36>/Optional feedback rolloff' */
    acf_rl_native_DW.Optionalfeedbackrolloff_states[0] =
      acf_rl_native_P.Optionalfeedbackrolloff_Initial[0];
    acf_rl_native_DW.Optionalfeedbackrolloff_states[1] =
      acf_rl_native_P.Optionalfeedbackrolloff_Initial[1];

    /* InitializeConditions for DiscreteTransferFcn: '<S35>/Filtered measured acceleration rate' */
    acf_rl_native_DW.Filteredmeasuredaccelerationrat =
      acf_rl_native_P.Filteredmeasuredaccelerationr_g;

    /* InitializeConditions for UnitDelay: '<S32>/Previous filtered drag ratio' */
    acf_rl_native_DW.Previousfiltereddragratio_DSTAT =
      acf_rl_native_P.Previousfiltereddragratio_Initi;

    /* InitializeConditions for DiscreteTransferFcn: '<S34>/Proper identified lower-link inverse' */
    acf_rl_native_DW.Properidentifiedlowerlinkinvers[0] =
      acf_rl_native_P.Properidentifiedlowerlinkinve_a[0];
    acf_rl_native_DW.Properidentifiedlowerlinkinvers[1] =
      acf_rl_native_P.Properidentifiedlowerlinkinve_a[1];
    acf_rl_native_DW.Properidentifiedlowerlinkinvers[2] =
      acf_rl_native_P.Properidentifiedlowerlinkinve_a[2];

    /* InitializeConditions for DiscreteTransferFcn: '<S33>/Optional structural-mode current notch' */
    acf_rl_native_DW.Optionalstructuralmodecurrentno[0] =
      acf_rl_native_P.Optionalstructuralmodecurrent_i[0];
    acf_rl_native_DW.Optionalstructuralmodecurrentno[1] =
      acf_rl_native_P.Optionalstructuralmodecurrent_i[1];

    /* InitializeConditions for Integrator: '<S21>/Position' */
    acf_rl_native_X.Position_CSTATE = acf_rl_native_P.Position_IC;

    /* InitializeConditions for Integrator: '<S22>/Position' */
    acf_rl_native_X.Position_CSTATE_p = acf_rl_native_P.Position_IC_n;

    /* InitializeConditions for UniformRandomNumber: '<S13>/Position noise' */
    tmp = floor(acf_rl_native_P.Positionnoise_Seed);
    if (rtIsNaN(tmp) || rtIsInf(tmp)) {
      tmp = 0.0;
    } else {
      tmp = fmod(tmp, 4.294967296E+9);
    }

    if (tmp < 0.0) {
      tseed = (uint32_T)-(int32_T)(uint32_T)-tmp;
    } else {
      tseed = (uint32_T)tmp;
    }

    r = tseed >> 16U;
    t = tseed & 32768U;
    tseed = ((((tseed - (r << 16U)) + t) << 16U) + t) + r;
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else if (tseed > 2147483646U) {
      tseed = 2147483646U;
    }

    acf_rl_native_DW.RandSeed = tseed;
    acf_rl_native_DW.Positionnoise_NextOutput =
      (acf_rl_native_P.Positionnoise_Maximum -
       acf_rl_native_P.Positionnoise_Minimum) * rt_urand_Upu32_Yd_f_pw_snf
      (&acf_rl_native_DW.RandSeed) + acf_rl_native_P.Positionnoise_Minimum;

    /* End of InitializeConditions for UniformRandomNumber: '<S13>/Position noise' */

    /* InitializeConditions for UnitDelay: '<S41>/Previous fast stable count' */
    acf_rl_native_DW.Previousfaststablecount_DSTATE =
      acf_rl_native_P.Previousfaststablecount_Initial;

    /* InitializeConditions for DiscreteTransferFcn: '<S39>/Force trend filter' */
    acf_rl_native_DW.Forcetrendfilter_states =
      acf_rl_native_P.Forcetrendfilter_InitialStates;

    /* InitializeConditions for UnitDelay: '<S39>/Previous force trend' */
    acf_rl_native_DW.Previousforcetrend_DSTATE =
      acf_rl_native_P.Previousforcetrend_InitialCondi;

    /* InitializeConditions for DiscreteTransferFcn: '<S41>/Fast normal force mean' */
    acf_rl_native_DW.Fastnormalforcemean_states =
      acf_rl_native_P.Fastnormalforcemean_InitialStat;

    /* InitializeConditions for DiscreteTransferFcn: '<S41>/Fast force variance' */
    acf_rl_native_DW.Fastforcevariance_states =
      acf_rl_native_P.Fastforcevariance_InitialStates;

    /* InitializeConditions for UnitDelay: '<S39>/Previous stable count' */
    acf_rl_native_DW.Previousstablecount_DSTATE =
      acf_rl_native_P.Previousstablecount_InitialCond;

    /* InitializeConditions for Integrator: '<S22>/Velocity' */
    acf_rl_native_X.Velocity_CSTATE = acf_rl_native_P.Velocity_IC;

    /* InitializeConditions for Integrator: '<S21>/Velocity' */
    acf_rl_native_X.Velocity_CSTATE_m = acf_rl_native_P.Velocity_IC_j;

    /* InitializeConditions for Integrator: '<S25>/Winding current' */
    acf_rl_native_X.Windingcurrent_CSTATE = acf_rl_native_P.Windingcurrent_IC;

    /* InitializeConditions for Delay: '<S14>/Robot command samples' */
    for (i = 0; i < 1000; i++) {
      acf_rl_native_DW.Robotcommandsamples_DSTATE[i] =
        acf_rl_native_P.Robotcommandsamples_InitialCond;
    }

    acf_rl_native_DW.CircBufIdx_e = 0U;

    /* End of InitializeConditions for Delay: '<S14>/Robot command samples' */

    /* InitializeConditions for Integrator: '<S15>/Robot velocity' */
    acf_rl_native_X.Robotvelocity_CSTATE = acf_rl_native_P.Robotvelocity_IC;

    /* InitializeConditions for StateSpace: '<S42>/Continuous analog front end' */
    acf_rl_native_X.Continuousanalogfrontend_CSTATE[0] =
      acf_rl_native_P.Continuousanalogfrontend_Initia[0];
    acf_rl_native_X.Continuousanalogfrontend_CSTATE[1] =
      acf_rl_native_P.Continuousanalogfrontend_Initia[1];

    /* InitializeConditions for StateSpace: '<S42>/ADC analog input filter' */
    acf_rl_native_X.ADCanaloginputfilter_CSTATE =
      acf_rl_native_P.ADCanaloginputfilter_InitialCon;

    /* InitializeConditions for DiscreteTransferFcn: '<S42>/Native converter impulse response' */
    memcpy(&acf_rl_native_DW.Nativeconverterimpulseresponse_[0],
           &acf_rl_native_P.Nativeconverterimpulserespons_h[0], 30U * sizeof
           (real_T));

    /* InitializeConditions for RandomNumber: '<S42>/Independent converter output noise' */
    tmp = floor(acf_rl_native_P.Independentconverteroutputnoi_e);
    if (rtIsNaN(tmp) || rtIsInf(tmp)) {
      tmp = 0.0;
    } else {
      tmp = fmod(tmp, 4.294967296E+9);
    }

    if (tmp < 0.0) {
      tseed = (uint32_T)-(int32_T)(uint32_T)-tmp;
    } else {
      tseed = (uint32_T)tmp;
    }

    r = tseed >> 16U;
    t = tseed & 32768U;
    tseed = ((((tseed - (r << 16U)) + t) << 16U) + t) + r;
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else if (tseed > 2147483646U) {
      tseed = 2147483646U;
    }

    acf_rl_native_DW.RandSeed_f = tseed;
    acf_rl_native_DW.NextOutput = rt_nrand_Upu32_Yd_f_pw_snf
      (&acf_rl_native_DW.RandSeed_f) *
      acf_rl_native_P.Independentconverteroutputnoi_o +
      acf_rl_native_P.Independentconverteroutputnoise;

    /* End of InitializeConditions for RandomNumber: '<S42>/Independent converter output noise' */

    /* InitializeConditions for DiscreteTransferFcn: '<S42>/ADC output noise correlation' */
    acf_rl_native_DW.ADCoutputnoisecorrelation_state =
      acf_rl_native_P.ADCoutputnoisecorrelation_Initi;

    /* InitializeConditions for DiscreteTransferFcn: '<S42>/Fresh per-conversion EMA' */
    acf_rl_native_DW.FreshperconversionEMA_states =
      acf_rl_native_P.FreshperconversionEMA_InitialSt;

    /* InitializeConditions for RandomNumber: '<S16>/X raw noise' */
    tmp = floor(acf_rl_native_P.Xrawnoise_Seed);
    if (rtIsNaN(tmp) || rtIsInf(tmp)) {
      tmp = 0.0;
    } else {
      tmp = fmod(tmp, 4.294967296E+9);
    }

    if (tmp < 0.0) {
      tseed = (uint32_T)-(int32_T)(uint32_T)-tmp;
    } else {
      tseed = (uint32_T)tmp;
    }

    r = tseed >> 16U;
    t = tseed & 32768U;
    tseed = ((((tseed - (r << 16U)) + t) << 16U) + t) + r;
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else if (tseed > 2147483646U) {
      tseed = 2147483646U;
    }

    acf_rl_native_DW.RandSeed_c = tseed;
    acf_rl_native_DW.NextOutput_b = rt_nrand_Upu32_Yd_f_pw_snf
      (&acf_rl_native_DW.RandSeed_c) * acf_rl_native_P.Xrawnoise_StdDev +
      acf_rl_native_P.Xrawnoise_Mean;

    /* End of InitializeConditions for RandomNumber: '<S16>/X raw noise' */

    /* InitializeConditions for UnitDelay: '<S16>/X filter memory' */
    acf_rl_native_DW.Xfiltermemory_DSTATE =
      acf_rl_native_P.Xfiltermemory_InitialCondition;

    /* InitializeConditions for StateSpace: '<S43>/Continuous analog front end' */
    acf_rl_native_X.Continuousanalogfrontend_CSTA_f[0] =
      acf_rl_native_P.Continuousanalogfrontend_Init_k[0];
    acf_rl_native_X.Continuousanalogfrontend_CSTA_f[1] =
      acf_rl_native_P.Continuousanalogfrontend_Init_k[1];

    /* InitializeConditions for StateSpace: '<S43>/ADC analog input filter' */
    acf_rl_native_X.ADCanaloginputfilter_CSTATE_o =
      acf_rl_native_P.ADCanaloginputfilter_InitialC_c;

    /* InitializeConditions for DiscreteTransferFcn: '<S43>/Native converter impulse response' */
    memcpy(&acf_rl_native_DW.Nativeconverterimpulserespons_a[0],
           &acf_rl_native_P.Nativeconverterimpulserespon_dz[0], 30U * sizeof
           (real_T));

    /* InitializeConditions for RandomNumber: '<S43>/Independent converter output noise' */
    tmp = floor(acf_rl_native_P.Independentconverteroutputnoi_a);
    if (rtIsNaN(tmp) || rtIsInf(tmp)) {
      tmp = 0.0;
    } else {
      tmp = fmod(tmp, 4.294967296E+9);
    }

    if (tmp < 0.0) {
      tseed = (uint32_T)-(int32_T)(uint32_T)-tmp;
    } else {
      tseed = (uint32_T)tmp;
    }

    r = tseed >> 16U;
    t = tseed & 32768U;
    tseed = ((((tseed - (r << 16U)) + t) << 16U) + t) + r;
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else if (tseed > 2147483646U) {
      tseed = 2147483646U;
    }

    acf_rl_native_DW.RandSeed_l = tseed;
    acf_rl_native_DW.NextOutput_n = rt_nrand_Upu32_Yd_f_pw_snf
      (&acf_rl_native_DW.RandSeed_l) *
      acf_rl_native_P.Independentconverteroutputnoi_p +
      acf_rl_native_P.Independentconverteroutputnoi_k;

    /* End of InitializeConditions for RandomNumber: '<S43>/Independent converter output noise' */

    /* InitializeConditions for DiscreteTransferFcn: '<S43>/ADC output noise correlation' */
    acf_rl_native_DW.ADCoutputnoisecorrelation_sta_b =
      acf_rl_native_P.ADCoutputnoisecorrelation_Ini_f;

    /* InitializeConditions for DiscreteTransferFcn: '<S43>/Fresh per-conversion EMA' */
    acf_rl_native_DW.FreshperconversionEMA_states_f =
      acf_rl_native_P.FreshperconversionEMA_Initial_g;

    /* InitializeConditions for RandomNumber: '<S16>/Z raw noise' */
    tmp = floor(acf_rl_native_P.Zrawnoise_Seed);
    if (rtIsNaN(tmp) || rtIsInf(tmp)) {
      tmp = 0.0;
    } else {
      tmp = fmod(tmp, 4.294967296E+9);
    }

    if (tmp < 0.0) {
      tseed = (uint32_T)-(int32_T)(uint32_T)-tmp;
    } else {
      tseed = (uint32_T)tmp;
    }

    r = tseed >> 16U;
    t = tseed & 32768U;
    tseed = ((((tseed - (r << 16U)) + t) << 16U) + t) + r;
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else if (tseed > 2147483646U) {
      tseed = 2147483646U;
    }

    acf_rl_native_DW.RandSeed_p = tseed;
    acf_rl_native_DW.NextOutput_bf = rt_nrand_Upu32_Yd_f_pw_snf
      (&acf_rl_native_DW.RandSeed_p) * acf_rl_native_P.Zrawnoise_StdDev +
      acf_rl_native_P.Zrawnoise_Mean;

    /* End of InitializeConditions for RandomNumber: '<S16>/Z raw noise' */

    /* InitializeConditions for UnitDelay: '<S16>/Z filter memory' */
    acf_rl_native_DW.Zfiltermemory_DSTATE =
      acf_rl_native_P.Zfiltermemory_InitialCondition;

    /* InitializeConditions for StateSpace: '<S29>/Continuous analog front end' */
    acf_rl_native_X.Continuousanalogfrontend_CSTA_m[0] =
      acf_rl_native_P.Continuousanalogfrontend_Init_j[0];
    acf_rl_native_X.Continuousanalogfrontend_CSTA_m[1] =
      acf_rl_native_P.Continuousanalogfrontend_Init_j[1];

    /* InitializeConditions for StateSpace: '<S29>/ADC analog input filter' */
    acf_rl_native_X.ADCanaloginputfilter_CSTATE_e =
      acf_rl_native_P.ADCanaloginputfilter_InitialC_m;

    /* InitializeConditions for DiscreteTransferFcn: '<S29>/Native converter impulse response' */
    memcpy(&acf_rl_native_DW.Nativeconverterimpulserespons_e[0],
           &acf_rl_native_P.Nativeconverterimpulserespon_kc[0], 30U * sizeof
           (real_T));

    /* InitializeConditions for DiscreteTransferFcn: '<S29>/Fresh per-conversion EMA' */
    acf_rl_native_DW.FreshperconversionEMA_states_j =
      acf_rl_native_P.FreshperconversionEMA_Initial_h;

    /* InitializeConditions for DiscreteStateSpace: '<S27>/Position frame matching' */
    acf_rl_native_DW.Positionframematching_DSTATE =
      acf_rl_native_P.Positionframematching_InitialCo;

    /* InitializeConditions for StateSpace: '<S30>/Continuous analog front end' */
    acf_rl_native_X.Continuousanalogfrontend_CSTA_p[0] =
      acf_rl_native_P.Continuousanalogfrontend_Init_o[0];
    acf_rl_native_X.Continuousanalogfrontend_CSTA_p[1] =
      acf_rl_native_P.Continuousanalogfrontend_Init_o[1];

    /* InitializeConditions for StateSpace: '<S30>/ADC analog input filter' */
    acf_rl_native_X.ADCanaloginputfilter_CSTATE_g =
      acf_rl_native_P.ADCanaloginputfilter_InitialC_p;

    /* InitializeConditions for DiscreteTransferFcn: '<S30>/Native converter impulse response' */
    memcpy(&acf_rl_native_DW.Nativeconverterimpulserespons_k[0],
           &acf_rl_native_P.Nativeconverterimpulserespons_e[0], 30U * sizeof
           (real_T));

    /* InitializeConditions for DiscreteTransferFcn: '<S30>/Fresh per-conversion EMA' */
    acf_rl_native_DW.FreshperconversionEMA_states_h =
      acf_rl_native_P.FreshperconversionEMA_Initia_h0;

    /* InitializeConditions for DiscreteStateSpace: '<S27>/Robot acceleration frame matching' */
    acf_rl_native_DW.Robotaccelerationframematching_ =
      acf_rl_native_P.Robotaccelerationframematchi_ka;

    /* InitializeConditions for StateSpace: '<S31>/Continuous analog front end' */
    acf_rl_native_X.Continuousanalogfrontend_CSTA_c[0] =
      acf_rl_native_P.Continuousanalogfrontend_Init_l[0];
    acf_rl_native_X.Continuousanalogfrontend_CSTA_c[1] =
      acf_rl_native_P.Continuousanalogfrontend_Init_l[1];

    /* InitializeConditions for StateSpace: '<S31>/ADC analog input filter' */
    acf_rl_native_X.ADCanaloginputfilter_CSTATE_ot =
      acf_rl_native_P.ADCanaloginputfilter_InitialC_k;

    /* InitializeConditions for DiscreteTransferFcn: '<S31>/Native converter impulse response' */
    memcpy(&acf_rl_native_DW.Nativeconverterimpulserespons_m[0],
           &acf_rl_native_P.Nativeconverterimpulserespon_ew[0], 30U * sizeof
           (real_T));

    /* InitializeConditions for DiscreteTransferFcn: '<S31>/Fresh per-conversion EMA' */
    acf_rl_native_DW.FreshperconversionEMA_states_l =
      acf_rl_native_P.FreshperconversionEMA_Initial_j;

    /* InitializeConditions for DiscreteStateSpace: '<S27>/Robot velocity frame matching' */
    acf_rl_native_DW.Robotvelocityframematching_DSTA =
      acf_rl_native_P.Robotvelocityframematching_Init;

    /* InitializeConditions for StateSpace: '<S37>/Continuous analog front end' */
    acf_rl_native_X.Continuousanalogfrontend_CSTA_e[0] =
      acf_rl_native_P.Continuousanalogfrontend_Init_a[0];
    acf_rl_native_X.Continuousanalogfrontend_CSTA_e[1] =
      acf_rl_native_P.Continuousanalogfrontend_Init_a[1];

    /* InitializeConditions for StateSpace: '<S37>/ADC analog input filter' */
    acf_rl_native_X.ADCanaloginputfilter_CSTATE_e3 =
      acf_rl_native_P.ADCanaloginputfilter_Initial_me;

    /* InitializeConditions for DiscreteTransferFcn: '<S37>/Native converter impulse response' */
    memcpy(&acf_rl_native_DW.Nativeconverterimpulserespons_h[0],
           &acf_rl_native_P.Nativeconverterimpulserespon_hk[0], 30U * sizeof
           (real_T));

    /* InitializeConditions for RandomNumber: '<S37>/Independent converter output noise' */
    tmp = floor(acf_rl_native_P.Independentconverteroutputno_px);
    if (rtIsNaN(tmp) || rtIsInf(tmp)) {
      tmp = 0.0;
    } else {
      tmp = fmod(tmp, 4.294967296E+9);
    }

    if (tmp < 0.0) {
      tseed = (uint32_T)-(int32_T)(uint32_T)-tmp;
    } else {
      tseed = (uint32_T)tmp;
    }

    r = tseed >> 16U;
    t = tseed & 32768U;
    tseed = ((((tseed - (r << 16U)) + t) << 16U) + t) + r;
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else if (tseed > 2147483646U) {
      tseed = 2147483646U;
    }

    acf_rl_native_DW.RandSeed_j = tseed;
    acf_rl_native_DW.NextOutput_j = rt_nrand_Upu32_Yd_f_pw_snf
      (&acf_rl_native_DW.RandSeed_j) *
      acf_rl_native_P.Independentconverteroutputnoi_l +
      acf_rl_native_P.Independentconverteroutputno_pm;

    /* End of InitializeConditions for RandomNumber: '<S37>/Independent converter output noise' */

    /* InitializeConditions for DiscreteTransferFcn: '<S37>/ADC output noise correlation' */
    acf_rl_native_DW.ADCoutputnoisecorrelation_sta_o =
      acf_rl_native_P.ADCoutputnoisecorrelation_Ini_o;

    /* InitializeConditions for DiscreteTransferFcn: '<S37>/Fresh per-conversion EMA' */
    acf_rl_native_DW.FreshperconversionEMA_states_m =
      acf_rl_native_P.FreshperconversionEMA_Initial_a;

    /* InitializeConditions for UnitDelay: '<S5>/Previous filtered force' */
    acf_rl_native_DW.Previousfilteredforce_DSTATE =
      acf_rl_native_P.Previousfilteredforce_InitialCo;

    /* InitializeConditions for UniformRandomNumber: '<S5>/Uniform sensor noise' */
    tmp = floor(acf_rl_native_P.Uniformsensornoise_Seed);
    if (rtIsNaN(tmp) || rtIsInf(tmp)) {
      tmp = 0.0;
    } else {
      tmp = fmod(tmp, 4.294967296E+9);
    }

    if (tmp < 0.0) {
      tseed = (uint32_T)-(int32_T)(uint32_T)-tmp;
    } else {
      tseed = (uint32_T)tmp;
    }

    r = tseed >> 16U;
    t = tseed & 32768U;
    tseed = ((((tseed - (r << 16U)) + t) << 16U) + t) + r;
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else if (tseed > 2147483646U) {
      tseed = 2147483646U;
    }

    acf_rl_native_DW.RandSeed_fw = tseed;
    acf_rl_native_DW.Uniformsensornoise_NextOutput =
      (acf_rl_native_P.Uniformsensornoise_Maximum -
       acf_rl_native_P.Uniformsensornoise_Minimum) * rt_urand_Upu32_Yd_f_pw_snf(
      &acf_rl_native_DW.RandSeed_fw) +
      acf_rl_native_P.Uniformsensornoise_Minimum;

    /* End of InitializeConditions for UniformRandomNumber: '<S5>/Uniform sensor noise' */

    /* InitializeConditions for Integrator: '<S7>/Predicted relative position' */
    acf_rl_native_X.Predictedrelativeposition_CSTAT =
      acf_rl_native_P.Predictedrelativeposition_IC;

    /* InitializeConditions for Integrator: '<S15>/Robot position' */
    acf_rl_native_X.Robotposition_CSTATE = acf_rl_native_P.Robotposition_IC;

    /* InitializeConditions for StateSpace: '<S38>/Continuous analog front end' */
    acf_rl_native_X.Continuousanalogfrontend_CSTA_k[0] =
      acf_rl_native_P.Continuousanalogfrontend_Ini_j5[0];
    acf_rl_native_X.Continuousanalogfrontend_CSTA_k[1] =
      acf_rl_native_P.Continuousanalogfrontend_Ini_j5[1];

    /* InitializeConditions for StateSpace: '<S38>/ADC analog input filter' */
    acf_rl_native_X.ADCanaloginputfilter_CSTATE_h =
      acf_rl_native_P.ADCanaloginputfilter_InitialC_l;

    /* InitializeConditions for DiscreteTransferFcn: '<S38>/Native converter impulse response' */
    memcpy(&acf_rl_native_DW.Nativeconverterimpulserespons_g[0],
           &acf_rl_native_P.Nativeconverterimpulserespons_o[0], 30U * sizeof
           (real_T));

    /* InitializeConditions for RandomNumber: '<S38>/Independent converter output noise' */
    tmp = floor(acf_rl_native_P.Independentconverteroutputnoi_n);
    if (rtIsNaN(tmp) || rtIsInf(tmp)) {
      tmp = 0.0;
    } else {
      tmp = fmod(tmp, 4.294967296E+9);
    }

    if (tmp < 0.0) {
      tseed = (uint32_T)-(int32_T)(uint32_T)-tmp;
    } else {
      tseed = (uint32_T)tmp;
    }

    r = tseed >> 16U;
    t = tseed & 32768U;
    tseed = ((((tseed - (r << 16U)) + t) << 16U) + t) + r;
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else if (tseed > 2147483646U) {
      tseed = 2147483646U;
    }

    acf_rl_native_DW.RandSeed_n = tseed;
    acf_rl_native_DW.NextOutput_m = rt_nrand_Upu32_Yd_f_pw_snf
      (&acf_rl_native_DW.RandSeed_n) *
      acf_rl_native_P.Independentconverteroutputnoi_f +
      acf_rl_native_P.Independentconverteroutputno_pu;

    /* End of InitializeConditions for RandomNumber: '<S38>/Independent converter output noise' */

    /* InitializeConditions for DiscreteTransferFcn: '<S38>/ADC output noise correlation' */
    acf_rl_native_DW.ADCoutputnoisecorrelation_st_ov =
      acf_rl_native_P.ADCoutputnoisecorrelation_Ini_j;

    /* InitializeConditions for DiscreteTransferFcn: '<S38>/Fresh per-conversion EMA' */
    acf_rl_native_DW.FreshperconversionEMA_states_a =
      acf_rl_native_P.FreshperconversionEMA_Initial_m;

    /* InitializeConditions for UnitDelay: '<S8>/Filter memory' */
    acf_rl_native_DW.Filtermemory_DSTATE =
      acf_rl_native_P.Filtermemory_InitialCondition;

    /* InitializeConditions for UniformRandomNumber: '<S8>/Independent lateral noise' */
    tmp = floor(acf_rl_native_P.Independentlateralnoise_Seed);
    if (rtIsNaN(tmp) || rtIsInf(tmp)) {
      tmp = 0.0;
    } else {
      tmp = fmod(tmp, 4.294967296E+9);
    }

    if (tmp < 0.0) {
      tseed = (uint32_T)-(int32_T)(uint32_T)-tmp;
    } else {
      tseed = (uint32_T)tmp;
    }

    r = tseed >> 16U;
    t = tseed & 32768U;
    tseed = ((((tseed - (r << 16U)) + t) << 16U) + t) + r;
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else if (tseed > 2147483646U) {
      tseed = 2147483646U;
    }

    acf_rl_native_DW.RandSeed_h = tseed;
    acf_rl_native_DW.Independentlateralnoise_NextOut =
      (acf_rl_native_P.Independentlateralnoise_Maximum -
       acf_rl_native_P.Independentlateralnoise_Minimum) *
      rt_urand_Upu32_Yd_f_pw_snf(&acf_rl_native_DW.RandSeed_h) +
      acf_rl_native_P.Independentlateralnoise_Minimum;

    /* End of InitializeConditions for UniformRandomNumber: '<S8>/Independent lateral noise' */
  }
}

/* Model terminate function */
void acf_rl_native_terminate(void)
{
  /* (no terminate code required) */
}
