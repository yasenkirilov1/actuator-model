/*
 * acf_rl_native_private.h
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "acf_rl_native".
 *
 * Model version              : 1.48
 * Simulink Coder version : 26.2 (R2026b) 22-May-2026
 * C source code generated on : Mon Oct  5 17:13:05 2026
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef acf_rl_native_private_h_
#define acf_rl_native_private_h_
#include "rtwtypes.h"
#include "multiword_types.h"
#include "acf_rl_native_types.h"
#include "acf_rl_native.h"

/* Private macros used by the generated code to access rtModel */
#ifndef rtmIsMajorTimeStep
#define rtmIsMajorTimeStep(rtm)        (((rtm)->Timing.simTimeStep) == MAJOR_TIME_STEP)
#endif

#ifndef rtmIsMinorTimeStep
#define rtmIsMinorTimeStep(rtm)        (((rtm)->Timing.simTimeStep) == MINOR_TIME_STEP)
#endif

#ifndef rtmSetTPtr
#define rtmSetTPtr(rtm, val)           ((rtm)->Timing.t = (val))
#endif

extern real_T rt_urand_Upu32_Yd_f_pw_snf(uint32_T *u);
extern real_T rt_nrand_Upu32_Yd_f_pw_snf(uint32_T *u);
extern real_T rt_modd_snf(real_T u0, real_T u1);
extern real_T rt_roundd_snf(real_T u);
extern real_T rt_TDelayInterpolate(real_T tMinusDelay, real_T tStart, real_T
  *uBuf, int32_T bufSz, int32_T *lastIdx, int32_T oldestIdx, int32_T newIdx,
  real_T initOutput, boolean_T discrete, boolean_T
  minorStepAndTAtLastMajorOutput);

/* private model entry point functions */
extern void acf_rl_native_derivatives(void);

#endif                                 /* acf_rl_native_private_h_ */
