/*
 * acf_rl_native.h
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "acf_rl_native".
 *
 * Model version              : 1.48
 * Simulink Coder version : 26.2 (R2026b) 22-May-2026
 * C source code generated on : Mon Oct  5 15:45:43 2026
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef acf_rl_native_h_
#define acf_rl_native_h_
#ifndef acf_rl_native_COMMON_INCLUDES_
#define acf_rl_native_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "rt_nonfinite.h"
#include "math.h"
#endif                                 /* acf_rl_native_COMMON_INCLUDES_ */

#include "acf_rl_native_types.h"
#include <string.h>
#include "rtGetNaN.h"
#include <math.h>

/* Macros for accessing real-time model data structure */
#ifndef rtmGetContStateDisabled
#define rtmGetContStateDisabled(rtm)   ((rtm)->contStateDisabled)
#endif

#ifndef rtmSetContStateDisabled
#define rtmSetContStateDisabled(rtm, val) ((rtm)->contStateDisabled = (val))
#endif

#ifndef rtmGetContStates
#define rtmGetContStates(rtm)          ((rtm)->contStates)
#endif

#ifndef rtmSetContStates
#define rtmSetContStates(rtm, val)     ((rtm)->contStates = (val))
#endif

#ifndef rtmGetContTimeOutputInconsistentWithStateAtMajorStepFlag
#define rtmGetContTimeOutputInconsistentWithStateAtMajorStepFlag(rtm) ((rtm)->CTOutputIncnstWithState)
#endif

#ifndef rtmSetContTimeOutputInconsistentWithStateAtMajorStepFlag
#define rtmSetContTimeOutputInconsistentWithStateAtMajorStepFlag(rtm, val) ((rtm)->CTOutputIncnstWithState = (val))
#endif

#ifndef rtmGetDerivCacheNeedsReset
#define rtmGetDerivCacheNeedsReset(rtm) ((rtm)->derivCacheNeedsReset)
#endif

#ifndef rtmSetDerivCacheNeedsReset
#define rtmSetDerivCacheNeedsReset(rtm, val) ((rtm)->derivCacheNeedsReset = (val))
#endif

#ifndef rtmGetIntgData
#define rtmGetIntgData(rtm)            ((rtm)->intgData)
#endif

#ifndef rtmSetIntgData
#define rtmSetIntgData(rtm, val)       ((rtm)->intgData = (val))
#endif

#ifndef rtmGetOdeF
#define rtmGetOdeF(rtm)                ((rtm)->odeF)
#endif

#ifndef rtmSetOdeF
#define rtmSetOdeF(rtm, val)           ((rtm)->odeF = (val))
#endif

#ifndef rtmGetOdeY
#define rtmGetOdeY(rtm)                ((rtm)->odeY)
#endif

#ifndef rtmSetOdeY
#define rtmSetOdeY(rtm, val)           ((rtm)->odeY = (val))
#endif

#ifndef rtmGetPeriodicContStateIndices
#define rtmGetPeriodicContStateIndices(rtm) ((rtm)->periodicContStateIndices)
#endif

#ifndef rtmSetPeriodicContStateIndices
#define rtmSetPeriodicContStateIndices(rtm, val) ((rtm)->periodicContStateIndices = (val))
#endif

#ifndef rtmGetPeriodicContStateRanges
#define rtmGetPeriodicContStateRanges(rtm) ((rtm)->periodicContStateRanges)
#endif

#ifndef rtmSetPeriodicContStateRanges
#define rtmSetPeriodicContStateRanges(rtm, val) ((rtm)->periodicContStateRanges = (val))
#endif

#ifndef rtmGetZCCacheNeedsReset
#define rtmGetZCCacheNeedsReset(rtm)   ((rtm)->zCCacheNeedsReset)
#endif

#ifndef rtmSetZCCacheNeedsReset
#define rtmSetZCCacheNeedsReset(rtm, val) ((rtm)->zCCacheNeedsReset = (val))
#endif

#ifndef rtmGetdX
#define rtmGetdX(rtm)                  ((rtm)->derivs)
#endif

#ifndef rtmSetdX
#define rtmSetdX(rtm, val)             ((rtm)->derivs = (val))
#endif

#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetStopRequested
#define rtmGetStopRequested(rtm)       ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
#define rtmSetStopRequested(rtm, val)  ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
#define rtmGetStopRequestedPtr(rtm)    (&((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef rtmGetT
#define rtmGetT(rtm)                   (rtmGetTPtr((rtm))[0])
#endif

#ifndef rtmGetTPtr
#define rtmGetTPtr(rtm)                ((rtm)->Timing.t)
#endif

#ifndef rtmGetTStart
#define rtmGetTStart(rtm)              ((rtm)->Timing.tStart)
#endif

/* Block signals (default storage) */
typedef struct {
  real_T Sensorsampledelay;            /* '<S5>/Sensor sample delay' */
  real_T Zknownchanneldelay;           /* '<S16>/Z known channel delay' */
  real_T Sensorsampledelay_h;          /* '<S8>/Sensor sample delay' */
  real_T Xknownchanneldelay;           /* '<S16>/X known channel delay' */
  real_T Robotvelocityframematchingdelay;
                               /* '<S27>/Robot velocity frame matching delay' */
  real_T Fresh10kHzrobotvelocityframe;
                                 /* '<S27>/Fresh 10 kHz robot velocity frame' */
  real_T PositionADCdeliverypipeline;
                                    /* '<S27>/Position ADC delivery pipeline' */
  real_T Fresh10kHzpositionframe;      /* '<S27>/Fresh 10 kHz position frame' */
  real_T Robotaccelerationframematchingd;
                           /* '<S27>/Robot acceleration frame matching delay' */
  real_T Fresh10kHzrobotaccelerationfram;
                             /* '<S27>/Fresh 10 kHz robot acceleration frame' */
  real_T Estimatedheadtravel;          /* '<S27>/Estimated head travel' */
  real_T Predictedvelocity;            /* '<S7>/Predicted velocity' */
  real_T Predictedvelocitysampling;   /* '<Root>/Predicted velocity sampling' */
  real_T Expectedspeedlimit;           /* '<S7>/Expected speed limit' */
  real_T Expectedaccelerationlimit;    /* '<S7>/Expected acceleration limit' */
  real_T Robotpredictionsampling;      /* '<Root>/Robot prediction sampling' */
  real_T Enablerobotaccelerationpredicti;
                             /* '<Root>/Enable robot acceleration prediction' */
  real_T Boundedorconstantnormalprogram;
                                /* '<S17>/Bounded or constant normal program' */
  real_T Nominalpathvelocity;          /* '<S17>/Nominal path velocity' */
  real_T Nominalpathsampling;          /* '<S12>/Nominal path sampling' */
  real_T PositionnoiseRMS;             /* '<S13>/Position noise RMS' */
  real_T Measuredorestimatedtravel;   /* '<S12>/Measured or estimated travel' */
  real_T Robotfeedbacksampling;        /* '<S12>/Robot feedback sampling' */
  real_T Distancefrommidstroke;        /* '<S12>/Distance from mid stroke' */
  real_T Recentergain;                 /* '<S12>/Recenter gain' */
  real_T RLfresh100Hzrobotrequest;   /* '<S12>/RL fresh 100 Hz robot request' */
  real_T Switch2;                      /* '<S40>/Switch2' */
  real_T Gatecontactorcellerror;       /* '<S12>/Gate contact or cell error' */
  real_T Robotrateforcesample;         /* '<S39>/Robot-rate force sample' */
  real_T Robotspeedlimits;             /* '<S12>/Robot speed limits' */
  real_T Double17;                     /* '<S10>/Double 17' */
  real_T Velocity;                     /* '<S22>/Velocity' */
  real_T Velocity_j;                   /* '<S21>/Velocity' */
  real_T Requestedrobotspeed;          /* '<S15>/Requested robot speed' */
  real_T Robotvelocity;                /* '<S15>/Robot velocity' */
  real_T Robotaccelerationlimits;      /* '<S15>/Robot acceleration limits' */
  real_T Newtonacceleration;           /* '<S21>/Newton acceleration' */
  real_T Newtonacceleration_n;         /* '<S22>/Newton acceleration' */
  real_T Currentrequestlimit;          /* '<S25>/Current request limit' */
  real_T Currentlimitflag;             /* '<S25>/Current limit flag' */
  real_T AccelerationXframediagonal; /* '<S16>/Acceleration X frame diagonal' */
  real_T AccelerationZframecrossaxis;
                                   /* '<S16>/Acceleration Z frame cross axis' */
  real_T Continuousanalogfrontend;     /* '<S42>/Continuous analog front end' */
  real_T ADCanaloginputfilter;         /* '<S42>/ADC analog input filter' */
  real_T Nativeconverterimpulseresponse;
                                 /* '<S42>/Native converter impulse response' */
  real_T ActualADCoutputsamples;       /* '<S42>/Actual ADC output samples' */
  real_T FreshperconversionEMA;        /* '<S42>/Fresh per-conversion EMA' */
  real_T XrawnoiseRMS;                 /* '<S16>/X raw noise RMS' */
  real_T Xnoisysensor;                 /* '<S16>/X noisy sensor' */
  real_T Xoldsampleweight;             /* '<S16>/X old sample weight' */
  real_T Xidentifiedsensorresponse;   /* '<S16>/X identified sensor response' */
  real_T Xunknowndrift;                /* '<S16>/X unknown drift' */
  real_T Continuousanalogfrontend_j;   /* '<S43>/Continuous analog front end' */
  real_T ADCanaloginputfilter_g;       /* '<S43>/ADC analog input filter' */
  real_T Nativeconverterimpulserespons_j;
                                 /* '<S43>/Native converter impulse response' */
  real_T ActualADCoutputsamples_k;     /* '<S43>/Actual ADC output samples' */
  real_T FreshperconversionEMA_l;      /* '<S43>/Fresh per-conversion EMA' */
  real_T ZrawnoiseRMS;                 /* '<S16>/Z raw noise RMS' */
  real_T Znoisysensor;                 /* '<S16>/Z noisy sensor' */
  real_T Zoldsampleweight;             /* '<S16>/Z old sample weight' */
  real_T Zidentifiedsensorresponse;   /* '<S16>/Z identified sensor response' */
  real_T Zunknowndrift;                /* '<S16>/Z unknown drift' */
  real_T Inverseinductance;            /* '<S25>/Inverse inductance' */
  real_T Headgapcoordinatesign;        /* '<S27>/Head gap coordinate sign' */
  real_T Continuousanalogfrontend_n;   /* '<S29>/Continuous analog front end' */
  real_T ADCanaloginputfilter_o;       /* '<S29>/ADC analog input filter' */
  real_T Nativeconverterimpulserespons_g;
                                 /* '<S29>/Native converter impulse response' */
  real_T ActualADCoutputsamples_g;     /* '<S29>/Actual ADC output samples' */
  real_T FreshperconversionEMA_c;      /* '<S29>/Fresh per-conversion EMA' */
  real_T Positionacquisitionframe;     /* '<S27>/Position acquisition frame' */
  real_T Continuousanalogfrontend_m;   /* '<S30>/Continuous analog front end' */
  real_T ADCanaloginputfilter_c;       /* '<S30>/ADC analog input filter' */
  real_T Nativeconverterimpulserespons_n;
                                 /* '<S30>/Native converter impulse response' */
  real_T ActualADCoutputsamples_m;     /* '<S30>/Actual ADC output samples' */
  real_T FreshperconversionEMA_k;      /* '<S30>/Fresh per-conversion EMA' */
  real_T Robotaccelerationacquisitionfra;
                              /* '<S27>/Robot acceleration acquisition frame' */
  real_T Continuousanalogfrontend_mn;  /* '<S31>/Continuous analog front end' */
  real_T ADCanaloginputfilter_cf;      /* '<S31>/ADC analog input filter' */
  real_T Nativeconverterimpulserespons_l;
                                 /* '<S31>/Native converter impulse response' */
  real_T ActualADCoutputsamples_i;     /* '<S31>/Actual ADC output samples' */
  real_T FreshperconversionEMA_g;      /* '<S31>/Fresh per-conversion EMA' */
  real_T Robotvelocityacquisitionframe;
                                  /* '<S27>/Robot velocity acquisition frame' */
  real_T Continuousanalogfrontend_n1;  /* '<S37>/Continuous analog front end' */
  real_T ADCanaloginputfilter_j;       /* '<S37>/ADC analog input filter' */
  real_T Nativeconverterimpulserespons_o;
                                 /* '<S37>/Native converter impulse response' */
  real_T ActualADCoutputsamples_n;     /* '<S37>/Actual ADC output samples' */
  real_T Conditionedforcewithtareresidua;
                               /* '<S5>/Conditioned force with tare residual' */
  real_T Residualaxialcalibration;     /* '<S5>/Residual axial calibration' */
  real_T Rawoutputnoisenormalization;/* '<S5>/Raw output-noise normalization' */
  real_T Rawnoisyforcesample;          /* '<S5>/Raw noisy force sample' */
  real_T Selectidentifiedsensorresponse;
                                  /* '<S5>/Select identified sensor response' */
  real_T Continuousanalogfrontend_jz;  /* '<S38>/Continuous analog front end' */
  real_T ADCanaloginputfilter_f;       /* '<S38>/ADC analog input filter' */
  real_T Nativeconverterimpulserespon_nt;
                                 /* '<S38>/Native converter impulse response' */
  real_T ActualADCoutputsamples_b;     /* '<S38>/Actual ADC output samples' */
  real_T Conditionedforcewithtareresid_m;
                               /* '<S8>/Conditioned force with tare residual' */
  real_T Residuallateralcalibration;   /* '<S8>/Residual lateral calibration' */
  real_T Rawoutputnoisenormalization_g;
                                     /* '<S8>/Raw output-noise normalization' */
  real_T Rawnoisyforcesample_l;        /* '<S8>/Raw noisy force sample' */
  real_T Selectidentifiedsensorrespons_d;
                                  /* '<S8>/Select identified sensor response' */
  boolean_T Forcestablenow;            /* '<S39>/Force stable now' */
  boolean_T Fastcontinuousdwellcompleted;
                                   /* '<S41>/Fast continuous dwell completed' */
  boolean_T Fastreadyatrobotrate;      /* '<S39>/Fast ready at robot rate' */
  boolean_T Readyorbypassed;           /* '<S39>/Ready or bypassed' */
  boolean_T Invalidsizingrun;          /* '<S23>/Invalid sizing run' */
} B_acf_rl_native_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real_T Headgapandvelocityobserver_DSTA[2];
                                    /* '<S27>/Head gap and velocity observer' */
  real_T Expectedcommanddelay_DSTATE[12];/* '<S7>/Expected command delay' */
  real_T Normalforceerrorintegral_DSTATE;/* '<S36>/Normal force error integral' */
  real_T Normalerrorprevioussample_DSTAT;
                                      /* '<S36>/Normal error previous sample' */
  real_T Normalerrorderivativememory_DST;
                                    /* '<S36>/Normal error derivative memory' */
  real_T Optionalfeedbackrolloff_states[2];/* '<S36>/Optional feedback rolloff' */
  real_T Filteredmeasuredaccelerationrat;
                               /* '<S35>/Filtered measured acceleration rate' */
  real_T Previousfiltereddragratio_DSTAT;
                                      /* '<S32>/Previous filtered drag ratio' */
  real_T Properidentifiedlowerlinkinvers[3];
                              /* '<S34>/Proper identified lower-link inverse' */
  real_T Optionalstructuralmodecurrentno[2];
                            /* '<S33>/Optional structural-mode current notch' */
  real_T Previousfaststablecount_DSTATE;/* '<S41>/Previous fast stable count' */
  real_T Forcetrendfilter_states;      /* '<S39>/Force trend filter' */
  real_T Previousforcetrend_DSTATE;    /* '<S39>/Previous force trend' */
  real_T Fastnormalforcemean_states;   /* '<S41>/Fast normal force mean' */
  real_T Fastforcevariance_states;     /* '<S41>/Fast force variance' */
  real_T Previousstablecount_DSTATE;   /* '<S39>/Previous stable count' */
  real_T Robotcommandsamples_DSTATE[1000];/* '<S14>/Robot command samples' */
  real_T Nativeconverterimpulseresponse_[30];
                                 /* '<S42>/Native converter impulse response' */
  real_T ADCoutputnoisecorrelation_state;
                                      /* '<S42>/ADC output noise correlation' */
  real_T FreshperconversionEMA_states; /* '<S42>/Fresh per-conversion EMA' */
  real_T Xfiltermemory_DSTATE;         /* '<S16>/X filter memory' */
  real_T Nativeconverterimpulserespons_a[30];
                                 /* '<S43>/Native converter impulse response' */
  real_T ADCoutputnoisecorrelation_sta_b;
                                      /* '<S43>/ADC output noise correlation' */
  real_T FreshperconversionEMA_states_f;/* '<S43>/Fresh per-conversion EMA' */
  real_T Zfiltermemory_DSTATE;         /* '<S16>/Z filter memory' */
  real_T Nativeconverterimpulserespons_e[30];
                                 /* '<S29>/Native converter impulse response' */
  real_T FreshperconversionEMA_states_j;/* '<S29>/Fresh per-conversion EMA' */
  real_T Positionframematching_DSTATE; /* '<S27>/Position frame matching' */
  real_T Nativeconverterimpulserespons_k[30];
                                 /* '<S30>/Native converter impulse response' */
  real_T FreshperconversionEMA_states_h;/* '<S30>/Fresh per-conversion EMA' */
  real_T Robotaccelerationframematching_;
                                 /* '<S27>/Robot acceleration frame matching' */
  real_T Nativeconverterimpulserespons_m[30];
                                 /* '<S31>/Native converter impulse response' */
  real_T FreshperconversionEMA_states_l;/* '<S31>/Fresh per-conversion EMA' */
  real_T Robotvelocityframematching_DSTA;
                                     /* '<S27>/Robot velocity frame matching' */
  real_T Nativeconverterimpulserespons_h[30];
                                 /* '<S37>/Native converter impulse response' */
  real_T ADCoutputnoisecorrelation_sta_o;
                                      /* '<S37>/ADC output noise correlation' */
  real_T FreshperconversionEMA_states_m;/* '<S37>/Fresh per-conversion EMA' */
  real_T Previousfilteredforce_DSTATE; /* '<S5>/Previous filtered force' */
  real_T Nativeconverterimpulserespons_g[30];
                                 /* '<S38>/Native converter impulse response' */
  real_T ADCoutputnoisecorrelation_st_ov;
                                      /* '<S38>/ADC output noise correlation' */
  real_T FreshperconversionEMA_states_a;/* '<S38>/Fresh per-conversion EMA' */
  real_T Filtermemory_DSTATE;          /* '<S8>/Filter memory' */
  real_T Optionalfeedbackrolloff_tmp;  /* '<S36>/Optional feedback rolloff' */
  real_T Filteredmeasuredaccelerationr_f;
                               /* '<S35>/Filtered measured acceleration rate' */
  real_T Properidentifiedlowerlinkinve_k;
                              /* '<S34>/Proper identified lower-link inverse' */
  real_T Optionalstructuralmodecurrent_n;
                            /* '<S33>/Optional structural-mode current notch' */
  real_T Positionnoise_NextOutput;     /* '<S13>/Position noise' */
  real_T Fastnormalforcemean_tmp;      /* '<S41>/Fast normal force mean' */
  real_T Fastforcevariance_tmp;        /* '<S41>/Fast force variance' */
  real_T Nativeconverterimpulserespons_o;
                                 /* '<S42>/Native converter impulse response' */
  real_T NextOutput;            /* '<S42>/Independent converter output noise' */
  real_T ADCoutputnoisecorrelation_tmp;
                                      /* '<S42>/ADC output noise correlation' */
  real_T FreshperconversionEMA_tmp;    /* '<S42>/Fresh per-conversion EMA' */
  real_T NextOutput_b;                 /* '<S16>/X raw noise' */
  real_T Nativeconverterimpulserespon_gz;
                                 /* '<S43>/Native converter impulse response' */
  real_T NextOutput_n;          /* '<S43>/Independent converter output noise' */
  real_T ADCoutputnoisecorrelation_tmp_e;
                                      /* '<S43>/ADC output noise correlation' */
  real_T FreshperconversionEMA_tmp_k;  /* '<S43>/Fresh per-conversion EMA' */
  real_T NextOutput_bf;                /* '<S16>/Z raw noise' */
  real_T Nativeconverterimpulserespon_au;
                                 /* '<S29>/Native converter impulse response' */
  real_T FreshperconversionEMA_tmp_h;  /* '<S29>/Fresh per-conversion EMA' */
  real_T Nativeconverterimpulserespons_b;
                                 /* '<S30>/Native converter impulse response' */
  real_T FreshperconversionEMA_tmp_e;  /* '<S30>/Fresh per-conversion EMA' */
  real_T Nativeconverterimpulserespons_n;
                                 /* '<S31>/Native converter impulse response' */
  real_T FreshperconversionEMA_tmp_p;  /* '<S31>/Fresh per-conversion EMA' */
  real_T Nativeconverterimpulserespons_p;
                                 /* '<S37>/Native converter impulse response' */
  real_T NextOutput_j;          /* '<S37>/Independent converter output noise' */
  real_T ADCoutputnoisecorrelation_tmp_p;
                                      /* '<S37>/ADC output noise correlation' */
  real_T FreshperconversionEMA_tmp_ep; /* '<S37>/Fresh per-conversion EMA' */
  real_T Uniformsensornoise_NextOutput;/* '<S5>/Uniform sensor noise' */
  real_T Nativeconverterimpulserespons_l;
                                 /* '<S38>/Native converter impulse response' */
  real_T NextOutput_m;          /* '<S38>/Independent converter output noise' */
  real_T ADCoutputnoisecorrelation_tmp_n;
                                      /* '<S38>/ADC output noise correlation' */
  real_T FreshperconversionEMA_tmp_j;  /* '<S38>/Fresh per-conversion EMA' */
  real_T Independentlateralnoise_NextOut;/* '<S8>/Independent lateral noise' */
  real_T Sensorsampledelay_RWORK[2049];/* '<S5>/Sensor sample delay' */
  real_T Zknownchanneldelay_RWORK[2049];/* '<S16>/Z known channel delay' */
  real_T Sensorsampledelay_RWORK_k[2049];/* '<S8>/Sensor sample delay' */
  real_T Xknownchanneldelay_RWORK[2049];/* '<S16>/X known channel delay' */
  real_T Robotvelocityframematchingdelay[2049];
                               /* '<S27>/Robot velocity frame matching delay' */
  real_T PositionADCdeliverypipeline_RWO[2049];
                                    /* '<S27>/Position ADC delivery pipeline' */
  real_T Robotaccelerationframematchingd[2049];
                           /* '<S27>/Robot acceleration frame matching delay' */
  void *Sensorsampledelay_PWORK[2];    /* '<S5>/Sensor sample delay' */
  void *Zknownchanneldelay_PWORK[2];   /* '<S16>/Z known channel delay' */
  void *Sensorsampledelay_PWORK_f[2];  /* '<S8>/Sensor sample delay' */
  void *Xknownchanneldelay_PWORK[2];   /* '<S16>/X known channel delay' */
  void *Robotvelocityframematchingdel_l[2];
                               /* '<S27>/Robot velocity frame matching delay' */
  void *PositionADCdeliverypipeline_PWO[2];
                                    /* '<S27>/Position ADC delivery pipeline' */
  void *Robotaccelerationframematchin_h[2];
                           /* '<S27>/Robot acceleration frame matching delay' */
  uint32_T CircBufIdx;                 /* '<S7>/Expected command delay' */
  uint32_T RandSeed;                   /* '<S13>/Position noise' */
  uint32_T CircBufIdx_e;               /* '<S14>/Robot command samples' */
  uint32_T RandSeed_f;          /* '<S42>/Independent converter output noise' */
  uint32_T RandSeed_c;                 /* '<S16>/X raw noise' */
  uint32_T RandSeed_l;          /* '<S43>/Independent converter output noise' */
  uint32_T RandSeed_p;                 /* '<S16>/Z raw noise' */
  uint32_T RandSeed_j;          /* '<S37>/Independent converter output noise' */
  uint32_T RandSeed_fw;                /* '<S5>/Uniform sensor noise' */
  uint32_T RandSeed_n;          /* '<S38>/Independent converter output noise' */
  uint32_T RandSeed_h;                 /* '<S8>/Independent lateral noise' */
  int_T Sensorsampledelay_IWORK[4];    /* '<S5>/Sensor sample delay' */
  int_T Zknownchanneldelay_IWORK[4];   /* '<S16>/Z known channel delay' */
  int_T Sensorsampledelay_IWORK_i[4];  /* '<S8>/Sensor sample delay' */
  int_T Xknownchanneldelay_IWORK[4];   /* '<S16>/X known channel delay' */
  int_T Robotvelocityframematchingdel_c[4];
                               /* '<S27>/Robot velocity frame matching delay' */
  int_T PositionADCdeliverypipeline_IWO[4];
                                    /* '<S27>/Position ADC delivery pipeline' */
  int_T Robotaccelerationframematchin_n[4];
                           /* '<S27>/Robot acceleration frame matching delay' */
} DW_acf_rl_native_T;

/* Continuous states (default storage) */
typedef struct {
  real_T Predictedvelocity_CSTATE;     /* '<S7>/Predicted velocity' */
  real_T Position_CSTATE;              /* '<S21>/Position' */
  real_T Position_CSTATE_p;            /* '<S22>/Position' */
  real_T Velocity_CSTATE;              /* '<S22>/Velocity' */
  real_T Velocity_CSTATE_m;            /* '<S21>/Velocity' */
  real_T Windingcurrent_CSTATE;        /* '<S25>/Winding current' */
  real_T Robotvelocity_CSTATE;         /* '<S15>/Robot velocity' */
  real_T Continuousanalogfrontend_CSTATE[2];/* '<S42>/Continuous analog front end' */
  real_T ADCanaloginputfilter_CSTATE;  /* '<S42>/ADC analog input filter' */
  real_T Continuousanalogfrontend_CSTA_f[2];/* '<S43>/Continuous analog front end' */
  real_T ADCanaloginputfilter_CSTATE_o;/* '<S43>/ADC analog input filter' */
  real_T Continuousanalogfrontend_CSTA_m[2];/* '<S29>/Continuous analog front end' */
  real_T ADCanaloginputfilter_CSTATE_e;/* '<S29>/ADC analog input filter' */
  real_T Continuousanalogfrontend_CSTA_p[2];/* '<S30>/Continuous analog front end' */
  real_T ADCanaloginputfilter_CSTATE_g;/* '<S30>/ADC analog input filter' */
  real_T Continuousanalogfrontend_CSTA_c[2];/* '<S31>/Continuous analog front end' */
  real_T ADCanaloginputfilter_CSTATE_ot;/* '<S31>/ADC analog input filter' */
  real_T Continuousanalogfrontend_CSTA_e[2];/* '<S37>/Continuous analog front end' */
  real_T ADCanaloginputfilter_CSTATE_e3;/* '<S37>/ADC analog input filter' */
  real_T Predictedrelativeposition_CSTAT;/* '<S7>/Predicted relative position' */
  real_T Robotposition_CSTATE;         /* '<S15>/Robot position' */
  real_T Continuousanalogfrontend_CSTA_k[2];/* '<S38>/Continuous analog front end' */
  real_T ADCanaloginputfilter_CSTATE_h;/* '<S38>/ADC analog input filter' */
} X_acf_rl_native_T;

/* State derivatives (default storage) */
typedef struct {
  real_T Predictedvelocity_CSTATE;     /* '<S7>/Predicted velocity' */
  real_T Position_CSTATE;              /* '<S21>/Position' */
  real_T Position_CSTATE_p;            /* '<S22>/Position' */
  real_T Velocity_CSTATE;              /* '<S22>/Velocity' */
  real_T Velocity_CSTATE_m;            /* '<S21>/Velocity' */
  real_T Windingcurrent_CSTATE;        /* '<S25>/Winding current' */
  real_T Robotvelocity_CSTATE;         /* '<S15>/Robot velocity' */
  real_T Continuousanalogfrontend_CSTATE[2];/* '<S42>/Continuous analog front end' */
  real_T ADCanaloginputfilter_CSTATE;  /* '<S42>/ADC analog input filter' */
  real_T Continuousanalogfrontend_CSTA_f[2];/* '<S43>/Continuous analog front end' */
  real_T ADCanaloginputfilter_CSTATE_o;/* '<S43>/ADC analog input filter' */
  real_T Continuousanalogfrontend_CSTA_m[2];/* '<S29>/Continuous analog front end' */
  real_T ADCanaloginputfilter_CSTATE_e;/* '<S29>/ADC analog input filter' */
  real_T Continuousanalogfrontend_CSTA_p[2];/* '<S30>/Continuous analog front end' */
  real_T ADCanaloginputfilter_CSTATE_g;/* '<S30>/ADC analog input filter' */
  real_T Continuousanalogfrontend_CSTA_c[2];/* '<S31>/Continuous analog front end' */
  real_T ADCanaloginputfilter_CSTATE_ot;/* '<S31>/ADC analog input filter' */
  real_T Continuousanalogfrontend_CSTA_e[2];/* '<S37>/Continuous analog front end' */
  real_T ADCanaloginputfilter_CSTATE_e3;/* '<S37>/ADC analog input filter' */
  real_T Predictedrelativeposition_CSTAT;/* '<S7>/Predicted relative position' */
  real_T Robotposition_CSTATE;         /* '<S15>/Robot position' */
  real_T Continuousanalogfrontend_CSTA_k[2];/* '<S38>/Continuous analog front end' */
  real_T ADCanaloginputfilter_CSTATE_h;/* '<S38>/ADC analog input filter' */
} XDot_acf_rl_native_T;

/* State disabled  */
typedef struct {
  boolean_T Predictedvelocity_CSTATE;  /* '<S7>/Predicted velocity' */
  boolean_T Position_CSTATE;           /* '<S21>/Position' */
  boolean_T Position_CSTATE_p;         /* '<S22>/Position' */
  boolean_T Velocity_CSTATE;           /* '<S22>/Velocity' */
  boolean_T Velocity_CSTATE_m;         /* '<S21>/Velocity' */
  boolean_T Windingcurrent_CSTATE;     /* '<S25>/Winding current' */
  boolean_T Robotvelocity_CSTATE;      /* '<S15>/Robot velocity' */
  boolean_T Continuousanalogfrontend_CSTATE[2];/* '<S42>/Continuous analog front end' */
  boolean_T ADCanaloginputfilter_CSTATE;/* '<S42>/ADC analog input filter' */
  boolean_T Continuousanalogfrontend_CSTA_f[2];/* '<S43>/Continuous analog front end' */
  boolean_T ADCanaloginputfilter_CSTATE_o;/* '<S43>/ADC analog input filter' */
  boolean_T Continuousanalogfrontend_CSTA_m[2];/* '<S29>/Continuous analog front end' */
  boolean_T ADCanaloginputfilter_CSTATE_e;/* '<S29>/ADC analog input filter' */
  boolean_T Continuousanalogfrontend_CSTA_p[2];/* '<S30>/Continuous analog front end' */
  boolean_T ADCanaloginputfilter_CSTATE_g;/* '<S30>/ADC analog input filter' */
  boolean_T Continuousanalogfrontend_CSTA_c[2];/* '<S31>/Continuous analog front end' */
  boolean_T ADCanaloginputfilter_CSTATE_ot;/* '<S31>/ADC analog input filter' */
  boolean_T Continuousanalogfrontend_CSTA_e[2];/* '<S37>/Continuous analog front end' */
  boolean_T ADCanaloginputfilter_CSTATE_e3;/* '<S37>/ADC analog input filter' */
  boolean_T Predictedrelativeposition_CSTAT;/* '<S7>/Predicted relative position' */
  boolean_T Robotposition_CSTATE;      /* '<S15>/Robot position' */
  boolean_T Continuousanalogfrontend_CSTA_k[2];/* '<S38>/Continuous analog front end' */
  boolean_T ADCanaloginputfilter_CSTATE_h;/* '<S38>/ADC analog input filter' */
} XDis_acf_rl_native_T;

#ifndef ODE4_INTG
#define ODE4_INTG

/* ODE4 Integration Data */
typedef struct {
  real_T *y;                           /* output */
  real_T *f[4];                        /* derivatives */
} ODE4_IntgData;

#endif

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T rl_current_request_A;         /* '<Root>/rl_current_request_A' */
  real_T rl_robot_correction_m_s;      /* '<Root>/rl_robot_correction_m_s' */
  real_T rl_baseline_mode;             /* '<Root>/rl_baseline_mode' */
} ExtU_acf_rl_native_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T rl_observation[18];           /* '<Root>/rl_observation' */
  real_T rl_scoring[27];               /* '<Root>/rl_scoring' */
  real_T rl_comparator[3];             /* '<Root>/rl_comparator' */
} ExtY_acf_rl_native_T;

/* Parameters (default storage) */
struct P_acf_rl_native_T_ {
  real_T Guardprojectiondenominator_Uppe;/* Expression: inf
                                          * Referenced by: '<S32>/Guard projection denominator'
                                          */
  real_T Guardprojectiondenominator_Lowe;
                                    /* Expression: p.contact.dragRatioMinForce_N
                                     * Referenced by: '<S32>/Guard projection denominator'
                                     */
  real_T Boundsigneddragratio_UpperSat;/* Expression: 1
                                        * Referenced by: '<S32>/Bound signed drag ratio'
                                        */
  real_T Boundsigneddragratio_LowerSat;/* Expression: -1
                                        * Referenced by: '<S32>/Bound signed drag ratio'
                                        */
  real_T Dragcorrectionsign_Gain;      /* Expression: -1
                                        * Referenced by: '<S32>/Drag correction sign'
                                        */
  real_T Axialnormalforcegeometryfactor_;/* Expression: 1
                                          * Referenced by: '<S32>/Axial normal-force geometry factor'
                                          */
  real_T Normalforcereference_Gain;    /* Expression: p.force.reference
                                        * Referenced by: '<S32>/Normal force reference'
                                        */
  real_T Normalreferenceprojection_Gain;/* Expression: p.force.reference
                                         * Referenced by: '<S28>/Normal reference projection'
                                         */
  real_T Onemorefaststablesample_Bias; /* Expression: 1
                                        * Referenced by: '<S41>/One more fast stable sample'
                                        */
  real_T Resetfastdwell_Value;         /* Expression: 0
                                        * Referenced by: '<S41>/Reset fast dwell'
                                        */
  real_T Onemorestablesample_Bias;     /* Expression: 1
                                        * Referenced by: '<S39>/One more stable sample'
                                        */
  real_T Resetcount_Value;             /* Expression: 0
                                        * Referenced by: '<S39>/Reset count'
                                        */
  real_T Dwellsamplecount_Value;
           /* Expression: 1+ceil(p.robot.forceGate.dwellTime/p.robot.sampleTime)
            * Referenced by: '<S39>/Dwell sample count'
            */
  real_T Baselineorexternalrobot_Thresho;/* Expression: .5
                                          * Referenced by: '<S12>/Baseline or external robot'
                                          */
  real_T ToolSrelativetoA_Gain[2];     /* Expression: [-1 1]
                                        * Referenced by: '<S13>/Tool S relative to A'
                                        */
  real_T Rampcomponent1_Coefs[6];      /* Expression: [6 -15 10 0 0 0]
                                        * Referenced by: '<S17>/Ramp component 1'
                                        */
  real_T Bumpcomponent1_Coefs[7];      /* Expression: [-64 192 -192 64 0 0 0]
                                        * Referenced by: '<S17>/Bump component 1'
                                        */
  real_T Rampcomponent2_Coefs[5];      /* Expression: [30 -60 30 0 0]
                                        * Referenced by: '<S17>/Ramp component 2'
                                        */
  real_T Bumpcomponent2_Coefs[6];      /* Expression: [-384 960 -768 192 0 0]
                                        * Referenced by: '<S17>/Bump component 2'
                                        */
  real_T Rampcomponent3_Coefs[4];      /* Expression: [120 -180 60 0]
                                        * Referenced by: '<S17>/Ramp component 3'
                                        */
  real_T Bumpcomponent3_Coefs[5];      /* Expression: [-1920 3840 -2304 384 0]
                                        * Referenced by: '<S17>/Bump component 3'
                                        */
  real_T Requestedforce_Value;         /* Expression: p.force.reference
                                        * Referenced by: '<Root>/Requested force'
                                        */
  real_T Sensorsampledelay_Delay;
                      /* Expression: p.adcStudy.d.channels.force.pipelineDelay_s
                       * Referenced by: '<S5>/Sensor sample delay'
                       */
  real_T Sensorsampledelay_InitOutput;
  /* Expression: round((((p.force.reference*(1+p.uncertainty.forceScaleZ)*(cos(p.uncertainty.forceFrameAngle_rad)-p.contact.mu*(sin(p.uncertainty.forceFrameAngle_rad)+p.uncertainty.forceCrossZFromX))))+(p.adcStudy.d.forceGravityOffset_N))/(max(p.adcStudy.forceQuantum_N,1e-15)))*(max(p.adcStudy.forceQuantum_N,1e-15))-round((p.adcStudy.d.forceGravityOffset_N)/(max(p.adcStudy.forceQuantum_N,1e-15)))*(max(p.adcStudy.forceQuantum_N,1e-15))+(p.sensor.bias)
   * Referenced by: '<S5>/Sensor sample delay'
   */
  real_T Cellerrortodeviation_Gain;    /* Expression: -1
                                        * Referenced by: '<S26>/Cell error to deviation'
                                        */
  real_T Taredaxialcellforce_Bias;     /* Expression: p.force.reference
                                        * Referenced by: '<S26>/Tared axial cell force'
                                        */
  real_T Zknownchanneldelay_Delay;
               /* Expression: p.adcStudy.d.channels.acceleration.pipelineDelay_s
                * Referenced by: '<S16>/Z known channel delay'
                */
  real_T Zknownchanneldelay_InitOutput;/* Expression: p.accel.bias
                                        * Referenced by: '<S16>/Z known channel delay'
                                        */
  real_T DownstreamZinertia_Gain;    /* Expression: p.sensor.distalMass_model_kg
                                      * Referenced by: '<S26>/Downstream Z inertia'
                                      */
  real_T Sensorsampledelay_Delay_j;
                    /* Expression: p.adcStudy.d.channels.lateral.pipelineDelay_s
                     * Referenced by: '<S8>/Sensor sample delay'
                     */
  real_T Sensorsampledelay_InitOutput_h;
  /* Expression: round((((p.force.reference*(1+p.uncertainty.forceScaleX)*(-p.contact.mu*cos(p.uncertainty.forceFrameAngle_rad)-sin(p.uncertainty.forceFrameAngle_rad)+p.uncertainty.forceCrossXFromZ)))+(p.adcStudy.d.lateralGravityOffset_N))/(max(p.adcStudy.forceQuantum_N,1e-15)))*(max(p.adcStudy.forceQuantum_N,1e-15))-round((p.adcStudy.d.lateralGravityOffset_N)/(max(p.adcStudy.forceQuantum_N,1e-15)))*(max(p.adcStudy.forceQuantum_N,1e-15))+(p.contact.lateralBias_N)
   * Referenced by: '<S8>/Sensor sample delay'
   */
  real_T Xknownchanneldelay_Delay;
               /* Expression: p.adcStudy.d.channels.acceleration.pipelineDelay_s
                * Referenced by: '<S16>/X known channel delay'
                */
  real_T Xknownchanneldelay_InitOutput;
                                    /* Expression: p.uncertainty.accelBiasX_m_s2
                                     * Referenced by: '<S16>/X known channel delay'
                                     */
  real_T DownstreamXinertia_Gain;    /* Expression: p.sensor.distalMass_model_kg
                                      * Referenced by: '<S26>/Downstream X inertia'
                                      */
  real_T Robotvelocityframematchingdelay;
                      /* Expression: p.adcStudy.d.channels.frame.pipelineDelay_s
                       * Referenced by: '<S27>/Robot velocity frame matching delay'
                       */
  real_T Robotvelocityframematchingdel_j;/* Expression: 0
                                          * Referenced by: '<S27>/Robot velocity frame matching delay'
                                          */
  real_T PositionADCdeliverypipeline_Del;
                   /* Expression: p.adcStudy.d.channels.position.pipelineDelay_s
                    * Referenced by: '<S27>/Position ADC delivery pipeline'
                    */
  real_T PositionADCdeliverypipeline_Ini;/* Expression: 0
                                          * Referenced by: '<S27>/Position ADC delivery pipeline'
                                          */
  real_T Robotaccelerationframematchingd;
                      /* Expression: p.adcStudy.d.channels.frame.pipelineDelay_s
                       * Referenced by: '<S27>/Robot acceleration frame matching delay'
                       */
  real_T Robotaccelerationframematchin_f;/* Expression: 0
                                          * Referenced by: '<S27>/Robot acceleration frame matching delay'
                                          */
  real_T Headgapandvelocityobserver_A[4];
                             /* Computed Parameter: Headgapandvelocityobserver_A
                              * Referenced by: '<S27>/Head gap and velocity observer'
                              */
  real_T Headgapandvelocityobserver_B[6];
                             /* Computed Parameter: Headgapandvelocityobserver_B
                              * Referenced by: '<S27>/Head gap and velocity observer'
                              */
  real_T Headgapandvelocityobserver_C[3];
                             /* Computed Parameter: Headgapandvelocityobserver_C
                              * Referenced by: '<S27>/Head gap and velocity observer'
                              */
  real_T Headgapandvelocityobserver_D[2];
                             /* Computed Parameter: Headgapandvelocityobserver_D
                              * Referenced by: '<S27>/Head gap and velocity observer'
                              */
  real_T Headgapandvelocityobserver_Init[2];/* Expression: zeros(2,1)
                                             * Referenced by: '<S27>/Head gap and velocity observer'
                                             */
  real_T Causalsurfacedirection_Gain;
                               /* Expression: 1/p.contact.commandedScanSpeed_m_s
                                * Referenced by: '<S28>/Causal surface direction'
                                */
  real_T Squaredslopeplusone_Bias;     /* Expression: 1
                                        * Referenced by: '<S28>/Squared slope plus one'
                                        */
  real_T Estimatedheadtravel_Bias;   /* Expression: p.mechanics.midTravel+p.d.u0
                                      * Referenced by: '<S27>/Estimated head travel'
                                      */
  real_T Predictedvelocity_IC;         /* Expression: 0
                                        * Referenced by: '<S7>/Predicted velocity'
                                        */
  real_T RLresetfixedqueuelength_Value;
                                   /* Expression: p.robotPrediction.delaySamples
                                    * Referenced by: '<S7>/RL reset-fixed queue length'
                                    */
  real_T Expectedcommanddelay_InitialCon;
  /* Expression: p.robot.nominalVelocity*double(~p.robotPrediction.boundedNominalPath)
   * Referenced by: '<S7>/Expected command delay'
   */
  real_T Expectedspeedlimit_UpperSat;  /* Expression: p.robotPrediction.maxSpeed
                                        * Referenced by: '<S7>/Expected speed limit'
                                        */
  real_T Expectedspeedlimit_LowerSat; /* Expression: -p.robotPrediction.maxSpeed
                                       * Referenced by: '<S7>/Expected speed limit'
                                       */
  real_T Expectedvelocityresponse_Gain;
                                 /* Expression: 1/p.robotPrediction.responseTime
                                  * Referenced by: '<S7>/Expected velocity response'
                                  */
  real_T Expectedaccelerationlimit_Upper;
                                /* Expression: p.robotPrediction.maxAcceleration
                                 * Referenced by: '<S7>/Expected acceleration limit'
                                 */
  real_T Expectedaccelerationlimit_Lower;
                               /* Expression: -p.robotPrediction.maxAcceleration
                                * Referenced by: '<S7>/Expected acceleration limit'
                                */
  real_T Enablerobotaccelerationpredicti;
                                /* Expression: double(p.robotPrediction.enabled)
                                 * Referenced by: '<Root>/Enable robot acceleration prediction'
                                 */
  real_T Flatholdingcurrent_Value;
  /* Expression: (p.force.reference-(p.uncertainty.massB_model_kg+p.sensor.distalMass_model_kg)*p.d.g)/p.uncertainty.forceConstant_model_N_A
   * Referenced by: '<S33>/Flat holding current'
   */
  real_T Measurednormalforcedeviation_Bi;/* Expression: -p.force.reference
                                          * Referenced by: '<S28>/Measured normal force deviation'
                                          */
  real_T Normalforceerrorsign_Gain;    /* Expression: -1
                                        * Referenced by: '<S3>/Normal force error sign'
                                        */
  real_T Errortonormaldeviation_Gain;  /* Expression: -1
                                        * Referenced by: '<S36>/Error to normal deviation'
                                        */
  real_T Normalforceproportionalcurrent_;/* Expression: -p.inertial.forceKp
                                          * Referenced by: '<S36>/Normal force proportional current'
                                          */
  real_T Normalforceerrorintegral_gainva;
                          /* Computed Parameter: Normalforceerrorintegral_gainva
                           * Referenced by: '<S36>/Normal force error integral'
                           */
  real_T Normalforceerrorintegral_IC;  /* Expression: 0
                                        * Referenced by: '<S36>/Normal force error integral'
                                        */
  real_T Normalforceintegralcurrent_Gain;/* Expression: -p.inertial.forceKi
                                          * Referenced by: '<S36>/Normal force integral current'
                                          */
  real_T Normalerrorprevioussample_Initi;/* Expression: 0
                                          * Referenced by: '<S36>/Normal error previous sample'
                                          */
  real_T Normalerrornewderivativeweight_;
  /* Expression: (1-exp(-p.force.sampleTime/p.inertial.derivativeFilter))/p.force.sampleTime
   * Referenced by: '<S36>/Normal error new derivative weight'
   */
  real_T Normalerrorderivativememory_Ini;/* Expression: 0
                                          * Referenced by: '<S36>/Normal error derivative memory'
                                          */
  real_T Normalerroroldderivativeweight_;
             /* Expression: exp(-p.force.sampleTime/p.inertial.derivativeFilter)
              * Referenced by: '<S36>/Normal error old derivative weight'
              */
  real_T Normalforcederivativecurrent_Ga;
        /* Expression: -p.inertial.forceKd*double(strcmp(p.coil.strategy,'pid'))
         * Referenced by: '<S36>/Normal force derivative current'
         */
  real_T Optionalfeedbackrolloff_NumCoef[3];
                                     /* Expression: p.d.feedbackFilter.numerator
                                      * Referenced by: '<S36>/Optional feedback rolloff'
                                      */
  real_T Optionalfeedbackrolloff_DenCoef[3];
                                   /* Expression: p.d.feedbackFilter.denominator
                                    * Referenced by: '<S36>/Optional feedback rolloff'
                                    */
  real_T Optionalfeedbackrolloff_Initial[2];
                                 /* Expression: p.d.feedbackFilter.initialStates
                                  * Referenced by: '<S36>/Optional feedback rolloff'
                                  */
  real_T Filteredmeasuredaccelerationrat[2];
                   /* Expression: p.d.accelerationPrediction.derivativeNumerator
                    * Referenced by: '<S35>/Filtered measured acceleration rate'
                    */
  real_T Filteredmeasuredaccelerationr_f[2];
                 /* Expression: p.d.accelerationPrediction.derivativeDenominator
                  * Referenced by: '<S35>/Filtered measured acceleration rate'
                  */
  real_T Filteredmeasuredaccelerationr_g;/* Expression: 0
                                          * Referenced by: '<S35>/Filtered measured acceleration rate'
                                          */
  real_T AccelerationFFpredictiontime_Ga;
                              /* Expression: p.inertial.accelerationPrediction_s
                               * Referenced by: '<S35>/Acceleration FF prediction time'
                               */
  real_T Measuredaccelerationcurrentfeed;
  /* Expression: -p.inertial.accelerationFeedforwardScale*(p.uncertainty.massB_model_kg+p.sensor.distalMass_model_kg)/p.uncertainty.forceConstant_model_N_A
   * Referenced by: '<S35>/Measured acceleration current feedforward'
   */
  real_T Minimumobservablenormalforce_Va;
                                    /* Expression: p.contact.dragRatioMinForce_N
                                     * Referenced by: '<S32>/Minimum observable normal force'
                                     */
  real_T Previousfiltereddragratio_Initi;/* Expression: 0
                                          * Referenced by: '<S32>/Previous filtered drag ratio'
                                          */
  real_T Newdragestimateweight_Gain;   /* Expression: p.d.dragRatio.inputWeight
                                        * Referenced by: '<S32>/New drag estimate weight'
                                        */
  real_T Previousdragestimateweight_Gain;/* Expression: p.d.dragRatio.memoryWeight
                                          * Referenced by: '<S32>/Previous drag estimate weight'
                                          */
  real_T Measureddraggeometryselected_Va;
     /* Expression: double(strcmp(p.contact.geometryMode,'measured_drag_ratio'))
      * Referenced by: '<S28>/Measured drag geometry selected'
      */
  real_T Selectaxialforcegeometryrequest;/* Expression: .5
                                          * Referenced by: '<S28>/Select axial force geometry request'
                                          */
  real_T Axialreferencedeviation_Bias; /* Expression: -p.force.reference
                                        * Referenced by: '<S28>/Axial reference deviation'
                                        */
  real_T Geometrycurrentfeedforward_Gain;
          /* Expression: p.contact.geometryFeedforwardScale/p.coil.forceConstant
           * Referenced by: '<S35>/Geometry current feedforward'
           */
  real_T Identifiedguidepredictiontime_G;
                             /* Expression: p.inertial.guideVelocityPrediction_s
                              * Referenced by: '<S35>/Identified guide prediction time'
                              */
  real_T Guideviscousfrictioncurrent_Gai;
  /* Expression: p.inertial.guideFrictionFeedforwardScale*p.coil.viscousFriction/p.coil.forceConstant
   * Referenced by: '<S35>/Guide viscous friction current'
   */
  real_T Guidefrictionvelocityscale_Gain;/* Expression: 1/p.coil.frictionSpeed
                                          * Referenced by: '<S35>/Guide friction velocity scale'
                                          */
  real_T GuideCoulombfrictioncurrent_Gai;
  /* Expression: p.inertial.guideFrictionFeedforwardScale*p.coil.coulombForce/p.coil.forceConstant
   * Referenced by: '<S35>/Guide Coulomb friction current'
   */
  real_T KnowndownstreamSinertia_Gain;
                                     /* Expression: p.sensor.distalMass_model_kg
                                      * Referenced by: '<S34>/Known downstream S inertia'
                                      */
  real_T Properidentifiedlowerlinkinvers[4];
                            /* Expression: p.inertial.lowerLoadInverse.numerator
                             * Referenced by: '<S34>/Proper identified lower-link inverse'
                             */
  real_T Properidentifiedlowerlinkinve_p[4];
                          /* Expression: p.inertial.lowerLoadInverse.denominator
                           * Referenced by: '<S34>/Proper identified lower-link inverse'
                           */
  real_T Properidentifiedlowerlinkinve_a[3];/* Expression: zeros(3,1)
                                             * Referenced by: '<S34>/Proper identified lower-link inverse'
                                             */
  real_T Optionalinversefeedforwardscale;
  /* Expression: double(p.inertial.lowerLoadInverse.enabled)*p.inertial.lowerLoadInverse.scale
   * Referenced by: '<S34>/Optional inverse feedforward scale'
   */
  real_T Baselineorexternalcurrent_Thres;/* Expression: .5
                                          * Referenced by: '<S33>/Baseline or external current'
                                          */
  real_T Optionalstructuralmodecurrentno[3];/* Expression: p.d.currentNotch.numerator
                                             * Referenced by: '<S33>/Optional structural-mode current notch'
                                             */
  real_T Optionalstructuralmodecurrent_c[3];
                                     /* Expression: p.d.currentNotch.denominator
                                      * Referenced by: '<S33>/Optional structural-mode current notch'
                                      */
  real_T Optionalstructuralmodecurrent_i[2];
                                   /* Expression: p.d.currentNotch.initialStates
                                    * Referenced by: '<S33>/Optional structural-mode current notch'
                                    */
  real_T Currentlimits_UpperSat;       /* Expression: p.coil.peakCurrent
                                        * Referenced by: '<S33>/Current limits'
                                        */
  real_T Currentlimits_LowerSat;       /* Expression: -p.coil.peakCurrent
                                        * Referenced by: '<S33>/Current limits'
                                        */
  real_T Normalcommandchange1_Time;    /* Expression: 1
                                        * Referenced by: '<S17>/Normal command change 1'
                                        */
  real_T Normalcommandchange1_Y0;      /* Expression: 0
                                        * Referenced by: '<S17>/Normal command change 1'
                                        */
  real_T Normalcommandchange1_YFinal;  /* Expression: 1
                                        * Referenced by: '<S17>/Normal command change 1'
                                        */
  real_T Normalcommandchange2_Time;    /* Expression: 2
                                        * Referenced by: '<S17>/Normal command change 2'
                                        */
  real_T Normalcommandchange2_Y0;      /* Expression: 0
                                        * Referenced by: '<S17>/Normal command change 2'
                                        */
  real_T Normalcommandchange2_YFinal;  /* Expression: -2
                                        * Referenced by: '<S17>/Normal command change 2'
                                        */
  real_T Normalcommandchange3_Time;    /* Expression: 3
                                        * Referenced by: '<S17>/Normal command change 3'
                                        */
  real_T Normalcommandchange3_Y0;      /* Expression: 0
                                        * Referenced by: '<S17>/Normal command change 3'
                                        */
  real_T Normalcommandchange3_YFinal;  /* Expression: 1
                                        * Referenced by: '<S17>/Normal command change 3'
                                        */
  real_T Nominalrobotvelocity_Value;   /* Expression: p.robot.nominalVelocity
                                        * Referenced by: '<S17>/Nominal robot velocity'
                                        */
  real_T Boundedprogramselected_Value;
                     /* Expression: double(p.robotPrediction.boundedNominalPath)
                      * Referenced by: '<S17>/Bounded program selected'
                      */
  real_T Boundedorconstantnormalprogram_;/* Expression: .5
                                          * Referenced by: '<S17>/Bounded or constant normal program'
                                          */
  real_T Eventstart_Value;             /* Expression: p.surface.startTime
                                        * Referenced by: '<S17>/Event start'
                                        */
  real_T Roboterrortimescale_Gain;     /* Expression: 1/p.robot.errorRiseTime
                                        * Referenced by: '<S17>/Robot error time scale'
                                        */
  real_T Roboterrortransition_UpperSat;/* Expression: 1
                                        * Referenced by: '<S17>/Robot error transition'
                                        */
  real_T Roboterrortransition_LowerSat;/* Expression: 0
                                        * Referenced by: '<S17>/Robot error transition'
                                        */
  real_T Smoothroboterrorslope_Coefs[5];/* Expression: [30 -60 30 0 0]
                                         * Referenced by: '<S17>/Smooth robot error slope'
                                         */
  real_T Robotpositioningerrorvelocity_G;
  /* Expression: -p.robot.errorForce/p.d.keq/p.robot.errorRiseTime*double(strcmp(p.surface.kind,'robot_error'))
   * Referenced by: '<S17>/Robot positioning error velocity'
   */
  real_T Midstroke_Value;              /* Expression: p.mechanics.midTravel
                                        * Referenced by: '<S12>/Mid stroke'
                                        */
  real_T Toolsensorneedsgapestimate_Valu;/* Expression: p.sensor.positionModel
                                          * Referenced by: '<S12>/Tool sensor needs gap estimate'
                                          */
  real_T Position_IC;                  /* Expression: p.d.u0
                                        * Referenced by: '<S21>/Position'
                                        */
  real_T Repeatinterval_Value;         /* Expression: p.surface.repeatPeriod
                                        * Referenced by: '<S17>/Repeat interval'
                                        */
  real_T Repeatselected_Value;
                        /* Expression: double(strcmp(p.surface.kind,'repeated'))
                         * Referenced by: '<S17>/Repeat selected'
                         */
  real_T Scanspeedoverwidth_Gain;      /* Expression: 1/p.d.duration
                                        * Referenced by: '<S17>/Scan speed over width'
                                        */
  real_T Positionwithinevent_UpperSat; /* Expression: 1
                                        * Referenced by: '<S17>/Position within event'
                                        */
  real_T Positionwithinevent_LowerSat; /* Expression: 0
                                        * Referenced by: '<S17>/Position within event'
                                        */
  real_T Rampselected_Value;/* Expression: double(strcmp(p.surface.kind,'ramp'))
                             * Referenced by: '<S17>/Ramp selected'
                             */
  real_T Surfacedisplacementenabled_Valu;
      /* Expression: double(~any(strcmp(p.surface.kind,{'flat','robot_error'})))
       * Referenced by: '<S17>/Surface displacement enabled'
       */
  real_T Zero_Value;                   /* Expression: 0
                                        * Referenced by: '<S17>/Zero'
                                        */
  real_T Disabled_Value;               /* Expression: 0
                                        * Referenced by: '<S17>/Disabled'
                                        */
  real_T SurfaceSIscale1_Gain;         /* Expression: p.surface.height
                                        * Referenced by: '<S17>/Surface SI scale 1'
                                        */
  real_T Coordinatereference_Value;    /* Expression: p.mechanics.midTravel
                                        * Referenced by: '<S13>/Coordinate reference'
                                        */
  real_T Usetoolheadposition_Value;    /* Expression: p.sensor.positionModel
                                        * Referenced by: '<S13>/Use toolhead position'
                                        */
  real_T Position_IC_n;                /* Expression: 0
                                        * Referenced by: '<S22>/Position'
                                        */
  real_T Midtravel_Value;              /* Expression: p.mechanics.midTravel
                                        * Referenced by: '<S23>/Mid travel'
                                        */
  real_T Positioncalibratedzero_Bias;
                                  /* Expression: -p.d.positionMeasurementInitial
                                   * Referenced by: '<S13>/Position calibrated zero'
                                   */
  real_T Residualpositioncalibration_Gai;
                                    /* Expression: 1+p.uncertainty.positionScale
                                     * Referenced by: '<S13>/Residual position calibration'
                                     */
  real_T Positioncalibratedorigin_Bias;
                                   /* Expression: p.d.positionMeasurementInitial
                                    * Referenced by: '<S13>/Position calibrated origin'
                                    */
  real_T Positionnoise_Minimum;        /* Expression: -sqrt(3)
                                        * Referenced by: '<S13>/Position noise'
                                        */
  real_T Positionnoise_Maximum;        /* Expression: sqrt(3)
                                        * Referenced by: '<S13>/Position noise'
                                        */
  real_T Positionnoise_Seed;           /* Expression: p.sensor.seed+1
                                        * Referenced by: '<S13>/Position noise'
                                        */
  real_T PositionnoiseRMS_Gain;
                             /* Expression: p.adcStudy.d.positionNoiseInputRms_m
                              * Referenced by: '<S13>/Position noise RMS'
                              */
  real_T Measurementdelay_Gain;        /* Expression: 1
                                        * Referenced by: '<S13>/Measurement delay'
                                        */
  real_T Recenterdeadband_Start;       /* Expression: -p.robot.deadband
                                        * Referenced by: '<S12>/Recenter deadband'
                                        */
  real_T Recenterdeadband_End;         /* Expression: p.robot.deadband
                                        * Referenced by: '<S12>/Recenter deadband'
                                        */
  real_T Recentergain_Gain;            /* Expression: p.uncertainty.recenterGain
                                        * Referenced by: '<S12>/Recenter gain'
                                        */
  real_T Holdrecentering_Value;        /* Expression: 0
                                        * Referenced by: '<S12>/Hold recentering'
                                        */
  real_T Gatedisabled_Value;   /* Expression: double(~p.robot.forceGate.enabled)
                                * Referenced by: '<S39>/Gate disabled'
                                */
  real_T Previousfaststablecount_Initial;/* Expression: 0
                                          * Referenced by: '<S41>/Previous fast stable count'
                                          */
  real_T Usereconstructedcontacterror_Va;
  /* Expression: double(p.sensor.distalMass_kg>0 && strcmp(p.actuator.kind,'voice_coil') && strcmp(p.coil.strategy,'observer'))
   * Referenced by: '<S12>/Use reconstructed contact error'
   */
  real_T Forceband_Value;           /* Expression: p.robot.forceGate.errorBand_N
                                     * Referenced by: '<S39>/Force band'
                                     */
  real_T Forcetrendfilter_NumCoef;
          /* Expression: 1-exp(-p.robot.sampleTime/p.robot.forceGate.filterTime)
           * Referenced by: '<S39>/Force trend filter'
           */
  real_T Forcetrendfilter_DenCoef[2];
       /* Expression: [1 -exp(-p.robot.sampleTime/p.robot.forceGate.filterTime)]
        * Referenced by: '<S39>/Force trend filter'
        */
  real_T Forcetrendfilter_InitialStates;/* Expression: 0
                                         * Referenced by: '<S39>/Force trend filter'
                                         */
  real_T Previousforcetrend_InitialCondi;/* Expression: 0
                                          * Referenced by: '<S39>/Previous force trend'
                                          */
  real_T Changepersecond_Gain;         /* Expression: 1/p.robot.sampleTime
                                        * Referenced by: '<S39>/Change per second'
                                        */
  real_T Forceratelimit_Value;      /* Expression: p.robot.forceGate.maxRate_N_s
                                     * Referenced by: '<S39>/Force rate limit'
                                     */
  real_T Normalforceabovetarget_Gain;  /* Expression: -1
                                        * Referenced by: '<S41>/Normal force above target'
                                        */
  real_T Fastnormalforcemean_NumCoef[2];
                                  /* Expression: p.d.variationGate.meanNumerator
                                   * Referenced by: '<S41>/Fast normal force mean'
                                   */
  real_T Fastnormalforcemean_DenCoef[2];
                                /* Expression: p.d.variationGate.meanDenominator
                                 * Referenced by: '<S41>/Fast normal force mean'
                                 */
  real_T Fastnormalforcemean_InitialStat;/* Expression: 0
                                          * Referenced by: '<S41>/Fast normal force mean'
                                          */
  real_T Fastforcevariance_NumCoef[2];
                              /* Expression: p.d.variationGate.varianceNumerator
                               * Referenced by: '<S41>/Fast force variance'
                               */
  real_T Fastforcevariance_DenCoef[2];
                            /* Expression: p.d.variationGate.varianceDenominator
                             * Referenced by: '<S41>/Fast force variance'
                             */
  real_T Fastforcevariance_InitialStates;/* Expression: 0
                                          * Referenced by: '<S41>/Fast force variance'
                                          */
  real_T Allowedforcevariance_Value;
                         /* Expression: p.robot.forceGate.variation.rmsLimit_N^2
                          * Referenced by: '<S41>/Allowed force variance'
                          */
  real_T Capfaststablecount_UpperSat;
                                   /* Expression: p.d.variationGate.dwellSamples
                                    * Referenced by: '<S41>/Cap fast stable count'
                                    */
  real_T Capfaststablecount_LowerSat;  /* Expression: 0
                                        * Referenced by: '<S41>/Cap fast stable count'
                                        */
  real_T Requiredfastdwellcount_Value;
                                   /* Expression: p.d.variationGate.dwellSamples
                                    * Referenced by: '<S41>/Required fast dwell count'
                                    */
  real_T Fastvariationgateenabled_Value;
                      /* Expression: double(p.robot.forceGate.variation.enabled)
                       * Referenced by: '<S39>/Fast variation gate enabled'
                       */
  real_T Previousstablecount_InitialCond;/* Expression: 0
                                          * Referenced by: '<S39>/Previous stable count'
                                          */
  real_T Capstablecount_UpperSat;
           /* Expression: 1+ceil(p.robot.forceGate.dwellTime/p.robot.sampleTime)
            * Referenced by: '<S39>/Cap stable count'
            */
  real_T Capstablecount_LowerSat;      /* Expression: 0
                                        * Referenced by: '<S39>/Cap stable count'
                                        */
  real_T Selectcontinuousfastdwell_Thres;/* Expression: .5
                                          * Referenced by: '<S39>/Select continuous fast dwell'
                                          */
  real_T Robotspeedlimits_UpperSat;    /* Expression: p.robot.maxSpeed
                                        * Referenced by: '<S12>/Robot speed limits'
                                        */
  real_T Robotspeedlimits_LowerSat;    /* Expression: -p.robot.maxSpeed
                                        * Referenced by: '<S12>/Robot speed limits'
                                        */
  real_T Preload_Value;                /* Expression: p.d.q10
                                        * Referenced by: '<S20>/Preload'
                                        */
  real_T SurfaceSIscale2_Gain;      /* Expression: p.surface.height/p.d.duration
                                     * Referenced by: '<S17>/Surface SI scale 2'
                                     */
  real_T Velocity_IC;                  /* Expression: 0
                                        * Referenced by: '<S22>/Velocity'
                                        */
  real_T Stiffness_Gain;               /* Expression: p.mechanics.k1
                                        * Referenced by: '<S20>/Stiffness'
                                        */
  real_T Damping_Gain;                 /* Expression: p.mechanics.c1
                                        * Referenced by: '<S20>/Damping'
                                        */
  real_T SurfaceSIscale3_Gain;    /* Expression: p.surface.height/p.d.duration^2
                                   * Referenced by: '<S17>/Surface SI scale 3'
                                   */
  real_T Gravityalongflangeaxis_Value; /* Expression: p.d.g
                                        * Referenced by: '<S23>/Gravity along flange axis'
                                        */
  real_T Toolmass_Gain;                /* Expression: p.mechanics.mS
                                        * Referenced by: '<S23>/Tool mass'
                                        */
  real_T Unilateralcontactforce_UpperSat;/* Expression: inf
                                          * Referenced by: '<S23>/Unilateral contact force'
                                          */
  real_T Unilateralcontactforce_LowerSat;/* Expression: 0
                                          * Referenced by: '<S23>/Unilateral contact force'
                                          */
  real_T Surfaceslope_Gain;            /* Expression: 1/p.surface.scanSpeed
                                        * Referenced by: '<S18>/Surface slope'
                                        */
  real_T Squaredslopeplusone_Bias_i;   /* Expression: 1
                                        * Referenced by: '<S18>/Squared slope plus one'
                                        */
  real_T Unity_Value;                  /* Expression: 1
                                        * Referenced by: '<S18>/Unity'
                                        */
  real_T Frictiontimesslope_Gain;      /* Expression: p.contact.mu
                                        * Referenced by: '<S18>/Friction times slope'
                                        */
  real_T Velocity_IC_j;                /* Expression: 0
                                        * Referenced by: '<S21>/Velocity'
                                        */
  real_T Windingcurrent_IC;            /* Expression: p.d.coilHoldCurrent
                                        * Referenced by: '<S25>/Winding current'
                                        */
  real_T Coilforceconstant_Gain;       /* Expression: p.coil.forceConstant
                                        * Referenced by: '<S25>/Coil force constant'
                                        */
  real_T Frictionspeedscale_Gain;      /* Expression: 1/p.coil.frictionSpeed
                                        * Referenced by: '<S25>/Friction speed scale'
                                        */
  real_T Coulombfriction_Gain;         /* Expression: p.coil.coulombForce
                                        * Referenced by: '<S25>/Coulomb friction'
                                        */
  real_T Viscousfriction_Gain;         /* Expression: p.coil.viscousFriction
                                        * Referenced by: '<S25>/Viscous friction'
                                        */
  real_T RLresetfixedqueuelength_Value_n;/* Expression: p.d.robotDelaySamples
                                          * Referenced by: '<S14>/RL reset-fixed queue length'
                                          */
  real_T Robotcommandsamples_InitialCond;
  /* Expression: p.robot.nominalVelocity*double(~p.robotPrediction.boundedNominalPath)
   * Referenced by: '<S14>/Robot command samples'
   */
  real_T Requestedrobotspeed_UpperSat; /* Expression: p.robot.maxSpeed
                                        * Referenced by: '<S15>/Requested robot speed'
                                        */
  real_T Requestedrobotspeed_LowerSat; /* Expression: -p.robot.maxSpeed
                                        * Referenced by: '<S15>/Requested robot speed'
                                        */
  real_T Robotvelocity_IC;             /* Expression: 0
                                        * Referenced by: '<S15>/Robot velocity'
                                        */
  real_T Velocityresponse_Gain;        /* Expression: 1/p.robot.responseTime
                                        * Referenced by: '<S15>/Velocity response'
                                        */
  real_T Robotaccelerationlimits_UpperSa;/* Expression: p.robot.maxAcceleration
                                          * Referenced by: '<S15>/Robot acceleration limits'
                                          */
  real_T Robotaccelerationlimits_LowerSa;/* Expression: -p.robot.maxAcceleration
                                          * Referenced by: '<S15>/Robot acceleration limits'
                                          */
  real_T Gravityatupperrigidattachment_V;/* Expression: p.d.g
                                          * Referenced by: '<S2>/Gravity at upper rigid attachment'
                                          */
  real_T Upperattachmentinertia_Gain;  /* Expression: p.mechanics.mA
                                        * Referenced by: '<S2>/Upper attachment inertia'
                                        */
  real_T Upperspringloadshare_Gain;    /* Expression: p.d.beta
                                        * Referenced by: '<S2>/Upper spring load share'
                                        */
  real_T Lowerspringloadshare_Gain;    /* Expression: p.d.alpha
                                        * Referenced by: '<S2>/Lower spring load share'
                                        */
  real_T Inversereducedmass_Gain;      /* Expression: 1/p.d.mu
                                        * Referenced by: '<S25>/Inverse reduced mass'
                                        */
  real_T Inversemass_Gain;             /* Expression: 1/p.mechanics.mA
                                        * Referenced by: '<S21>/Inverse mass'
                                        */
  real_T Gravity_Value;                /* Expression: p.d.g
                                        * Referenced by: '<S21>/Gravity'
                                        */
  real_T Inversemass_Gain_h;           /* Expression: 1/p.mechanics.mB
                                        * Referenced by: '<S22>/Inverse mass'
                                        */
  real_T Gravity_Value_i;              /* Expression: p.d.g
                                        * Referenced by: '<S22>/Gravity'
                                        */
  real_T Resistance_Gain;              /* Expression: p.coil.resistance
                                        * Referenced by: '<S25>/Resistance'
                                        */
  real_T BackEMF_Gain;                 /* Expression: p.coil.forceConstant
                                        * Referenced by: '<S25>/Back EMF'
                                        */
  real_T Currentrequestlimit_UpperSat; /* Expression: p.coil.peakCurrent
                                        * Referenced by: '<S25>/Current request limit'
                                        */
  real_T Currentrequestlimit_LowerSat; /* Expression: -p.coil.peakCurrent
                                        * Referenced by: '<S25>/Current request limit'
                                        */
  real_T Currentregulator_Gain;
                         /* Expression: p.coil.inductance/p.coil.currentResponse
                          * Referenced by: '<S25>/Current regulator'
                          */
  real_T Supplyvoltagelimits_UpperSat; /* Expression: p.coil.voltage
                                        * Referenced by: '<S25>/Supply voltage limits'
                                        */
  real_T Supplyvoltagelimits_LowerSat; /* Expression: -p.coil.voltage
                                        * Referenced by: '<S25>/Supply voltage limits'
                                        */
  real_T Zero_Value_i;                 /* Expression: 0
                                        * Referenced by: '<S25>/Zero'
                                        */
  real_T Equivalentscrewangularspeed_Gai;/* Expression: 1/p.d.ell
                                          * Referenced by: '<S23>/Equivalent screw angular speed'
                                          */
  real_T Zero_Value_d;                 /* Expression: 0
                                        * Referenced by: '<S23>/Zero'
                                        */
  real_T Minimumtravel_Value;          /* Expression: p.mechanics.travelMin
                                        * Referenced by: '<S23>/Minimum travel'
                                        */
  real_T Maximumtravel_Value;          /* Expression: p.mechanics.travelMax
                                        * Referenced by: '<S23>/Maximum travel'
                                        */
  real_T Peakcurrentthreshold_Value;  /* Expression: p.coil.peakCurrent*(1-1e-9)
                                       * Referenced by: '<S25>/Peak current threshold'
                                       */
  real_T Supplyvoltage_Value;          /* Expression: p.coil.voltage
                                        * Referenced by: '<S25>/Supply voltage'
                                        */
  real_T PhysicalSaccelerationX_Value; /* Expression: 0
                                        * Referenced by: '<S16>/Physical S acceleration X'
                                        */
  real_T Xgravity_Value;
                       /* Expression: p.mechanics.gravity*sin(p.mechanics.angle)
                        * Referenced by: '<S16>/X gravity'
                        */
  real_T AccelerationXframediagonal_Gain;
                           /* Expression: cos(p.uncertainty.accelFrameAngle_rad)
                            * Referenced by: '<S16>/Acceleration X frame diagonal'
                            */
  real_T Zgravity_Value;               /* Expression: p.d.g
                                        * Referenced by: '<S16>/Z gravity'
                                        */
  real_T AccelerationXframecrossaxis_Gai;
                          /* Expression: -sin(p.uncertainty.accelFrameAngle_rad)
                           * Referenced by: '<S16>/Acceleration X frame cross axis'
                           */
  real_T AccelerationZframediagonal_Gain;
                           /* Expression: cos(p.uncertainty.accelFrameAngle_rad)
                            * Referenced by: '<S16>/Acceleration Z frame diagonal'
                            */
  real_T AccelerationZframecrossaxis_Gai;
                           /* Expression: sin(p.uncertainty.accelFrameAngle_rad)
                            * Referenced by: '<S16>/Acceleration Z frame cross axis'
                            */
  real_T Continuousanalogfrontend_A_pr[3];
                            /* Computed Parameter: Continuousanalogfrontend_A_pr
                             * Referenced by: '<S42>/Continuous analog front end'
                             */
  real_T Continuousanalogfrontend_B_pr;
                            /* Computed Parameter: Continuousanalogfrontend_B_pr
                             * Referenced by: '<S42>/Continuous analog front end'
                             */
  real_T Continuousanalogfrontend_C_pr;
                            /* Computed Parameter: Continuousanalogfrontend_C_pr
                             * Referenced by: '<S42>/Continuous analog front end'
                             */
  real_T Continuousanalogfrontend_Initia[2];
  /* Expression: (((1+p.accel.scaleError)*(cos(p.uncertainty.accelFrameAngle_rad)*(p.mechanics.gravity*sin(p.mechanics.angle))-sin(p.uncertainty.accelFrameAngle_rad)*(p.d.g))))*p.adcStudy.d.channels.acceleration.analogSteadyStatePerUnitInput
   * Referenced by: '<S42>/Continuous analog front end'
   */
  real_T ADCanaloginputfilter_A_pr;
                                /* Computed Parameter: ADCanaloginputfilter_A_pr
                                 * Referenced by: '<S42>/ADC analog input filter'
                                 */
  real_T ADCanaloginputfilter_D_pr;
                                /* Computed Parameter: ADCanaloginputfilter_D_pr
                                 * Referenced by: '<S42>/ADC analog input filter'
                                 */
  real_T ADCanaloginputfilter_InitialCon;
  /* Expression: (((1+p.accel.scaleError)*(cos(p.uncertainty.accelFrameAngle_rad)*(p.mechanics.gravity*sin(p.mechanics.angle))-sin(p.uncertainty.accelFrameAngle_rad)*(p.d.g))))*p.adcStudy.d.converterAnalog.steadyStatePerUnitInput
   * Referenced by: '<S42>/ADC analog input filter'
   */
  real_T Nativeconverterimpulseresponse_[31];
                                 /* Expression: p.adcStudy.d.converter.numerator
                                  * Referenced by: '<S42>/Native converter impulse response'
                                  */
  real_T Nativeconverterimpulserespons_b[31];
                               /* Expression: p.adcStudy.d.converter.denominator
                                * Referenced by: '<S42>/Native converter impulse response'
                                */
  real_T Nativeconverterimpulserespons_h[30];
  /* Expression: (((1+p.accel.scaleError)*(cos(p.uncertainty.accelFrameAngle_rad)*(p.mechanics.gravity*sin(p.mechanics.angle))-sin(p.uncertainty.accelFrameAngle_rad)*(p.d.g))))*p.adcStudy.d.converter.steadyStatePerUnitInput
   * Referenced by: '<S42>/Native converter impulse response'
   */
  real_T Independentconverteroutputnoise;/* Expression: 0
                                          * Referenced by: '<S42>/Independent converter output noise'
                                          */
  real_T Independentconverteroutputnoi_o;
                          /* Computed Parameter: Independentconverteroutputnoi_o
                           * Referenced by: '<S42>/Independent converter output noise'
                           */
  real_T Independentconverteroutputnoi_e;/* Expression: p.sensor.seed+3540
                                          * Referenced by: '<S42>/Independent converter output noise'
                                          */
  real_T ConverteroutputnoiseRMS_Gain;
                          /* Expression: p.adcStudy.accelerationAdcNoiseRms_m_s2
                           * Referenced by: '<S42>/Converter output noise RMS'
                           */
  real_T ADCoutputnoisecorrelation_NumCo[2];
                            /* Expression: p.adcStudy.d.converterNoise.numerator
                             * Referenced by: '<S42>/ADC output noise correlation'
                             */
  real_T ADCoutputnoisecorrelation_DenCo[2];
                          /* Expression: p.adcStudy.d.converterNoise.denominator
                           * Referenced by: '<S42>/ADC output noise correlation'
                           */
  real_T ADCoutputnoisecorrelation_Initi;/* Expression: 0
                                          * Referenced by: '<S42>/ADC output noise correlation'
                                          */
  real_T Restorephysicalstaticoffset_Bia;/* Expression: 0
                                          * Referenced by: '<S42>/Restore physical static offset'
                                          */
  real_T ActualADCinputrange_UpperSat;
  /* Expression: min(p.adcStudy.accelerationAdcRange_g,p.accel.range_g)*p.mechanics.gravity
   * Referenced by: '<S42>/Actual ADC input range'
   */
  real_T ActualADCinputrange_LowerSat;
  /* Expression: -(min(p.adcStudy.accelerationAdcRange_g,p.accel.range_g)*p.mechanics.gravity)
   * Referenced by: '<S42>/Actual ADC input range'
   */
  real_T Actualconverterquantization_Int;
                   /* Expression: max(p.adcStudy.accelerationQuantum_m_s2,1e-15)
                    * Referenced by: '<S42>/Actual converter quantization'
                    */
  real_T Knownquantizedstaticoffset_Bias;
  /* Expression: -round((0)/(max(p.adcStudy.accelerationQuantum_m_s2,1e-15)))*(max(p.adcStudy.accelerationQuantum_m_s2,1e-15))
   * Referenced by: '<S42>/Known quantized static offset'
   */
  real_T FreshperconversionEMA_NumCoef[2];
                  /* Expression: p.adcStudy.d.channels.acceleration.emaNumerator
                   * Referenced by: '<S42>/Fresh per-conversion EMA'
                   */
  real_T FreshperconversionEMA_DenCoef[2];
                /* Expression: p.adcStudy.d.channels.acceleration.emaDenominator
                 * Referenced by: '<S42>/Fresh per-conversion EMA'
                 */
  real_T FreshperconversionEMA_InitialSt;
  /* Expression: (round(((((1+p.accel.scaleError)*(cos(p.uncertainty.accelFrameAngle_rad)*(p.mechanics.gravity*sin(p.mechanics.angle))-sin(p.uncertainty.accelFrameAngle_rad)*(p.d.g))))+(0))/(max(p.adcStudy.accelerationQuantum_m_s2,1e-15)))*(max(p.adcStudy.accelerationQuantum_m_s2,1e-15))-round((0)/(max(p.adcStudy.accelerationQuantum_m_s2,1e-15)))*(max(p.adcStudy.accelerationQuantum_m_s2,1e-15)))*p.adcStudy.d.channels.acceleration.emaSteadyStatePerUnitInput
   * Referenced by: '<S42>/Fresh per-conversion EMA'
   */
  real_T Xuseengineeredacquisition_Value;
                                    /* Expression: double(p.acquisition.enabled)
                                     * Referenced by: '<S16>/X use engineered acquisition'
                                     */
  real_T Xsensorscale_Gain;            /* Expression: 1+p.accel.scaleError
                                        * Referenced by: '<S16>/X sensor scale'
                                        */
  real_T Xrawnoise_Mean;               /* Expression: 0
                                        * Referenced by: '<S16>/X raw noise'
                                        */
  real_T Xrawnoise_StdDev;             /* Computed Parameter: Xrawnoise_StdDev
                                        * Referenced by: '<S16>/X raw noise'
                                        */
  real_T Xrawnoise_Seed;               /* Expression: p.sensor.seed+336
                                        * Referenced by: '<S16>/X raw noise'
                                        */
  real_T XrawnoiseRMS_Gain;
                      /* Expression: p.adcStudy.d.accelerationNoiseInputRms_m_s2
                       * Referenced by: '<S16>/X raw noise RMS'
                       */
  real_T Xanalogvibrationprobe_Amp;
                 /* Expression: p.adcStudy.accelerationXAnalogToneAmplitude_m_s2
                  * Referenced by: '<S16>/X analog vibration probe'
                  */
  real_T Xanalogvibrationprobe_Bias;   /* Expression: 0
                                        * Referenced by: '<S16>/X analog vibration probe'
                                        */
  real_T Xanalogvibrationprobe_Freq;
                           /* Expression: 2*pi*p.adcStudy.analogToneFrequency_Hz
                            * Referenced by: '<S16>/X analog vibration probe'
                            */
  real_T Xanalogvibrationprobe_Phase;
                                   /* Expression: p.adcStudy.analogTonePhase_rad
                                    * Referenced by: '<S16>/X analog vibration probe'
                                    */
  real_T Xnewsampleweight_Gain;
                   /* Expression: 1-exp(-p.sensor.sampleTime/p.accel.filterTime)
                    * Referenced by: '<S16>/X new sample weight'
                    */
  real_T Xfiltermemory_InitialCondition;
  /* Expression: (1+p.accel.scaleError)*(cos(p.uncertainty.accelFrameAngle_rad)*(p.mechanics.gravity*sin(p.mechanics.angle))-sin(p.uncertainty.accelFrameAngle_rad)*(p.d.g))
   * Referenced by: '<S16>/X filter memory'
   */
  real_T Xoldsampleweight_Gain;
                     /* Expression: exp(-p.sensor.sampleTime/p.accel.filterTime)
                      * Referenced by: '<S16>/X old sample weight'
                      */
  real_T Xselectidentifiedsensorresponse;/* Expression: .5
                                          * Referenced by: '<S16>/X select identified sensor response'
                                          */
  real_T Xaccelerationrange_UpperSat;  /* Expression: inf
                                        * Referenced by: '<S16>/X acceleration range'
                                        */
  real_T Xaccelerationrange_LowerSat;  /* Expression: -inf
                                        * Referenced by: '<S16>/X acceleration range'
                                        */
  real_T XADCquantization_Interval;    /* Expression: 1e-15
                                        * Referenced by: '<S16>/X ADC quantization'
                                        */
  real_T Xdriftaftertare_Value;     /* Expression: p.uncertainty.accelBiasX_m_s2
                                     * Referenced by: '<S16>/X drift after tare'
                                     */
  real_T Xknownstatictare_Value;
  /* Expression: round((((1+p.accel.scaleError)*(cos(p.uncertainty.accelFrameAngle_rad)*(p.mechanics.gravity*sin(p.mechanics.angle))-sin(p.uncertainty.accelFrameAngle_rad)*(p.d.g))))/p.accel.quantum)*p.accel.quantum
   * Referenced by: '<S16>/X known static tare'
   */
  real_T Continuousanalogfrontend_A_pr_g[3];
                          /* Computed Parameter: Continuousanalogfrontend_A_pr_g
                           * Referenced by: '<S43>/Continuous analog front end'
                           */
  real_T Continuousanalogfrontend_B_pr_a;
                          /* Computed Parameter: Continuousanalogfrontend_B_pr_a
                           * Referenced by: '<S43>/Continuous analog front end'
                           */
  real_T Continuousanalogfrontend_C_pr_m;
                          /* Computed Parameter: Continuousanalogfrontend_C_pr_m
                           * Referenced by: '<S43>/Continuous analog front end'
                           */
  real_T Continuousanalogfrontend_Init_k[2];
  /* Expression: (((1+p.accel.scaleError)*(cos(p.uncertainty.accelFrameAngle_rad)*(p.d.g)+sin(p.uncertainty.accelFrameAngle_rad)*(p.mechanics.gravity*sin(p.mechanics.angle)))))*p.adcStudy.d.channels.acceleration.analogSteadyStatePerUnitInput
   * Referenced by: '<S43>/Continuous analog front end'
   */
  real_T ADCanaloginputfilter_A_pr_g;
                              /* Computed Parameter: ADCanaloginputfilter_A_pr_g
                               * Referenced by: '<S43>/ADC analog input filter'
                               */
  real_T ADCanaloginputfilter_D_pr_g;
                              /* Computed Parameter: ADCanaloginputfilter_D_pr_g
                               * Referenced by: '<S43>/ADC analog input filter'
                               */
  real_T ADCanaloginputfilter_InitialC_c;
  /* Expression: (((1+p.accel.scaleError)*(cos(p.uncertainty.accelFrameAngle_rad)*(p.d.g)+sin(p.uncertainty.accelFrameAngle_rad)*(p.mechanics.gravity*sin(p.mechanics.angle)))))*p.adcStudy.d.converterAnalog.steadyStatePerUnitInput
   * Referenced by: '<S43>/ADC analog input filter'
   */
  real_T Nativeconverterimpulserespons_k[31];
                                 /* Expression: p.adcStudy.d.converter.numerator
                                  * Referenced by: '<S43>/Native converter impulse response'
                                  */
  real_T Nativeconverterimpulserespons_d[31];
                               /* Expression: p.adcStudy.d.converter.denominator
                                * Referenced by: '<S43>/Native converter impulse response'
                                */
  real_T Nativeconverterimpulserespon_dz[30];
  /* Expression: (((1+p.accel.scaleError)*(cos(p.uncertainty.accelFrameAngle_rad)*(p.d.g)+sin(p.uncertainty.accelFrameAngle_rad)*(p.mechanics.gravity*sin(p.mechanics.angle)))))*p.adcStudy.d.converter.steadyStatePerUnitInput
   * Referenced by: '<S43>/Native converter impulse response'
   */
  real_T Independentconverteroutputnoi_k;/* Expression: 0
                                          * Referenced by: '<S43>/Independent converter output noise'
                                          */
  real_T Independentconverteroutputnoi_p;
                          /* Computed Parameter: Independentconverteroutputnoi_p
                           * Referenced by: '<S43>/Independent converter output noise'
                           */
  real_T Independentconverteroutputnoi_a;/* Expression: p.sensor.seed+3542
                                          * Referenced by: '<S43>/Independent converter output noise'
                                          */
  real_T ConverteroutputnoiseRMS_Gain_f;
                          /* Expression: p.adcStudy.accelerationAdcNoiseRms_m_s2
                           * Referenced by: '<S43>/Converter output noise RMS'
                           */
  real_T ADCoutputnoisecorrelation_Num_p[2];
                            /* Expression: p.adcStudy.d.converterNoise.numerator
                             * Referenced by: '<S43>/ADC output noise correlation'
                             */
  real_T ADCoutputnoisecorrelation_Den_f[2];
                          /* Expression: p.adcStudy.d.converterNoise.denominator
                           * Referenced by: '<S43>/ADC output noise correlation'
                           */
  real_T ADCoutputnoisecorrelation_Ini_f;/* Expression: 0
                                          * Referenced by: '<S43>/ADC output noise correlation'
                                          */
  real_T Restorephysicalstaticoffset_B_a;/* Expression: 0
                                          * Referenced by: '<S43>/Restore physical static offset'
                                          */
  real_T ActualADCinputrange_UpperSat_e;
  /* Expression: min(p.adcStudy.accelerationAdcRange_g,p.accel.range_g)*p.mechanics.gravity
   * Referenced by: '<S43>/Actual ADC input range'
   */
  real_T ActualADCinputrange_LowerSat_i;
  /* Expression: -(min(p.adcStudy.accelerationAdcRange_g,p.accel.range_g)*p.mechanics.gravity)
   * Referenced by: '<S43>/Actual ADC input range'
   */
  real_T Actualconverterquantization_I_f;
                   /* Expression: max(p.adcStudy.accelerationQuantum_m_s2,1e-15)
                    * Referenced by: '<S43>/Actual converter quantization'
                    */
  real_T Knownquantizedstaticoffset_Bi_g;
  /* Expression: -round((0)/(max(p.adcStudy.accelerationQuantum_m_s2,1e-15)))*(max(p.adcStudy.accelerationQuantum_m_s2,1e-15))
   * Referenced by: '<S43>/Known quantized static offset'
   */
  real_T FreshperconversionEMA_NumCoef_m[2];
                  /* Expression: p.adcStudy.d.channels.acceleration.emaNumerator
                   * Referenced by: '<S43>/Fresh per-conversion EMA'
                   */
  real_T FreshperconversionEMA_DenCoef_h[2];
                /* Expression: p.adcStudy.d.channels.acceleration.emaDenominator
                 * Referenced by: '<S43>/Fresh per-conversion EMA'
                 */
  real_T FreshperconversionEMA_Initial_g;
  /* Expression: (round(((((1+p.accel.scaleError)*(cos(p.uncertainty.accelFrameAngle_rad)*(p.d.g)+sin(p.uncertainty.accelFrameAngle_rad)*(p.mechanics.gravity*sin(p.mechanics.angle)))))+(0))/(max(p.adcStudy.accelerationQuantum_m_s2,1e-15)))*(max(p.adcStudy.accelerationQuantum_m_s2,1e-15))-round((0)/(max(p.adcStudy.accelerationQuantum_m_s2,1e-15)))*(max(p.adcStudy.accelerationQuantum_m_s2,1e-15)))*p.adcStudy.d.channels.acceleration.emaSteadyStatePerUnitInput
   * Referenced by: '<S43>/Fresh per-conversion EMA'
   */
  real_T Zuseengineeredacquisition_Value;
                                    /* Expression: double(p.acquisition.enabled)
                                     * Referenced by: '<S16>/Z use engineered acquisition'
                                     */
  real_T Zsensorscale_Gain;            /* Expression: 1+p.accel.scaleError
                                        * Referenced by: '<S16>/Z sensor scale'
                                        */
  real_T Zrawnoise_Mean;               /* Expression: 0
                                        * Referenced by: '<S16>/Z raw noise'
                                        */
  real_T Zrawnoise_StdDev;             /* Computed Parameter: Zrawnoise_StdDev
                                        * Referenced by: '<S16>/Z raw noise'
                                        */
  real_T Zrawnoise_Seed;               /* Expression: p.sensor.seed+239
                                        * Referenced by: '<S16>/Z raw noise'
                                        */
  real_T ZrawnoiseRMS_Gain;
                      /* Expression: p.adcStudy.d.accelerationNoiseInputRms_m_s2
                       * Referenced by: '<S16>/Z raw noise RMS'
                       */
  real_T Zanalogvibrationprobe_Amp;
                 /* Expression: p.adcStudy.accelerationZAnalogToneAmplitude_m_s2
                  * Referenced by: '<S16>/Z analog vibration probe'
                  */
  real_T Zanalogvibrationprobe_Bias;   /* Expression: 0
                                        * Referenced by: '<S16>/Z analog vibration probe'
                                        */
  real_T Zanalogvibrationprobe_Freq;
                           /* Expression: 2*pi*p.adcStudy.analogToneFrequency_Hz
                            * Referenced by: '<S16>/Z analog vibration probe'
                            */
  real_T Zanalogvibrationprobe_Phase;
                                   /* Expression: p.adcStudy.analogTonePhase_rad
                                    * Referenced by: '<S16>/Z analog vibration probe'
                                    */
  real_T Znewsampleweight_Gain;
                   /* Expression: 1-exp(-p.sensor.sampleTime/p.accel.filterTime)
                    * Referenced by: '<S16>/Z new sample weight'
                    */
  real_T Zfiltermemory_InitialCondition;
  /* Expression: (1+p.accel.scaleError)*(cos(p.uncertainty.accelFrameAngle_rad)*(p.d.g)+sin(p.uncertainty.accelFrameAngle_rad)*(p.mechanics.gravity*sin(p.mechanics.angle)))
   * Referenced by: '<S16>/Z filter memory'
   */
  real_T Zoldsampleweight_Gain;
                     /* Expression: exp(-p.sensor.sampleTime/p.accel.filterTime)
                      * Referenced by: '<S16>/Z old sample weight'
                      */
  real_T Zselectidentifiedsensorresponse;/* Expression: .5
                                          * Referenced by: '<S16>/Z select identified sensor response'
                                          */
  real_T Zaccelerationrange_UpperSat;  /* Expression: inf
                                        * Referenced by: '<S16>/Z acceleration range'
                                        */
  real_T Zaccelerationrange_LowerSat;  /* Expression: -inf
                                        * Referenced by: '<S16>/Z acceleration range'
                                        */
  real_T ZADCquantization_Interval;    /* Expression: 1e-15
                                        * Referenced by: '<S16>/Z ADC quantization'
                                        */
  real_T Zdriftaftertare_Value;        /* Expression: p.accel.bias
                                        * Referenced by: '<S16>/Z drift after tare'
                                        */
  real_T Zknownstatictare_Value;
  /* Expression: round((((1+p.accel.scaleError)*(cos(p.uncertainty.accelFrameAngle_rad)*(p.d.g)+sin(p.uncertainty.accelFrameAngle_rad)*(p.mechanics.gravity*sin(p.mechanics.angle)))))/p.accel.quantum)*p.accel.quantum
   * Referenced by: '<S16>/Z known static tare'
   */
  real_T Slopeplusfriction_Bias;       /* Expression: p.contact.mu
                                        * Referenced by: '<S18>/Slope plus friction'
                                        */
  real_T Lateralsign_Gain;             /* Expression: -1
                                        * Referenced by: '<S18>/Lateral sign'
                                        */
  real_T Inverseinductance_Gain;       /* Expression: 1/p.coil.inductance
                                        * Referenced by: '<S25>/Inverse inductance'
                                        */
  real_T Measuredheadcoordinatedeviation;
                                      /* Expression: p.d.positionMeasurementBias
                                       * Referenced by: '<S27>/Measured head coordinate deviation'
                                       */
  real_T Headgapcoordinatesign_Gain;   /* Expression: -1
                                        * Referenced by: '<S27>/Head gap coordinate sign'
                                        */
  real_T Continuousanalogfrontend_A_pr_a[3];
                          /* Computed Parameter: Continuousanalogfrontend_A_pr_a
                           * Referenced by: '<S29>/Continuous analog front end'
                           */
  real_T Continuousanalogfrontend_B_pr_d;
                          /* Computed Parameter: Continuousanalogfrontend_B_pr_d
                           * Referenced by: '<S29>/Continuous analog front end'
                           */
  real_T Continuousanalogfrontend_C_pr_d;
                          /* Computed Parameter: Continuousanalogfrontend_C_pr_d
                           * Referenced by: '<S29>/Continuous analog front end'
                           */
  real_T Continuousanalogfrontend_Init_j[2];
  /* Expression: (0)*p.adcStudy.d.channels.position.analogSteadyStatePerUnitInput
   * Referenced by: '<S29>/Continuous analog front end'
   */
  real_T ADCanaloginputfilter_A_pr_o;
                              /* Computed Parameter: ADCanaloginputfilter_A_pr_o
                               * Referenced by: '<S29>/ADC analog input filter'
                               */
  real_T ADCanaloginputfilter_D_pr_f;
                              /* Computed Parameter: ADCanaloginputfilter_D_pr_f
                               * Referenced by: '<S29>/ADC analog input filter'
                               */
  real_T ADCanaloginputfilter_InitialC_m;
         /* Expression: (0)*p.adcStudy.d.converterAnalog.steadyStatePerUnitInput
          * Referenced by: '<S29>/ADC analog input filter'
          */
  real_T Nativeconverterimpulserespon_kh[31];
                                 /* Expression: p.adcStudy.d.converter.numerator
                                  * Referenced by: '<S29>/Native converter impulse response'
                                  */
  real_T Nativeconverterimpulserespons_n[31];
                               /* Expression: p.adcStudy.d.converter.denominator
                                * Referenced by: '<S29>/Native converter impulse response'
                                */
  real_T Nativeconverterimpulserespon_kc[30];
               /* Expression: (0)*p.adcStudy.d.converter.steadyStatePerUnitInput
                * Referenced by: '<S29>/Native converter impulse response'
                */
  real_T FreshperconversionEMA_NumCoef_l[2];
                      /* Expression: p.adcStudy.d.channels.position.emaNumerator
                       * Referenced by: '<S29>/Fresh per-conversion EMA'
                       */
  real_T FreshperconversionEMA_DenCoef_j[2];
                    /* Expression: p.adcStudy.d.channels.position.emaDenominator
                     * Referenced by: '<S29>/Fresh per-conversion EMA'
                     */
  real_T FreshperconversionEMA_Initial_h;
    /* Expression: (0)*p.adcStudy.d.channels.position.emaSteadyStatePerUnitInput
     * Referenced by: '<S29>/Fresh per-conversion EMA'
     */
  real_T Positionuseengineeredacquisitio;
                                    /* Expression: double(p.acquisition.enabled)
                                     * Referenced by: '<S27>/Position use engineered acquisition'
                                     */
  real_T Positionframematching_A; /* Computed Parameter: Positionframematching_A
                                   * Referenced by: '<S27>/Position frame matching'
                                   */
  real_T Positionframematching_B; /* Computed Parameter: Positionframematching_B
                                   * Referenced by: '<S27>/Position frame matching'
                                   */
  real_T Positionframematching_C; /* Computed Parameter: Positionframematching_C
                                   * Referenced by: '<S27>/Position frame matching'
                                   */
  real_T Positionframematching_D; /* Computed Parameter: Positionframematching_D
                                   * Referenced by: '<S27>/Position frame matching'
                                   */
  real_T Positionframematching_InitialCo;/* Expression: 0
                                          * Referenced by: '<S27>/Position frame matching'
                                          */
  real_T Positionacquisitionframe_Thresh;/* Expression: .5
                                          * Referenced by: '<S27>/Position acquisition frame'
                                          */
  real_T Continuousanalogfrontend_A_pr_i[3];
                          /* Computed Parameter: Continuousanalogfrontend_A_pr_i
                           * Referenced by: '<S30>/Continuous analog front end'
                           */
  real_T Continuousanalogfrontend_B_pr_j;
                          /* Computed Parameter: Continuousanalogfrontend_B_pr_j
                           * Referenced by: '<S30>/Continuous analog front end'
                           */
  real_T Continuousanalogfrontend_C_pr_e;
                          /* Computed Parameter: Continuousanalogfrontend_C_pr_e
                           * Referenced by: '<S30>/Continuous analog front end'
                           */
  real_T Continuousanalogfrontend_Init_o[2];
    /* Expression: (0)*p.adcStudy.d.channels.frame.analogSteadyStatePerUnitInput
     * Referenced by: '<S30>/Continuous analog front end'
     */
  real_T ADCanaloginputfilter_A_pr_h;
                              /* Computed Parameter: ADCanaloginputfilter_A_pr_h
                               * Referenced by: '<S30>/ADC analog input filter'
                               */
  real_T ADCanaloginputfilter_D_pr_j;
                              /* Computed Parameter: ADCanaloginputfilter_D_pr_j
                               * Referenced by: '<S30>/ADC analog input filter'
                               */
  real_T ADCanaloginputfilter_InitialC_p;
         /* Expression: (0)*p.adcStudy.d.converterAnalog.steadyStatePerUnitInput
          * Referenced by: '<S30>/ADC analog input filter'
          */
  real_T Nativeconverterimpulserespon_dh[31];
                                 /* Expression: p.adcStudy.d.converter.numerator
                                  * Referenced by: '<S30>/Native converter impulse response'
                                  */
  real_T Nativeconverterimpulserespon_dv[31];
                               /* Expression: p.adcStudy.d.converter.denominator
                                * Referenced by: '<S30>/Native converter impulse response'
                                */
  real_T Nativeconverterimpulserespons_e[30];
               /* Expression: (0)*p.adcStudy.d.converter.steadyStatePerUnitInput
                * Referenced by: '<S30>/Native converter impulse response'
                */
  real_T FreshperconversionEMA_NumCoef_o[2];
                         /* Expression: p.adcStudy.d.channels.frame.emaNumerator
                          * Referenced by: '<S30>/Fresh per-conversion EMA'
                          */
  real_T FreshperconversionEMA_DenCoef_p[2];
                       /* Expression: p.adcStudy.d.channels.frame.emaDenominator
                        * Referenced by: '<S30>/Fresh per-conversion EMA'
                        */
  real_T FreshperconversionEMA_Initia_h0;
       /* Expression: (0)*p.adcStudy.d.channels.frame.emaSteadyStatePerUnitInput
        * Referenced by: '<S30>/Fresh per-conversion EMA'
        */
  real_T Robotaccelerationuseengineereda;
                                    /* Expression: double(p.acquisition.enabled)
                                     * Referenced by: '<S27>/Robot acceleration use engineered acquisition'
                                     */
  real_T Robotaccelerationframematching_;
                          /* Computed Parameter: Robotaccelerationframematching_
                           * Referenced by: '<S27>/Robot acceleration frame matching'
                           */
  real_T Robotaccelerationframematchin_k;
                          /* Computed Parameter: Robotaccelerationframematchin_k
                           * Referenced by: '<S27>/Robot acceleration frame matching'
                           */
  real_T Robotaccelerationframematchi_f0;
                          /* Computed Parameter: Robotaccelerationframematchi_f0
                           * Referenced by: '<S27>/Robot acceleration frame matching'
                           */
  real_T Robotaccelerationframematchin_d;
                          /* Computed Parameter: Robotaccelerationframematchin_d
                           * Referenced by: '<S27>/Robot acceleration frame matching'
                           */
  real_T Robotaccelerationframematchi_ka;/* Expression: 0
                                          * Referenced by: '<S27>/Robot acceleration frame matching'
                                          */
  real_T Robotaccelerationacquisitionfra;/* Expression: .5
                                          * Referenced by: '<S27>/Robot acceleration acquisition frame'
                                          */
  real_T Continuousanalogfrontend_A_pr_j[3];
                          /* Computed Parameter: Continuousanalogfrontend_A_pr_j
                           * Referenced by: '<S31>/Continuous analog front end'
                           */
  real_T Continuousanalogfrontend_B_p_jg;
                          /* Computed Parameter: Continuousanalogfrontend_B_p_jg
                           * Referenced by: '<S31>/Continuous analog front end'
                           */
  real_T Continuousanalogfrontend_C_pr_f;
                          /* Computed Parameter: Continuousanalogfrontend_C_pr_f
                           * Referenced by: '<S31>/Continuous analog front end'
                           */
  real_T Continuousanalogfrontend_Init_l[2];
    /* Expression: (0)*p.adcStudy.d.channels.frame.analogSteadyStatePerUnitInput
     * Referenced by: '<S31>/Continuous analog front end'
     */
  real_T ADCanaloginputfilter_A_pr_m;
                              /* Computed Parameter: ADCanaloginputfilter_A_pr_m
                               * Referenced by: '<S31>/ADC analog input filter'
                               */
  real_T ADCanaloginputfilter_D_pr_a;
                              /* Computed Parameter: ADCanaloginputfilter_D_pr_a
                               * Referenced by: '<S31>/ADC analog input filter'
                               */
  real_T ADCanaloginputfilter_InitialC_k;
         /* Expression: (0)*p.adcStudy.d.converterAnalog.steadyStatePerUnitInput
          * Referenced by: '<S31>/ADC analog input filter'
          */
  real_T Nativeconverterimpulserespon_hz[31];
                                 /* Expression: p.adcStudy.d.converter.numerator
                                  * Referenced by: '<S31>/Native converter impulse response'
                                  */
  real_T Nativeconverterimpulserespons_a[31];
                               /* Expression: p.adcStudy.d.converter.denominator
                                * Referenced by: '<S31>/Native converter impulse response'
                                */
  real_T Nativeconverterimpulserespon_ew[30];
               /* Expression: (0)*p.adcStudy.d.converter.steadyStatePerUnitInput
                * Referenced by: '<S31>/Native converter impulse response'
                */
  real_T FreshperconversionEMA_NumCoef_b[2];
                         /* Expression: p.adcStudy.d.channels.frame.emaNumerator
                          * Referenced by: '<S31>/Fresh per-conversion EMA'
                          */
  real_T FreshperconversionEMA_DenCoe_pq[2];
                       /* Expression: p.adcStudy.d.channels.frame.emaDenominator
                        * Referenced by: '<S31>/Fresh per-conversion EMA'
                        */
  real_T FreshperconversionEMA_Initial_j;
       /* Expression: (0)*p.adcStudy.d.channels.frame.emaSteadyStatePerUnitInput
        * Referenced by: '<S31>/Fresh per-conversion EMA'
        */
  real_T Robotvelocityuseengineeredacqui;
                                    /* Expression: double(p.acquisition.enabled)
                                     * Referenced by: '<S27>/Robot velocity use engineered acquisition'
                                     */
  real_T Robotvelocityframematching_A;
                             /* Computed Parameter: Robotvelocityframematching_A
                              * Referenced by: '<S27>/Robot velocity frame matching'
                              */
  real_T Robotvelocityframematching_B;
                             /* Computed Parameter: Robotvelocityframematching_B
                              * Referenced by: '<S27>/Robot velocity frame matching'
                              */
  real_T Robotvelocityframematching_C;
                             /* Computed Parameter: Robotvelocityframematching_C
                              * Referenced by: '<S27>/Robot velocity frame matching'
                              */
  real_T Robotvelocityframematching_D;
                             /* Computed Parameter: Robotvelocityframematching_D
                              * Referenced by: '<S27>/Robot velocity frame matching'
                              */
  real_T Robotvelocityframematching_Init;/* Expression: 0
                                          * Referenced by: '<S27>/Robot velocity frame matching'
                                          */
  real_T Robotvelocityacquisitionframe_T;/* Expression: .5
                                          * Referenced by: '<S27>/Robot velocity acquisition frame'
                                          */
  real_T Integralantiwindup_Gain;
                       /* Expression: -p.force.antiWindupRate/p.inertial.forceKi
                        * Referenced by: '<S33>/Integral anti windup'
                        */
  real_T Axialgravity_Value;           /* Expression: p.d.g
                                        * Referenced by: '<S5>/Axial gravity'
                                        */
  real_T Continuousanalogfrontend_A_pr_n[3];
                          /* Computed Parameter: Continuousanalogfrontend_A_pr_n
                           * Referenced by: '<S37>/Continuous analog front end'
                           */
  real_T Continuousanalogfrontend_B_pr_b;
                          /* Computed Parameter: Continuousanalogfrontend_B_pr_b
                           * Referenced by: '<S37>/Continuous analog front end'
                           */
  real_T Continuousanalogfrontend_C_p_ft;
                          /* Computed Parameter: Continuousanalogfrontend_C_p_ft
                           * Referenced by: '<S37>/Continuous analog front end'
                           */
  real_T Continuousanalogfrontend_Init_a[2];
  /* Expression: ((p.force.reference*(1+p.uncertainty.forceScaleZ)*(cos(p.uncertainty.forceFrameAngle_rad)-p.contact.mu*(sin(p.uncertainty.forceFrameAngle_rad)+p.uncertainty.forceCrossZFromX))))*p.adcStudy.d.channels.force.analogSteadyStatePerUnitInput
   * Referenced by: '<S37>/Continuous analog front end'
   */
  real_T ADCanaloginputfilter_A_pr_f;
                              /* Computed Parameter: ADCanaloginputfilter_A_pr_f
                               * Referenced by: '<S37>/ADC analog input filter'
                               */
  real_T ADCanaloginputfilter_D_pr_p;
                              /* Computed Parameter: ADCanaloginputfilter_D_pr_p
                               * Referenced by: '<S37>/ADC analog input filter'
                               */
  real_T ADCanaloginputfilter_Initial_me;
  /* Expression: ((p.force.reference*(1+p.uncertainty.forceScaleZ)*(cos(p.uncertainty.forceFrameAngle_rad)-p.contact.mu*(sin(p.uncertainty.forceFrameAngle_rad)+p.uncertainty.forceCrossZFromX))))*p.adcStudy.d.converterAnalog.steadyStatePerUnitInput
   * Referenced by: '<S37>/ADC analog input filter'
   */
  real_T Nativeconverterimpulserespons_j[31];
                                 /* Expression: p.adcStudy.d.converter.numerator
                                  * Referenced by: '<S37>/Native converter impulse response'
                                  */
  real_T Nativeconverterimpulserespons_l[31];
                               /* Expression: p.adcStudy.d.converter.denominator
                                * Referenced by: '<S37>/Native converter impulse response'
                                */
  real_T Nativeconverterimpulserespon_hk[30];
  /* Expression: ((p.force.reference*(1+p.uncertainty.forceScaleZ)*(cos(p.uncertainty.forceFrameAngle_rad)-p.contact.mu*(sin(p.uncertainty.forceFrameAngle_rad)+p.uncertainty.forceCrossZFromX))))*p.adcStudy.d.converter.steadyStatePerUnitInput
   * Referenced by: '<S37>/Native converter impulse response'
   */
  real_T Independentconverteroutputno_pm;/* Expression: 0
                                          * Referenced by: '<S37>/Independent converter output noise'
                                          */
  real_T Independentconverteroutputnoi_l;
                          /* Computed Parameter: Independentconverteroutputnoi_l
                           * Referenced by: '<S37>/Independent converter output noise'
                           */
  real_T Independentconverteroutputno_px;/* Expression: p.sensor.seed+2594
                                          * Referenced by: '<S37>/Independent converter output noise'
                                          */
  real_T ConverteroutputnoiseRMS_Gain_l;
                                    /* Expression: p.adcStudy.forceAdcNoiseRms_N
                                     * Referenced by: '<S37>/Converter output noise RMS'
                                     */
  real_T ADCoutputnoisecorrelation_Nu_pq[2];
                            /* Expression: p.adcStudy.d.converterNoise.numerator
                             * Referenced by: '<S37>/ADC output noise correlation'
                             */
  real_T ADCoutputnoisecorrelation_Den_d[2];
                          /* Expression: p.adcStudy.d.converterNoise.denominator
                           * Referenced by: '<S37>/ADC output noise correlation'
                           */
  real_T ADCoutputnoisecorrelation_Ini_o;/* Expression: 0
                                          * Referenced by: '<S37>/ADC output noise correlation'
                                          */
  real_T Restorephysicalstaticoffset_B_l;
                                /* Expression: p.adcStudy.d.forceGravityOffset_N
                                 * Referenced by: '<S37>/Restore physical static offset'
                                 */
  real_T ActualADCinputrange_UpperSat_i;/* Expression: p.adcStudy.forceAdcRange_N
                                         * Referenced by: '<S37>/Actual ADC input range'
                                         */
  real_T ActualADCinputrange_LowerSat_c;
                                    /* Expression: -(p.adcStudy.forceAdcRange_N)
                                     * Referenced by: '<S37>/Actual ADC input range'
                                     */
  real_T Actualconverterquantization_I_i;
                             /* Expression: max(p.adcStudy.forceQuantum_N,1e-15)
                              * Referenced by: '<S37>/Actual converter quantization'
                              */
  real_T Knownquantizedstaticoffset_Bi_c;
  /* Expression: -round((p.adcStudy.d.forceGravityOffset_N)/(max(p.adcStudy.forceQuantum_N,1e-15)))*(max(p.adcStudy.forceQuantum_N,1e-15))
   * Referenced by: '<S37>/Known quantized static offset'
   */
  real_T FreshperconversionEMA_NumCoef_n[2];
                         /* Expression: p.adcStudy.d.channels.force.emaNumerator
                          * Referenced by: '<S37>/Fresh per-conversion EMA'
                          */
  real_T FreshperconversionEMA_DenCoef_g[2];
                       /* Expression: p.adcStudy.d.channels.force.emaDenominator
                        * Referenced by: '<S37>/Fresh per-conversion EMA'
                        */
  real_T FreshperconversionEMA_Initial_a;
  /* Expression: (round((((p.force.reference*(1+p.uncertainty.forceScaleZ)*(cos(p.uncertainty.forceFrameAngle_rad)-p.contact.mu*(sin(p.uncertainty.forceFrameAngle_rad)+p.uncertainty.forceCrossZFromX))))+(p.adcStudy.d.forceGravityOffset_N))/(max(p.adcStudy.forceQuantum_N,1e-15)))*(max(p.adcStudy.forceQuantum_N,1e-15))-round((p.adcStudy.d.forceGravityOffset_N)/(max(p.adcStudy.forceQuantum_N,1e-15)))*(max(p.adcStudy.forceQuantum_N,1e-15)))*p.adcStudy.d.channels.force.emaSteadyStatePerUnitInput
   * Referenced by: '<S37>/Fresh per-conversion EMA'
   */
  real_T Sensorbias_Value;             /* Expression: p.sensor.bias
                                        * Referenced by: '<S5>/Sensor bias'
                                        */
  real_T Previousfilteredforce_InitialCo;
  /* Expression: p.force.reference*(1+p.uncertainty.forceScaleZ)*(cos(p.uncertainty.forceFrameAngle_rad)-p.contact.mu*(sin(p.uncertainty.forceFrameAngle_rad)+p.uncertainty.forceCrossZFromX))
   * Referenced by: '<S5>/Previous filtered force'
   */
  real_T Filtermemory_Gain;
  /* Expression: exp(-p.sensor.sampleTime/max(p.sensor.filterTime,realmin))*double(p.sensor.filterTime>0)
   * Referenced by: '<S5>/Filter memory'
   */
  real_T Knowndownstreamtoolmass_Gain; /* Expression: p.sensor.distalMass_kg
                                        * Referenced by: '<S5>/Known downstream tool mass'
                                        */
  real_T Knownweighttare_Value;      /* Expression: p.sensor.distalMass_kg*p.d.g
                                      * Referenced by: '<S5>/Known weight tare'
                                      */
  real_T ForceZframediagonal_Gain;
                           /* Expression: cos(p.uncertainty.forceFrameAngle_rad)
                            * Referenced by: '<S5>/Force Z frame diagonal'
                            */
  real_T ForceZframecrossaxis_Gain;
  /* Expression: sin(p.uncertainty.forceFrameAngle_rad)+p.uncertainty.forceCrossZFromX
   * Referenced by: '<S5>/Force Z frame cross axis'
   */
  real_T Forceanalogvibrationprobe_Amp;
                            /* Expression: p.adcStudy.forceAnalogToneAmplitude_N
                             * Referenced by: '<S5>/Force analog vibration probe'
                             */
  real_T Forceanalogvibrationprobe_Bias;/* Expression: 0
                                         * Referenced by: '<S5>/Force analog vibration probe'
                                         */
  real_T Forceanalogvibrationprobe_Freq;
                           /* Expression: 2*pi*p.adcStudy.analogToneFrequency_Hz
                            * Referenced by: '<S5>/Force analog vibration probe'
                            */
  real_T Forceanalogvibrationprobe_Phase;
                                   /* Expression: p.adcStudy.analogTonePhase_rad
                                    * Referenced by: '<S5>/Force analog vibration probe'
                                    */
  real_T Residualaxialcalibration_Gain;
                                      /* Expression: 1+p.uncertainty.forceScaleZ
                                       * Referenced by: '<S5>/Residual axial calibration'
                                       */
  real_T Newsampleweight_Gain;
  /* Expression: 1-(exp(-p.sensor.sampleTime/max(p.sensor.filterTime,realmin))*double(p.sensor.filterTime>0))
   * Referenced by: '<S5>/New sample weight'
   */
  real_T Uniformsensornoise_Minimum;   /* Expression: -sqrt(3)
                                        * Referenced by: '<S5>/Uniform sensor noise'
                                        */
  real_T Uniformsensornoise_Maximum;   /* Expression: sqrt(3)
                                        * Referenced by: '<S5>/Uniform sensor noise'
                                        */
  real_T Uniformsensornoise_Seed;      /* Expression: p.sensor.seed
                                        * Referenced by: '<S5>/Uniform sensor noise'
                                        */
  real_T SensornoiseRMS_Gain;   /* Expression: p.adcStudy.d.forceNoiseInputRms_N
                                 * Referenced by: '<S5>/Sensor noise RMS'
                                 */
  real_T Rawoutputnoisenormalization_Gai;/* Expression: 1
                                          * Referenced by: '<S5>/Raw output-noise normalization'
                                          */
  real_T Useengineeredacquisition_Value;
                                    /* Expression: double(p.acquisition.enabled)
                                     * Referenced by: '<S5>/Use engineered acquisition'
                                     */
  real_T Selectidentifiedsensorresponse_;/* Expression: .5
                                          * Referenced by: '<S5>/Select identified sensor response'
                                          */
  real_T Predictedrelativeposition_IC; /* Expression: 0
                                        * Referenced by: '<S7>/Predicted relative position'
                                        */
  real_T Robotposition_IC;             /* Expression: p.d.r0
                                        * Referenced by: '<S15>/Robot position'
                                        */
  real_T Continuousanalogfrontend_A_pr_p[3];
                          /* Computed Parameter: Continuousanalogfrontend_A_pr_p
                           * Referenced by: '<S38>/Continuous analog front end'
                           */
  real_T Continuousanalogfrontend_B_pr_f;
                          /* Computed Parameter: Continuousanalogfrontend_B_pr_f
                           * Referenced by: '<S38>/Continuous analog front end'
                           */
  real_T Continuousanalogfrontend_C_p_ms;
                          /* Computed Parameter: Continuousanalogfrontend_C_p_ms
                           * Referenced by: '<S38>/Continuous analog front end'
                           */
  real_T Continuousanalogfrontend_Ini_j5[2];
  /* Expression: ((p.force.reference*(1+p.uncertainty.forceScaleX)*(-p.contact.mu*cos(p.uncertainty.forceFrameAngle_rad)-sin(p.uncertainty.forceFrameAngle_rad)+p.uncertainty.forceCrossXFromZ)))*p.adcStudy.d.channels.lateral.analogSteadyStatePerUnitInput
   * Referenced by: '<S38>/Continuous analog front end'
   */
  real_T ADCanaloginputfilter_A_pr_n;
                              /* Computed Parameter: ADCanaloginputfilter_A_pr_n
                               * Referenced by: '<S38>/ADC analog input filter'
                               */
  real_T ADCanaloginputfilter_D_pr_p5;
                             /* Computed Parameter: ADCanaloginputfilter_D_pr_p5
                              * Referenced by: '<S38>/ADC analog input filter'
                              */
  real_T ADCanaloginputfilter_InitialC_l;
  /* Expression: ((p.force.reference*(1+p.uncertainty.forceScaleX)*(-p.contact.mu*cos(p.uncertainty.forceFrameAngle_rad)-sin(p.uncertainty.forceFrameAngle_rad)+p.uncertainty.forceCrossXFromZ)))*p.adcStudy.d.converterAnalog.steadyStatePerUnitInput
   * Referenced by: '<S38>/ADC analog input filter'
   */
  real_T Nativeconverterimpulserespon_e5[31];
                                 /* Expression: p.adcStudy.d.converter.numerator
                                  * Referenced by: '<S38>/Native converter impulse response'
                                  */
  real_T Nativeconverterimpulserespon_al[31];
                               /* Expression: p.adcStudy.d.converter.denominator
                                * Referenced by: '<S38>/Native converter impulse response'
                                */
  real_T Nativeconverterimpulserespons_o[30];
  /* Expression: ((p.force.reference*(1+p.uncertainty.forceScaleX)*(-p.contact.mu*cos(p.uncertainty.forceFrameAngle_rad)-sin(p.uncertainty.forceFrameAngle_rad)+p.uncertainty.forceCrossXFromZ)))*p.adcStudy.d.converter.steadyStatePerUnitInput
   * Referenced by: '<S38>/Native converter impulse response'
   */
  real_T Independentconverteroutputno_pu;/* Expression: 0
                                          * Referenced by: '<S38>/Independent converter output noise'
                                          */
  real_T Independentconverteroutputnoi_f;
                          /* Computed Parameter: Independentconverteroutputnoi_f
                           * Referenced by: '<S38>/Independent converter output noise'
                           */
  real_T Independentconverteroutputnoi_n;/* Expression: p.sensor.seed+2808
                                          * Referenced by: '<S38>/Independent converter output noise'
                                          */
  real_T ConverteroutputnoiseRMS_Gain_c;
                                    /* Expression: p.adcStudy.forceAdcNoiseRms_N
                                     * Referenced by: '<S38>/Converter output noise RMS'
                                     */
  real_T ADCoutputnoisecorrelation_Nu_pw[2];
                            /* Expression: p.adcStudy.d.converterNoise.numerator
                             * Referenced by: '<S38>/ADC output noise correlation'
                             */
  real_T ADCoutputnoisecorrelation_Den_j[2];
                          /* Expression: p.adcStudy.d.converterNoise.denominator
                           * Referenced by: '<S38>/ADC output noise correlation'
                           */
  real_T ADCoutputnoisecorrelation_Ini_j;/* Expression: 0
                                          * Referenced by: '<S38>/ADC output noise correlation'
                                          */
  real_T Restorephysicalstaticoffset_B_b;
                              /* Expression: p.adcStudy.d.lateralGravityOffset_N
                               * Referenced by: '<S38>/Restore physical static offset'
                               */
  real_T ActualADCinputrange_UpperSat_j;/* Expression: p.adcStudy.forceAdcRange_N
                                         * Referenced by: '<S38>/Actual ADC input range'
                                         */
  real_T ActualADCinputrange_LowerSat_n;
                                    /* Expression: -(p.adcStudy.forceAdcRange_N)
                                     * Referenced by: '<S38>/Actual ADC input range'
                                     */
  real_T Actualconverterquantization_I_b;
                             /* Expression: max(p.adcStudy.forceQuantum_N,1e-15)
                              * Referenced by: '<S38>/Actual converter quantization'
                              */
  real_T Knownquantizedstaticoffset_Bi_o;
  /* Expression: -round((p.adcStudy.d.lateralGravityOffset_N)/(max(p.adcStudy.forceQuantum_N,1e-15)))*(max(p.adcStudy.forceQuantum_N,1e-15))
   * Referenced by: '<S38>/Known quantized static offset'
   */
  real_T FreshperconversionEMA_NumCoe_b0[2];
                       /* Expression: p.adcStudy.d.channels.lateral.emaNumerator
                        * Referenced by: '<S38>/Fresh per-conversion EMA'
                        */
  real_T FreshperconversionEMA_DenCoef_k[2];
                     /* Expression: p.adcStudy.d.channels.lateral.emaDenominator
                      * Referenced by: '<S38>/Fresh per-conversion EMA'
                      */
  real_T FreshperconversionEMA_Initial_m;
  /* Expression: (round((((p.force.reference*(1+p.uncertainty.forceScaleX)*(-p.contact.mu*cos(p.uncertainty.forceFrameAngle_rad)-sin(p.uncertainty.forceFrameAngle_rad)+p.uncertainty.forceCrossXFromZ)))+(p.adcStudy.d.lateralGravityOffset_N))/(max(p.adcStudy.forceQuantum_N,1e-15)))*(max(p.adcStudy.forceQuantum_N,1e-15))-round((p.adcStudy.d.lateralGravityOffset_N)/(max(p.adcStudy.forceQuantum_N,1e-15)))*(max(p.adcStudy.forceQuantum_N,1e-15)))*p.adcStudy.d.channels.lateral.emaSteadyStatePerUnitInput
   * Referenced by: '<S38>/Fresh per-conversion EMA'
   */
  real_T Knowntareresidualbias_Value;  /* Expression: p.contact.lateralBias_N
                                        * Referenced by: '<S8>/Known tare residual bias'
                                        */
  real_T Filtermemory_InitialCondition;
  /* Expression: p.force.reference*(1+p.uncertainty.forceScaleX)*(-p.contact.mu*cos(p.uncertainty.forceFrameAngle_rad)-sin(p.uncertainty.forceFrameAngle_rad)+p.uncertainty.forceCrossXFromZ)
   * Referenced by: '<S8>/Filter memory'
   */
  real_T ForceXframediagonal_Gain;
                           /* Expression: cos(p.uncertainty.forceFrameAngle_rad)
                            * Referenced by: '<S8>/Force X frame diagonal'
                            */
  real_T ForceXframecrossaxis_Gain;
  /* Expression: -sin(p.uncertainty.forceFrameAngle_rad)+p.uncertainty.forceCrossXFromZ
   * Referenced by: '<S8>/Force X frame cross axis'
   */
  real_T Independentlateralnoise_Minimum;/* Expression: -sqrt(3)
                                          * Referenced by: '<S8>/Independent lateral noise'
                                          */
  real_T Independentlateralnoise_Maximum;/* Expression: sqrt(3)
                                          * Referenced by: '<S8>/Independent lateral noise'
                                          */
  real_T Independentlateralnoise_Seed; /* Expression: p.sensor.seed+197
                                        * Referenced by: '<S8>/Independent lateral noise'
                                        */
  real_T Lateralanalogvibrationprobe_Amp;
                          /* Expression: p.adcStudy.lateralAnalogToneAmplitude_N
                           * Referenced by: '<S8>/Lateral analog vibration probe'
                           */
  real_T Lateralanalogvibrationprobe_Bia;/* Expression: 0
                                          * Referenced by: '<S8>/Lateral analog vibration probe'
                                          */
  real_T Lateralanalogvibrationprobe_Fre;
                           /* Expression: 2*pi*p.adcStudy.analogToneFrequency_Hz
                            * Referenced by: '<S8>/Lateral analog vibration probe'
                            */
  real_T Lateralanalogvibrationprobe_Pha;
                                   /* Expression: p.adcStudy.analogTonePhase_rad
                                    * Referenced by: '<S8>/Lateral analog vibration probe'
                                    */
  real_T Residuallateralcalibration_Gain;
                                      /* Expression: 1+p.uncertainty.forceScaleX
                                       * Referenced by: '<S8>/Residual lateral calibration'
                                       */
  real_T Newsampleweight_Gain_j;
  /* Expression: 1-exp(-p.sensor.sampleTime/max(p.sensor.filterTime,realmin))*double(p.sensor.filterTime>0)
   * Referenced by: '<S8>/New sample weight'
   */
  real_T NoiseRMS_Gain;       /* Expression: p.adcStudy.d.lateralNoiseInputRms_N
                               * Referenced by: '<S8>/Noise RMS'
                               */
  real_T Oldsampleweight_Gain;
  /* Expression: exp(-p.sensor.sampleTime/max(p.sensor.filterTime,realmin))*double(p.sensor.filterTime>0)
   * Referenced by: '<S8>/Old sample weight'
   */
  real_T Rawoutputnoisenormalization_G_a;/* Expression: 1
                                          * Referenced by: '<S8>/Raw output-noise normalization'
                                          */
  real_T Useengineeredacquisition_Valu_f;
                                    /* Expression: double(p.acquisition.enabled)
                                     * Referenced by: '<S8>/Use engineered acquisition'
                                     */
  real_T Selectidentifiedsensorrespons_i;/* Expression: .5
                                          * Referenced by: '<S8>/Select identified sensor response'
                                          */
  uint32_T Continuousanalogfrontend_A_ir[3];
                            /* Computed Parameter: Continuousanalogfrontend_A_ir
                             * Referenced by: '<S42>/Continuous analog front end'
                             */
  uint32_T Continuousanalogfrontend_A_jc[3];
                            /* Computed Parameter: Continuousanalogfrontend_A_jc
                             * Referenced by: '<S42>/Continuous analog front end'
                             */
  uint32_T Continuousanalogfrontend_B_ir;
                            /* Computed Parameter: Continuousanalogfrontend_B_ir
                             * Referenced by: '<S42>/Continuous analog front end'
                             */
  uint32_T Continuousanalogfrontend_B_jc[2];
                            /* Computed Parameter: Continuousanalogfrontend_B_jc
                             * Referenced by: '<S42>/Continuous analog front end'
                             */
  uint32_T Continuousanalogfrontend_C_ir;
                            /* Computed Parameter: Continuousanalogfrontend_C_ir
                             * Referenced by: '<S42>/Continuous analog front end'
                             */
  uint32_T Continuousanalogfrontend_C_jc[3];
                            /* Computed Parameter: Continuousanalogfrontend_C_jc
                             * Referenced by: '<S42>/Continuous analog front end'
                             */
  uint32_T ADCanaloginputfilter_A_ir;
                                /* Computed Parameter: ADCanaloginputfilter_A_ir
                                 * Referenced by: '<S42>/ADC analog input filter'
                                 */
  uint32_T ADCanaloginputfilter_A_jc[2];
                                /* Computed Parameter: ADCanaloginputfilter_A_jc
                                 * Referenced by: '<S42>/ADC analog input filter'
                                 */
  uint32_T ADCanaloginputfilter_D_ir;
                                /* Computed Parameter: ADCanaloginputfilter_D_ir
                                 * Referenced by: '<S42>/ADC analog input filter'
                                 */
  uint32_T ADCanaloginputfilter_D_jc[2];
                                /* Computed Parameter: ADCanaloginputfilter_D_jc
                                 * Referenced by: '<S42>/ADC analog input filter'
                                 */
  uint32_T Continuousanalogfrontend_A_ir_l[3];
                          /* Computed Parameter: Continuousanalogfrontend_A_ir_l
                           * Referenced by: '<S43>/Continuous analog front end'
                           */
  uint32_T Continuousanalogfrontend_A_jc_n[3];
                          /* Computed Parameter: Continuousanalogfrontend_A_jc_n
                           * Referenced by: '<S43>/Continuous analog front end'
                           */
  uint32_T Continuousanalogfrontend_B_ir_n;
                          /* Computed Parameter: Continuousanalogfrontend_B_ir_n
                           * Referenced by: '<S43>/Continuous analog front end'
                           */
  uint32_T Continuousanalogfrontend_B_jc_m[2];
                          /* Computed Parameter: Continuousanalogfrontend_B_jc_m
                           * Referenced by: '<S43>/Continuous analog front end'
                           */
  uint32_T Continuousanalogfrontend_C_ir_m;
                          /* Computed Parameter: Continuousanalogfrontend_C_ir_m
                           * Referenced by: '<S43>/Continuous analog front end'
                           */
  uint32_T Continuousanalogfrontend_C_jc_p[3];
                          /* Computed Parameter: Continuousanalogfrontend_C_jc_p
                           * Referenced by: '<S43>/Continuous analog front end'
                           */
  uint32_T ADCanaloginputfilter_A_ir_c;
                              /* Computed Parameter: ADCanaloginputfilter_A_ir_c
                               * Referenced by: '<S43>/ADC analog input filter'
                               */
  uint32_T ADCanaloginputfilter_A_jc_i[2];
                              /* Computed Parameter: ADCanaloginputfilter_A_jc_i
                               * Referenced by: '<S43>/ADC analog input filter'
                               */
  uint32_T ADCanaloginputfilter_D_ir_n;
                              /* Computed Parameter: ADCanaloginputfilter_D_ir_n
                               * Referenced by: '<S43>/ADC analog input filter'
                               */
  uint32_T ADCanaloginputfilter_D_jc_k[2];
                              /* Computed Parameter: ADCanaloginputfilter_D_jc_k
                               * Referenced by: '<S43>/ADC analog input filter'
                               */
  uint32_T Continuousanalogfrontend_A_ir_a[3];
                          /* Computed Parameter: Continuousanalogfrontend_A_ir_a
                           * Referenced by: '<S29>/Continuous analog front end'
                           */
  uint32_T Continuousanalogfrontend_A_jc_f[3];
                          /* Computed Parameter: Continuousanalogfrontend_A_jc_f
                           * Referenced by: '<S29>/Continuous analog front end'
                           */
  uint32_T Continuousanalogfrontend_B_ir_o;
                          /* Computed Parameter: Continuousanalogfrontend_B_ir_o
                           * Referenced by: '<S29>/Continuous analog front end'
                           */
  uint32_T Continuousanalogfrontend_B_jc_c[2];
                          /* Computed Parameter: Continuousanalogfrontend_B_jc_c
                           * Referenced by: '<S29>/Continuous analog front end'
                           */
  uint32_T Continuousanalogfrontend_C_ir_d;
                          /* Computed Parameter: Continuousanalogfrontend_C_ir_d
                           * Referenced by: '<S29>/Continuous analog front end'
                           */
  uint32_T Continuousanalogfrontend_C_jc_o[3];
                          /* Computed Parameter: Continuousanalogfrontend_C_jc_o
                           * Referenced by: '<S29>/Continuous analog front end'
                           */
  uint32_T ADCanaloginputfilter_A_ir_f;
                              /* Computed Parameter: ADCanaloginputfilter_A_ir_f
                               * Referenced by: '<S29>/ADC analog input filter'
                               */
  uint32_T ADCanaloginputfilter_A_jc_f[2];
                              /* Computed Parameter: ADCanaloginputfilter_A_jc_f
                               * Referenced by: '<S29>/ADC analog input filter'
                               */
  uint32_T ADCanaloginputfilter_D_ir_f;
                              /* Computed Parameter: ADCanaloginputfilter_D_ir_f
                               * Referenced by: '<S29>/ADC analog input filter'
                               */
  uint32_T ADCanaloginputfilter_D_jc_e[2];
                              /* Computed Parameter: ADCanaloginputfilter_D_jc_e
                               * Referenced by: '<S29>/ADC analog input filter'
                               */
  uint32_T Continuousanalogfrontend_A_ir_d[3];
                          /* Computed Parameter: Continuousanalogfrontend_A_ir_d
                           * Referenced by: '<S30>/Continuous analog front end'
                           */
  uint32_T Continuousanalogfrontend_A_jc_p[3];
                          /* Computed Parameter: Continuousanalogfrontend_A_jc_p
                           * Referenced by: '<S30>/Continuous analog front end'
                           */
  uint32_T Continuousanalogfrontend_B_ir_a;
                          /* Computed Parameter: Continuousanalogfrontend_B_ir_a
                           * Referenced by: '<S30>/Continuous analog front end'
                           */
  uint32_T Continuousanalogfrontend_B_jc_a[2];
                          /* Computed Parameter: Continuousanalogfrontend_B_jc_a
                           * Referenced by: '<S30>/Continuous analog front end'
                           */
  uint32_T Continuousanalogfrontend_C_ir_p;
                          /* Computed Parameter: Continuousanalogfrontend_C_ir_p
                           * Referenced by: '<S30>/Continuous analog front end'
                           */
  uint32_T Continuousanalogfrontend_C_jc_l[3];
                          /* Computed Parameter: Continuousanalogfrontend_C_jc_l
                           * Referenced by: '<S30>/Continuous analog front end'
                           */
  uint32_T ADCanaloginputfilter_A_ir_n;
                              /* Computed Parameter: ADCanaloginputfilter_A_ir_n
                               * Referenced by: '<S30>/ADC analog input filter'
                               */
  uint32_T ADCanaloginputfilter_A_jc_ir[2];
                             /* Computed Parameter: ADCanaloginputfilter_A_jc_ir
                              * Referenced by: '<S30>/ADC analog input filter'
                              */
  uint32_T ADCanaloginputfilter_D_ir_fn;
                             /* Computed Parameter: ADCanaloginputfilter_D_ir_fn
                              * Referenced by: '<S30>/ADC analog input filter'
                              */
  uint32_T ADCanaloginputfilter_D_jc_c[2];
                              /* Computed Parameter: ADCanaloginputfilter_D_jc_c
                               * Referenced by: '<S30>/ADC analog input filter'
                               */
  uint32_T Continuousanalogfrontend_A_ir_i[3];
                          /* Computed Parameter: Continuousanalogfrontend_A_ir_i
                           * Referenced by: '<S31>/Continuous analog front end'
                           */
  uint32_T Continuousanalogfrontend_A_j_nh[3];
                          /* Computed Parameter: Continuousanalogfrontend_A_j_nh
                           * Referenced by: '<S31>/Continuous analog front end'
                           */
  uint32_T Continuousanalogfrontend_B_ir_p;
                          /* Computed Parameter: Continuousanalogfrontend_B_ir_p
                           * Referenced by: '<S31>/Continuous analog front end'
                           */
  uint32_T Continuousanalogfrontend_B_jc_f[2];
                          /* Computed Parameter: Continuousanalogfrontend_B_jc_f
                           * Referenced by: '<S31>/Continuous analog front end'
                           */
  uint32_T Continuousanalogfrontend_C_ir_l;
                          /* Computed Parameter: Continuousanalogfrontend_C_ir_l
                           * Referenced by: '<S31>/Continuous analog front end'
                           */
  uint32_T Continuousanalogfrontend_C_jc_n[3];
                          /* Computed Parameter: Continuousanalogfrontend_C_jc_n
                           * Referenced by: '<S31>/Continuous analog front end'
                           */
  uint32_T ADCanaloginputfilter_A_ir_m;
                              /* Computed Parameter: ADCanaloginputfilter_A_ir_m
                               * Referenced by: '<S31>/ADC analog input filter'
                               */
  uint32_T ADCanaloginputfilter_A_jc_o[2];
                              /* Computed Parameter: ADCanaloginputfilter_A_jc_o
                               * Referenced by: '<S31>/ADC analog input filter'
                               */
  uint32_T ADCanaloginputfilter_D_ir_fr;
                             /* Computed Parameter: ADCanaloginputfilter_D_ir_fr
                              * Referenced by: '<S31>/ADC analog input filter'
                              */
  uint32_T ADCanaloginputfilter_D_jc_a[2];
                              /* Computed Parameter: ADCanaloginputfilter_D_jc_a
                               * Referenced by: '<S31>/ADC analog input filter'
                               */
  uint32_T Continuousanalogfrontend_A_ir_g[3];
                          /* Computed Parameter: Continuousanalogfrontend_A_ir_g
                           * Referenced by: '<S37>/Continuous analog front end'
                           */
  uint32_T Continuousanalogfrontend_A_jc_g[3];
                          /* Computed Parameter: Continuousanalogfrontend_A_jc_g
                           * Referenced by: '<S37>/Continuous analog front end'
                           */
  uint32_T Continuousanalogfrontend_B_ir_l;
                          /* Computed Parameter: Continuousanalogfrontend_B_ir_l
                           * Referenced by: '<S37>/Continuous analog front end'
                           */
  uint32_T Continuousanalogfrontend_B_j_co[2];
                          /* Computed Parameter: Continuousanalogfrontend_B_j_co
                           * Referenced by: '<S37>/Continuous analog front end'
                           */
  uint32_T Continuousanalogfrontend_C_ir_h;
                          /* Computed Parameter: Continuousanalogfrontend_C_ir_h
                           * Referenced by: '<S37>/Continuous analog front end'
                           */
  uint32_T Continuousanalogfrontend_C_jc_d[3];
                          /* Computed Parameter: Continuousanalogfrontend_C_jc_d
                           * Referenced by: '<S37>/Continuous analog front end'
                           */
  uint32_T ADCanaloginputfilter_A_ir_b;
                              /* Computed Parameter: ADCanaloginputfilter_A_ir_b
                               * Referenced by: '<S37>/ADC analog input filter'
                               */
  uint32_T ADCanaloginputfilter_A_jc_e[2];
                              /* Computed Parameter: ADCanaloginputfilter_A_jc_e
                               * Referenced by: '<S37>/ADC analog input filter'
                               */
  uint32_T ADCanaloginputfilter_D_ir_a;
                              /* Computed Parameter: ADCanaloginputfilter_D_ir_a
                               * Referenced by: '<S37>/ADC analog input filter'
                               */
  uint32_T ADCanaloginputfilter_D_jc_f[2];
                              /* Computed Parameter: ADCanaloginputfilter_D_jc_f
                               * Referenced by: '<S37>/ADC analog input filter'
                               */
  uint32_T Continuousanalogfrontend_A_ir_m[3];
                          /* Computed Parameter: Continuousanalogfrontend_A_ir_m
                           * Referenced by: '<S38>/Continuous analog front end'
                           */
  uint32_T Continuousanalogfrontend_A_jc_m[3];
                          /* Computed Parameter: Continuousanalogfrontend_A_jc_m
                           * Referenced by: '<S38>/Continuous analog front end'
                           */
  uint32_T Continuousanalogfrontend_B_i_ow;
                          /* Computed Parameter: Continuousanalogfrontend_B_i_ow
                           * Referenced by: '<S38>/Continuous analog front end'
                           */
  uint32_T Continuousanalogfrontend_B_jc_g[2];
                          /* Computed Parameter: Continuousanalogfrontend_B_jc_g
                           * Referenced by: '<S38>/Continuous analog front end'
                           */
  uint32_T Continuousanalogfrontend_C_i_lc;
                          /* Computed Parameter: Continuousanalogfrontend_C_i_lc
                           * Referenced by: '<S38>/Continuous analog front end'
                           */
  uint32_T Continuousanalogfrontend_C_jc_k[3];
                          /* Computed Parameter: Continuousanalogfrontend_C_jc_k
                           * Referenced by: '<S38>/Continuous analog front end'
                           */
  uint32_T ADCanaloginputfilter_A_ir_nf;
                             /* Computed Parameter: ADCanaloginputfilter_A_ir_nf
                              * Referenced by: '<S38>/ADC analog input filter'
                              */
  uint32_T ADCanaloginputfilter_A_jc_j[2];
                              /* Computed Parameter: ADCanaloginputfilter_A_jc_j
                               * Referenced by: '<S38>/ADC analog input filter'
                               */
  uint32_T ADCanaloginputfilter_D_ir_e;
                              /* Computed Parameter: ADCanaloginputfilter_D_ir_e
                               * Referenced by: '<S38>/ADC analog input filter'
                               */
  uint32_T ADCanaloginputfilter_D_jc_fu[2];
                             /* Computed Parameter: ADCanaloginputfilter_D_jc_fu
                              * Referenced by: '<S38>/ADC analog input filter'
                              */
};

/* Real-time Model Data Structure */
struct tag_RTM_acf_rl_native_T {
  const char_T *errorStatus;
  RTWSolverInfo solverInfo;
  X_acf_rl_native_T *contStates;
  int_T *periodicContStateIndices;
  real_T *periodicContStateRanges;
  real_T *derivs;
  XDis_acf_rl_native_T *contStateDisabled;
  boolean_T zCCacheNeedsReset;
  boolean_T derivCacheNeedsReset;
  boolean_T CTOutputIncnstWithState;
  real_T odeY[30];
  real_T odeF[4][30];
  ODE4_IntgData intgData;

  /*
   * Sizes:
   * The following substructure contains sizes information
   * for many of the model attributes such as inputs, outputs,
   * dwork, sample times, etc.
   */
  struct {
    int_T numContStates;
    int_T numPeriodicContStates;
    int_T numSampTimes;
  } Sizes;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    uint32_T clockTick0;
    uint32_T clockTickH0;
    time_T stepSize0;
    uint32_T clockTick1;
    uint32_T clockTickH1;
    uint32_T clockTick4;
    uint32_T clockTickH4;
    struct {
      uint16_T TID[5];
    } TaskCounters;

    time_T tStart;
    SimTimeStep simTimeStep;
    boolean_T stopRequestedFlag;
    time_T *t;
    time_T tArray[5];
  } Timing;
};

/* Block parameters (default storage) */
extern P_acf_rl_native_T acf_rl_native_P;

/* Block signals (default storage) */
extern B_acf_rl_native_T acf_rl_native_B;

/* Continuous states (default storage) */
extern X_acf_rl_native_T acf_rl_native_X;

/* Disabled states (default storage) */
extern XDis_acf_rl_native_T acf_rl_native_XDis;

/* Block states (default storage) */
extern DW_acf_rl_native_T acf_rl_native_DW;

/* External inputs (root inport signals with default storage) */
extern ExtU_acf_rl_native_T acf_rl_native_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_acf_rl_native_T acf_rl_native_Y;

/* Model entry point functions */
extern void acf_rl_native_initialize(void);
extern void acf_rl_native_step(void);
extern void acf_rl_native_terminate(void);

/* Real-time Model object */
extern RT_MODEL_acf_rl_native_T *const acf_rl_native_M;

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
 * '<Root>' : 'acf_rl_native'
 * '<S1>'   : 'acf_rl_native/ADC acquisition diagnostics'
 * '<S2>'   : 'acf_rl_native/Actuator and RABS mechanics'
 * '<S3>'   : 'acf_rl_native/Contact force measurement'
 * '<S4>'   : 'acf_rl_native/Force controller'
 * '<S5>'   : 'acf_rl_native/Force measurement'
 * '<S6>'   : 'acf_rl_native/Inertial sensor recordings'
 * '<S7>'   : 'acf_rl_native/Issued-command robot predictor'
 * '<S8>'   : 'acf_rl_native/Lateral B-S force measurement'
 * '<S9>'   : 'acf_rl_native/RL rl_comparator'
 * '<S10>'  : 'acf_rl_native/RL rl_observation'
 * '<S11>'  : 'acf_rl_native/RL rl_scoring'
 * '<S12>'  : 'acf_rl_native/Recenter travel'
 * '<S13>'  : 'acf_rl_native/Relative position measurement'
 * '<S14>'  : 'acf_rl_native/Robot command delay'
 * '<S15>'  : 'acf_rl_native/Robot motion'
 * '<S16>'  : 'acf_rl_native/S accelerometer and ADC'
 * '<S17>'  : 'acf_rl_native/Surface and nominal path'
 * '<S18>'  : 'acf_rl_native/Surface contact force'
 * '<S19>'  : 'acf_rl_native/Actuator and RABS mechanics/Actuator'
 * '<S20>'  : 'acf_rl_native/Actuator and RABS mechanics/B to S spring and damper'
 * '<S21>'  : 'acf_rl_native/Actuator and RABS mechanics/Mass A'
 * '<S22>'  : 'acf_rl_native/Actuator and RABS mechanics/Mass B'
 * '<S23>'  : 'acf_rl_native/Actuator and RABS mechanics/Measurements and safety'
 * '<S24>'  : 'acf_rl_native/Actuator and RABS mechanics/R to A spring and damper'
 * '<S25>'  : 'acf_rl_native/Actuator and RABS mechanics/Actuator/Voice coil'
 * '<S26>'  : 'acf_rl_native/Contact force measurement/Contact force reconstruction'
 * '<S27>'  : 'acf_rl_native/Contact force measurement/Head motion estimator'
 * '<S28>'  : 'acf_rl_native/Contact force measurement/Normal force projection'
 * '<S29>'  : 'acf_rl_native/Contact force measurement/Head motion estimator/Position common AA and EMA'
 * '<S30>'  : 'acf_rl_native/Contact force measurement/Head motion estimator/Robot acceleration common AA and EMA'
 * '<S31>'  : 'acf_rl_native/Contact force measurement/Head motion estimator/Robot velocity common AA and EMA'
 * '<S32>'  : 'acf_rl_native/Contact force measurement/Normal force projection/Measured drag geometry'
 * '<S33>'  : 'acf_rl_native/Force controller/Coil PI and PID'
 * '<S34>'  : 'acf_rl_native/Force controller/Coil PI and PID/Lower spring inverse feedforward'
 * '<S35>'  : 'acf_rl_native/Force controller/Coil PI and PID/Measured motion feedforward'
 * '<S36>'  : 'acf_rl_native/Force controller/Coil PI and PID/Normal force PI and PID'
 * '<S37>'  : 'acf_rl_native/Force measurement/Engineered common AA and EMA'
 * '<S38>'  : 'acf_rl_native/Lateral B-S force measurement/Engineered common AA and EMA'
 * '<S39>'  : 'acf_rl_native/Recenter travel/Force settling gate'
 * '<S40>'  : 'acf_rl_native/Recenter travel/RL hard sign and gain envelope'
 * '<S41>'  : 'acf_rl_native/Recenter travel/Force settling gate/Fast force variation and dwell'
 * '<S42>'  : 'acf_rl_native/S accelerometer and ADC/X engineered common AA and EMA'
 * '<S43>'  : 'acf_rl_native/S accelerometer and ADC/Z engineered common AA and EMA'
 */
#endif                                 /* acf_rl_native_h_ */
