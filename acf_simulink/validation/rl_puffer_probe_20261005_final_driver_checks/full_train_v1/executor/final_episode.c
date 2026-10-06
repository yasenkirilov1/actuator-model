/* Whole-episode policy reward evaluation. Physical integration remains solely
 * in the accepted Simulink-generated native shared library. */
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "policy_inference.h"
void *acf_create(int scenario, int seed, int baseline);
void acf_destroy(void *handle);
int acf_snapshot(void *, double *, double *, double *, double *);
int acf_step(void *, double, double, double *, double *, double *, double *);
void acf_set_record(void *, double *, size_t);
size_t acf_record_count(void *);

/* c: reference, Ieq, midTravel, angle, mS_id, mB_id, mA, k_id,
 * c_id,Kf_id,R,L,V,Imax,stroke,currenttau,robotdelay,robotresponse,
 * AA,EMA,pipeline,duration,accepted_recenter_band,robot_action_scale. All identified quantities are causal context. */
void acf_search_features(const double *s,const double *c,int64_t step,float *o) {
  o[0]=(s[0]-c[0])/.5; o[1]=(s[1]-c[0])/5; o[2]=s[2]/50;
  o[3]=s[3]/10; o[4]=s[4]/10; o[5]=s[5]/.025; o[6]=s[6]/.02;
  o[7]=(s[7]-c[2])/.005; o[8]=s[8]/.2;
  o[9]=s[9]/.005; o[10]=s[10]/.2; o[11]=s[11]/.005;
  o[12]=s[12]/.2; o[13]=(s[13]-c[1])/2.; o[14]=s[14]/.005;
  o[15]=s[15]/.005; o[16]=s[16]; o[17]=s[17]/.005;
  o[18]=c[0]/50.; o[19]=c[4]/2.; o[20]=c[5]; o[21]=c[6]/2.;
  o[22]=sin(c[3]); o[23]=cos(c[3]); o[24]=c[7]/20000.;
  o[25]=c[8]/60.; o[26]=c[9]/25.; o[27]=c[10]/10.;
  o[28]=c[11]/.004; o[29]=c[12]/32.; o[30]=c[13]/3.5;
  o[31]=c[14]/.025; o[32]=c[1]/3.5; o[33]=c[15]/.0001;
  o[34]=c[16]/.08; o[35]=c[17]/.06; o[36]=c[18]/8000.;
  o[37]=c[19]/.00005; o[38]=c[20]/.00002;
  o[39]=2.*(step%100)/99.-1.;
  for(int j=0;j<40;j++)o[j]=fminf(100.f,fmaxf(-100.f,o[j]));
}

/* stats: peakerr,rms,mse,terminaltravel,travelrms,currentpeak,voltagepeak,
 * clipfraction,currentlimitfraction,contact,stop,software,completed,
 * native_samples,controller_decisions,end_s,issuedrobot_frames,
 * eligible_robot_decisions,issued_robot_peak,initial_error,maxtravel,mintravel,
 * force_outside_fraction,policy_active_seconds,record_count,
 * band_excess_integral_upper_m2_s,max_abs_travel_error_m,
 * time_outside_band_upper_s,initial_travel_error_m. */
int acf_search_stats_count(void) { return 29; }
int acf_search_episode(int scenario,int seed,const double *c,const float *w,
   int repeat,int baseline,double *stats,double *record,size_t capacity,
   double *actions,size_t action_capacity) {
  if(ACF_RL_OBS!=40 || repeat<1 || 100%repeat) return -2;
  void *ctx=acf_create(scenario,seed,baseline);
  if(!ctx)return -1;
  if(record)acf_set_record(ctx,record,capacity);
  double obs[18],truth[27],comp[3],metric[24],ssq=0,tsq=0,n=0;
  double clips=0,limits=0,outside=0;
  float state[ACF_RL_LAYERS*ACF_RL_HIDDEN]={0},feature[ACF_RL_OBS],mean[2],value;
  memset(stats,0,29*sizeof(double));
  acf_snapshot(ctx,obs,truth,comp,metric);
  stats[0]=fabs(truth[0]-c[0]);stats[19]=stats[0];
  stats[20]=stats[21]=truth[2];
  stats[28]=truth[2]-c[2];stats[26]=fabs(stats[28]);
  int64_t maxsteps=llround(c[21]/.0001), step=0;
  int physical=0,software=0;
  while(step<maxsteps && !physical && !software){
    acf_search_features(obs,c,step,feature);
    if (baseline) { mean[0]=mean[1]=0; value=0; } else acf_unified_policy_step(w,feature,state,step==0,mean,&value);
    int eligible=(obs[16]>.5 && step%100==0);
    double current=c[1]+2.*(double)tanhf(mean[0]);
    double robot=eligible ? c[23]*(double)tanhf(mean[1]) : 0;
    stats[17]+=eligible;
    if(actions && (size_t)stats[14]<action_capacity){
       double *row=actions+45*(size_t)stats[14];
       row[0]=step*.0001;row[1]=current;row[2]=robot;
       for(int k=0;k<40;k++)row[3+k]=feature[k];
       row[43]=mean[0];row[44]=mean[1];
    }
    for(int k=0;k<repeat && step<maxsteps && !physical && !software;k++){
      acf_step(ctx,current,robot,obs,truth,comp,metric);
      step++;
      n+=metric[3];ssq+=metric[7]*metric[3];tsq+=metric[10]*metric[3];
      stats[0]=fmax(stats[0],metric[6]);stats[5]=fmax(stats[5],metric[11]);
      stats[6]=fmax(stats[6],metric[12]);clips+=metric[14];limits+=metric[15];
      outside+=metric[16];stats[9]=fmax(stats[9],metric[17]);
      stats[10]=fmax(stats[10],metric[18]);stats[11]=fmax(stats[11],metric[19]);
      physical=metric[17]||metric[18];software=metric[20]&&!physical;
      stats[15]=metric[2]; stats[16]+=fabs(obs[15])>1e-12;
      stats[18]=fmax(stats[18],fabs(obs[15]));
      stats[20]=fmax(stats[20],metric[9]);stats[21]=fmin(stats[21],metric[8]);
      double abs_travel=fmax(fabs(metric[8]-c[2]),fabs(metric[9]-c[2]));
      double excess=fmax(abs_travel-c[22],0.);
      stats[25]+=excess*excess*metric[0];stats[26]=fmax(stats[26],abs_travel);
      stats[27]+=metric[0]*(excess>0);
    }
    stats[14]++;
  }
  stats[1]=sqrt(ssq/fmax(n,1));stats[2]=ssq/fmax(n,1);
  stats[3]=truth[2]-c[2];stats[4]=sqrt(tsq/fmax(n,1));
  stats[7]=clips/fmax(n,1);stats[8]=limits/fmax(n,1);
  stats[11]=fmax(stats[11],software);stats[12]=!physical&&!software&&step>=maxsteps;
  stats[13]=n;stats[22]=outside/fmax(n,1);stats[23]=step*.0001;
  stats[24]=acf_record_count(ctx);acf_destroy(ctx);return 0;
}
