#define _POSIX_C_SOURCE 200809L
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#define PI 3.14159265358979323846264338327950288
#define NST 21
#define NCOL 23
#define MAXPAD 121

typedef struct {
  double actuator,pad_points,dt,duration,force_target,mass_a,mass_b,mass_s,gravity,
    angle_deg,upper_k,cell_k,pad_k,pad_c,pad_diameter,pad_thickness,pad_compression_limit,
    bump_height,bump_width,scan_speed,bump_start,nut_mass,lead,rotor_inertia,sea_k,
    sea_c,spring_deflection_limit,nut_travel_limit,tool_travel_limit,force_constant,
    resistance,inductance,current_response,voltage_limit,current_limit,
    continuous_current,guide_coulomb,guide_viscous,guide_eps,shaft_coulomb,
    shaft_viscous,shaft_eps,analog_cutoff,adc_rate,control_rate,ema_time,pipeline_time,
    force_noise_rms,accel_noise_rms,position_noise_rms,noise_reference_enbw,
    force_quantum,accel_quantum,position_quantum,force_bias,accel_bias,position_bias,
    force_scale,accel_scale,identified_mass_s,force_kp,force_ki,velocity_gain,
    inertia_gain,inertia_cutoff,velocity_leak,servo_hz,servo_zeta,
    servo_target_cutoff,encoder_quantum,noise_enabled,seed,robot_enabled,
    robot_gate_rms,robot_gate_mean,robot_dwell,robot_rate,robot_delay,robot_tau,
    robot_max_velocity,robot_gain,robot_soft_band,external_current,
    external_frequency,external_amplitude,external_start,external_robot_amplitude,
    external_robot_frequency,servo_velocity_feedforward,force_feedback_cutoff;
} Parameters;

typedef struct {
  double steps,stop_code,stop_time,force_peak_error,force_rms_error,force_peak_time,
    force_min,force_max,current_peak,current_rms,voltage_peak_request,
    voltage_clipped_fraction,current_clipped_fraction,tool_peak_deviation,
    nut_peak_deviation,spring_peak_deflection,pad_peak_compression,
    force_reconstruction_peak_error,force_balance_peak,centering_band_fraction,
    robot_command_count,runtime_s,adc_count,control_count,force_noise_filtered_rms,
    accel_noise_filtered_rms,flat_peak_error,bump_window_peak_error,nut_velocity_peak,
    rotor_speed_peak_rpm;
} Metrics;

typedef struct {
  const Parameters *p;
  int n;
  double weight[MAXPAD],offset[MAXPAD],noise[3],command;
  uint64_t random_state;
} Context;

static double clip(double x,double lo,double hi){return fmax(lo,fmin(hi,x));}
static double quantize(double x,double q){return q>0?round(x/q)*q:x;}
static double uniform(Context *c){
  uint64_t x=c->random_state;x^=x>>12;x^=x<<25;x^=x>>27;c->random_state=x;
  return ((x*UINT64_C(2685821657736338717))>>11)*0x1.0p-53;
}
static double normal(Context *c){
  double a=fmax(uniform(c),0x1.0p-53),b=uniform(c);
  return sqrt(-2*log(a))*cos(2*PI*b);
}
static void initialize_context(Context *c,const Parameters *p){
  memset(c,0,sizeof(*c));c->p=p;
  c->random_state=(uint64_t)p->seed+UINT64_C(0x9e3779b97f4a7c15);
  c->n=p->pad_diameter>0?(int)p->pad_points:1;
  if(c->n<1)c->n=1;
  if(c->n>MAXPAD)c->n=MAXPAD;
  if(c->n==1){c->weight[0]=1;c->offset[0]=0;return;}
  double radius=p->pad_diameter/2,area=PI*radius*radius;
  for(int j=0;j<c->n;j++){
    double a=-radius+2*radius*j/c->n,b=-radius+2*radius*(j+1)/c->n;
    double ia=a*sqrt(fmax(0,radius*radius-a*a))+radius*radius*asin(clip(a/radius,-1,1));
    double ib=b*sqrt(fmax(0,radius*radius-b*b))+radius*radius*asin(clip(b/radius,-1,1));
    c->weight[j]=(ib-ia)/area;c->offset[j]=(a+b)/2;
  }
}
static void profile(const Parameters *p,double t,double shift,double *h,double *v){
  double r=((t-p->bump_start)*p->scan_speed+shift)/p->bump_width;
  if(r<=0||r>=1){*h=0;*v=0;return;}
  double z=r*(1-r);
  *h=64*p->bump_height*z*z*z;
  *v=192*p->bump_height*z*z*(1-2*r)*p->scan_speed/p->bump_width;
}
static double profile_acceleration(const Parameters *p,double t){
  double r=(t-p->bump_start)*p->scan_speed/p->bump_width;
  if(r<=0||r>=1)return 0;
  double z=r*(1-r),d=1-2*r,rate=p->scan_speed/p->bump_width;
  return 192*p->bump_height*(2*z*d*d-2*z*z)*rate*rate;
}
static double robot_position(const Parameters *p,double t){
  return p->external_robot_amplitude*sin(2*PI*p->external_robot_frequency*t);
}
static void plant_rhs(Context *c,double t,const double *x,double command,double robot,
                      double *dx,double *o){
  const Parameters *p=c->p;
  const double g=p->gravity*cos(p->angle_deg*PI/180),ell=p->lead/(2*PI);
  double fc=0,compression=0,zs=x[4],vs=x[5],rigid_acceleration=0;
  if(p->pad_k==0){
    profile(p,t,0,&zs,&vs);rigid_acceleration=profile_acceleration(p,t);
  }
  for(int j=0;j<c->n&&p->pad_k>0;j++){
    double h,hv;profile(p,t,c->offset[j],&h,&hv);
    double d=h-x[4];compression=fmax(compression,d);
    if(d>0)fc+=c->weight[j]*fmax(0,p->pad_k*d+p->pad_c*(hv-x[5]));
  }
  const double cell=p->cell_k*(zs-x[2]),upper=p->upper_k*(x[0]-robot);
  if(p->pad_k==0)fc=cell+p->mass_s*(rigid_acceleration+g);
  const double u=x[0]-x[2],uv=x[1]-x[3];
  const double guide=p->guide_coulomb*tanh(uv/p->guide_eps)+p->guide_viscous*uv;
  double fa,aA,qdd=0,speed,fm=0;
  if(p->actuator==0){
    fa=p->force_constant*x[8];aA=(fa-guide-upper)/p->mass_a-g;speed=uv;
  }else{
    fa=p->sea_k*(x[6]-u)+p->sea_c*(x[7]-uv);speed=x[7]/ell;
    double friction=p->shaft_coulomb*tanh(speed/p->shaft_eps)+p->shaft_viscous*speed;
    fm=(p->force_constant*x[8]-friction)/ell;
    double ja=p->rotor_inertia/(ell*ell),aa=p->mass_a+p->nut_mass;
    double bb=p->nut_mass+ja,det=aa*bb-p->nut_mass*p->nut_mass;
    double ra=fa-guide-upper-aa*g,rq=fm-fa+p->nut_mass*g;
    aA=(bb*ra+p->nut_mass*rq)/det;
    qdd=(p->nut_mass*ra+aa*rq)/det;
  }
  double aB=(cell-fa+guide)/p->mass_b-g,aS=(fc-cell)/p->mass_s-g;
  double icmd=clip(command,-p->current_limit,p->current_limit);
  double demand=p->resistance*x[8]+p->force_constant*speed+
    p->inductance/p->current_response*(icmd-x[8]);
  double voltage=clip(demand,-p->voltage_limit,p->voltage_limit);
  dx[0]=x[1];dx[1]=aA;dx[2]=x[3];dx[3]=aB;
  dx[4]=p->pad_k==0?vs:x[5];dx[5]=p->pad_k==0?rigid_acceleration:aS;
  dx[6]=p->actuator==0?0:x[7];dx[7]=qdd;
  dx[8]=(voltage-p->resistance*x[8]-p->force_constant*speed)/p->inductance;
  if(o){
    o[0]=fc;o[1]=cell;o[2]=fa;o[3]=aS;o[4]=demand;o[5]=compression;
    o[6]=guide;o[7]=upper;o[8]=fm;o[9]=qdd;
  }
}
static void rhs(Context *c,double t,const double *x,double *dx){
  const Parameters *p=c->p;double o[10];
  plant_rhs(c,t,x,c->command,robot_position(p,t),dx,o);
  double g=p->gravity*cos(p->angle_deg*PI/180),w=2*PI*p->analog_cutoff;
  double input[3]={p->force_scale*(o[1]+p->mass_s*g)+p->force_bias,
    p->accel_scale*o[3]+p->accel_bias,x[4]-x[0]+p->position_bias};
  for(int k=0;k<3;k++){
    int j=9+2*k;dx[j]=x[j+1];dx[j+1]=w*w*(input[k]+c->noise[k]-x[j])-sqrt(2)*w*x[j+1];
    int n=15+2*k;dx[n]=x[n+1];dx[n+1]=w*w*(c->noise[k]-x[n])-sqrt(2)*w*x[n+1];
  }
}
void engine_initialize(const Parameters *p,double *x,double *equilibrium){
  memset(x,0,9*sizeof(double));double g=p->gravity*cos(p->angle_deg*PI/180);
  double fa=p->force_target-(p->mass_b+p->mass_s)*g;
  x[4]=p->pad_k==0?0:-p->force_target/p->pad_k;
  x[2]=x[4]-(p->force_target-p->mass_s*g)/p->cell_k;
  x[0]=(fa-(p->mass_a+(p->actuator!=0?p->nut_mass:0))*g)/p->upper_k;
  double ell=p->lead/(2*PI);x[6]=p->actuator==0?0:x[0]-x[2]+fa/p->sea_k;
  x[8]=p->actuator==0?fa/p->force_constant:(fa-p->nut_mass*g)*ell/p->force_constant;
  if(equilibrium){equilibrium[0]=fa;equilibrium[1]=g;}
}
void engine_rhs(const Parameters *p,double t,const double *x,double command,double robot,
                double *dx,double *out){
  Context c;initialize_context(&c,p);plant_rhs(&c,t,x,command,robot,dx,out);
}

static double elapsed(struct timespec a,struct timespec b){return b.tv_sec-a.tv_sec+(b.tv_nsec-a.tv_nsec)/1e9;}
int engine_simulate(const Parameters *p,Metrics *m,double *trace,int capacity,int stride){
  if(p->dt<=0||p->duration<=0||p->adc_rate<=0||p->control_rate<=0||p->pad_points>MAXPAD)return -1;
  if(p->robot_enabled)return -2; /* This fixed-robot diagnostic does not implement recentering. */
  Context c;initialize_context(&c,p);memset(m,0,sizeof(*m));
  double x[NST]={0},eq[2];engine_initialize(p,x,eq);
  double dx[NST],k1[NST],k2[NST],k3[NST],k4[NST],tmp[NST],o[10];
  x[9]=p->force_scale*p->force_target+p->force_bias;
  x[11]=p->accel_bias;x[13]=x[4]-x[0]+p->position_bias;
  double measured[3]={x[9],x[11],x[13]},ema[3]={x[9],x[11],x[13]},noise_ema[3]={0};
  double queue_time[16],queue_value[16][3];int qhead=0,qtail=0;
  double alpha=p->ema_time>0?exp(-1/p->adc_rate/p->ema_time):0;
  const double delay=p->pipeline_time; /* Includes declared ADC49.6us delay in config. */
  double integral=0,aff=0,vhat=0,fa_requested=eq[0],qtarget=x[6],qtarget_previous=x[6];
  double force_estimate=p->force_target,noise_force_sum=0,noise_accel_sum=0;
  double encoder_previous=quantize(x[6],p->encoder_quantum),encoder_velocity=0;
  double force_feedback=p->force_target;
  double error_sum=0,current_sum=0,clip_voltage=0,clip_current=0,band=0;
  double previous_time=0;
  int recorded=0,base=0,adc=0,control=0;double t=0,next_trace=0;
  c.command=x[8];m->force_min=INFINITY;m->force_max=-INFINITY;
  struct timespec start,end;clock_gettime(CLOCK_MONOTONIC,&start);
  for(;;){
    /* Major events occur before endpoint scoring and before the next RK4 step. */
    if(t+1e-12 >= adc/p->adc_rate){
      for(int j=0;j<3;j++){
        double quantum=j==0?p->force_quantum:j==1?p->accel_quantum:p->position_quantum;
        double val=j==0?clip(x[9+2*j],-160,160):j==1?clip(x[9+2*j],-117.6798,117.6798):x[9+2*j];
        val=quantize(val,quantum);ema[j]=alpha*ema[j]+(1-alpha)*val;
        noise_ema[j]=alpha*noise_ema[j]+(1-alpha)*x[15+2*j];
        queue_value[qtail][j]=ema[j];
      }
      queue_time[qtail]=t+delay;qtail=(qtail+1)%16;
      noise_force_sum+=noise_ema[0]*noise_ema[0];noise_accel_sum+=noise_ema[1]*noise_ema[1];adc++;
    }
    while(qhead!=qtail&&queue_time[qhead]<=t+1e-12){
      for(int j=0;j<3;j++)measured[j]=queue_value[qhead][j];
      qhead=(qhead+1)%16;
    }
    if(t+1e-12 >= control/p->control_rate){
      double ts=1/p->control_rate,aa=exp(-2*PI*p->inertia_cutoff*ts);
      aff=aa*aff+(1-aa)*measured[1];
      double vl=2*PI*p->velocity_leak;
      vhat=vl>0?exp(-vl*ts)*vhat+(1-exp(-vl*ts))/vl*aff:vhat+ts*aff;
      force_estimate=measured[0]+p->identified_mass_s*measured[1];
      double af=p->force_feedback_cutoff>0?exp(-2*PI*p->force_feedback_cutoff*ts):0;
      force_feedback=af*force_feedback+(1-af)*force_estimate;
      double err=p->force_target-force_feedback;
      double static_fa=p->force_target-(p->mass_b+p->identified_mass_s)*eq[1];
      double force_cap=p->actuator==0?p->force_constant*p->current_limit:p->sea_k*p->spring_deflection_limit;
      double candidate_integral=integral+ts*p->force_ki*err;
      double unconstrained=static_fa+p->force_kp*err+candidate_integral-
        p->inertia_gain*(p->mass_b+p->identified_mass_s)*aff+p->velocity_gain*vhat;
      if(fabs(unconstrained)<=force_cap||err*unconstrained<0)integral=candidate_integral;
      integral=clip(integral,-force_cap,force_cap);
      fa_requested=clip(static_fa+p->force_kp*err+integral-
        p->inertia_gain*(p->mass_b+p->identified_mass_s)*aff+p->velocity_gain*vhat,-force_cap,force_cap);
      if(p->actuator==0)c.command=fa_requested/p->force_constant;
      else{
        double u_est=-measured[2]+(measured[0]-p->identified_mass_s*eq[1])/p->cell_k;
        double desired=u_est+fa_requested/p->sea_k;
        double at=exp(-2*PI*p->servo_target_cutoff*ts);
        qtarget=at*qtarget+(1-at)*desired;
        double vt=(qtarget-qtarget_previous)/ts;qtarget_previous=qtarget;
        double ell=p->lead/(2*PI),ja=p->rotor_inertia/(ell*ell)+p->nut_mass;
        double ws=2*PI*p->servo_hz;
        double qenc=quantize(x[6],p->encoder_quantum);
        double av=exp(-2*PI*1000*ts);
        encoder_velocity=av*encoder_velocity+(1-av)*(qenc-encoder_previous)/ts;
        encoder_previous=qenc;
        double fm=fa_requested-p->nut_mass*eq[1]+ja*ws*ws*(qtarget-qenc)+
          2*p->servo_zeta*ja*ws*(p->servo_velocity_feedforward*vt-encoder_velocity);
        c.command=fm*ell/p->force_constant;
      }
      if(p->external_current){
        c.command=p->actuator==0?eq[0]/p->force_constant:
          (eq[0]-p->nut_mass*eq[1])*p->lead/(2*PI)/p->force_constant;
        if(t>=p->external_start)c.command+=
          p->external_amplitude*sin(2*PI*p->external_frequency*(t-p->external_start));
      }
      control++;
    }
    plant_rhs(&c,t,x,c.command,robot_position(p,t),dx,o);
    double er=fabs(o[0]-p->force_target);m->steps++;
    double scored_interval=t-previous_time;previous_time=t;
    error_sum+=scored_interval*(o[0]-p->force_target)*(o[0]-p->force_target);
    current_sum+=scored_interval*x[8]*x[8];
    if(er>m->force_peak_error){m->force_peak_error=er;m->force_peak_time=t;}
    double early=p->bump_start-p->pad_diameter/(2*p->scan_speed);
    double late=p->bump_start+p->bump_width/p->scan_speed+p->pad_diameter/(2*p->scan_speed);
    if(t>=early&&t<=late)m->bump_window_peak_error=fmax(m->bump_window_peak_error,er);
    else m->flat_peak_error=fmax(m->flat_peak_error,er);
    m->force_min=fmin(m->force_min,o[0]);m->force_max=fmax(m->force_max,o[0]);
    m->current_peak=fmax(m->current_peak,fabs(x[8]));
    m->voltage_peak_request=fmax(m->voltage_peak_request,fabs(o[4]));
    clip_voltage+=scored_interval*(fabs(o[4])>p->voltage_limit+1e-10);
    clip_current+=scored_interval*(fabs(c.command)>p->current_limit+1e-10);
    double u=x[0]-x[2];m->tool_peak_deviation=fmax(m->tool_peak_deviation,fabs(u));
    m->nut_peak_deviation=fmax(m->nut_peak_deviation,fabs(x[6]));
    m->nut_velocity_peak=fmax(m->nut_velocity_peak,fabs(x[7]));
    m->rotor_speed_peak_rpm=fmax(m->rotor_speed_peak_rpm,fabs(x[7])*60/p->lead);
    m->spring_peak_deflection=fmax(m->spring_peak_deflection,p->actuator!=0?fabs(x[6]-u):0);
    m->pad_peak_compression=fmax(m->pad_peak_compression,o[5]);
    m->force_reconstruction_peak_error=fmax(m->force_reconstruction_peak_error,fabs(force_estimate-o[0]));
    m->force_balance_peak=fmax(m->force_balance_peak,fabs(o[0]-o[1]-p->mass_s*(dx[5]+eq[1])));
    band+=scored_interval*(fabs(u)>p->robot_soft_band);
    if(trace&&recorded<capacity&&(t>=next_trace-1e-12||t>=p->duration-1e-12)){
      double *tr=trace+recorded*NCOL;
      tr[0]=t;tr[1]=o[0];tr[2]=force_estimate;tr[3]=o[1];tr[4]=dx[5];
      tr[5]=x[8];tr[6]=c.command;tr[7]=o[4];
      for(int j=0;j<8;j++)tr[8+j]=x[j];
      tr[16]=o[2];tr[17]=robot_position(p,t);tr[18]=0;tr[19]=o[5];
      tr[20]=fa_requested;tr[21]=vhat;tr[22]=measured[2];recorded++;
      next_trace+=stride*p->dt;
    }
    int reason=0;
    if(!isfinite(x[8])||!isfinite(o[0])||!isfinite(x[0]))reason=6;
    else if(o[0]<=0)reason=1;
    else if(fabs(u)>p->tool_travel_limit)reason=2;
    else if(p->actuator!=0&&fabs(x[6])>p->nut_travel_limit)reason=3;
    else if(p->actuator!=0&&fabs(x[6]-u)>p->spring_deflection_limit)reason=4;
    else if(p->pad_compression_limit>0&&o[5]>p->pad_compression_limit)reason=5;
    if(reason){m->stop_code=reason;m->stop_time=t;break;}
    if(t>=p->duration-1e-12)break;
    if(t+1e-12>=base*p->dt){
      double scale=1/sqrt(2*p->dt*p->noise_reference_enbw);
      c.noise[0]=p->noise_enabled*p->force_noise_rms*scale*sqrt(3)*(2*uniform(&c)-1);
      c.noise[1]=p->noise_enabled*p->accel_noise_rms*scale*normal(&c);
      c.noise[2]=p->noise_enabled*p->position_noise_rms*scale*normal(&c);base++;
    }
    double next=fmin(p->duration,fmin(base*p->dt,fmin(adc/p->adc_rate,control/p->control_rate)));
    if(qhead!=qtail)next=fmin(next,queue_time[qhead]);
    double h=next-t;if(h<=1e-13)return -3;
    rhs(&c,t,x,k1);
    for(int j=0;j<NST;j++)tmp[j]=x[j]+h*k1[j]/2;
    rhs(&c,t+h/2,tmp,k2);
    for(int j=0;j<NST;j++)tmp[j]=x[j]+h*k2[j]/2;
    rhs(&c,t+h/2,tmp,k3);
    for(int j=0;j<NST;j++)tmp[j]=x[j]+h*k3[j];
    rhs(&c,t+h,tmp,k4);
    for(int j=0;j<NST;j++)x[j]+=h*(k1[j]+2*k2[j]+2*k3[j]+k4[j])/6;
    t=next;
  }
  double elapsed_time=fmax(t,p->dt);
  m->force_rms_error=sqrt(error_sum/elapsed_time);m->current_rms=sqrt(current_sum/elapsed_time);
  m->voltage_clipped_fraction=clip_voltage/elapsed_time;m->current_clipped_fraction=clip_current/elapsed_time;
  m->centering_band_fraction=band/elapsed_time;m->adc_count=adc;m->control_count=control;
  m->force_noise_filtered_rms=sqrt(noise_force_sum/adc);m->accel_noise_filtered_rms=sqrt(noise_accel_sum/adc);
  clock_gettime(CLOCK_MONOTONIC,&end);m->runtime_s=elapsed(start,end);return recorded;
}

/* Independent open-loop physical plant replay on exactly regular RK4 steps.
 * commands are10kHz major-time samples, held through each RK4 endpoint.
 * output columns: [Fpad,zA,vA,zB,vB,zS,vS,q,vq,i]. No controller/filter states.
 */
int engine_replay(const Parameters *p,const double *commands,int ncommands,
                  double *output,int noutput){
  int steps=(int)llround(p->duration/p->dt);
  int interval=(int)llround(1/(p->control_rate*p->dt));
  if(interval<1||fabs(interval*p->dt*p->control_rate-1)>1e-8||noutput<steps+1)return -1;
  if(ncommands<steps/interval+1)return -2;
  Context c;initialize_context(&c,p);double x[9],eq[2],tmp[9],k1[9],k2[9],k3[9],k4[9],o[10];
  engine_initialize(p,x,eq);
  for(int n=0;n<=steps;n++){
    double t=n*p->dt,command=commands[n/interval];
    plant_rhs(&c,t,x,command,robot_position(p,t),k1,o);
    output[n*10]=o[0];for(int j=0;j<9;j++)output[n*10+1+j]=x[j];
    if(n==steps)break;
    for(int j=0;j<9;j++)tmp[j]=x[j]+p->dt*k1[j]/2;
    plant_rhs(&c,t+p->dt/2,tmp,command,robot_position(p,t+p->dt/2),k2,NULL);
    for(int j=0;j<9;j++)tmp[j]=x[j]+p->dt*k2[j]/2;
    plant_rhs(&c,t+p->dt/2,tmp,command,robot_position(p,t+p->dt/2),k3,NULL);
    for(int j=0;j<9;j++)tmp[j]=x[j]+p->dt*k3[j];
    plant_rhs(&c,t+p->dt,tmp,command,robot_position(p,t+p->dt),k4,NULL);
    for(int j=0;j<9;j++)x[j]+=p->dt*(k1[j]+2*k2[j]+2*k3[j]+k4[j])/6;
  }
  return steps+1;
}
