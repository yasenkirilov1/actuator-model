/* Bounded deterministic translation of the attached solver, NOT a MATLAB
 * execution or a qualified flange plant. Gaussian noise is disabled; fixed
 * sensor errors are selectively retained. No full histories are allocated.
 * Source lines 629-768 are transcribed with the original update order.
 */
#include <math.h>
#include <stdio.h>

static const double pi = 3.14159265358979323846;

static void run_case(const char *name, int flat, int pid, int sensor_errors,
                     double target, double theta, double dt, double buzz,
                     double buzz_hz, int trace) {
    const double mass=1.726, kp=.22, ki=2.20, kd=.004;
    const double kpad=20000, dpad=80, kf=25.6, resistance=5.7;
    const double radius=.077/2, feed=.050, duration=20, amp=.00015;
    const long count=lround(duration/dt), ticks=lround(.001/dt);
    double offsets[35], weights[35], total_weight=0;
    for(int j=0;j<35;j++) {
        offsets[j]=-radius+2*radius*j/34;
        weights[j]=2*sqrt(fmax(0,radius*radius-offsets[j]*offsets[j]));
        total_weight+=weights[j];
    }
    for(int j=0;j<35;j++) weights[j]/=total_weight;
    const double gravity=mass*9.81*cos(theta*pi/180);
    const double feedforce=target-gravity, nominal=target/kpad;
    const double drive_alpha=1-exp(-dt/.00004), afe_alpha=1-exp(-dt/.0001);
    double y=nominal, velocity=0, current=feedforce/kf, command=current;
    double integrator=0, previous_error=0, afe=target, measured=target;
    double peak_all=0, peak_after_point2=0, peak_after5=0;
    double square5=0,sine=0,cosine=0, max_accel=0,min_stroke=1e9,max_stroke=-1e9;
    double peak_current=0,mean_power=0,mean_error=0,peak_inertial_cell_error=0;
    long samples5=0,contact_loss=0,stroke_outside=0,adc_overflow=0;
    for(long index=0;index<count;index++) {
        const double time=duration*index/(count-1);
        const double x=feed*time;
        const double robot=amp*sin(2*pi*1.2*time);
        const double robot_velocity=2*pi*1.2*amp*cos(2*pi*1.2*time);
        double effective_penetration=0;
        for(int j=0;j<35;j++) {
            double z=0,hole_fraction=0;
            if(!flat) {
                const double local=x+offsets[j];
                const double macro=.00015*sin(2*pi*local/.300)
                    +.00008*.5*(1+tanh((local-.620)/.020))
                    +(local>=.900?.00005*sin(2*pi*local/.025):0);
                const double strip_width=2*sqrt(fmax(0,radius*radius-offsets[j]*offsets[j]));
                const double bump_width=2*sqrt(fmax(0,.008*.008-(local-.220)*(local-.220)));
                const double hole_width=2*sqrt(fmax(0,.0025*.0025-(local-.420)*(local-.420)));
                z=macro+.00015*fmin(1,bump_width/fmax(.001,strip_width));
                hole_fraction=fmin(1,hole_width/fmax(.001,strip_width));
            }
            effective_penetration+=fmax(0,y+z)*(1-hole_fraction)*weights[j];
        }
        const double contact=effective_penetration>0?fmax(0,kpad*effective_penetration+dpad*velocity):0;
        double raw=contact;
        if(sensor_errors)
            raw+=4*.10*(contact/100)*(1-contact/100)+.05*(contact/100)*tanh(10*velocity);
        afe+=afe_alpha*(raw-afe);
        if((index+1)%ticks==0) {
            const double q=50.0/4096;
            measured=round(afe/q)*q;
            if(round(afe/q)>4095||round(afe/q)<0) adc_overflow++;
            const double error=target-measured;
            const double derivative=(error-previous_error)/.001;
            previous_error=error;
            double force=feedforce;
            if(pid) {
                integrator=fmax(-15,fmin(15,integrator+ki*.001*error));
                force+=kp*error+integrator+kd*derivative;
            }
            command=fmax(-202.3,fmin(202.3,force))/kf;
        }
        current+=drive_alpha*(command-current);
        const double force=kf*current;
        const double friction=12*(velocity-robot_velocity)+.08*tanh(100*(velocity-robot_velocity));
        const double acceleration=(force+gravity-contact-friction+buzz*sin(2*pi*buzz_hz*time))/mass;
        velocity+=dt*acceleration;
        y+=dt*velocity;
        const double stroke=15-(y-nominal-robot)*1000;
        const double error=contact-target;
        peak_all=fmax(peak_all,fabs(error));
        if(time>=.2) peak_after_point2=fmax(peak_after_point2,fabs(error));
        if(time>=5) {
            peak_after5=fmax(peak_after5,fabs(error));square5+=error*error;
            sine+=error*sin(2*pi*1.2*time);cosine+=error*cos(2*pi*1.2*time);
            mean_error+=error;samples5++;
        }
        max_accel=fmax(max_accel,fabs(acceleration));
        peak_inertial_cell_error=fmax(peak_inertial_cell_error,1.1*fabs(acceleration));
        min_stroke=fmin(min_stroke,stroke);max_stroke=fmax(max_stroke,stroke);
        peak_current=fmax(peak_current,fabs(current));mean_power+=current*current*resistance;
        if(contact<=0)contact_loss++;
        if(stroke<0||stroke>30)stroke_outside++;
        if(trace&&index%100==0)
            fprintf(stderr,"%s,%.9f,%.9f\n",name,time,error);
    }
    printf("{\"name\":\"%s\",\"steps\":%ld,\"dt_s\":%.9g,\"peakAll_N\":%.9g,\"peakAfter0p2_N\":%.9g,\"peakAfter5_N\":%.9g,\"rmsAfter5_N\":%.9g,\"meanErrorAfter5_N\":%.9g,\"robotFrequencyAmplitude_N\":%.9g,\"maxAcceleration_m_s2\":%.9g,\"omittedToolInertiaPeak_N\":%.9g,\"minStroke_mm\":%.9g,\"maxStroke_mm\":%.9g,\"peakCurrent_A\":%.9g,\"meanColdCopperPower_W\":%.9g,\"contactLossSamples\":%ld,\"strokeOutsideSamples\":%ld,\"illegal12bitAdcCodes\":%ld}\n",
           name,count,dt,peak_all,peak_after_point2,peak_after5,sqrt(square5/samples5),
           mean_error/samples5,2*hypot(sine,cosine)/samples5,max_accel,peak_inertial_cell_error,
           min_stroke,max_stroke,peak_current,mean_power/count,contact_loss,stroke_outside,adc_overflow);
}

int main(void) {
    run_case("flat_robot_pid_off",1,0,0,25,0,.0001,0,66.7,1);
    run_case("flat_robot_pid_on",1,1,0,25,0,.0001,0,66.7,1);
    run_case("profile_polishing_deterministic",0,1,1,25,0,.0001,.2,66.7,0);
    run_case("profile_polishing_half_step",0,1,1,25,0,.00005,.2,66.7,0);
    run_case("profile_fine_sanding_deterministic",0,1,1,25,0,.0001,1,116.7,0);
    run_case("profile_coarse_sanding_deterministic",0,1,1,25,0,.0001,2.4,166.7,0);
    run_case("profile_horizontal_50N_deterministic",0,1,1,50,90,.0001,.2,66.7,0);
    return 0;
}
