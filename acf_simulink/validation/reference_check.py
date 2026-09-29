"""Independent Python translation for numerical checks, not a Simulink runtime."""
from pathlib import Path
from types import SimpleNamespace as NS
import re, math, json, copy, time
import numpy as np

ROOT=Path(__file__).resolve().parents[1]

def params(kind='leadscrew', scenario='bump'):
    p=NS()
    for line in (ROOT/'acf_parameters.m').read_text().splitlines():
        match=re.match(r'p\.(\w+)\.(\w+)\s*=\s*([^;]+);',line)
        if not match: continue
        section,key,value=match.groups()
        if not hasattr(p,section): setattr(p,section,NS())
        if value.strip()=='kind': value=kind
        elif value.strip()=='scenario': value=scenario
        else: value=eval(value,{'__builtins__':{}},{'pi':math.pi,'inf':float('inf')})
        setattr(getattr(p,section),key,value)
    if scenario=='sharp_bump': p.surface.width=.01
    if scenario=='ramp': p.surface.height=.003; p.surface.width=.2
    if scenario=='repeated': p.sim.stopTime=8
    return prepare(p)

def prepare(p):
    m=p.mechanics; d=p.d=NS()
    d.M=m.mA+m.mB; d.alpha=m.mA/d.M; d.beta=m.mB/d.M
    d.mu=m.mA*m.mB/d.M; d.g=m.gravity*math.cos(m.angle)
    d.keq=m.k1*m.k2/(m.k1+m.k2); d.ell=p.screw.lead/(2*math.pi)
    d.q10=p.force.reference-m.mS*d.g
    d.q20=p.force.reference-(d.M+m.mS)*d.g
    d.holdForce=p.force.reference-(m.mB+m.mS)*d.g
    d.r0=-p.robot.initialExcessForce/d.keq; d.u0=d.r0; d.z0=d.alpha*d.u0
    for k,t in [('forceTicks',p.force.sampleTime),('sensorTicks',p.sensor.sampleTime),('robotTicks',p.robot.sampleTime)]:
        setattr(d,k,round(t/p.sim.step))
    d.sensorDelaySamples=round(p.sensor.delay/p.sensor.sampleTime)
    d.robotDelaySamples=round(p.robot.delay/p.robot.sampleTime)
    d.duration=p.surface.width/p.surface.scanSpeed
    d.screwHoldCurrent=d.ell*d.holdForce/p.screw.torqueConstant
    d.coilHoldCurrent=d.holdForce/p.coil.forceConstant
    J=p.screw.rotorInertia+d.mu*d.ell**2
    d.speedKp=2*p.screw.speedDamping*p.screw.speedBandwidth*J/p.screw.torqueConstant
    d.speedKi=p.screw.speedBandwidth**2*J/p.screw.torqueConstant
    if p.actuator.kind=='leadscrew':
        d.forceKp=p.screw.forceKp/d.ell; d.forceKi=p.screw.forceKi/d.ell
        d.forceFeedforward=0; d.commandMax=p.screw.maxLinearSpeedRequest/d.ell; d.commandMin=-d.commandMax
    elif p.actuator.kind=='voice_coil':
        d.forceKp=p.coil.forceKp; d.forceKi=p.coil.forceKi
        d.forceFeedforward=d.coilHoldCurrent; d.commandMax=p.coil.peakCurrent; d.commandMin=-d.commandMax
    else:
        d.forceKp=p.ideal.forceKp; d.forceKi=p.ideal.forceKi
        d.forceFeedforward=d.u0; d.commandMin=m.travelMin-m.midTravel; d.commandMax=m.travelMax-m.midTravel
    return p

def clip(x,lo,hi): return min(max(x,lo),hi)

def smooth(q):
    if q<=0: return 0.,0.,0.
    if q>=1: return 1.,0.,0.
    return 10*q**3-15*q**4+6*q**5,30*q**2-60*q**3+30*q**4,60*q-180*q**2+120*q**3

def surface(t,p):
    y=np.zeros(3); nv=p.robot.nominalVelocity; dt=t-p.surface.startTime
    kind=p.surface.kind
    if kind=='robot_error':
        nv-=p.robot.errorForce/p.d.keq*smooth(dt/p.robot.errorRiseTime)[1]/p.robot.errorRiseTime
        return y,nv
    if kind=='flat' or dt<0: return y,nv
    if kind=='repeated': dt=dt%p.surface.repeatPeriod
    q=dt/p.d.duration
    if kind=='ramp': b,d,dd=smooth(q)
    elif q>=1: return y,nv
    else:
        b=64*q**3*(1-q)**3
        d=192*q**2-768*q**3+960*q**4-384*q**5
        dd=384*q-2304*q**2+3840*q**3-1920*q**4
    return p.surface.height*np.array([b,d/p.d.duration,dd/p.d.duration**2]),nv

def initial(p):
    x=np.zeros(13); x[0]=p.d.z0; x[2]=p.d.u0; x[8]=1; x[12]=p.d.forceFeedforward
    if p.actuator.kind=='voice_coil': x[6]=p.d.coilHoldCurrent
    elif p.actuator.kind=='leadscrew': x[6]=p.d.screwHoldCurrent
    return x

def rhs(x,cmd,r,y,contact,p):
    m=p.mechanics; d=p.d; z,vz,u,w,s,vs,i,si=x[:8]
    aPos=z+d.beta*u; aVel=vz+d.beta*w
    bPos=z-d.alpha*u; bVel=vz-d.alpha*w
    if contact: s,vs=y[:2]
    q1=d.q10+m.k1*(s-bPos)+m.c1*(vs-bVel)
    q2=d.q20+m.k2*(aPos-r[0])+m.c2*(aVel-r[1])
    az=(q1-q2)/d.M-d.g; load=d.beta*q2+d.alpha*q1; omega=w/d.ell
    di=siRate=volt=ireq=torque=es=vsat=0.
    if p.actuator.kind=='leadscrew':
        c=p.screw; e=cmd-omega
        rawi=d.screwHoldCurrent+d.speedKp*e+si; ireq=clip(rawi,-c.peakCurrent,c.peakCurrent)
        siRate=d.speedKi*e+c.speedAntiWindup*(ireq-rawi)
        rawv=c.resistance*i+c.backEmfConstant*omega+c.inductance*(ireq-i)/c.currentResponse
        volt=clip(rawv,-c.voltage,c.voltage)
        di=(volt-c.resistance*i-c.backEmfConstant*omega)/c.inductance
        torque=c.torqueConstant*i
        fric=c.coulombTorque*math.tanh(omega/c.frictionSpeed)+c.viscousTorque*omega
        au=((torque-fric)/d.ell-load)/(d.mu+c.rotorInertia/d.ell**2)
        es=float(abs(rawi)>c.peakCurrent); vsat=float(abs(rawv)>c.voltage)
    elif p.actuator.kind=='voice_coil':
        c=p.coil; ireq=clip(cmd,-c.peakCurrent,c.peakCurrent)
        rawv=c.resistance*i+c.forceConstant*w+c.inductance*(ireq-i)/c.currentResponse
        volt=clip(rawv,-c.voltage,c.voltage); di=(volt-c.resistance*i-c.forceConstant*w)/c.inductance
        fric=c.coulombForce*math.tanh(w/c.frictionSpeed)+c.viscousFriction*w
        au=(c.forceConstant*i-fric-load)/d.mu
        es=float(abs(cmd)>=c.peakCurrent*(1-1e-9)); vsat=float(abs(rawv)>c.voltage)
    else:
        c=p.ideal; vreq=clip(c.positionGain*(cmd-u),-c.maxSpeed,c.maxSpeed)
        rawa=(vreq-w)/c.velocityTime; au=clip(rawa,-c.maxAcceleration,c.maxAcceleration)
        wanted=d.mu*au+load; available=clip(wanted,-c.maxAxialForce,c.maxAxialForce)
        au=(available-load)/d.mu
        es=float(abs(rawa)>c.maxAcceleration or abs(wanted)>c.maxAxialForce)
    lo=m.travelMin-m.midTravel; hi=m.travelMax-m.midTravel
    if (u<=lo+1e-12 and w<=0 and au<0) or (u>=hi-1e-12 and w>=0 and au>0): au=0.
    fa=d.mu*au+load
    aS=y[2] if contact else -q1/m.mS-d.g
    fc=q1+m.mS*(d.g+aS) if contact else 0.
    return np.array([vz,az,w,au,vs,aS,di,siRate]),np.array([fc,fa,az,au,az+d.beta*au,az-d.alpha*au,q1,q2,i,volt,torque,omega,ireq,es,vsat])

def move_surface(y,h): return np.array([y[0]+y[1]*h+.5*y[2]*h*h,y[1]+y[2]*h,y[2]])

def step(x,cmd,r,y,p):
    h=p.sim.step; xn=x.copy(); v=x[:8].copy(); contact=x[8]>.5; xn[10]=0
    if contact:
        v[4:6]=y[:2]
        if rhs(v,cmd,r,y,True,p)[1][0]<0: contact=False
    def f(xx,dt): return rhs(xx,cmd,[r[0]+r[1]*dt,r[1]],move_surface(y,dt),contact,p)[0]
    k1=f(v,0); k2=f(v+h*k1/2,h/2); k3=f(v+h*k2/2,h/2); k4=f(v+h*k3,h)
    v+=h*(k1+2*k2+2*k3+k4)/6
    yy=move_surface(y,h); rr=[r[0]+r[1]*h,r[1]]
    if contact:
        v[4:6]=yy[:2]
        if rhs(v,cmd,rr,yy,True,p)[1][0]<0: contact=False
    elif v[4]<yy[0]:
        J=max(0,p.mechanics.mS*(yy[1]-v[5])); v[4:6]=yy[:2]; contact=True; xn[10]=J
        if J>1e-10: xn[9]+=1
    lo=p.mechanics.travelMin-p.mechanics.midTravel; hi=p.mechanics.travelMax-p.mechanics.midTravel
    if v[2]<=lo: v[2]=lo; v[3]=max(0,v[3]); xn[11]=1
    elif v[2]>=hi: v[2]=hi; v[3]=min(0,v[3]); xn[11]=1
    xn[:8]=v; xn[8]=float(contact); xn[12]=cmd
    return xn

def observe(x,r,y,p):
    a=rhs(x[:8],x[12],r,y,x[8]>.5,p)[1]; d=p.d
    s,vs=y[:2] if x[8]>.5 else x[4:6]
    return np.array([max(0,a[0]),p.mechanics.midTravel+x[2],x[3],a[3],x[0]+d.beta*x[2],x[0]-d.alpha*x[2],s,x[1]+d.beta*x[3],x[1]-d.alpha*x[3],vs,a[4],a[5],a[1],a[8],a[9],a[10],a[11],x[8],x[9],x[10],x[11],a[13],a[14],a[6],a[7]])

def run(p):
    prepare(p); n=round(p.sim.stopTime/p.sim.step)+1
    tt=np.arange(n)*p.sim.step; data=np.zeros((n,35)); xp=initial(p); r=np.array([p.d.r0,0.])
    fint=0.; cmd=p.d.forceFeedforward; sf=p.force.reference; measured=sf+p.sensor.bias; seed=p.sensor.seed
    sq=[measured]*p.d.sensorDelaySamples; vreq=vdel=p.robot.nominalVelocity
    dq=[vdel]*p.d.robotDelaySamples
    for k,t in enumerate(tt):
        y,nv=surface(t,p); obs=observe(xp,r,y,p)
        if k%p.d.sensorTicks==0:
            alpha=math.exp(-p.sensor.sampleTime/p.sensor.filterTime) if p.sensor.filterTime else 0
            sf=alpha*sf+(1-alpha)*obs[0]; seed=(16807*seed)%2147483647
            samp=sf+p.sensor.bias+math.sqrt(3)*p.sensor.noiseRms*(2*seed/2147483647-1)
            if sq: measured=sq.pop(0); sq.append(samp)
            else: measured=samp
        if k%p.d.forceTicks==0:
            e=p.force.reference-measured; raw=p.d.forceFeedforward+p.d.forceKp*e+fint
            cmd=clip(raw,p.d.commandMin,p.d.commandMax)
            fint+=p.force.sampleTime*(p.d.forceKi*e+p.force.antiWindupRate*(cmd-raw))
        if k%p.d.robotTicks==0:
            e=p.mechanics.midTravel-obs[1]; dz=math.copysign(max(0,abs(e)-p.robot.deadband),e)
            vreq=clip(nv+p.robot.recenterGain*dz,-p.robot.maxSpeed,p.robot.maxSpeed)
            if dq: vdel=dq.pop(0); dq.append(vreq)
            else: vdel=vreq
        data[k]=np.r_[obs,r,y,p.force.reference,measured,cmd,vreq,vdel]
        if k==n-1: break
        xp=step(xp,cmd,r,y,p)
        dv=clip((vdel-r[1])*(1-math.exp(-p.sim.step/p.robot.responseTime)),-p.robot.maxAcceleration*p.sim.step,p.robot.maxAcceleration*p.sim.step)
        r=np.array([r[0]+p.sim.step*(r[1]+.5*dv),r[1]+dv])
    assert np.isfinite(data).all()
    err=data[:,0]-p.force.reference
    metrics=dict(actuator=p.actuator.kind,scenario=p.surface.kind,peak_error_N=float(max(abs(err))),rms_error_N=float(np.sqrt(np.mean(err**2))),peak_gap_accel=float(max(abs(data[:,3]))),peak_axial_force=float(max(abs(data[:,12]))),peak_current=float(max(abs(data[:,13]))),lost_contact=bool(np.any(data[:,17]<.5)),impacts=int(data[-1,18]),stroke_stop=bool(np.any(data[:,20]>.5)),time_step=p.sim.step)
    metrics['pass']=bool(metrics['peak_error_N']<=p.force.tolerance and not(metrics['lost_contact'] or metrics['impacts'] or metrics['stroke_stop']))
    return tt,data,metrics

def invariants():
    rng=np.random.default_rng(5); checked=0
    for kind in ['leadscrew','voice_coil','ideal_position']:
        p=params(kind,'flat'); x=initial(p); r=np.array([p.d.r0,0.]); y=np.zeros(3)
        dx,a=rhs(x[:8],x[12],r,y,True,p)
        assert np.max(abs(dx))<1e-9,(kind,dx)
        assert abs(a[0]-p.force.reference)<1e-12
        for j in range(50):
            v=x[:8]+rng.normal(size=8)*np.array([.0001,.001,.0001,.001,.0001,.001,.1,.01])
            dx,a=rhs(v,x[12],r,y,True,p)
            m=p.mechanics; d=p.d
            assert abs(m.mA*a[4]-(a[1]-a[7]-m.mA*d.g))<1e-9
            assert abs(m.mB*a[5]-(a[6]-a[1]-m.mB*d.g))<1e-9
            assert abs(a[0]-a[6]-m.mS*d.g)<1e-9
            if kind=='leadscrew':
                omega=v[3]/d.ell
                fr=p.screw.coulombTorque*math.tanh(omega/p.screw.frictionSpeed)+p.screw.viscousTorque*omega
                assert abs(a[10]-(d.ell*a[1]+p.screw.rotorInertia*a[3]/d.ell+fr))<1e-10
            checked+=1
    return checked

if __name__=='__main__':
    print('Algebraic equilibrium and Newton/screw checks:',invariants(),flush=True)
    reports=[]; start=time.time()
    for kind,scenario in [('leadscrew','flat'),('leadscrew','bump'),('voice_coil','bump'),('ideal_position','bump'),('leadscrew','sharp_bump'),('leadscrew','ramp')]:
        p=params(kind,scenario); t,data,metrics=run(p); reports.append(metrics)
        print(json.dumps(metrics),flush=True)
        if kind=='leadscrew' and scenario=='bump':
            np.savez(str(ROOT/'validation/reference_baseline.npz'),time=t,data=data)
    (ROOT/'validation/reproduced_results.json').write_text(json.dumps(reports,indent=2))
    print('Elapsed seconds',time.time()-start,flush=True)
