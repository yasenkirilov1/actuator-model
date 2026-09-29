function [dx,a] = acf_plant_rhs(x,command,robot,surface,contact,p)
%ACF_PLANT_RHS Coupled mechanics and actuator. Positive translation is UP.
% u=xA-xB is a CHANGE in gap; actual travel=midTravel+u.
% No contact stiffness or contact damper exists in this model.
m=p.mechanics; d=p.d;
z=x(1); vz=x(2); u=x(3); w=x(4); current=x(7);
xA=z+d.beta*u; vA=vz+d.beta*w;
xB=z-d.alpha*u; vB=vz-d.alpha*w;
if contact, s=surface(1); vs=surface(2); else, s=x(5); vs=x(6); end
q1=d.q10+m.k1*(s-xB)+m.c1*(vs-vB);
q2=d.q20+m.k2*(xA-robot(1))+m.c2*(vA-robot(2));
az=(q1-q2)/d.M-d.g;
loadForce=d.beta*q2+d.alpha*q1;
omega=w/d.ell;
di=0; speedIntegralRate=0; voltage=0; currentRequest=0;
torque=0; effortLimited=0; voltageLimited=0;
switch p.actuator.kind
    case 'leadscrew'
        c=p.screw;
        speedError=command-omega;
        rawCurrent=d.screwHoldCurrent+d.speedKp*speedError+x(8);
        currentRequest=clip(rawCurrent,-c.peakCurrent,c.peakCurrent);
        speedIntegralRate=d.speedKi*speedError+c.speedAntiWindup*(currentRequest-rawCurrent);
        rawVoltage=c.resistance*current+c.backEmfConstant*omega+ ...
            c.inductance*(currentRequest-current)/c.currentResponse;
        voltage=clip(rawVoltage,-c.voltage,c.voltage);
        di=(voltage-c.resistance*current-c.backEmfConstant*omega)/c.inductance;
        torque=c.torqueConstant*current;
        friction=c.coulombTorque*tanh(omega/c.frictionSpeed)+c.viscousTorque*omega;
        au=((torque-friction)/d.ell-loadForce)/(d.mu+c.rotorInertia/d.ell^2);
        effortLimited=double(abs(rawCurrent) > c.peakCurrent);
        voltageLimited=double(abs(rawVoltage) > c.voltage);
    case 'voice_coil'
        c=p.coil;
        currentRequest=clip(command,-c.peakCurrent,c.peakCurrent);
        rawVoltage=c.resistance*current+c.forceConstant*w+ ...
            c.inductance*(currentRequest-current)/c.currentResponse;
        voltage=clip(rawVoltage,-c.voltage,c.voltage);
        di=(voltage-c.resistance*current-c.forceConstant*w)/c.inductance;
        friction=c.coulombForce*tanh(w/c.frictionSpeed)+c.viscousFriction*w;
        au=(c.forceConstant*current-friction-loadForce)/d.mu;
        effortLimited=double(abs(command) >= c.peakCurrent*(1-1e-9));
        voltageLimited=double(abs(rawVoltage) > c.voltage);
    otherwise
        c=p.ideal;
        velocityRequest=clip(c.positionGain*(command-u),-c.maxSpeed,c.maxSpeed);
        rawAcceleration=(velocityRequest-w)/c.velocityTime;
        au=clip(rawAcceleration,-c.maxAcceleration,c.maxAcceleration);
        wantedForce=d.mu*au+loadForce;
        availableForce=clip(wantedForce,-c.maxAxialForce,c.maxAxialForce);
        au=(availableForce-loadForce)/d.mu;
        effortLimited=double(abs(rawAcceleration)>c.maxAcceleration || abs(wantedForce)>c.maxAxialForce);
end
% Inelastic stroke stop. Reaching a stop is always a failed sizing run.
lo=m.travelMin-m.midTravel; hi=m.travelMax-m.midTravel;
if (u<=lo+1e-12 && w<=0 && au<0) || (u>=hi-1e-12 && w>=0 && au>0), au=0; end
Fa=d.mu*au+loadForce; % transmitted internal force, + on A and - on B
if contact
    aS=surface(3);
    Fc=q1+m.mS*(d.g+aS);
else
    aS=-q1/m.mS-d.g;
    Fc=0;
end
dx=[vz;az;w;au;vs;aS;di;speedIntegralRate];
% Dynamics/diagnostics are in a fixed order; see acf_signals.m.
a=[Fc;Fa;az;au;az+d.beta*au;az-d.alpha*au; ...
    q1;q2;current;voltage;torque;omega;currentRequest;effortLimited;voltageLimited];
end

function y=clip(x,lo,hi)
y=min(max(x,lo),hi);
end
