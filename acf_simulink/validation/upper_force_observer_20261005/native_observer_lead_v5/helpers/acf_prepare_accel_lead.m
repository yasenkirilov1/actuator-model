function p=acf_prepare_accel_lead(p)
% Proper causal present-time prediction, restricted to force-current FF.
assert(isfield(p,'accelLead')&&p.accelLead.enabled);
dt=p.force.sampleTime;tauP=p.accelLead.poleTime_s;tauA=p.accelLead.advanceTime_s;
assert(abs(dt-1e-4)<1e-15&&tauP>0&&tauA>=0);
alpha=2/dt;
num=[1+alpha*(tauP+tauA),1-alpha*(tauP+tauA)]/(1+alpha*tauP);
den=[1,(1-alpha*tauP)/(1+alpha*tauP)];
assert(abs(den(2))<1&&abs(sum(num)/sum(den)-1)<1e-12);
p.d.accelLead=struct('numerator',num,'denominator',den,'sampleTime_s',dt, ...
 'pole',-den(2),'steadyGain',1,'highFrequencyGain',1+tauA/tauP, ...
 'continuousNumerator',[tauP+tauA,1],'continuousDenominator',[tauP,1]);
end
