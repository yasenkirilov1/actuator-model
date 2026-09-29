function [measured,xn] = acf_sensor(t,x,trueForce,p)
% An already compensated contact-force channel, with editable residual errors.
% This is NOT a model of an arbitrarily placed raw load cell.
xn=x; measured=x(2);
if mod(round(t/p.sim.step),p.d.sensorTicks)~=0, return; end
if p.sensor.filterTime==0, alpha=0;
else, alpha=exp(-p.sensor.sampleTime/p.sensor.filterTime); end
xn(1)=alpha*x(1)+(1-alpha)*trueForce;
xn(3)=mod(16807*x(3),2147483647); % repeatable Park-Miller sequence
noise=sqrt(3)*p.sensor.noiseRms*(2*xn(3)/2147483647-1);
sample=xn(1)+p.sensor.bias+noise;
if p.d.sensorDelaySamples==0
    measured=sample;
else
    measured=x(4);
    xn(4:end)=[x(5:end);sample];
end
xn(2)=measured;
end
