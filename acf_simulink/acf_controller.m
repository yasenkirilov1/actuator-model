function [command,xn] = acf_controller(t,x,error,p)
% Sampled force PI. Command units differ by actuator; see acf_prepare.
xn=x; command=x(2);
if mod(round(t/p.sim.step),p.d.forceTicks)~=0, return; end
raw=p.d.forceFeedforward+p.d.forceKp*error+x(1);
command=min(max(raw,p.d.commandMin),p.d.commandMax);
xn(1)=x(1)+p.force.sampleTime*(p.d.forceKi*error+ ...
    p.force.antiWindupRate*(command-raw));
xn(2)=command;
end
