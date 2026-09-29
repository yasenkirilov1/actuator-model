function [delayed,xn] = acf_delay(t,x,command,p)
delayed=x(1); xn=x;
if mod(round(t/p.sim.step),p.d.robotTicks)~=0, return; end
if p.d.robotDelaySamples==0
    delayed=command;
else
    delayed=x(2);
    xn(2:end)=[x(3:end);command];
end
xn(1)=delayed;
end
