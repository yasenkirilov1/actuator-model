function [command,xn] = acf_recenter(t,x,travel,nominalVelocity,p)
command=x; xn=x;
if mod(round(t/p.sim.step),p.d.robotTicks)~=0, return; end
e=p.mechanics.midTravel-travel;
e=sign(e)*max(0,abs(e)-p.robot.deadband);
command=nominalVelocity+p.robot.recenterGain*e;
command=min(max(command,-p.robot.maxSpeed),p.robot.maxSpeed);
xn=command;
end
