function xn = acf_robot_step(x,command,p)
% Robot is a prescribed motion boundary with finite velocity/acceleration.
% The delayed command is a normal-axis velocity; there is no robot force input.
vtarget=min(max(command,-p.robot.maxSpeed),p.robot.maxSpeed);
dv=(vtarget-x(2))*(1-exp(-p.sim.step/p.robot.responseTime));
dv=min(max(dv,-p.robot.maxAcceleration*p.sim.step),p.robot.maxAcceleration*p.sim.step);
vnext=x(2)+dv;
xn=[x(1)+0.5*p.sim.step*(x(2)+vnext);vnext];
end
