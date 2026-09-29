function [surface, nominalVelocity] = acf_surface(t,p)
%ACF_SURFACE Analytic position, velocity and acceleration; no derivatives of steps.
% The robot is NOT told the measured surface height.
surface = zeros(3,1);
nominalVelocity = p.robot.nominalVelocity;
dt = t-p.surface.startTime;
kind = p.surface.kind;
if strcmp(kind,'robot_error')
    [~,d,~] = smoothstep(dt/p.robot.errorRiseTime);
    nominalVelocity = nominalVelocity-p.robot.errorForce/p.d.keq*d/p.robot.errorRiseTime;
    return
end
if strcmp(kind,'flat') || dt < 0, return; end
if strcmp(kind,'repeated'), dt = mod(dt,p.surface.repeatPeriod); end
q = dt/p.d.duration;
if strcmp(kind,'ramp')
    [b,d,dd] = smoothstep(q);
else
    if q >= 1, return; end
    b = 64*q^3*(1-q)^3;
    d = 192*q^2-768*q^3+960*q^4-384*q^5;
    dd = 384*q-2304*q^2+3840*q^3-1920*q^4;
end
surface = p.surface.height*[b;d/p.d.duration;dd/p.d.duration^2];
end

function [b,d,dd] = smoothstep(q)
if q <= 0, b=0; d=0; dd=0;
elseif q >= 1, b=1; d=0; dd=0;
else
    b=10*q^3-15*q^4+6*q^5;
    d=30*q^2-60*q^3+30*q^4;
    dd=60*q-180*q^2+120*q^3;
end
end
