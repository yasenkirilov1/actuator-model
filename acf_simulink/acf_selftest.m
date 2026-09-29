function acf_selftest(mdl,p)
%ACF_SELFTEST Checks physics and, optionally, Simulink versus MATLAB execution.
% acf_selftest                 : equilibrium, mechanics, contact, delay checks
% acf_selftest(acfModel,acfParameters) : also compare a 0.25 s Simulink run
kinds={'leadscrew','voice_coil','ideal_position'};
for j=1:numel(kinds)
    q=acf_parameters(kinds{j},'flat'); x=acf_initial('plant',q);
    robot=[q.d.r0;0]; surface=zeros(3,1);
    [dx,a]=acf_plant_rhs(x(1:8),x(13),robot,surface,true,q);
    assert(max(abs(dx))<1e-9,'Initial equilibrium is inconsistent.');
    assert(abs(a(1)-q.force.reference)<1e-12);
    % Unequal-mass kinematics plus equal/opposite internal actuator force.
    v=x(1:8); v(1)=v(1)+1e-4; v(3)=v(3)+2e-4; v(4)=1e-3;
    [~,a]=acf_plant_rhs(v,x(13),robot,surface,true,q);
    assert(abs(q.mechanics.mA*a(5)-(a(2)-a(8)-q.mechanics.mA*q.d.g))<1e-9);
    assert(abs(q.mechanics.mB*a(6)-(a(7)-a(2)-q.mechanics.mB*q.d.g))<1e-9);
    if strcmp(q.actuator.kind,'leadscrew')
        omega=v(4)/q.d.ell;
        friction=q.screw.coulombTorque*tanh(omega/q.screw.frictionSpeed)+q.screw.viscousTorque*omega;
        assert(abs(a(11)-(q.d.ell*a(2)+q.screw.rotorInertia*a(4)/q.d.ell+friction))<1e-10);
    end
end
q=acf_parameters('voice_coil','flat'); q.robot.recenterGain=0; q=acf_prepare(q);
x=acf_initial('plant',q); x(9)=0; x(5)=1e-5; x(6)=-0.1;
xn=acf_plant_step(x,x(13),[q.d.r0;0],zeros(3,1),q);
assert(xn(9)==1 && xn(10)==1 && xn(11)>0,'Rigid re-impact was not recorded.');
assert(xn(5)==0 && xn(6)==0,'Rigid contact failed to project onto the surface.');
xd=acf_initial('delay',q);
for k=0:round(q.robot.delay/q.sim.step)
    [v,xd]=acf_delay(k*q.sim.step,xd,1,q);
    if k*q.sim.step<q.robot.delay-1e-10, assert(v==0); else, assert(v==1); end
end
fprintf('Physics, rigid-contact and delay checks passed.\n');
if nargin>=1
    if nargin<2, p=getVariable(get_param(mdl,'ModelWorkspace'),'p'); end
    original=p; p.sim.stopTime=0.25; p.sim.stopTime=round(p.sim.stopTime/p.sim.step)*p.sim.step;
    a=acf_run(mdl,p,false); b=acf_reference(p);
    err=max(max(abs(a.data-b.data)));
    assignin(get_param(mdl,'ModelWorkspace'),'p',original);
    assert(err<1e-8,'Simulink/reference mismatch: max absolute signal difference %.4g.',err);
    fprintf('Simulink/reference execution agrees (max absolute difference %.4g).\n',err);
end
end
