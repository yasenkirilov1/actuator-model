function r = acf_reference(p)
%ACF_REFERENCE Run the SAME component functions in plain MATLAB, no Simulink.
% Also enables direct Simulink/reference comparison via acf_selftest(model,p).
p=acf_prepare(p); n=round(p.sim.stopTime/p.sim.step)+1;
r.time=(0:n-1)'*p.sim.step; r.data=zeros(n,35);
xp=acf_initial('plant',p); xr=acf_initial('robot',p);
xf=acf_initial('force',p); xs=acf_initial('sensor',p);
xc=acf_initial('recenter',p); xd=acf_initial('delay',p);
for k=1:n
    t=r.time(k);
    [surface,vnom]=acf_surface(t,p);
    obs=acf_observe(xp,xr,surface,p);
    [measured,xsn]=acf_sensor(t,xs,obs(1),p);
    [command,xfn]=acf_controller(t,xf,p.force.reference-measured,p);
    [vrequest,xcn]=acf_recenter(t,xc,obs(2),vnom,p);
    [vdelayed,xdn]=acf_delay(t,xd,vrequest,p);
    r.data(k,:)=[obs;xr;surface;p.force.reference;measured;command;vrequest;vdelayed]';
    if k<n
        xp=acf_plant_step(xp,command,xr,surface,p);
        xr=acf_robot_step(xr,vdelayed,p);
        xs=xsn; xf=xfn; xc=xcn; xd=xdn;
    end
end
r.parameters=p; r.names=acf_signals; r.metrics=acf_metrics(r);
end
