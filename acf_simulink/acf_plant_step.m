function xn = acf_plant_step(x,command,robot,surface,p)
%ACF_PLANT_STEP RK4 smooth dynamics + unilateral rigid-contact projection.
% Contact events are resolved to the numerical timestep. Re-impact is
% perfectly inelastic. Its impulse is logged separately, never as a spring.
h=p.sim.step; xn=x; xc=x(1:8); contact=x(9)>0.5;
xn(11)=0;
if contact
    xc(5:6)=surface(1:2);
    [~,a]=acf_plant_rhs(xc,command,robot,surface,true,p);
    if a(1)<0, contact=false; end
end
k1=rhs(xc,0);
k2=rhs(xc+h*k1/2,h/2);
k3=rhs(xc+h*k2/2,h/2);
k4=rhs(xc+h*k3,h);
xc=xc+h*(k1+2*k2+2*k3+k4)/6;
lastSurface=move_surface(surface,h);
lastRobot=[robot(1)+robot(2)*h;robot(2)];
if contact
    xc(5:6)=lastSurface(1:2);
    [~,a]=acf_plant_rhs(xc,command,lastRobot,lastSurface,true,p);
    if a(1)<0, contact=false; end
else
    if xc(5)<lastSurface(1)
        impulse=max(0,p.mechanics.mS*(lastSurface(2)-xc(6)));
        xc(5:6)=lastSurface(1:2);
        contact=true;
        xn(11)=impulse;
        if impulse>1e-10, xn(10)=xn(10)+1; end
    end
end
lo=p.mechanics.travelMin-p.mechanics.midTravel;
hi=p.mechanics.travelMax-p.mechanics.midTravel;
if xc(3)<=lo
    xc(3)=lo; xc(4)=max(0,xc(4)); xn(12)=1;
elseif xc(3)>=hi
    xc(3)=hi; xc(4)=min(0,xc(4)); xn(12)=1;
end
xn(1:8)=xc; xn(9)=double(contact); xn(13)=command;
assert(all(isfinite(xn)), 'Nonfinite plant state: reduce timestep or check parameters.');

    function dx=rhs(v,dt)
        rr=[robot(1)+robot(2)*dt;robot(2)];
        yy=move_surface(surface,dt);
        dx=acf_plant_rhs(v,command,rr,yy,contact,p);
    end
end

function yy=move_surface(y,dt)
yy=[y(1)+y(2)*dt+0.5*y(3)*dt^2;y(2)+y(3)*dt;y(3)];
end
