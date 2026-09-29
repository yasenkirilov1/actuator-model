function r = acf_run(mdl,p,makePlots)
%ACF_RUN Configure a generated model, simulate, and return named signals/metrics.
if nargin<2, p=getVariable(get_param(mdl,'ModelWorkspace'),'p'); end
if nargin<3, makePlots=true; end
p=acf_prepare(p);
mw=get_param(mdl,'ModelWorkspace'); assignin(mw,'p',p);
set_param(mdl,'FixedStep',num2str(p.sim.step,17),'StopTime',num2str(p.sim.stopTime,17));
out=sim(mdl);
a=out.get('plant_log'); b=out.get('robot_log');
c=out.get('surface_log'); d=out.get('control_log');
r.time=a.Time(:);
assert(isequal(r.time,b.Time(:),c.Time(:),d.Time(:)), 'Log sample grids do not match.');
r.data=[rows(a,25),rows(b,2),rows(c,3),rows(d,5)];
r.parameters=p; r.names=acf_signals; r.metrics=acf_metrics(r);
fprintf('%s / %s: %s | peak force error %.4g N | peak gap accel %.4g m/s^2\n', ...
    p.actuator.kind,p.surface.kind,r.metrics.reason,r.metrics.peakAbsForceError_N, ...
    r.metrics.peakGapAcceleration_m_s2);
if makePlots, acf_plot(r); end
end

function d=rows(ts,width)
n=numel(ts.Time); d=ts.Data;
if size(d,1)~=n
    assert(size(d,ndims(d))==n,'Unexpected timeseries array shape.');
    d=permute(d,[ndims(d),1:ndims(d)-1]);
end
d=reshape(d,n,width);
end
