function acf_plot(r)
p=r.parameters; t=r.time; d=r.data;
if isfield(r,'normalContact'),d(:,1)=r.normalContact.data(:,1);end
% Let the figure and axes use the same MATLAB light/dark theme.
f=figure('Name',['ACF: ' p.actuator.kind ' / ' p.surface.kind]);
if exist('acf_visualize','file')==2
    uimenu(f,'Label','2D replay','Callback',@(~,~) acf_visualize(r));
end
subplot(3,2,1);
plot(t,d(:,31)-d(:,1),'LineWidth',1.2); hold on;
if isfield(r,'normalContact')
    plot(t,p.force.reference-r.normalContact.data(:,4),':');
    estimateLabel='Estimated normal';
elseif isfield(r,'contactEstimate')
    plot(r.contactEstimate.time,p.force.reference-r.contactEstimate.force_N,':');
    estimateLabel='Estimated normal';
else
    plot(t,d(:,31)-d(:,32),':');estimateLabel='Cell reading';
end
plot(t,0*t+p.force.tolerance,'--r',t,0*t-p.force.tolerance,'--r');
ylabel('Normal-force error (N)'); grid on; legend('True',estimateLabel,'Tolerance','Location','best');
title(r.metrics.reason,'Interpreter','none');
subplot(3,2,2);
plot(t,1000*d(:,2)); hold on;
plot(t,0*t+1000*p.mechanics.midTravel,'--');
ylabel('Actuator travel (mm)'); grid on;
subplot(3,2,3);
plot(t,1000*d(:,26),t,1000*d(:,28));
ylabel('Position change (mm)'); grid on; legend('Robot','Surface','Location','best');
subplot(3,2,4);
plot(t,d(:,13)); ylabel('Actuator axial force (N)'); grid on;
subplot(3,2,5);
plot(t,d(:,4)); ylabel('Gap acceleration (m/s^2)'); xlabel('Time (s)'); grid on;
subplot(3,2,6);
if strcmp(p.actuator.kind,'ideal_position')
    plot(t,d(:,3)); ylabel('Gap speed (m/s)');
else
    plot(t,d(:,14)); ylabel('Motor / coil current (A)');
end
xlabel('Time (s)'); grid on;
if r.metrics.impactCount>0
    fprintf('Rigid impacts: %d; total impulse %.4g N s. Finite force plots exclude the impulses.\n', ...
        r.metrics.impactCount,r.metrics.totalImpactImpulse_Ns);
end
end
