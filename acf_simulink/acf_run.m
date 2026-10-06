function r = acf_run(mdl,p,makePlots)
%ACF_RUN Apply settings to a saved model, simulate, and return signals/metrics.
mw=get_param(mdl,'ModelWorkspace');
if nargin<2
    if getSimulinkBlockHandle([mdl '/Actuator selection'])~=-1
        p=acf_select_actuator(mdl);
    else
        p=getVariable(get_param(mdl,'ModelWorkspace'),'p');
    end
    p.sim.stopTime=evalin(mw,get_param(mdl,'StopTime'));
    p.sim.step=evalin(mw,get_param(mdl,'FixedStep'));
end
if nargin<3, makePlots=true; end
if hasVariable(mw,'acfImplementation') && strcmp(getVariable(mw,'acfImplementation'),'simple')
    if getSimulinkBlockHandle([mdl '/Actuator selection'])~=-1
        assignin(mw,'p',p);
        p=acf_select_actuator(mdl,p.actuator.kind);
    else
        p=acf_prepare(p);assignin(mw,'p',p);
        kinds={'leadscrew','voice_coil','ideal_position'};
        labels={'Leadscrew','Voice coil','Ideal position'};
        set_param([mdl '/Actuator and RABS mechanics/Actuator'], ...
            'LabelModeActiveChoice',labels{strcmp(kinds,p.actuator.kind)});
    end
else
    p=acf_prepare(p);assignin(mw,'p',p);
end
set_param(mdl,'FixedStep',num2str(p.sim.step,17),'StopTime',num2str(p.sim.stopTime,17));
out=sim(mdl);
a=out.get('plant_log'); b=out.get('robot_log');
c=out.get('surface_log'); d=out.get('control_log');
r.time=a.Time(:);
assert(isequal(r.time,b.Time(:),c.Time(:),d.Time(:)), 'Log sample grids do not match.');
r.data=[rows(a,25),rows(b,2),rows(c,3),rows(d,5)];
r.parameters=p; r.names=acf_signals;
if any(strcmp(who(out),'normal_contact_log'))
    logged=out.get('normal_contact_log');
    r.normalContact=struct('time',logged.Time(:),'data',rows(logged,8), ...
        'names',{{'normalForce_N','lateralForce_N','trueSlope', ...
        'estimatedNormalForce_N','estimatedSlope','requestedAxialForce_N', ...
        'measuredLateralForce_N','predictedRobotVelocity_m_s'}});
    assert(isequal(r.time,r.normalContact.time),'Normal-force sample grid differs.');
    % The public force channel and pass/fail metrics describe the workpiece.
    % Retain the axial component separately for balance checks and diagnosis.
    r.axialContact=struct('time',r.time,'force_N',r.data(:,1));
    r.data(:,1)=r.normalContact.data(:,1);
    r.contactEstimate=struct('time',r.time, ...
        'force_N',r.normalContact.data(:,4), ...
        'source','Recorded native normal-force reconstruction');
end
if any(strcmp(who(out),'accelerometer_log'))
    logged=out.get('accelerometer_log');
    r.accelerometer=struct('time',logged.Time(:),'data',rows(logged,6), ...
        'names',{{'measuredAccelerationZ_m_s2','measuredAccelerationX_m_s2', ...
        'physicalAccelerationZ_m_s2','physicalAccelerationX_m_s2', ...
        'reconstructedAxialContact_N','reconstructedLateralContact_N'}});
    logged=out.get('inertial_motion_log');
    r.motionObserver=struct('time',logged.Time(:),'data',rows(logged,2), ...
        'names',{{'headGapDeviation_m','headGapVelocity_m_s'}});
    assert(isequal(r.time,r.accelerometer.time,r.motionObserver.time), ...
        'Acceleration or motion sample grid differs.');
end
if any(strcmp(who(out),'measured_drag_log'))
    logged=out.get('measured_drag_log');
    r.measuredDrag=struct('time',logged.Time(:),'data',rows(logged,3), ...
        'names',{{'filteredDragRatio','boundedRawDragRatio','estimateValid'}});
    assert(isequal(r.time,r.measuredDrag.time),'Drag-estimate sample grid differs.');
end
if any(strcmp(who(out),'force_variation_log'))
    logged=out.get('force_variation_log');
    r.forceVariation=struct('time',logged.Time(:),'data',rows(logged,5), ...
        'names',{{'meanError_N','rmsDeviation_N','stableSampleCount','stableNow','ready'}});
    assert(isequal(r.time,r.forceVariation.time),'Force-stability sample grid differs.');
end
if any(strcmp(who(out),'acceleration_feedforward_log'))
    logged=out.get('acceleration_feedforward_log');
    width=numel(logged.Data)/numel(logged.Time);
    names={'measuredAz_m_s2','predictedFeedforwardAz_m_s2','filteredFeedforwardAz_m_s2'};
    assert(any(width==[2,3]),'Unexpected acceleration-feedforward log width.');
    r.accelerationFeedforward=struct('time',logged.Time(:),'data',rows(logged,width), ...
        'names',{names(1:width)});
    assert(isequal(r.time,r.accelerationFeedforward.time),'Feedforward sample grid differs.');
end
if any(strcmp(who(out),'lower_inverse_log'))
    logged=out.get('lower_inverse_log');
    r.lowerInverse=struct('time',logged.Time(:),'data',rows(logged,4), ...
        'names',{{'requestedAxialForceChange_N','identifiedToolInertia_N', ...
        'desiredLowerLinkLoadChange_N','inverseCurrent_A'}});
    assert(isequal(r.time,r.lowerInverse.time),'Lower-link inverse sample grid differs.');
end
if any(strcmp(who(out),'robot_prediction_observer_log'))
    logged=out.get('robot_prediction_observer_log');
    r.observerStates=struct('time',logged.Time(:),'data',rows(logged,9));
end
if any(strcmp(who(out),'robot_prediction_log'))
    logged=out.get('robot_prediction_log');
    r.commandPrediction=struct('time',logged.Time(:),'data',rows(logged,3));
end
if any(strcmp(who(out),'robot_gate_log'))
    gate=out.get('robot_gate_log'); values=rows(gate,4);
    r.robotGate=struct('time',gate.Time(:),'ready',values(:,1), ...
        'filteredError_N',values(:,2),'forceRate_N_s',values(:,3), ...
        'stableNow',values(:,4));
end
r.metrics=acf_metrics(r);
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
