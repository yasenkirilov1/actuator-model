function x = acf_initial(role,p)
% State definitions are deliberately ordinary MATLAB vectors.
switch role
    case 'plant'
        % [z vz u w s vs i speedPI contact impacts lastImpulse stopHit lastCommand]
        x = zeros(13,1);
        x(1)=p.d.z0; x(3)=p.d.u0; x(9)=1;
        if strcmp(p.actuator.kind,'voice_coil'), x(7)=p.d.coilHoldCurrent;
        elseif strcmp(p.actuator.kind,'leadscrew'), x(7)=p.d.screwHoldCurrent;
        end
        x(13)=p.d.forceFeedforward;
    case 'force'
        x=[0;p.d.forceFeedforward]; % integral contribution, held command
    case 'sensor'
        % filter state, held output, PRNG seed, sample delay queue
        x=[p.force.reference;p.force.reference+p.sensor.bias;p.sensor.seed; ...
            repmat(p.force.reference+p.sensor.bias,p.d.sensorDelaySamples,1)];
    case 'recenter'
        x=p.robot.nominalVelocity;
    case 'delay'
        x=repmat(p.robot.nominalVelocity,1+p.d.robotDelaySamples,1);
    case 'robot'
        x=[p.d.r0;0]; % actual boundary position, actual velocity
    otherwise
        x=0;
end
end
