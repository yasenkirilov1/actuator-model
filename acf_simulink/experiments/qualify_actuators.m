function [report,traces] = qualify_actuators(snapshotFolder,outputFolder,study,candidateFile)
%QUALIFY_ACTUATORS Run electrical and actuator screens in an isolated snapshot.
% This does not save the SLX or use a second implementation of the physics.
% study is 'coil', 'leadscrew', 'leadscrew_refined', 'leadscrew_high_gain'
% or 'coil_final'. The high-gain/final studies can resume their own checkpoints.
if nargin<1,snapshotFolder=fullfile(getenv('HOME'),'.local','state','actuator-model','qualification-20260930');end
if nargin<2,outputFolder=fullfile(fileparts(fileparts(mfilename('fullpath'))),'validation');end
if nargin<3,study='coil';end
if nargin<4,candidateFile='';end
assert(any(strcmp(study,{'coil','leadscrew','leadscrew_refined','leadscrew_high_gain','coil_final'})),'Unknown actuator study.');
if strcmp(study,'coil_final')
    [report,traces]=final_winding(snapshotFolder,outputFolder,candidateFile);return;
end
if ~isfolder(outputFolder),mkdir(outputFolder);end
modelFile=fullfile(snapshotFolder,'acf_force_control_simple.slx');
mdl='acf_force_control_simple';
if bdIsLoaded(mdl)
    assert(strcmp(get_param(mdl,'FileName'),modelFile),'Use a separate MATLAB process for the snapshot study.');
else,load_system(modelFile);end
base=load(fullfile(snapshotFolder,'base_parameters.mat'));base=base.p;
catalog=jsondecode(fileread(fullfile(snapshotFolder,'spring_catalog.json')));
mw=get_param(mdl,'ModelWorkspace');original=getVariable(mw,'p');
oldStep=get_param(mdl,'FixedStep');oldStop=get_param(mdl,'StopTime');
oldChoice=get_param([mdl '/Actuator selection'],'ActuatorChoice');
delayBlock=[mdl '/Force measurement/Sensor sample delay'];oldDelay=get_param(delayBlock,'DelayLength');
cleanup=onCleanup(@() restore_model(mdl,mw,original,oldStep,oldStop,oldChoice,delayBlock,oldDelay)); %#ok<NASGU>
maxNumCompThreads(2);
base.sim.stopTime=4;base.sim.evaluateFrom=0;
base.surface.kind='sharp_bump';base.surface.startTime=2;
base.surface.height=.001;base.surface.width=.01;base.surface.scanSpeed=.1;
base.mechanics.mA=.4;base.mechanics.mB=.6;base.mechanics.mS=1.4;
base.force.tolerance=.5;base.sensor.noiseRms=0;base.sensor.seed=1729;
base.robot.forceGate.enabled=true;
strong=base;strong.mechanics.lowerRigid=false;strong.mechanics.upperRigid=false;
strong.springs.lower=catalog.springs(4);strong.springs.upper=catalog.springs(4);
strong.springs.commonPreload_N=32;
strong.mechanics.k1=2*catalog.springs(4).rate_N_m;strong.mechanics.k2=strong.mechanics.k1;
strong.mechanics.c1=30;strong.mechanics.c2=5;
rigid=base;rigid.mechanics.lowerRigid=true;rigid.mechanics.upperRigid=true;
rigid.mechanics.k1=0;rigid.mechanics.k2=0;rigid.mechanics.c1=0;rigid.mechanics.c2=0;
rows=struct([]);configs={};traces={};started=tic;
prefix=fullfile(outputFolder,['actuator_qualification_' study]);
standardResumedRows=0;standardPreviousSeconds=0;standardPreviousSource='';
standardCaseCounter=0;standardStatus='running';
if strcmp(study,'leadscrew_high_gain') && isfile([prefix '_checkpoint.mat'])
    try,saved=load([prefix '_checkpoint.mat'],'rows','configs','traces');
    catch
        companion=load([prefix '.mat'],'report','configs','traces');
        saved=struct('rows',companion.report.rows,'configs',{companion.configs},'traces',{companion.traces});
    end
    prior=jsondecode(fileread([prefix '.json']));
    assert(strcmp(prior.sourceModelSha256,sha256(modelFile)),'Model changed since checkpoint.');
    rows=saved.rows;configs=saved.configs;traces=saved.traces;
    standardResumedRows=numel(rows);standardPreviousSeconds=prior.elapsedSeconds;
    if isfield(prior,'previousSourceStudySha256') && ~isempty(prior.previousSourceStudySha256)
        standardPreviousSource=prior.previousSourceStudySha256;
    else,standardPreviousSource=prior.sourceStudySha256;end
    fprintf('ACTUATOR_RESUME %s %d recorded cases retained\n',study,standardResumedRows);
end
if strcmp(study,'coil')
    modes={rigid,strong};modeNames={'both_rigid','strong_compliant'};
    % Matched-controller OFAT variations: the observer/state feedback is
    % recalculated for Kf/tau, with its original design weights unchanged.
    fields={'forceConstant','resistance','inductance','voltage','peakCurrent','currentResponse'};
    values={[12 16 20 24 30],[2 4 6 8],[.001 .003 .006 .01], ...
        [12 24 36 48],[2 2.5 3 4 5],[.0002 .0004 .0008 .0015 .003]};
    for mode=1:2
        for stress=0:1
            batch_start(modes{mode},stress);
            for field=1:numel(fields)
                for value=values{field}
                    q=modes{mode};q.force.reference=50;q.mechanics.angle=pi/2;
                    q.coil.(fields{field})=value;
                    run_case(q,'coil_ofat',modeNames{mode},fields{field},value,stress,7);
                end
            end
            batch_end;
        end
        checkpoint;
    end
    % Joint cases intentionally form conditional electrical targets. They
    % are not claimed to be simultaneously available real coil windings.
    pairs=[16 4;20 4;24 4;20 6];
    for mode=1:2
        for stress=0:1
            batch_start(modes{mode},stress);
            for pair=1:size(pairs,1)
                for inductance=[.001 .006]
                    for voltage=[12 24]
                        for tau=[.0004 .0015]
                            q=modes{mode};q.force.reference=50;q.mechanics.angle=pi/2;
                            q.coil.forceConstant=pairs(pair,1);q.coil.resistance=pairs(pair,2);
                            q.coil.inductance=inductance;q.coil.voltage=voltage;q.coil.currentResponse=tau;
                            run_case(q,'coil_joint',modeNames{mode},'joint',pair,stress,7);
                        end
                    end
                end
            end
            batch_end;
        end
        checkpoint;
    end
    % All four force/orientation corners, including seed 29 confirmation.
    for mode=1:2
        for stress=0:2
            batch_start(modes{mode},double(stress>0));
            for force=[5 50]
                for angle=[0 pi/2]
                    q=modes{mode};q.force.reference=force;q.mechanics.angle=angle;
                    seeds=[1729 7 29];
                    [row,r]=run_case(q,'coil_corner',modeNames{mode},'base',0,double(stress>0),seeds(stress+1));
                    if ~isempty(r),r.label=sprintf('%s / %g N / %g deg / seed %d',modeNames{mode},force,angle*180/pi,row.seed);traces{end+1}=r;end %#ok<AGROW>
                end
            end
            batch_end;
        end
    end
elseif strcmp(study,'leadscrew')
    strong.actuator.kind='leadscrew';
    % These are PI controllers already implemented by the native model.
    % The screw velocity/current loops and physical motor parameters remain
    % unchanged while the force PI is tuned, except in labelled upgrade cases.
    batch_start(strong,0);
    for lead=[.002 .005 .010 .020]
        for kp=[.0005 .002 .005 .02 .05]
            for ki=[.002 .008 .02 .08 .2]
                q=strong;q.force.reference=50;q.mechanics.angle=pi/2;
                q.screw.lead=lead;q.screw.forceKp=kp;q.screw.forceKi=ki;
                run_case(q,'lead_pi_tuning','strong_compliant','PI',lead,0,1729);
            end
        end
    end
    batch_end;checkpoint;
    good=find(strcmp({rows.group},'lead_pi_tuning') & [rows.simulated] & [rows.completed] & ...
        ~[rows.lostContact] & ~[rows.strokeStopHit] & ~[rows.overCurrent] & [rows.springSizingPassed]);
    if isempty(good),good=find(strcmp({rows.group},'lead_pi_tuning') & [rows.simulated]);end
    [~,order]=sort([rows(good).peakError_N]);good=good(order);
    bestLead=configs{good(1)};
    % Mechanical/electrical upgrade sensitivity. Raising current alone does
    % not overcome rotor inertia or a delayed force controller.
    batch_start(bestLead,0);
    for lead=[.005 .010 .020]
        for inertia=[5e-6 2e-5 1e-4]
            for bandwidth=[150 300 600]
                q=bestLead;q.screw.lead=lead;q.screw.rotorInertia=inertia;
                q.screw.speedBandwidth=bandwidth;q.screw.peakCurrent=5;q.screw.voltage=36;
                run_case(q,'lead_upgrade','strong_compliant','lead/inertia/bandwidth',lead,0,1729);
            end
        end
    end
    batch_end;checkpoint;
    good=find(strcmp({rows.group},'lead_upgrade') & [rows.simulated] & [rows.completed] & ...
        ~[rows.lostContact] & ~[rows.strokeStopHit] & ~[rows.overCurrent] & [rows.springSizingPassed]);
    if isempty(good),good=find(strcmp({rows.group},'lead_upgrade') & [rows.simulated]);end
    [~,order]=sort([rows(good).peakError_N]);good=good(order);
    bestUpgrade=configs{good(1)};
    candidates={strong,bestLead,bestUpgrade};names={'default_PI','retuned_PI','retuned_upgraded_screw'};
    for actuator=1:3
        for stress=0:1
            batch_start(candidates{actuator},stress);
            for force=[5 50]
                for angle=[0 pi/2]
                    q=candidates{actuator};q.force.reference=force;q.mechanics.angle=angle;
                    [~,r]=run_case(q,'lead_corner',names{actuator},'base',0,stress,7);
                    if ~isempty(r),r.label=sprintf('%s / %g N / %g deg / stress %d',names{actuator},force,angle*180/pi,stress);traces{end+1}=r;end %#ok<AGROW>
                end
            end
            batch_end;
        end
        set_param(delayBlock,'DelayLength','p.d.sensorDelaySamples');
        q=candidates{actuator};q.force.reference=20;q.mechanics.angle=0;
        q.surface.kind='bump';q.surface.width=.060;q.surface.scanSpeed=.020;q.sim.stopTime=8;
        [~,r]=run_case(q,'lead_slow',names{actuator},'slow_bump',0,0,1729);
        if ~isempty(r),r.label=[names{actuator} ' / slow 20 N bump'];traces{end+1}=r;end %#ok<AGROW>
    end
    % The ideal position source is a non-purchasable reference, not an
    % unlimited-force physical actuator comparison. Keep its configured
    % speed, acceleration and position-servo lag limitations visible.
    q=strong;q.actuator.kind='ideal_position';
    batch_start(q,0);
    keq=q.mechanics.k1*q.mechanics.k2/(q.mechanics.k1+q.mechanics.k2);
    for kp=[.1 .3 1]
        for ki=[30 120 300]
            a=q;a.force.reference=50;a.mechanics.angle=pi/2;
            a.ideal.forceKp=kp/keq;a.ideal.forceKi=ki/keq;
            run_case(a,'ideal_tuning','strong_compliant','ideal PI',kp,0,1729);
        end
    end
    good=find(strcmp({rows.group},'ideal_tuning') & [rows.simulated] & [rows.completed] & ~[rows.lostContact] & ~[rows.strokeStopHit]);
    if isempty(good),good=find(strcmp({rows.group},'ideal_tuning') & [rows.simulated]);end
    [~,order]=sort([rows(good).peakError_N]);q=configs{good(order(1))};
    for force=[5 50]
        for angle=[0 pi/2]
            q.force.reference=force;q.mechanics.angle=angle;
            [~,r]=run_case(q,'ideal_reference','strong_compliant','ideal',0,0,1729);
            if ~isempty(r),r.label=sprintf('Ideal position reference / %g N / %g deg',force,angle*180/pi);traces{end+1}=r;end %#ok<AGROW>
        end
    end
    batch_end;
elseif strcmp(study,'leadscrew_refined')
    strong.actuator.kind='leadscrew';
    drives={strong,strong,strong};
    drives{1}.screw.lead=.005;drives{1}.screw.rotorInertia=2e-5;drives{1}.screw.speedBandwidth=300;
    drives{2}.screw.lead=.010;drives{2}.screw.rotorInertia=5e-6;drives{2}.screw.speedBandwidth=600;
    drives{3}.screw.lead=.020;drives{3}.screw.rotorInertia=5e-6;drives{3}.screw.speedBandwidth=600;
    for drive=1:numel(drives)
        drives{drive}.screw.peakCurrent=5;drives{drive}.screw.voltage=36;
        q=drives{drive};q.force.reference=50;q.mechanics.angle=pi/2;
        batch_start(q,0);
        name=sprintf('upgraded_drive_%d',drive);
        for kp=[.005 .01 .02 .03 .05 .08]
            for ki=[.005 .02 .05 .1 .2 .5]
                a=q;a.screw.forceKp=kp;a.screw.forceKi=ki;
                run_case(a,'lead_refined_tuning',name,'PI',drive,0,1729);
            end
        end
        batch_end;checkpoint;
    end
    candidates={};candidateNames={};
    for drive=1:numel(drives)
        name=sprintf('upgraded_drive_%d',drive);
        good=find(strcmp({rows.group},'lead_refined_tuning') & strcmp({rows.mechanics},name) & ...
            [rows.simulated] & [rows.completed] & ~[rows.lostContact] & ~[rows.strokeStopHit] & ~[rows.overCurrent] & [rows.springSizingPassed]);
        if isempty(good),good=find(strcmp({rows.group},'lead_refined_tuning') & strcmp({rows.mechanics},name) & [rows.simulated]);end
        [~,order]=sort([rows(good).peakError_N]);good=good(order);
        for rank=1:min(3,numel(good))
            candidates{end+1}=configs{good(rank)};candidateNames{end+1}=sprintf('drive_%d_PI_rank_%d',drive,rank); %#ok<AGROW>
        end
    end
    for candidate=1:numel(candidates)
        for stress=0:2
            batch_start(candidates{candidate},double(stress>0));
            seeds=[1729 7 29];
            for force=[5 50]
                for angle=[0 pi/2]
                    q=candidates{candidate};q.force.reference=force;q.mechanics.angle=angle;
                    run_case(q,'lead_refined_corner',candidateNames{candidate},'base',candidate,double(stress>0),seeds(stress+1));
                end
            end
            batch_end;
        end
        checkpoint;
    end
    score=inf(1,numel(candidates));
    for candidate=1:numel(candidates)
        ix=find(strcmp({rows.group},'lead_refined_corner') & strcmp({rows.mechanics},candidateNames{candidate}));
        if all([rows(ix).simulated]) && all([rows(ix).completed]) && all([rows(ix).springSizingPassed]) && ...
                ~any([rows(ix).lostContact] | [rows(ix).strokeStopHit] | [rows(ix).overCurrent])
            score(candidate)=max([rows(ix).peakError_N]);
        end
    end
    [bestScore,best]=min(score);assert(isfinite(bestScore),'No upgraded screw candidate stayed mechanically valid.');
    selected=candidates{best};selectedName=candidateNames{best};
    for stress=0:2
        batch_start(selected,double(stress>0));seeds=[1729 7 29];
        for force=[5 50]
            for angle=[0 pi/2]
                q=selected;q.force.reference=force;q.mechanics.angle=angle;
                [~,r]=run_case(q,'lead_refined_selected',selectedName,'selected',best,double(stress>0),seeds(stress+1));
                if ~isempty(r),r.label=sprintf('Refined screw / %g N / %g deg / seed%d',force,angle*180/pi,seeds(stress+1));traces{end+1}=r;end %#ok<AGROW>
            end
        end
        batch_end;
    end
    set_param(delayBlock,'DelayLength','p.d.sensorDelaySamples');
    q=selected;q.force.reference=20;q.mechanics.angle=0;
    q.surface.kind='bump';q.surface.width=.060;q.surface.scanSpeed=.020;q.sim.stopTime=8;
    [~,r]=run_case(q,'lead_refined_slow',selectedName,'slow_bump',0,0,1729);
    if ~isempty(r),r.label='Refined screw / slow 20 N bump';traces{end+1}=r;end %#ok<AGROW>
    % Broader ideal-position controller/servo envelope. This reference still
    % has its stated velocity and acceleration caps and no force preview.
    q=strong;q.actuator.kind='ideal_position';q.force.reference=50;q.mechanics.angle=pi/2;
    keq=q.mechanics.k1*q.mechanics.k2/(q.mechanics.k1+q.mechanics.k2);
    batch_start(q,0);
    for positionGain=[100 500 1000]
        for velocityTime=[.005 .001 .0004]
            for kp=[.3 1 3 10]
                for ki=[120 300 1000]
                    a=q;a.ideal.positionGain=positionGain;a.ideal.velocityTime=velocityTime;
                    a.ideal.forceKp=kp/keq;a.ideal.forceKi=ki/keq;
                    run_case(a,'ideal_refined_tuning','strong_compliant','ideal PI/servo',positionGain,0,1729);
                end
            end
        end
    end
    batch_end;checkpoint;
    good=find(strcmp({rows.group},'ideal_refined_tuning') & [rows.simulated] & [rows.completed] & ...
        ~[rows.lostContact] & ~[rows.strokeStopHit] & [rows.springSizingPassed]);
    if isempty(good),good=find(strcmp({rows.group},'ideal_refined_tuning') & [rows.simulated]);end
    [~,order]=sort([rows(good).peakError_N]);q=configs{good(order(1))};
    for stress=0:2
        batch_start(q,double(stress>0));seeds=[1729 7 29];
        for force=[5 50]
            for angle=[0 pi/2]
                q.force.reference=force;q.mechanics.angle=angle;
                [~,r]=run_case(q,'ideal_refined_corner','strong_compliant','selected ideal',0,double(stress>0),seeds(stress+1));
                if ~isempty(r),r.label=sprintf('Fast ideal-position reference / %g N / %g deg / seed%d',force,angle*180/pi,seeds(stress+1));traces{end+1}=r;end %#ok<AGROW>
            end
        end
        batch_end;
    end
else
    strong.actuator.kind='leadscrew';strong.screw.rotorInertia=5e-6;
    strong.screw.speedBandwidth=600;strong.screw.peakCurrent=5;strong.screw.voltage=36;
    candidates={};nominal=struct([]);scores=[];
    for stress=0:1
        batch_start(strong,stress);number=0;
        for lead=[.010 .020]
            for filter=[.0003 .0006 .001 .002]
                for kp=[.08 .12 .2 .32]
                    for ki=[.05 .5 2]
                        number=number+1;q=strong;q.force.reference=50;q.mechanics.angle=pi/2;
                        q.screw.lead=lead;q.screw.forceKp=kp;q.screw.forceKi=ki;q.sensor.filterTime=filter;
                        [row,~]=run_case(q,'lead_high_gain_screen',sprintf('%gmm_low_inertia',lead*1000),'PI/filter',number,stress,7);
                        if stress==0
                            candidates{number}=q;if isempty(nominal),nominal=row;else,nominal(number)=row;end
                            scores(number)=inf; %#ok<AGROW>
                        elseif row.simulated && row.completed && row.springSizingPassed && ~row.lostContact && ~row.strokeStopHit && ~row.overCurrent && ...
                                nominal(number).simulated && nominal(number).completed && nominal(number).springSizingPassed && ...
                                ~nominal(number).lostContact && ~nominal(number).strokeStopHit && ~nominal(number).overCurrent
                            scores(number)=max(nominal(number).peakError_N,row.peakError_N);
                        end
                    end
                end
            end
        end
        batch_end;checkpoint;
    end
    valid=find(isfinite(scores));assert(~isempty(valid),'No physically valid high-gain screw candidate.');
    [~,order]=sort(scores(valid));valid=valid(order);valid=valid(1:min(3,numel(valid)));
    worst=inf(1,numel(valid));
    for rank=1:numel(valid)
        number=valid(rank);q=candidates{number};name=sprintf('high_gain_candidate_%d',number);
        for stress=0:2
            batch_start(q,double(stress>0));seeds=[1729 7 29];
            for force=[5 50]
                for angle=[0 pi/2]
                    q.force.reference=force;q.mechanics.angle=angle;
                    run_case(q,'lead_high_gain_corner',name,'candidate',number,double(stress>0),seeds(stress+1));
                end
            end
            batch_end;
        end
        ix=find(strcmp({rows.group},'lead_high_gain_corner') & strcmp({rows.mechanics},name));
        if all([rows(ix).simulated]) && all([rows(ix).completed]) && all([rows(ix).springSizingPassed]) && ...
                ~any([rows(ix).lostContact] | [rows(ix).strokeStopHit] | [rows(ix).overCurrent])
            worst(rank)=max([rows(ix).peakError_N]);
        end
        checkpoint;
    end
    [score,rank]=min(worst);assert(isfinite(score),'High-gain corner candidates were all invalid.');
    number=valid(rank);q=candidates{number};
    for stress=0:2
        batch_start(q,double(stress>0));seeds=[1729 7 29];
        for force=[5 50]
            for angle=[0 pi/2]
                q.force.reference=force;q.mechanics.angle=angle;
                [~,r]=run_case(q,'lead_high_gain_selected',sprintf('candidate_%d',number),'selected',number,double(stress>0),seeds(stress+1));
                if ~isempty(r),r.label=sprintf('Higher-gain filtered screw / %g N / %g deg / seed%d',force,angle*180/pi,seeds(stress+1));traces{end+1}=r;end %#ok<AGROW>
            end
        end
        batch_end;
    end
    set_param(delayBlock,'DelayLength','p.d.sensorDelaySamples');q.force.reference=20;q.mechanics.angle=0;
    q.surface.kind='bump';q.surface.width=.060;q.surface.scanSpeed=.020;q.sim.stopTime=8;
    [~,r]=run_case(q,'lead_high_gain_slow',sprintf('candidate_%d',number),'slow_bump',number,0,1729);
    if ~isempty(r),r.label='Higher-gain filtered screw / slow bump';traces{end+1}=r;end %#ok<AGROW>
end
standardStatus='complete';checkpoint;
fprintf('ACTUATOR_STUDY_COMPLETE %s: %d rows, %d simulations, %.1f seconds\n',study,numel(rows),sum([rows.simulated]),standardPreviousSeconds+toc(started));

    function batch_start(q,extraDelay)
        set_param(mdl,'FastRestart','off');
        set_param(delayBlock,'DelayLength',sprintf('p.d.sensorDelaySamples+%d',extraDelay));
        q=acf_prepare(q);assignin(mw,'p',q);acf_select_actuator(mdl,q.actuator.kind);
        set_param(mdl,'FixedStep',num2str(q.sim.step,17),'StopTime',num2str(q.sim.stopTime,17));
        set_param(mdl,'FastRestart','on');
    end
    function batch_end
        set_param(mdl,'FastRestart','off');
        Simulink.sdi.clear;
    end
    function [row,r]=run_case(q,group,mechanics,varied,value,stress,seed)
        r=[];q.sensor.seed=seed;q.sensor.noiseRms=.02*stress;
        standardCaseCounter=standardCaseCounter+1;
        if standardCaseCounter<=standardResumedRows
            row=rows(standardCaseCounter);
            assert(strcmp(row.group,group) && strcmp(row.mechanics,mechanics) && row.seed==seed && row.stress==logical(stress),'Checkpoint case sequence changed.');
            assert(row.force_N==q.force.reference && abs(row.angle_deg-q.mechanics.angle*180/pi)<1e-12 && row.lead_mm==q.screw.lead*1000 && ...
                row.forceKp==q.screw.forceKp && row.forceKi==q.screw.forceKi && row.forceFilter_ms==q.sensor.filterTime*1000,'Checkpoint parameters changed.');
            return;
        end
        row=struct('id',numel(rows)+1,'group',group,'mechanics',mechanics,'actuator',q.actuator.kind, ...
            'variedParameter',varied,'variedValue',value,'force_N',q.force.reference,'angle_deg',q.mechanics.angle*180/pi, ...
            'stress',logical(stress),'seed',seed,'extraForceDelay_ms',.2*stress, ...
            'forceFilter_ms',q.sensor.filterTime*1000, ...
            'Kf_N_A',q.coil.forceConstant,'R_ohm',q.coil.resistance,'L_mH',q.coil.inductance*1000, ...
            'Vlimit_V',q.coil.voltage,'Ilimit_A',q.coil.peakCurrent,'currentTau_ms',q.coil.currentResponse*1000, ...
            'lead_mm',q.screw.lead*1000,'rotorInertia_kg_m2',q.screw.rotorInertia, ...
            'speedBandwidth_rad_s',q.screw.speedBandwidth,'screwIlimit_A',q.screw.peakCurrent,'screwVlimit_V',q.screw.voltage, ...
            'forceKp',q.screw.forceKp,'forceKi',q.screw.forceKi, ...
            'idealForceKp',q.ideal.forceKp,'idealForceKi',q.ideal.forceKi, ...
            'idealPositionGain_s_1',q.ideal.positionGain,'idealVelocityTime_ms',q.ideal.velocityTime*1000, ...
            'simulated',false,'passed',false,'dynamicsPassed',false,'springSizingPassed',false, ...
            'peakError_N',NaN,'rmsError_N',NaN,'peakCurrent_A',NaN,'rmsCurrent_A',NaN,'peakVoltage_V',NaN, ...
            'peakAxialForce_N',NaN,'peakGapSpeed_m_s',NaN,'peakGapAcceleration_m_s2',NaN, ...
            'effortSaturationFraction',NaN,'voltageSaturationFraction',NaN,'completed',false, ...
            'lostContact',false,'strokeStopHit',false,'overCurrent',false,'holdCurrent_A',NaN,'holdCopperLoss_W',NaN, ...
            'rmsCopperLoss_W',NaN,'failurePhase','','reason','');
        if strcmp(q.actuator.kind,'voice_coil')
            row.holdCurrent_A=(q.force.reference-(q.mechanics.mB+q.mechanics.mS)*q.mechanics.gravity*cos(q.mechanics.angle))/q.coil.forceConstant;
            row.holdCopperLoss_W=row.holdCurrent_A^2*q.coil.resistance;
        elseif strcmp(q.actuator.kind,'leadscrew')
            row.holdCurrent_A=(q.screw.lead/(2*pi))*(q.force.reference-(q.mechanics.mB+q.mechanics.mS)*q.mechanics.gravity*cos(q.mechanics.angle))/q.screw.torqueConstant;
            row.holdCopperLoss_W=row.holdCurrent_A^2*q.screw.resistance;
        end
        try
            q=acf_prepare(q);
        catch e
            row.failurePhase='preparation';row.reason=e.message;
            append(row,q);return;
        end
        if strcmp(q.actuator.kind,'voice_coil')
            row.holdCurrent_A=q.d.coilHoldCurrent;row.holdCopperLoss_W=row.holdCurrent_A^2*q.coil.resistance;
        elseif strcmp(q.actuator.kind,'leadscrew')
            row.holdCurrent_A=q.d.screwHoldCurrent;row.holdCopperLoss_W=row.holdCurrent_A^2*q.screw.resistance;
        end
        try
            quiet=evalc('r=acf_run(mdl,q,false);'); %#ok<NASGU>
            m=r.metrics;row.simulated=true;row.passed=m.passed;
            row.dynamicsPassed=m.completed && m.peakAbsForceError_N<=q.force.tolerance && ~m.lostContact && ~m.strokeStopHit && ~m.overCurrent;
            row.springSizingPassed=m.springSizing.passed;
            row.peakError_N=m.peakAbsForceError_N;row.rmsError_N=m.rmsForceError_N;
            row.peakCurrent_A=m.peakCurrent_A;row.rmsCurrent_A=m.rmsCurrent_A;row.peakVoltage_V=m.peakVoltage_V;
            row.peakAxialForce_N=m.peakAxialForce_N;row.peakGapSpeed_m_s=m.peakGapSpeed_m_s;
            row.peakGapAcceleration_m_s2=m.peakGapAcceleration_m_s2;
            row.effortSaturationFraction=m.effortSaturationFraction;row.voltageSaturationFraction=m.voltageSaturationFraction;
            row.completed=m.completed;row.lostContact=m.lostContact;row.strokeStopHit=m.strokeStopHit;row.overCurrent=m.overCurrent;row.reason=m.reason;
            if strcmp(q.actuator.kind,'voice_coil'),row.rmsCopperLoss_W=m.rmsCurrent_A^2*q.coil.resistance;
            elseif strcmp(q.actuator.kind,'leadscrew'),row.rmsCopperLoss_W=m.rmsCurrent_A^2*q.screw.resistance;end
        catch e
            row.failurePhase='simulation';row.reason=e.message;
        end
        append(row,q);
    end
    function append(row,q)
        if isempty(rows),rows=row;else,rows(end+1)=row;end %#ok<AGROW>
        configs{end+1}=q; %#ok<AGROW>
        if mod(numel(rows),10)==0
            fprintf('ACTUATOR_PROGRESS %s %d rows/%d simulations, %.1f s, last error %.5g N %s\n',study,numel(rows),sum([rows.simulated]),toc(started),row.peakError_N,row.reason);
        end
        if mod(numel(rows),30)==0,checkpoint;end
    end
    function checkpoint
        % Save actual results before metadata generation can fail.
        save([prefix '_checkpoint.pending.mat'],'rows','configs','traces','-v7.3');
        movefile([prefix '_checkpoint.pending.mat'],[prefix '_checkpoint.mat'],'f');
        report=struct('study',study,'status',standardStatus,'date','2026-09-30','matlabVersion',version,'sourceModel',modelFile, ...
            'sourceModelSha256',sha256(modelFile),'sourcePrepareSha256',sha256(fullfile(snapshotFolder,'helpers','acf_prepare.m')), ...
            'sourceStudySha256',sha256(mfilename('fullpath')), ...
            'previousSourceStudySha256',standardPreviousSource,'resumedRows',standardResumedRows, ...
            'rows',rows,'simulations',sum([rows.simulated]),'preparationRejections',sum(strcmp({rows.failurePhase},'preparation')), ...
            'simulationErrors',sum(strcmp({rows.failurePhase},'simulation')),'passedRows',sum([rows.passed]),'elapsedSeconds',standardPreviousSeconds+toc(started), ...
            'scope','1 mm high, 10 mm wide bump at 100 mm/s, B+S=2 kg, A=0.4 kg, force/orientation corners and labelled electrical variations.', ...
            'stressDefinition','0.02 N RMS force noise, stated seed, plus an unmodelled extra 0.2 ms force delay; stroke noise 1 micrometer RMS with configured 0.2 ms delay.', ...
            'controllerPolicy','Native controller architecture. Voice-coil design weights fixed to the snapshot; matrices are recalculated for each matched mechanical/Kf/current-response configuration. Leadscrew outer PI is tuned in the stated grid.', ...
            'electricalCaveat','Independent Kf/R/L changes are conditional specifications, not guaranteed combinations in a real winding. Rms copper loss excludes iron/friction/electronics losses, cooling and temperature dependence; thermal qualification is absent.', ...
            'actuatorCaveat','Leadscrew results only test the implemented PI/current/speed architecture and labelled motor parameters. They cannot prove that every screw actuator or advanced screw controller fails. Pneumatic, hydraulic and other linear servo dynamics are not simulated. Ideal-position reference is not a physical product.');
        fid=fopen([prefix '.json.pending'],'w');assert(fid>=0);fprintf(fid,'%s',jsonencode(report,PrettyPrint=true));fclose(fid);
        movefile([prefix '.json.pending'],[prefix '.json'],'f');
        writetable(struct2table(rows),[prefix '.csv.pending'],'FileType','text');
        movefile([prefix '.csv.pending'],[prefix '.csv'],'f');
        save([prefix '.pending.mat'],'report','configs','traces','-v7.3');
        movefile([prefix '.pending.mat'],[prefix '.mat'],'f');
    end
end

function restore_model(mdl,mw,p,step,stop,choice,delayBlock,delay)
set_param(mdl,'FastRestart','off');set_param(delayBlock,'DelayLength',delay);
assignin(mw,'p',p);acf_select_actuator(mdl,choice);
set_param(mdl,'FixedStep',step,'StopTime',stop);
end

function h=sha256(filename)
if ~isfile(filename) && isfile([filename '.m']),filename=[filename '.m'];end
% MATLAB in this workspace has no Java runtime. Quote filenames for the
% system checksum utility; the bytes themselves never pass through a shell.
quote=char(39);quoted=[quote,strrep(filename,quote,[quote,char(34),quote,char(34),quote]),quote];
[status,value]=system(['sha256sum -- ' quoted]);assert(status==0,'Could not hash source.');
h=regexp(value,'^[0-9a-f]{64}','match','once');assert(~isempty(h),'Invalid source checksum.');
end

function [report,traces]=final_winding(snapshotFolder,outputFolder,candidateFile)
% Joint winding tolerances retain the selected force-controller coefficients.
assert(isfile(candidateFile),'Pass the selected upper-rigid parameter MAT.');
candidate=load(candidateFile,'p');base=candidate.p;
assert(base.mechanics.upperRigid && ~base.mechanics.lowerRigid,'Expected upper-rigid candidate.');
base.sim.stopTime=4;base.sim.evaluateFrom=0;base.actuator.kind='voice_coil';
base.sensor.noiseRms=.02;base.surface.kind='sharp_bump';base.surface.height=.001;
base.surface.width=.010;base.surface.scanSpeed=.1;base.surface.startTime=2;
base.windingActual=struct('forceFactor',1,'resistanceFactor',1,'inductanceFactor',1,'currentTauFactor',1);
modelFile=fullfile(snapshotFolder,'acf_force_control_simple.slx');mdl='acf_force_control_simple';
if ~bdIsLoaded(mdl),load_system(modelFile);end
assert(strcmp(get_param(mdl,'FileName'),modelFile),'Use the immutable model in a private MATLAB.');
mw=get_param(mdl,'ModelWorkspace');original=getVariable(mw,'p');
set_param(mdl,'FastRestart','off');
drive=[mdl '/Actuator and RABS mechanics/Actuator/Voice coil'];
paths=cellfun(@(n)[drive '/' n],{'Coil force constant','Back EMF','Resistance','Inverse inductance','Current regulator'},UniformOutput=false);
oldGains=cellfun(@(n)get_param(n,'Gain'),paths,UniformOutput=false);
robotFrom=[mdl '/Receive actuator_travel_m 6'];oldRobotTag=get_param(robotFrom,'GotoTag');
delayBlock=[mdl '/Force measurement/Sensor sample delay'];oldDelay=get_param(delayBlock,'DelayLength');
cleanup=onCleanup(@()restore_winding_model(mdl,mw,original,paths,oldGains,robotFrom,oldRobotTag,delayBlock,oldDelay)); %#ok<NASGU>
gains={'p.coil.forceConstant*p.windingActual.forceFactor', ...
    'p.coil.forceConstant*p.windingActual.forceFactor', ...
    'p.coil.resistance*p.windingActual.resistanceFactor', ...
    '1/(p.coil.inductance*p.windingActual.inductanceFactor)', ...
    '(p.coil.inductance*p.windingActual.inductanceFactor)/(p.coil.currentResponse*p.windingActual.currentTauFactor)'};
for j=1:numel(paths),set_param(paths{j},'Gain',gains{j});end
set_param(robotFrom,'GotoTag','measured_magnet_position');
set_param(delayBlock,'DelayLength','p.d.sensorDelaySamples+1');
base=acf_prepare(base);assignin(mw,'p',base);acf_select_actuator(mdl,'voice_coil');
set_param(mdl,'FixedStep',num2str(base.sim.step,17),'StopTime','4');
rows=struct([]);traces={};started=tic;status='running';
prefix=fullfile(outputFolder,'actuator_qualification_final_winding');
previousElapsed=0;resumedRows=0;previousSourceStudySha256='';
if isfile([prefix '_checkpoint.mat'])
    try
        saved=load([prefix '_checkpoint.mat'],'report','traces');
    catch
        try,saved=load([prefix '.mat'],'report','traces');
        catch,saved=struct('report',jsondecode(fileread([prefix '.json'])),'traces',{{}});end
    end
    assert(strcmp(saved.report.candidateSha256,sha256(candidateFile)),'Candidate changed since checkpoint.');
    assert(strcmp(saved.report.sourceModelSha256,sha256(modelFile)),'Model changed since checkpoint.');
    rows=saved.report.rows(:).';traces=saved.traces;resumedRows=numel(rows);
    previousElapsed=saved.report.elapsedSeconds;
    if isfield(saved.report,'previousSourceStudySha256') && ~isempty(saved.report.previousSourceStudySha256)
        previousSourceStudySha256=saved.report.previousSourceStudySha256;
    else,previousSourceStudySha256=saved.report.sourceStudySha256;end
    fprintf('FINAL_WINDING_RESUME %d recorded rows retained\n',resumedRows);
end
caseCounter=0;
% Send the proposed 3.5 A/24 V drive through every joint condition first.
limits=[3.5 24;3 24;4 24;3 18;3.5 18;4 18;3 30;3.5 30;4 30];
factors=[1 1 1 1;.9 1 1 1;.9 1.3 1.3 1.25];
labels={'nominal_winding','unknown_Kf_minus10','joint_Kf_minus10_R_L_plus30_tau_plus25'};
for limit=1:size(limits,1)
    for scenario=1:size(factors,1)
        set_param(mdl,'FastRestart','off');set_param(mdl,'FastRestart','on');
        for seed=[7 29 1729 31415]
            for force=[5 50]
                for angle=[0 pi/2]
                    caseCounter=caseCounter+1;
                    if caseCounter<=resumedRows,continue;end
                    q=base;q.coil.peakCurrent=limits(limit,1);q.coil.voltage=limits(limit,2);
                    q.force.reference=force;q.mechanics.angle=angle;q.sensor.seed=seed;
                    q.windingActual=struct('forceFactor',factors(scenario,1),'resistanceFactor',factors(scenario,2), ...
                        'inductanceFactor',factors(scenario,3),'currentTauFactor',factors(scenario,4));
                    row=struct('id',numel(rows)+1,'scenario',labels{scenario},'Ilimit_A',q.coil.peakCurrent, ...
                        'Vlimit_V',q.coil.voltage,'force_N',force,'angle_deg',angle*180/pi,'seed',seed, ...
                        'forceNoiseRms_N',q.sensor.noiseRms,'strokeNoiseRms_m',q.sensor.strokeNoiseRms,'extraForceDelay_ms',.2, ...
                        'nominalKf_N_A',q.coil.forceConstant,'actualKf_N_A',q.coil.forceConstant*factors(scenario,1), ...
                        'actualR_ohm',q.coil.resistance*factors(scenario,2),'actualL_mH',q.coil.inductance*factors(scenario,3)*1000, ...
                        'nominalCurrentTau_ms',q.coil.currentResponse*1000,'actualCurrentTau_ms',q.coil.currentResponse*factors(scenario,4)*1000, ...
                        'simulated',false,'passed',false,'peakError_N',NaN,'peakCurrent_A',NaN,'currentReserve_A',NaN, ...
                        'rmsCurrent_A',NaN,'peakVoltage_V',NaN,'voltageReserve_V',NaN,'peakRequestedVoltage_V',NaN, ...
                        'effortSaturationFraction',NaN,'voltageSaturationFraction',NaN,'completed',false, ...
                        'springSizingPassed',false,'lostContact',false,'strokeStopHit',false,'overCurrent',false, ...
                        'holdCurrent_A',NaN,'holdCopperLoss_W',NaN,'rmsCopperLoss_W',NaN,'reason','');
                    try
                        quiet=evalc('r=acf_run(mdl,q,false);'); %#ok<NASGU>
                        m=r.metrics;row.simulated=true;row.passed=m.passed;row.peakError_N=m.peakAbsForceError_N;
                        row.peakCurrent_A=m.peakCurrent_A;row.currentReserve_A=q.coil.peakCurrent-m.peakCurrent_A;
                        row.rmsCurrent_A=m.rmsCurrent_A;row.peakVoltage_V=m.peakVoltage_V;row.voltageReserve_V=q.coil.voltage-m.peakVoltage_V;
                        row.completed=m.completed;row.springSizingPassed=m.springSizing.passed;row.lostContact=m.lostContact;
                        row.strokeStopHit=m.strokeStopHit;row.overCurrent=m.overCurrent;row.reason=m.reason;
                        row.effortSaturationFraction=m.effortSaturationFraction;row.voltageSaturationFraction=m.voltageSaturationFraction;
                        row.holdCurrent_A=(force-(q.mechanics.mB+q.mechanics.mS)*q.mechanics.gravity*cos(angle))/row.actualKf_N_A;
                        row.holdCopperLoss_W=row.holdCurrent_A^2*row.actualR_ohm;
                        row.rmsCopperLoss_W=m.rmsCurrent_A^2*row.actualR_ohm;
                        d=r.data;raw=(row.actualL_mH/1000)/(row.actualCurrentTau_ms/1000)* ...
                            (min(max(d(:,33),-q.coil.peakCurrent),q.coil.peakCurrent)-d(:,14))+ ...
                            row.actualR_ohm*d(:,14)+row.actualKf_N_A*d(:,3);
                        row.peakRequestedVoltage_V=max(abs(raw));
                        assert(max(abs(min(max(raw,-q.coil.voltage),q.coil.voltage)-d(:,15)))<1e-8,'Voltage reconstruction mismatch.');
                        if limit==1 && (seed==7 || (force==50 && angle>0))
                            r.label=sprintf('Final upper-rigid / %s / %g N / %g deg / seed%d',labels{scenario},force,angle*180/pi,seed);
                            traces{end+1}=r; %#ok<AGROW>
                        end
                    catch e,row.passed=false;row.reason=e.message;end
                    if isempty(rows),rows=row;else,rows(end+1)=row;end %#ok<AGROW>
                    if mod(numel(rows),8)==0,checkpoint;end
                end
            end
        end
        set_param(mdl,'FastRestart','off');Simulink.sdi.clear;checkpoint;
        fprintf('FINAL_WINDING_LIMIT I%.1f V%g %s: worst%.6gN, minIreserve%.6gA, pass%d/16\n', ...
            limits(limit,1),limits(limit,2),labels{scenario},max([rows(end-15:end).peakError_N]), ...
            min([rows(end-15:end).currentReserve_A]),sum([rows(end-15:end).passed]));
    end
end
status='complete';checkpoint;
fprintf('FINAL_WINDING_COMPLETE %d simulations, %.1f seconds\n',sum([rows.simulated]),previousElapsed+toc(started));

    function checkpoint
        summary=struct([]);
        for n=1:size(limits,1)
            ix=find([rows.Ilimit_A]==limits(n,1) & [rows.Vlimit_V]==limits(n,2));if isempty(ix),continue;end
            one=struct('Ilimit_A',limits(n,1),'Vlimit_V',limits(n,2),'rows',numel(ix),'completedSet',numel(ix)==48, ...
                'passed',numel(ix)==48 && all([rows(ix).passed]),'worstError_N',max([rows(ix).peakError_N]), ...
                'forceMargin_N',.5-max([rows(ix).peakError_N]),'minimumCurrentReserve_A',min([rows(ix).currentReserve_A]), ...
                'minimumVoltageReserve_V',min([rows(ix).voltageReserve_V]),'maximumRequestedVoltage_V',max([rows(ix).peakRequestedVoltage_V]));
            if isempty(summary),summary=one;else,summary(end+1)=one;end %#ok<AGROW>
        end
        report=struct('date','2026-09-30','status',status,'matlabVersion',version,'sourceModel',modelFile, ...
            'sourceModelSha256',sha256(modelFile),'sourceStudySha256',sha256(mfilename('fullpath')), ...
            'previousSourceStudySha256',previousSourceStudySha256,'resumedRows',resumedRows, ...
            'candidateFile',candidateFile,'candidateSha256',sha256(candidateFile),'parameters',base, ...
            'rows',rows,'summary',summary,'simulations',sum([rows.simulated]),'passedRows',sum([rows.passed]), ...
            'elapsedSeconds',previousElapsed+toc(started),'robotTravelMeasurement','Noisy and delayed modeled magnet measurement supplied to fast control and slow recentering.', ...
            'scope','Selected upper-rigid candidate, four 5/50 N and 0/90 deg corners, four noise seeds, extra unknown 0.2 ms force delay, direct contact force measurement.', ...
            'tolerancePolicy','Force-controller coefficients retain nominal Kf and current response. Native winding/force gains implement actual factors simultaneously; current/voltage limits are known controller settings.', ...
            'currentDriverCalibration','The inner current drive is calibrated to actual winding resistance, inductance and back EMF. Its actual response is 25 percent slower in the joint condition, unmodeled by the force observer. No uncalibrated current-sensor bias or current-drive DC regulation error is modeled.', ...
            'initialCurrentPolicy','Native nominal starting current retained; no hidden re-equilibration to the actual Kf.', ...
            'caveat','The joint tests are actual simultaneous tolerance simulations, but are a finite corner/seed set and a calibrated-driver assumption, not a hardware thermal or continuous-domain proof. Copper-loss values require a separately adequate continuous thermal design.');
        save([prefix '_checkpoint.pending.mat'],'report','traces','-v7.3');
        movefile([prefix '_checkpoint.pending.mat'],[prefix '_checkpoint.mat'],'f');
        fid=fopen([prefix '.json.pending'],'w');assert(fid>=0);fprintf(fid,'%s',jsonencode(report,PrettyPrint=true));fclose(fid);
        movefile([prefix '.json.pending'],[prefix '.json'],'f');
        writetable(struct2table(rows),[prefix '.csv.pending'],'FileType','text');
        movefile([prefix '.csv.pending'],[prefix '.csv'],'f');
        save([prefix '.pending.mat'],'report','traces','-v7.3');
        movefile([prefix '.pending.mat'],[prefix '.mat'],'f');
    end
end
function restore_winding_model(mdl,mw,original,paths,oldGains,robotFrom,oldRobotTag,delayBlock,oldDelay)
set_param(mdl,'FastRestart','off');
for k=1:numel(paths),set_param(paths{k},'Gain',oldGains{k});end
set_param(robotFrom,'GotoTag',oldRobotTag);set_param(delayBlock,'DelayLength',oldDelay);
assignin(mw,'p',original);acf_select_actuator(mdl,original.actuator.kind);
end
