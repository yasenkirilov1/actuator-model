function [summary,details] = acf_sweep(cases,path1,values1,path2,values2)
%ACF_SWEEP Test a rectangular parameter grid across specified scenarios.
% Returns every case and configurations passing ALL cases. No global minimum
% or optimal controller claim is made. The native model is reused; preparation
% updates known-mass, sensing and inverse coefficients for each parameter set.
%
% p=acf_parameters('voice_coil','sharp_bump');
% [T,D]=acf_sweep({p},'inertial.forceKp',[0.15 0.3 0.45], ...
%     'inertial.forceKi',[10 20 30]);
% T(T.allPassed,:)  % candidates passing this particular test envelope
%
% Uses the saved Simulink diagram, including any loaded equation edits.
if nargin<1, cases={acf_parameters}; end
if ~iscell(cases), cases={cases}; end
if nargin<2, path1='inertial.forceKp'; end
if nargin<3, values1=[0.15 0.3 0.45]; end
if nargin<4, path2='inertial.forceKi'; end
if nargin<5, values2=[10 20 30]; end
mdl=acf_open;mw=get_param(mdl,'ModelWorkspace');original=getVariable(mw,'p');
step=get_param(mdl,'FixedStep');stop=get_param(mdl,'StopTime');
choice=get_param([mdl '/Actuator selection'],'ActuatorChoice');
cleanup=onCleanup(@() restore(mdl,original,step,stop,choice)); %#ok<NASGU>
rows={}; groups={};
for i=1:numel(values1)
    for j=1:numel(values2)
        allPassed=true; worstError=0; peakForce=0; peakAccel=0;
        for k=1:numel(cases)
            p=put(cases{k},path1,values1(i)); p=put(p,path2,values2(j));
            try
                p=acf_prepare(p);
            catch err
                rows(end+1,:)={values1(i),values2(j),p.surface.kind,false, ...
                    NaN,NaN,NaN,['Invalid parameters: ' err.message]}; %#ok<AGROW>
                allPassed=false; worstError=inf;
                continue
            end
            r=acf_run(mdl,p,false);
            m=r.metrics;
            rows(end+1,:)={values1(i),values2(j),p.surface.kind,m.passed, ...
                m.peakAbsForceError_N,m.peakAxialForce_N,m.peakGapAcceleration_m_s2,m.reason}; %#ok<AGROW>
            allPassed=allPassed && m.passed;
            worstError=max(worstError,m.peakAbsForceError_N);
            peakForce=max(peakForce,m.peakAxialForce_N);
            peakAccel=max(peakAccel,m.peakGapAcceleration_m_s2);
        end
        groups(end+1,:)={values1(i),values2(j),allPassed,worstError,peakForce,peakAccel}; %#ok<AGROW>
    end
end
summary=cell2table(groups,'VariableNames', ...
    {'value1','value2','allPassed','worstError_N','peakAxialForce_N','peakGapAcceleration_m_s2'});
details=cell2table(rows,'VariableNames', ...
    {'value1','value2','scenario','passed','peakError_N','peakAxialForce_N','peakGapAcceleration_m_s2','reason'});
summary.Properties.VariableDescriptions{1}=path1;
summary.Properties.VariableDescriptions{2}=path2;
fprintf('Sweep %s x %s: %d/%d configurations passed every specified case.\n', ...
    path1,path2,sum(summary.allPassed),height(summary));
end

function p=put(p,path,value)
parts=strsplit(path,'.');
assert(numel(parts)==2 && isfield(p,parts{1}) && isfield(p.(parts{1}),parts{2}), ...
    'Use an existing two-level parameter path, e.g. screw.peakCurrent.');
p.(parts{1}).(parts{2})=value;
end

function restore(mdl,p,step,stop,choice)
assignin(get_param(mdl,'ModelWorkspace'),'p',p);
acf_select_actuator(mdl,choice);
set_param(mdl,'FixedStep',step,'StopTime',stop);
end
