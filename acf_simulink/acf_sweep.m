function [summary,details] = acf_sweep(cases,path1,values1,path2,values2,backend)
%ACF_SWEEP Test a rectangular parameter grid across specified scenarios.
% Returns every case and configurations passing ALL cases. No global minimum
% or optimal controller claim is made. Controller gains stay fixed.
%
% p=acf_parameters('ideal_position','bump');
% [T,D]=acf_sweep({p},'ideal.maxAxialForce',[15 20 30], ...
%     'ideal.maxAcceleration',[0.1 0.5 2],'simulink');
% T(T.allPassed,:)  % candidates passing this particular test envelope
%
% backend: 'simulink' (default) or 'reference' (plain MATLAB, same equations).
if nargin<1, cases={acf_parameters}; end
if ~iscell(cases), cases={cases}; end
if nargin<2, path1='screw.peakCurrent'; end
if nargin<3, values1=[0.5 1 2 3]; end
if nargin<4, path2='robot.delay'; end
if nargin<5, values2=[0 0.08 0.2]; end
if nargin<6, backend='simulink'; end
assert(any(strcmp(backend,{'simulink','reference'})),'Unknown backend.');
if strcmp(backend,'simulink'), mdl=acf_build(cases{1}); end
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
            if strcmp(backend,'simulink'), r=acf_run(mdl,p,false);
            else, r=acf_reference(p); end
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
