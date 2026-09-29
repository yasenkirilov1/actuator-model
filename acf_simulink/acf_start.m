function [result,model,p] = acf_start(kind,scenario)
%ACF_START One command: build/open native SLX, run a scenario, show results.
%   acf_start
%   [r,mdl,p] = acf_start('voice_coil','sharp_bump');
if nargin<1, kind='leadscrew'; end
if nargin<2, scenario='bump'; end
addpath(fileparts(mfilename('fullpath')));
p=acf_parameters(kind,scenario);
model=acf_build(p);
result=acf_run(model,p,true);
if nargout==0
    assignin('base','acfResult',result);
    assignin('base','acfModel',model);
    assignin('base','acfParameters',p);
end
end
