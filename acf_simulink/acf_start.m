function [result,model,p] = acf_start(kind,scenario)
%ACF_START Open and run the saved model with its current settings.
% Optional names select an actuator/scenario; block edits are always retained.
addpath(fileparts(mfilename('fullpath')));
model=acf_open;
if nargin<1,kind=[];end
if nargin<2,scenario=[];end
p=acf_parameters(kind,scenario);result=acf_run(model,p,true);
if nargout==0
    assignin('base','acfResult',result);
    assignin('base','acfModel',model);
    assignin('base','acfParameters',result.parameters);
end
end
