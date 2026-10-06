function [results,viewerFile,model] = acf_start_visualizer(kind,scenarios,openBrowser)
%ACF_START_VISUALIZER Run the saved Simulink model and open its recorded replay.
% No arguments uses the current model settings.
% acf_start_visualizer('voice_coil',{'bump','sharp_bump','ramp'})
addpath(fileparts(mfilename('fullpath')));
if nargin<3,openBrowser=true;end
assert(islogical(openBrowser) && isscalar(openBrowser),'openBrowser must be true or false.');
model=acf_open;
current=acf_parameters;
if nargin<1 || isempty(kind),kind=current.actuator.kind;end
if nargin<2 || isempty(scenarios),scenarios=current.surface.kind;end
if ischar(scenarios),scenarios={scenarios};end
if isstring(scenarios),scenarios=cellstr(scenarios);end
assert(iscell(scenarios) && ~isempty(scenarios),'Provide one or more scenarios.');
presets=cell(1,numel(scenarios));
for k=1:numel(scenarios),presets{k}=acf_parameters(kind,char(scenarios{k}));end
items=cell(1,numel(scenarios));
for k=1:numel(scenarios)
    r=acf_run(model,presets{k},false);r.source='Simulink';items{k}=r;
end
viewerFile=acf_visualize(items,[],openBrowser);
if numel(items)==1,results=items{1};else,results=items;end
if nargout==0
    assignin('base','acfResult',results);
    assignin('base','acfViewerFile',viewerFile);
    assignin('base','acfModel',model);
end
end
