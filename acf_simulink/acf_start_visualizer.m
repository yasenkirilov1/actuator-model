function [results, viewerFile, model] = acf_start_visualizer(kind, scenarios, backend)
%ACF_START_VISUALIZER One command: simulate, export and open the 2D replay.
%   acf_start_visualizer
%   acf_start_visualizer('voice_coil',{'bump','sharp_bump','ramp'})
%   acf_start_visualizer('voice_coil','bump','reference') % no Simulink needed
% Outputs: one result struct (or a cell for several runs), HTML path, model.
if nargin<1 || isempty(kind), kind='voice_coil'; end
if nargin<2 || isempty(scenarios), scenarios='bump'; end
if nargin<3 || isempty(backend), backend='simulink'; end
if ischar(scenarios), scenarios={scenarios}; end
if isstring(scenarios), scenarios=cellstr(scenarios); end
assert(iscell(scenarios) && ~isempty(scenarios),'Provide one or more scenarios.');
assert(any(strcmp(backend,{'simulink','reference'})),'Backend must be simulink or reference.');
addpath(fileparts(mfilename('fullpath')));
items=cell(1,numel(scenarios)); model='';
for k=1:numel(scenarios)
    p=acf_parameters(kind,char(scenarios{k}));
    if strcmp(backend,'simulink')
        if k==1, model=acf_build(p); end
        r=acf_run(model,p,false); r.source='Simulink';
    else
        r=acf_reference(p); r.source='MATLAB reference model';
    end
    items{k}=r;
end
viewerFile=acf_visualize(items);
if numel(items)==1, results=items{1}; else, results=items; end
if nargout==0
    assignin('base','acfResult',results);
    assignin('base','acfViewerFile',viewerFile);
    if ~isempty(model), assignin('base','acfModel',model); end
end
end
