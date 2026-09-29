function htmlFile = acf_visualize(results, htmlFile, openBrowser)
%ACF_VISUALIZE Export recorded ACF results to an offline, interactive 2D viewer.
%   acf_visualize(r)                  % result from acf_run / acf_reference
%   acf_visualize({r1,r2,r3})         % choose among several recorded runs
%   file=acf_visualize(r,'replay.html',false) % export without opening browser
%   acf_visualize                    % uses acfResult in the base workspace
%
% All logged samples are exported. Playback uses recorded samples, without
% interpolating discrete contact/impact states or rerunning the physics.
% The generated HTML is self-contained and can be shared or opened offline.
if nargin < 1
    if evalin('base','exist(''acfResult'',''var'')')
        results=evalin('base','acfResult');
    else
        error('ACF:Viewer:NoResult', ...
            'Run acf_start_visualizer, or pass a result: acf_visualize(r).');
    end
end
if nargin < 3 || isempty(openBrowser), openBrowser=true; end
validateattributes(openBrowser,{'logical','numeric'},{'scalar'},mfilename,'openBrowser');
root=fileparts(mfilename('fullpath'));
if nargin < 2 || isempty(htmlFile)
    folder=fullfile(root,'generated');
    if ~exist(folder,'dir'), mkdir(folder); end
    stem=['acf_replay_' datestr(now,'yyyymmdd_HHMMSS_FFF')];
    htmlFile=fullfile(folder,[stem '.html']); n=2;
    while exist(htmlFile,'file')
        htmlFile=fullfile(folder,sprintf('%s_%d.html',stem,n)); n=n+1;
    end
else
    htmlFile=char(htmlFile);
    [folder,~,ext]=fileparts(htmlFile);
    assert(any(strcmpi(ext,{'.html','.htm'})), ...
        'ACF:Viewer:FileType','Output filename must end in .html or .htm.');
    if isempty(regexp(htmlFile,'^([A-Za-z]:[\\/]|[\\/])','once'))
        htmlFile=fullfile(pwd,htmlFile); folder=fileparts(htmlFile);
    end
    if ~isempty(folder) && ~exist(folder,'dir'), mkdir(folder); end
end
if isstruct(results), results=num2cell(results); end
assert(iscell(results) && ~isempty(results),'ACF:Viewer:Results', ...
    'Pass an ACF result struct, or a cell array of results.');
names=acf_signals; runs=cell(1,numel(results));
for k=1:numel(results)
    r=results{k};
    assert(isstruct(r) && isscalar(r) && ...
        all(isfield(r,{'time','data','parameters','names'})), ...
        'ACF:Viewer:Result','Result %d needs time, data, parameters and names.',k);
    tt=r.time(:); dd=r.data;
    validateattributes(tt,{'numeric'},{'real','finite','nonempty'},mfilename,'time');
    validateattributes(dd,{'numeric'},{'real','finite','2d'},mfilename,'data');
    assert(numel(tt)>=2 && all(diff(tt)>0),'ACF:Viewer:Time', ...
        'Result %d needs at least two strictly increasing times.',k);
    assert(size(dd,1)==numel(tt) && size(dd,2)==numel(r.names), ...
        'ACF:Viewer:Dimensions','Result %d has mismatched signal dimensions.',k);
    [present,order]=ismember(names,r.names);
    assert(all(present) && numel(unique(order))==numel(names), ...
        'ACF:Viewer:Signals','Result %d is missing required ACF signals.',k);
    canonical=r; canonical.time=tt; canonical.data=dd(:,order); canonical.names=names;
    p=r.parameters;
    assert(all(isfield(p,{'actuator','mechanics','force','robot','surface','sim'})), ...
        'ACF:Viewer:Parameters','Result %d is missing model parameters.',k);
    one=struct;
    one.label=sprintf('%s / %s',p.actuator.kind,p.surface.kind);
    if isfield(r,'label'), one.label=char(r.label); end
    one.source='Recorded ACF result';
    if isfield(r,'source'), one.source=char(r.source); end
    one.time=tt; one.data=canonical.data; one.names=names;
    one.parameters=p; one.metrics=acf_metrics(canonical);
    runs{k}=one;
end
bundle=struct('schema','acf-replay-v1','runs',{runs});
encoded=jsonencode(bundle);
% Keep user-supplied labels inside the data script, even if they contain HTML.
encoded=strrep(encoded,'<','\u003c');
encoded=strrep(encoded,'>','\u003e');
encoded=strrep(encoded,'&','\u0026');
templateFile=fullfile(root,'viewer','acf_viewer_template.html');
assert(exist(templateFile,'file')==2,'ACF:Viewer:Template', ...
    'Keep viewer/acf_viewer_template.html beside the add-on MATLAB files.');
template=fileread(templateFile); token='__ACF_DATA_JSON__';
assert(numel(strfind(template,token))==1,'ACF:Viewer:Template', ...
    'The viewer template must contain exactly one data placeholder.');
document=strrep(template,token,encoded);
fid=fopen(htmlFile,'w','n','UTF-8');
assert(fid>=0,'ACF:Viewer:Write','Cannot write viewer: %s',htmlFile);
cleanup=onCleanup(@() fclose(fid));
count=fprintf(fid,'%s',document);
assert(count>=0,'ACF:Viewer:Write','Failed while writing viewer: %s',htmlFile);
clear cleanup;
fprintf('ACF 2D replay saved: %s\n',htmlFile);
if openBrowser
    try
        status=web(htmlFile,'-browser');
        if status~=0
            warning('ACF:Viewer:Browser','Open this saved HTML file in your browser: %s',htmlFile);
        end
    catch err
        warning('ACF:Viewer:Browser','Viewer was saved. Open it manually: %s\n%s',htmlFile,err.message);
    end
end
end
