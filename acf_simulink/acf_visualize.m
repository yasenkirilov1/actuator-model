function htmlFile = acf_visualize(results, htmlFile, openBrowser, springSweep)
%ACF_VISUALIZE Export recorded ACF results to an offline, interactive 2D viewer.
%   acf_visualize(r)                  % result from acf_run
%   acf_visualize({r1,r2,r3})         % choose among several recorded runs
%   file=acf_visualize(r,'replay.html',false) % export without opening browser
%   acf_visualize(runs,'sweep.html',true,report) % include sweep_springs results
%   acf_visualize                    % uses acfResult in the base workspace
%
% Long histories use an extrema/event-preserving display subset. Metrics are
% calculated from the complete original recording before display reduction.
% Playback uses recorded samples without interpolating contact/impact states.
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
    if isfield(r,'normalContact')
        assert(isequal(tt,r.normalContact.time) && size(r.normalContact.data,2)==8, ...
            'ACF:Viewer:NormalForce','Invalid native normal-force recording.');
        canonical.data(:,1)=r.normalContact.data(:,1);
    end
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
    if isfield(r,'normalContact')
        one.forceComponent='surface_normal';
        one.contactEstimate=struct('time',tt, ...
            'force_N',r.normalContact.data(:,4), ...
            'source','Recorded native normal-force reconstruction');
    elseif isfield(r,'contactEstimate')
        estimate=r.contactEstimate;
        assert(numel(estimate.time)==numel(estimate.force_N) && ...
            all(diff(estimate.time)>0) && all(isfinite(estimate.force_N)), ...
            'ACF:Viewer:Estimate','Invalid recorded normal-force estimate.');
        one.contactEstimate=estimate;
    elseif isfield(r,'observerStates') && strcmp(p.actuator.kind,'voice_coil') && ...
            strcmp(p.coil.strategy,'observer')
        observed=r.observerStates;
        assert(size(observed.data,1)==numel(observed.time) && ...
            size(observed.data,2)==numel(p.d.coilControl.force) && ...
            all(isfinite(observed.data),'all') && all(diff(observed.time)>0), ...
            'ACF:Viewer:Observer','Invalid recorded observer states.');
        reconstructed=p.force.reference+observed.data*p.d.coilControl.force';
        if p.mechanics.lowerRigid && p.coil.directCoulombCorrection
            reconstructed=reconstructed-p.coil.coulombForce* ...
                tanh(observed.data(:,6)/p.coil.frictionSpeed);
        end
        one.contactEstimate=struct('time',observed.time, ...
            'force_N',reconstructed,'source','Recorded native state estimate');
    end
    if isfield(r,'robotGate')
        gate=r.robotGate; fields={'time','ready','filteredError_N','forceRate_N_s','stableNow'};
        assert(all(isfield(gate,fields)) && ~isempty(gate.time) && all(diff(gate.time)>0), ...
            'ACF:Viewer:Gate','Invalid force-settling diagnostic time samples.');
        for j=1:numel(fields)
            values=gate.(fields{j});
            assert(numel(values)==numel(gate.time) && all(isfinite(values)), ...
                'ACF:Viewer:Gate','Invalid force-settling diagnostic samples.');
        end
        one.robotGate=gate;
    end
    one=reduce_display(one);
    runs{k}=one;
end
bundle=struct('schema','acf-replay-v1','runs',{runs});
if nargin>=4 && ~isempty(springSweep)
    assert(isstruct(springSweep) && isscalar(springSweep) && ...
        all(isfield(springSweep,{'rows','selected','configurations','passedConfigurations'})), ...
        'ACF:Viewer:Sweep','Pass the report returned by sweep_springs.');
    bundle.springSweep=springSweep;
end
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

function one=reduce_display(one)
% Keep the portable replay small while retaining native peaks and events.
% The full recording remains in the caller's result; only display is reduced.
n=numel(one.time);
if n<=20000,return;end
nativeTime=one.time;
d=one.data;signals=[d(:,1)-d(:,31),d(:,2),d(:,14:15),d(:,32), ...
    d(:,7)-d(:,6),d(:,5)-d(:,26)];
if isfield(one,'contactEstimate') && isequal(one.time,one.contactEstimate.time)
    signals=[signals,one.contactEstimate.force_N];
end
% Each displayed bin retains its endpoints and every selected min/max.
edges=unique(round(linspace(1,n+1,1001)));
keep=false(n,1);keep([1,n])=true;
for b=1:numel(edges)-1
    first=edges(b);last=edges(b+1)-1;
    [~,lo]=min(signals(first:last,:),[],1);
    [~,hi]=max(signals(first:last,:),[],1);
    keep([first,last,first-1+lo,first-1+hi])=true;
end
% Preserve entry/exit samples for band, contact, stop and drive-limit events.
drive=one.parameters.coil;
if strcmp(one.parameters.actuator.kind,'leadscrew'),drive=one.parameters.screw;end
limit=inf;if ~strcmp(one.parameters.actuator.kind,'ideal_position'),limit=drive.peakCurrent;end
flags=[abs(d(:,1)-d(:,31))>one.parameters.force.tolerance, ...
    d(:,18)<.5,d(:,20)>1e-10,d(:,21:23)>.5,abs(d(:,14))>limit*1.0001];
events=find(any(diff(flags)~=0,2));keep([events;events+1])=true;
if isfield(one,'robotGate') && isequal(one.time,one.robotGate.time)
    events=find(diff(one.robotGate.ready)~=0);keep([events;events+1])=true;
end
valid=find(one.time>=one.metrics.evaluationStart_s);
[~,at]=max(abs(d(valid,1)-d(valid,31)));worst=valid(at);keep(worst)=true;
indices=find(keep);
one.display=struct('originalSamples',n,'exportedSamples',numel(indices), ...
    'method','Bin extrema and discrete transitions; metrics use complete native recording', ...
    'nativeWorstTime_s',one.time(worst));
one.time=one.time(indices);one.data=d(indices,:);
if isfield(one,'contactEstimate') && isequal(one.contactEstimate.time,nativeTime)
    one.contactEstimate.time=one.contactEstimate.time(indices);
    one.contactEstimate.force_N=one.contactEstimate.force_N(indices);
end
if isfield(one,'robotGate') && isequal(one.robotGate.time,nativeTime)
    for name={'time','ready','filteredError_N','forceRate_N_s','stableNow'}
        one.robotGate.(name{1})=one.robotGate.(name{1})(indices);
    end
end
end
