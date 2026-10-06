function checks=validate_public_force_helpers(recordingFile,outputFile)
%VALIDATE_PUBLIC_FORCE_HELPERS Check public scoring against a retained run.
% No plant simulation or qualification is performed by these software checks.
assert(~usejava('desktop'),'Run in an isolated MATLAB -batch process.');
root=fileparts(fileparts(mfilename('fullpath')));addpath(root,'-begin');
maxNumCompThreads(2);
saved=load(recordingFile,'r');r=saved.r;
assert(isfield(r,'normalContact') && isequal(r.time,r.normalContact.time));
expected=max(abs(r.normalContact.data(:,1)-r.data(:,31)));
m=acf_metrics(r);assert(abs(m.peakAbsForceError_N-expected)<1e-12);
checks=struct('status','complete','kind','public_helper_software_checks', ...
 'sourceRecording',recordingFile,'newNativeSimulations',0, ...
 'retainedNativeSamples',numel(r.time),'nativePeakError_N',expected, ...
 'publicMetricMatchesNativeNormalForce',true);

% A nonzero evaluation start must never hide startup in a normal-force run.
startup=r;startup.parameters.sim.evaluateFrom=r.time(end)/2;
startup.normalContact.data(1,1)=startup.data(1,31)+2;
sm=acf_metrics(startup);
assert(sm.peakAbsForceError_N>=2-1e-12 && ~sm.passed && sm.evaluationStart_s==r.time(1));
checks.startupViolationRetained=true;
contact=r;contact.data(end,18)=0;cm=acf_metrics(contact);
assert(cm.lostContact && ~cm.passed);checks.terminalContactFailureRetained=true;
stop=r;stop.data(end,21)=1;tm=acf_metrics(stop);
assert(tm.strokeStopHit && ~tm.passed);checks.terminalTravelFailureRetained=true;
incomplete=r;incomplete.parameters.sim.stopTime=r.time(end)+1;
im=acf_metrics(incomplete);assert(~im.completed && ~im.passed);
checks.incompleteRecordingRejected=true;

% Keep this browser fixture small; it is explicitly not a native experiment.
fixture=startup;indices=unique([1;2;find(abs(r.normalContact.data(:,1)-r.data(:,31))==expected,1);numel(r.time)]);
fixture.time=r.time(indices);fixture.data=r.data(indices,:);
fixture.normalContact.time=fixture.time;
fixture.normalContact.data=startup.normalContact.data(indices,:);
for field={'observerStates','contactEstimate','robotGate','commandPrediction', ...
 'accelerometer','motionObserver','forceVariation','accelerationFeedforward','measuredDrag'}
 if isfield(fixture,field{1}),fixture=rmfield(fixture,field{1});end
end
fixture.label='Software check: startup force violation';
fixture.source='Subset fixture from retained history; not an experimental recording';
folder=tempname(fullfile(getenv('HOME'),'.local','state','actuator-model'));mkdir(folder);
fixtureFile=acf_visualize(fixture,fullfile(folder,'public_force_fixture.html'),false);
html=fileread(fixtureFile);
token=regexp(html,'<script id="acf-data" type="application/json">(.*?)</script>','tokens','once');
assert(~isempty(token));bundle=jsondecode(token{1});one=bundle.runs;
assert(strcmp(one.forceComponent,'surface_normal'));
assert(isequal(one.data(:,1),fixture.normalContact.data(:,1)));
assert(isequal(one.contactEstimate.force_N,fixture.normalContact.data(:,4)));
assert(one.metrics.peakAbsForceError_N>=2-1e-12 && ~one.metrics.passed);
checks.viewerForceAndEstimateMatchNativeChannels=true;
checks.viewerFixture=fixtureFile;

% Long replay reduction must preserve native metrics and individual events.
displayFixture=r;
displayFixture.label='Software check: reduced display with single-sample events';
displayFixture.source='Modified retained recording for export checks; not a native experiment';
eventIndex=round(numel(r.time)/3);
displayFixture.data(eventIndex,18)=0;
displayFixture.data(eventIndex+2,21)=1;
reducedFile=acf_visualize(displayFixture,fullfile(folder,'reduced_display_fixture.html'),false);
html=fileread(reducedFile);
token=regexp(html,'<script id="acf-data" type="application/json">(.*?)</script>','tokens','once');
bundle=jsondecode(token{1});one=bundle.runs;
assert(one.display.originalSamples==numel(r.time) && numel(one.time)<numel(r.time)/5);
[present,indices]=ismember(one.time,r.time);assert(all(present));
expectedData=displayFixture.data(indices,:);expectedData(:,1)=r.normalContact.data(indices,1);
assert(max(abs(one.data-expectedData),[],'all')<1e-12);
assert(abs(one.metrics.peakAbsForceError_N-expected)<1e-12);
assert(one.metrics.lostContact && one.metrics.strokeStopHit && ~one.metrics.passed);
assert(any(one.time==r.time(eventIndex)) && any(one.time==r.time(eventIndex+1)) && ...
    any(one.time==r.time(eventIndex+2)) && any(one.time==r.time(eventIndex+3)));
assert(max(abs(one.data(:,1)-one.data(:,31)))==expected);
assert(max(abs(one.contactEstimate.force_N-r.normalContact.data(indices,4)))<1e-12);
checks.viewerReducedDisplayPreservesNativeMetrics=true;
checks.viewerSingleSampleEventsRetained=true;
checks.viewerExportedSamples=numel(one.time);
checks.viewerReducedFixture=reducedFile;

files={'acf_run.m','acf_metrics.m','acf_plot.m','acf_visualize.m','acf_signals.m'};
issues=struct([]);
for k=1:numel(files)
 found=checkcode(fullfile(root,files{k}),'-id');
 issues(k).file=files{k};issues(k).messages=found; %#ok<AGROW>
end
checks.codeAnalyzer=issues;
checks.codeAnalyzerMessageCount=sum(arrayfun(@(x)numel(x.messages),issues));
fid=fopen(outputFile,'w');assert(fid>=0);cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'%s\n',jsonencode(checks));
fprintf('Public force helpers checked: %.6g N native peak; %d analyzer messages.\n', ...
 expected,checks.codeAnalyzerMessageCount);
end
