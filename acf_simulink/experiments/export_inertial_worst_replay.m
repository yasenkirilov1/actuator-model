function report = export_inertial_worst_replay(root, targetHtml)
%EXPORT_INERTIAL_WORST_REPLAY Read-only export of the audited native worst.
% No model is loaded and no simulation is run. The input native history and
% full-history metric are verified before the public viewer reduces display.
if nargin < 1, root=fileparts(fileparts(fileparts(mfilename('fullpath')))); end
if nargin < 2
    targetHtml=fullfile(root,'acf_simulink','generated', ...
        'active_contact_flange_inertial_worst_20260930.html');
end
assert(~isfile(targetHtml),'Preserve the existing replay; choose a new name.');
addpath(fullfile(root,'acf_simulink'),'-begin');
summaryFile=fullfile(root,'acf_simulink','validation', ...
    'normal_mass_controller_final_grid_20260930_summary.json');
summary=jsondecode(fileread(summaryFile));
assert(strcmp(summary.status,'complete_audited_finite_nominal_grid'));
assert(summary.coverage.auditPassed && summary.coverage.complete);
traceFile=summary.worstNativeTrace;
if ~isfile(traceFile)
    % Portable delivery keeps this member under the same project-relative path.
    traceFile=fullfile(root,'acf_simulink','validation','final_grid','part1', ...
        'normal_mass_controller_inertial_full_range_mass_20260930_v8_final_part1_r2_worst_native_trace.mat');
end
traceHash=fileHash(traceFile);
assert(strcmp(traceHash,summary.worstNativeTraceSHA256));
recorded=load(traceFile,'worst'); r=recorded.worst;
assert(numel(r.time)==400001 && r.time(1)==0 && r.time(end)==4);
peak=max(abs(r.normalContact.data(:,1)-r.data(:,31)));
assert(abs(peak-summary.worstCase.peakNormalForceError_N)<1e-12);
r.label='Full-grid worst: S=2 kg / 50 N / 75 deg / seed97';
r.source='Audited native Simulink grid global3489; read-only replay export';
html=acf_visualize(r,targetHtml,false);
metrics=acf_metrics(r);
assert(abs(metrics.peakAbsForceError_N-peak)<1e-12);
report=struct('status','complete_read_only_native_replay_export', ...
    'newSimulations',0,'sourceNativeTrace',traceFile, ...
    'sourceNativeTraceSHA256',traceHash,'summarySHA256',fileHash(summaryFile), ...
    'nativeSamplesScored',numel(r.time),'peakNormalForceError_N',peak, ...
    'htmlFile',html,'htmlSHA256',fileHash(html),'metrics',metrics, ...
    'displayScope','Full-history scores calculated before extrema/event-preserving display reduction.');
output=fullfile(root,'acf_simulink','validation','inertial_worst_replay_export_20260930.json');
assert(~isfile(output),'Preserve an existing export receipt.');
f=fopen(output,'w'); assert(f>=0); cleanup=onCleanup(@() fclose(f));
fprintf(f,'%s\n',jsonencode(report,'PrettyPrint',true));
end

function value=fileHash(file)
% Works in Linux -nojvm MATLAB; quote paths without command substitution.
quoted=['''' strrep(char(file),'''','''"''"''') ''''];
[code,output]=system(['sha256sum -- ' quoted]);
assert(code==0,'SHA256 failed.'); value=lower(strtrim(output(1:64)));
assert(numel(value)==64 && all(ismember(value,'0123456789abcdef')));
end
