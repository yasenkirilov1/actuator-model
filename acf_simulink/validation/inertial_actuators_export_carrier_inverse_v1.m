function replay=inertial_actuators_export_carrier_inverse_v1(reportFile,outputFile)
% Export existing paired native histories. This function runs no simulation.
assert(isfile(reportFile));assert(~isfile(outputFile),'Retained exports are immutable.');
report=jsondecode(fileread(reportFile));
assert(strcmp(report.status,'complete') && report.executedCases==8);
assert(strcmp(report.phase,'carrier_inverse_comparison_v1'));
oldThreads=maxNumCompThreads;maxNumCompThreads(2);
cleanup=onCleanup(@()maxNumCompThreads(oldThreads)); %#ok<NASGU>
replay=struct('status','reviewed_complete_native_recordings', ...
 'reportFile',reportFile,'reportSHA256',digest(reportFile), ...
 'sourceModelSHA256',report.sourceModelSHA256, ...
 'privateModelSHA256',report.privateModelSHA256, ...
 'recipeSHA256',report.recipeSHA256, ...
 'columns',{{'time_s','trueNormalForce_N','estimatedNormalForce_N', ...
 'normalForceError_N','current_A','voltage_V','travel_mm','surfaceHeight_mm', ...
 'trueSlope','estimatedSlope','robotVelocity_mm_s'}}, ...
 'plotSampling','Approximately 1ms plus native global extrema. Qualification uses every native timestamp, including startup.', ...
 'forceTolerance_N',.5,'cases',struct([]));
for caseIndex=1:8
 row=report.rows(caseIndex);assert(row.caseIndex==caseIndex && row.executed);
 assert(row.physicalScanSpeed_m_s==row.commandedScanSpeed_m_s);
 assert(row.forceFilterTime_s==100e-6 && row.accelerationFilterTime_s==100e-6);
 loaded=load(row.nativeTraceFile,'r');r=loaded.r;
 t=r.time;d=r.data;n=r.normalContact.data;
 assert(isequal(t,r.normalContact.time));
 error=n(:,1)-r.parameters.force.reference;fullPeak=max(abs(error));
 assert(abs(fullPeak-row.peakNormalForceError_N)<1e-10);
 stride=max(1,round(.001/r.parameters.sim.step));indices=(1:stride:numel(t))';
 y=[n(:,1) n(:,4) d(:,14) d(:,15) d(:,2)];
 [~,mins]=min(y,[],1);[~,maxs]=max(y,[],1);
 [~,peak]=max(abs(error));indices=unique([indices;numel(t);mins';maxs';peak]);
 plotted=[t(indices) n(indices,1) n(indices,4) error(indices) ...
  d(indices,14) d(indices,15) d(indices,2)*1000 d(indices,28)*1000 ...
  n(indices,3) n(indices,5) d(indices,27)*1000];
 assert(max(abs(plotted(:,4)))==fullPeak,'Replay must retain the full-run force peak.');
 entry=struct('row',row,'referenceForce_N',r.parameters.force.reference, ...
  'nativeTraceSHA256',digest(row.nativeTraceFile),'nativeSamples',numel(t), ...
  'plotSamples',numel(indices),'fullNativePeakError_N',fullPeak, ...
  'fullNativeRmsError_N',sqrt(mean(error.^2)), ...
  'peakErrorTime_s',t(peak),'peakSignedError_N',error(peak),'data',plotted);
 replay.cases=[replay.cases;entry]; %#ok<AGROW>
 clear loaded r d n y plotted;
end
fid=fopen(outputFile,'w');assert(fid>=0);close=onCleanup(@()fclose(fid)); %#ok<NASGU>
fwrite(fid,jsonencode(replay),'char');
fprintf('CARRIER_ACTUATOR_REPLAY: %d existing cases; every plot retains the full native force peak.\n',numel(replay.cases));
end

function value=digest(path)
quote=char(39);escaped=strrep(path,quote,[quote char(34) quote char(34) quote]);
[status,out]=system(['sha256sum -- ' quote escaped quote]);assert(status==0);value=out(1:64);
end
