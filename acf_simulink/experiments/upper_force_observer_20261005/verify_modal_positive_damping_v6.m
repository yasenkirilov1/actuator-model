function report=verify_modal_positive_damping_v6(mdl,configs,folder)
% Native sinusoidal command-to-force sign/phase guard, not qualification.
indices=[45,111,177];sys=[mdl '/Force measurement/Engineered common AA and EMA'];
target=get_param([sys '/Continuous analog front end'],'PortHandles');line=get_param(target.Inport(1),'Line');
original=get_param(line,'SrcPortHandle');delete_line(sys,original,target.Inport(1));
stim=[sys '/Owned modal damping phase stimulus'];
add_block('simulink/Sources/Sine Wave',stim,'Amplitude','.1','Bias','p.force.reference', ...
 'Frequency','1','Phase','0','SampleTime','0','Position',[20 -240 350 -180]);
ports=get_param(stim,'PortHandles');add_line(sys,ports.Outport(1),target.Inport(1),'autorouting','on');
coil=[mdl '/Actuator and RABS mechanics/Actuator/Voice coil'];log=[coil '/Owned modal diagnostic current log'];
add_block('simulink/Sinks/To Workspace',log,'VariableName','owned_modal_diagnostic_current', ...
 'SaveFormat','Timeseries','MaxDataPoints','inf','SampleTime','p.sim.step','Position',[1000 1200 1450 1280]);
add_line(coil,'Winding current/1','Owned modal diagnostic current log/1','autorouting','on');
oldRecord=Simulink.sdi.getRecordData;Simulink.sdi.setRecordData(true);
cleanup=onCleanup(@()restore(sys,stim,log,target.Inport(1),original,oldRecord)); %#ok<NASGU>
rows=struct([]);duration=.04;discard=.012;
for index=indices
 p=configs(index).p;p.adcStudy.forceAdcNoiseRms_N=0;p=acf_prepare(p);assignin(get_param(mdl,'ModelWorkspace'),'p',p);
 w=2*pi*p.cellModal.carrier_Hz;set_param(stim,'Frequency',num2str(w,17));
 input=[0,p.d.coilHoldCurrent,0,0;duration,p.d.coilHoldCurrent,0,0];
 assignin('base','owned_modal_diagnostic_external_input',input);
 output=sim(mdl,'StopTime',num2str(duration,17),'SimulationMode','normal', ...
  'LoadExternalInput','on','ExternalInput','owned_modal_diagnostic_external_input','ReturnWorkspaceOutputs','on');
 data=output.owned_modal_diagnostic_current;t=double(data.Time(:));
 force=p.coil.forceConstant*(double(data.Data(:))-p.d.coilHoldCurrent);use=t>=discard;
 fit=[sin(w*t(use)),cos(w*t(use)),ones(sum(use),1)];beta=fit\force(use);
 actual=(beta(1)+1i*beta(2))/.1;
 desired=-1i*w*p.cellModal.virtualDamping_Ns_m/p.inertial.lowerLoadInverse.identifiedStiffness_N_m;
 ratio=actual/desired;residual=force(use)-fit*beta;
 velocity=-.1*w/p.inertial.lowerLoadInverse.identifiedStiffness_N_m*cos(w*t(use));
 meanPower=-mean(force(use).*velocity);
 row=struct('configurationIndex1',index,'carrier_Hz',p.cellModal.carrier_Hz, ...
  'filterDamping',p.cellModal.filterDamping,'virtualDamping_Ns_m',p.cellModal.virtualDamping_Ns_m, ...
  'actualElectromagneticForcePerRawCellReal',real(actual),'actualElectromagneticForcePerRawCellImag',imag(actual), ...
  'desiredElectromagneticForcePerRawCellReal',real(desired),'desiredElectromagneticForcePerRawCellImag',imag(desired), ...
  'normalizedPositiveDampingReal',real(ratio),'normalizedPositiveDampingImag',imag(ratio), ...
  'gainRatio',abs(ratio),'phaseError_deg',angle(ratio)*180/pi, ...
  'meanVirtualBModePower_W',meanPower,'fitResidualRms_N',sqrt(mean(residual.^2)), ...
  'actualTimes_s',t,'actualDiagnosticElectromagneticForce_N',force,'fittedCoefficientSinCosOffset',beta, ...
  'physicalCurrentUsedOnlyForGuard',true,'qualificationEpisodes',0);
 if isempty(rows),rows=row;else,rows(end+1)=row;end
 write_json(fullfile(folder,'modal_positive_damping_guard_progress.json'),struct('status','running_native_modal_phase_guard','rows',rows));
 assert(abs(abs(ratio)-1)<.03&&abs(angle(ratio)*180/pi)<3&&meanPower<0, ...
  'Native modal feedback has the wrong transmitted phase/sign or carrier gain.');
 Simulink.sdi.clear;
end
report=struct('status','native_modal_positive_damping_sign_and_phase_passed','qualificationEpisodes',0, ...
 'sinusoidalWiringHarnessCount',numel(rows),'durationPerHarness_s',duration, ...
 'rows',rows,'scope','Native raw-cell acquisition, digital modal output,10k command hold and actual50us current response checked end-to-end at carrier. Physical current is a diagnostic log only, never a controller input. Baseline current held at loaded equilibrium; no candidate noise change. Harnesses excluded from force qualification.');
write_json(fullfile(folder,'modal_positive_damping_guard.json'),report);
end
function restore(sys,stim,log,inputPort,sourcePort,oldRecord)
if getSimulinkBlockHandle(log)>0,delete_block(log);end
if getSimulinkBlockHandle(stim)>0,delete_block(stim);end
line=get_param(inputPort,'Line');if line~=-1,delete_line(line);end
add_line(sys,sourcePort,inputPort,'autorouting','on');
evalin('base','clear owned_modal_diagnostic_external_input');
Simulink.sdi.clear;Simulink.sdi.setRecordData(oldRecord);
end
function write_json(file,value)
fid=fopen(file,'w');assert(fid>=0);fprintf(fid,'%s\n',jsonencode(value,'PrettyPrint',true));fclose(fid);
end
