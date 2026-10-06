function report=verify_emitted_current_input(folder)
% Codegen must preserve a live issued-current variable in current-lag xdot.
file=fullfile(folder,'code','acf_rl_native_grt_rtw','acf_rl_native.c');code=fileread(file);
section=regexp(code,'/\* Derivatives for StateSpace: ''<S\d+>/Identified causal current lag'' \*/(.*?)/\* End of Derivatives for StateSpace: ''<S\d+>/Identified causal current lag'' \*/','tokens','once');
assert(~isempty(section),'Missing emitted current lag.');body=section{1};
operand=regexp(body,'Identifiedcausalcurrentlag_B_pr\s*\*\s*([^;]+);','tokens','once');
assert(~isempty(operand));operand=strtrim(operand{1});
assert(~strcmp(operand,'0.0')&&~strcmp(operand,'0')&&contains(operand,'acf_rl_native_B.'),'Current lag input was grounded or constant in emittedC.');
name=regexp(operand,'acf_rl_native_B\.(\w+)','tokens','once');assert(~isempty(name));
assignment=regexp(code,['acf_rl_native_B\.' name{1} '\s*=\s*([^;]+);'],'tokens');assert(~isempty(assignment));
assert(any(cellfun(@(a)~strcmp(strtrim(a{1}),'0.0'),assignment)));
report=struct('status','emitted_current_lag_uses_live_selected_command','inputOperand',operand, ...
 'inputSignalAssignments',{assignment},'derivativeSection',body,'nativeCodeFile',file, ...
 'currentLagInputGrounded',false,'physicalQualificationEpisodes',0);
fid=fopen(fullfile(folder,'emitted_current_input_guard.json'),'w');assert(fid>=0);fprintf(fid,'%s\n',jsonencode(report,'PrettyPrint',true));fclose(fid);
end
