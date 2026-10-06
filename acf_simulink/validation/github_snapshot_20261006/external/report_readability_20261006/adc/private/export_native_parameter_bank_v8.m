function export_native_parameter_bank_v8(mdl,configs,folder)
% Parse Coder's parameter declarations, then evaluate the actual native block
% expressions. Computed sparse state-space coefficients retain Coder's exact
% sparse column-major and discrete row-major layouts. No physical equation is
% reimplemented here; exact emitted case0 bytes are checked by the C bridge.
header=fileread(fullfile(folder,'code',[mdl '_grt_rtw'],[mdl '.h']));
body=regexp(header,'struct P_acf_rl_native_T_ \{(.*?)\n\};','tokens','once');assert(~isempty(body));
fields=regexp(body{1},'(\w+)\s+(\w+[^;]*);\s*/\*(.*?)\*/','tokens');assert(numel(fields)>500);
map=struct([]);counters=containers.Map;
for j=1:numel(fields)
 f=fields{j};size=regexp(f{2},'\[(\d+)\]','tokens','once');count=1;if ~isempty(size),count=str2double(size{1});end
 f{2}=regexp(f{2},'^\w+','match','once');
 ref=regexp(f{3},'Referenced by: ''([^'']+)''','tokens','once');assert(~isempty(ref));
 expr=regexp(f{3},'Expression:\s*([^\n]+)','tokens','once');
 if isempty(expr)
  path=ref{1};token=regexp(path,'^<S(\d+)>/(.*)$','tokens','once');assert(~isempty(token));
  sid=str2double(token{1});sub=regexp(header,sprintf("'\\<S%d\\>'\\s*:\\s*'([^']+)'",sid),'tokens','once');
  if isempty(sub),sub=regexp(header,sprintf("'<S%d>'\\s*:\\s*'([^']+)'",sid),'tokens','once');end
  assert(~isempty(sub));block=[sub{1} '/' token{2}];type=get_param(block,'BlockType');
  if any(strcmp(type,{'StateSpace','DiscreteStateSpace'}))
   if strcmp(type,'StateSpace')
    matrix=regexp(f{2},'_([ABCD])_(pr|p_|ir|i_|jc|j_)','tokens','once');assert(~isempty(matrix));storage=matrix{2};matrix=matrix{1};
    if startsWith(f{1},'uint')
     if any(strcmp(storage,{'ir','i_'})),kind='sparse_row';else,kind='sparse_col';end
    else,kind='sparse_value';end
   else
    if ~isKey(counters,block),counters(block)=0;end;counters(block)=counters(block)+1;
    abcd='ABCD';matrix=abcd(counters(block));kind='nonzero_value';
   end
   expression=get_param(block,matrix);
  elseif strcmp(type,'RandomNumber'),kind='sqrt';expression=get_param(block,'Variance');
  elseif strcmp(type,'DiscreteIntegrator'),kind='plain';expression=get_param(block,'SampleTime');
  else,error('Unsupported native computed coefficient %s type%s.',f{2},type);end
 else,kind='plain';expression=strtrim(expr{1});block='';end
 row=struct('type',f{1},'name',f{2},'count',count,'kind',kind,'expression',expression,'block',block);
 if isempty(map),map=row;else,map(end+1)=row;end
end
fid=fopen(fullfile(folder,'native_parameter_bank.c'),'w');assert(fid>=0);cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
fprintf(fid,'#include "acf_rl_native.h"\nconst P_acf_rl_native_T acf_parameter_bank[%d] = {\n',numel(configs));
values=cell(numel(configs),numel(map));
for k=1:numel(configs)
 p=acf_prepare(configs(k).p);assignin(get_param(mdl,'ModelWorkspace'),'p',p);fprintf(fid,' {\n');
 for j=1:numel(map)
  m=map(j);v=evalin(get_param(mdl,'ModelWorkspace'),m.expression);
  switch m.kind
   case 'sqrt',v=sqrt(v);
   case 'sparse_value',v=v(v~=0);
   case 'nonzero_value',v=v.';v=v(v~=0);
   case 'sparse_row',[row,~,~]=find(v);v=row-1;
   case 'sparse_col',v=[0,cumsum(sum(v~=0,1))];
  end
  assert(isreal(v)&&numel(v)==m.count,'Native coefficient shape mismatch for case%d %s.',k,m.name);values{k,j}=v(:);
  fprintf(fid,'  .%s = ',m.name);
  if m.count>1,fprintf(fid,'{');end
  for n=1:numel(v)
   if n>1,fprintf(fid,',');end
   if isinf(v(n)),if v(n)<0,fprintf(fid,'-');end;fprintf(fid,'INFINITY');
   elseif isnan(v(n)),fprintf(fid,'NAN');else,fprintf(fid,'%.17g',v(n));end
  end
  if m.count>1,fprintf(fid,'}');end;fprintf(fid,',\n');
 end
 fprintf(fid,' },\n');
end
fprintf(fid,'};\nconst int acf_parameter_bank_count=%d;\n',numel(configs));
save(fullfile(folder,'parameter_mapping.mat'),'map','values','-v7');write_json(fullfile(folder,'parameter_mapping.json'),map);
scenarios=struct([]);
for k=1:numel(configs)
 p=acf_prepare(configs(k).p);
 row=struct('scenario',k-1,'identity',configs(k).identity,'identifiedMassB_kg',p.uncertainty.massB_model_kg, ...
  'knownEquilibriumCurrent_A',(p.force.reference-(p.uncertainty.massB_model_kg+p.sensor.distalMass_model_kg)*p.d.g)/p.uncertainty.forceConstant_model_N_A, ...
  'identifiedRobotDelay_s',p.robotPrediction.delay,'identifiedRobotResponseTime_s',p.robotPrediction.responseTime, ...
  'currentResponse_s',p.coil.currentResponse,'identifiedAntiAliasCutoff_Hz',p.adcStudy.identifiedAnalogCutoff_Hz*p.acquisition.identifiedAntiAliasCutoff_Hz/5100, ...
  'identifiedEmaTime_s',p.adcStudy.identifiedEmaTime_s*p.acquisition.identifiedEmaTime_s/50e-6, ...
  'identifiedPipelineDelay_s',p.adcStudy.d.channels.frame.pipelineDelay_s,'actualRobotDelaySamples',p.d.robotDelaySamples, ...
  'identifiedRobotDelaySamples',p.robotPrediction.delaySamples,'seed',p.sensor.seed);
 if isempty(scenarios),scenarios=row;else,scenarios(end+1)=row;end
end
write_json(fullfile(folder,'scenarios.json'),scenarios);
assignin(get_param(mdl,'ModelWorkspace'),'p',acf_prepare(configs(1).p));
end

function write_json(path,value)
fid=fopen(path,'w');assert(fid~=-1);cleanup=onCleanup(@()fclose(fid));fprintf(fid,'%s\n',jsonencode(value,'PrettyPrint',true)); %#ok<NASGU>
end
