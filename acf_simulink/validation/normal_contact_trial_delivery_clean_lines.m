function report=normal_contact_trial_delivery_clean_lines(work)
% Remove only destinationless line segments. No compilation or simulation.
if nargin<1,work=fullfile(getenv('HOME'),'.local','state','actuator-model','inertial-delivery-private-v8-layoutd-20260930');end
assert(~usejava('desktop'));mdl='acf_force_control_simple';
file=fullfile(work,'generated',[mdl '.slx']);
original=fullfile(work,'generated','acf_force_control_simple_before_line_cleanup.slx');
assert(~isfile(original),'Keep the saved metadata bridge and reviewed original immutable.');
copyfile(file,original);
load_system(file);cleanup=onCleanup(@()close_own_model(mdl)); %#ok<NASGU>
removed=0;
while true
 lines=find_system(mdl,'FindAll','on','LookUnderMasks','all', ...
  'MatchFilter',@Simulink.match.allVariants,'Type','line');
 dangling=[];
 for k=1:numel(lines)
  try
   children=get_param(lines(k),'LineChildren');dst=get_param(lines(k),'DstPortHandle');
  catch,continue;end
  if isempty(children)&&(isempty(dst)||all(dst==-1)),dangling=lines(k);break;end
 end
 if isempty(dangling),break;end
 delete_line(dangling);removed=removed+1;assert(removed<100,'Unexpected line population.');
end
assert(removed>0,'Expected the reviewed destinationless stubs.');
save_system(mdl);close_system(mdl,0);
root=fileparts(mfilename('fullpath'));
script=fullfile(root,'normal_contact_trial_delivery_line_audit.py');
output=fullfile(work,'line_cleanup_bridge.json');
for s={script,original,file,output},assert(~contains(string(s{1}),"'"));end
[status,message]=system(sprintf('python3 ''%s'' ''%s'' ''%s'' ''%s''',script,original,file,output));
if status~=0
 copyfile(original,file,'f');error('The layout audit failed; restored the metadata-only model: %s',message);
end
report=jsondecode(fileread(output));report.removedNativeLineSegments=removed;
fid=fopen(output,'w');assert(fid>=0);fprintf(fid,'%s\n',jsonencode(report,PrettyPrint=true));fclose(fid);
fprintf('DELIVERY DESTINATIONLESS LINE CLEANUP COMPLETE: %d SEGMENTS; NO SIMULATION\n',removed);
end
function close_own_model(mdl)
if bdIsLoaded(mdl),close_system(mdl,0);end
end
