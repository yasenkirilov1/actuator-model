function report=normal_contact_trial_delivery_compact(config)
% Keep standard native Gain icons compact and remove empty layout strips.
if nargin<1,config=struct;end
state=fullfile(getenv('HOME'),'.local','state','actuator-model');
if ~isfield(config,'sourceFolder'),config.sourceFolder=fullfile(state,'inertial-delivery-private-v8-layoutb-20260930');end
if ~isfield(config,'workFolder'),config.workFolder=fullfile(state,'inertial-delivery-private-v8-layoutc-20260930');end
assert(~usejava('desktop')&&~strcmp(config.sourceFolder,config.workFolder));
work=config.workFolder;assert(~isfolder(work));mkdir(work);
for folder={'generated','helpers'},copyfile(fullfile(config.sourceFolder,folder{1}),fullfile(work,folder{1}));end
for file={'delivery_candidate.mat','delivery_verification.json','delivery_defaults_verification.json','layout_changes.json'}
 copyfile(fullfile(config.sourceFolder,file{1}),fullfile(work,file{1}));
end
for folder={'cache','code','sdi','traces','figures'},mkdir(fullfile(work,folder{1}));end
addpath(fullfile(work,'helpers'),'-begin');clear acf_prepare acf_select_actuator acf_select_controller;
mdl='acf_force_control_simple';load_system(fullfile(work,'generated',[mdl '.slx']));
cleanup=onCleanup(@()close_system(mdl,0)); %#ok<NASGU>
pages=find_system(mdl,'MatchFilter',@Simulink.match.allVariants,'LookUnderMasks','all','BlockType','SubSystem');
for j=1:numel(pages)
 % ContentPreviewEnabled is a supported editor parameter which R2026b does
 % not list in ObjectParameters. Set it directly rather than skipping it.
 set_param(pages{j},'ContentPreviewEnabled','off');
 if strcmp(pages{j},mdl),continue;end
 children=find_system(pages{j},'SearchDepth',1,'Type','block');children=setdiff(children,pages(j),'stable');
 if isempty(children),continue;end
 positions=zeros(numel(children),4);
 for k=1:numel(children)
  pos=get_param(children{k},'Position');cx=(pos(1)+pos(3))/2;cy=(pos(2)+pos(4))/2;
  type=get_param(children{k},'BlockType');
  if strcmp(type,'Gain'),pos=[cx-90,cy-30,cx+90,cy+30];
  elseif strcmp(type,'SubSystem')
   ports=get_param(children{k},'PortHandles');height=max(90,35*max(numel(ports.Inport),numel(ports.Outport)));
   pos=[cx-170,cy-height/2,cx+170,cy+height/2];
  end
  positions(k,:)=pos;
 end
 positions(:,[1 3])=compact_axis(positions(:,[1 3]),90);
 positions(:,[2 4])=compact_axis(positions(:,[2 4]),80);
 positions(:,[1 3])=positions(:,[1 3])-min(positions(:,1))+30;
 positions(:,[2 4])=positions(:,[2 4])-min(positions(:,2))+30;
 for k=1:numel(children),set_param(children{k},'Position',round(positions(k,:)));end
 lines=find_system(pages{j},'FindAll','on','SearchDepth',1,'Type','line');
 if ~isempty(lines),Simulink.BlockDiagram.routeLine(lines);end
end
set_param(mdl,'SimulationCommand','update');assert(isempty(Simulink.BlockDiagram.getAlgebraicLoops(mdl)));save_system(mdl);
report=struct('status','compacted; native history pairing required','workFolder',work,'sourceFolder',config.sourceFolder, ...
 'changes','Standard Gain blocks keep their full editable expressions and compact K icons; content previews disabled; unused horizontal/vertical strips removed without changing block order or equations');
fid=fopen(fullfile(work,'compact_layout.json'),'w');fprintf(fid,'%s\n',jsonencode(report,PrettyPrint=true));fclose(fid);
fprintf('DELIVERY NATIVE PAGES COMPACTED\n');
end
function value=compact_axis(value,targetGap)
intervals=sortrows(value,1);gaps=zeros(0,2);right=intervals(1,2);
for j=2:size(intervals,1)
 if intervals(j,1)>right+targetGap,gaps(end+1,:)=[right+targetGap,intervals(j,1)];end %#ok<AGROW>
 right=max(right,intervals(j,2));
end
original=value;
for j=1:size(gaps,1),value=value-max(0,min(gaps(j,2)-gaps(j,1),original-gaps(j,1)));end
end
