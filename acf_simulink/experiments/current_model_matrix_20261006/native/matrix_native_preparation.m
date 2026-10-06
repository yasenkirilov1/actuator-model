function report=matrix_native_preparation(root,temp,out)
% Graph-only check: no physical/controller/acquisition recordings execute.
assert(~usejava('desktop'));maxNumCompThreads(2);
assert(isfile(fullfile(temp,'owned_matrix_native')));
here=fullfile(root,'acf_simulink','experiments','current_model_matrix_20261006','native');addpath(here);
base=fullfile(root,'acf_simulink','experiments','sea_pad_dynamic_20261006','native');addpath(base);
assert(~isempty(which('load_system'))&&~isempty(ver('simulink')));
sourceFiles={'matrix_native_build.m','run_matrix_native.m','matrix_native_preparation.m'};
warnings=struct([]);
for j=1:numel(sourceFiles)
 items=checkcode(fullfile(here,sourceFiles{j}),'-id');
 warnings(j).file=sourceFiles{j};warnings(j).items=items;
end
spec=jsondecode(fileread(fullfile(fileparts(here),'initial_banks.json')));rows=struct([]);
for mode=1:2
 c=spec.banks(1).config;if mode==2,c.pad_k=0;c.pad_thickness=0;c.pad_compression_limit=0;end
 p=sea_pad_native_from_engine(fullfile(fileparts(base),'study_contract.json'),c);
 p.rigidWide=p.rigidPoint&&p.padDiameter>0;p.eventEpsilon=1e-12;
 modelFile=fullfile(temp,sprintf('acf_matrix_graphcheck_%d.slx',mode));
 prepared=matrix_native_build(p,modelFile,temp);[~,mdl]=fileparts(modelFile);load_system(modelFile);
 cleanup=onCleanup(@()close_loaded(mdl)); %#ok<NASGU>
 set_param(mdl,'SimulationCommand','update');loops=numel(Simulink.BlockDiagram.getAlgebraicLoops(mdl));assert(loops==0);
 integrators=find_system(mdl,'LookUnderMasks','all','BlockType','Integrator');assert(numel(integrators)==9);
 functions=find_system(mdl,'LookUnderMasks','all','MaskType','MATLAB Function');assert(isempty(functions));
 directSubsystems=find_system(mdl,'SearchDepth',1,'BlockType','SubSystem');assert(numel(directSubsystems)==6);
 save_system(mdl,modelFile);close_system(mdl,0);load_system(modelFile);set_param(mdl,'SimulationCommand','update');
 reopenedLoops=numel(Simulink.BlockDiagram.getAlgebraicLoops(mdl));assert(reopenedLoops==0);
 mw=get_param(mdl,'ModelWorkspace');assert(isequaln(mw.getVariable('p'),p));
 if mode==2
  set_param(mdl,'PaperOrientation','landscape');print(['-s' mdl],'-dpng',fullfile(temp,'rigid_native_top.png'));
  print(['-s' mdl '/Pad total force'],'-dpng',fullfile(temp,'rigid_native_contact.png'));
 end
 close_system(mdl,0);clear cleanup;
 row=struct('rigidWide',p.rigidWide,'algebraicLoopCount',loops,'reopenedAlgebraicLoopCount',reopenedLoops, ...
  'ordinaryIntegratorCount',numel(integrators),'shallowSubsystemCount',numel(directSubsystems), ...
  'hiddenMatlabPlantFunctionCount',numel(functions),'graphPrepared',prepared,'savedReopened',true);
 if isempty(rows),rows=row;else,rows(end+1)=row;end %#ok<AGROW>
end
report=struct('status','native_graphs_prepared_reopened_no_simulation','simulationsExecuted',0, ...
 'qualification',false,'graphs',rows,'matlabCodeDiagnostics',warnings,'loadSystemPath',which('load_system'), ...
 'matlabVersion',version,'simulinkVersion',ver('simulink'));
fid=fopen(fullfile(out,'preparation_checks.json'),'w');assert(fid>=0);fprintf(fid,'%s\n',jsonencode(report,'PrettyPrint',true));fclose(fid);
fprintf('MATRIX_NATIVE_GRAPH_CHECK_DONE graphs=%d simulations=0\n',numel(rows));
end
function close_loaded(mdl),try,if bdIsLoaded(mdl),close_system(mdl,0);end,catch,end,end
