function evidence=verify_native_package
%VERIFY_NATIVE_PACKAGE Run saved-model checks in an isolated MATLAB batch.
assert(~usejava('desktop'),'Run verify_native_package with MATLAB -batch.');
root=fileparts(fileparts(mfilename('fullpath')));addpath(root);
storage=tempname;mkdir(storage);
previousStorage=Simulink.sdi.getStorageLocation;
previousRecord=Simulink.sdi.getRecordData;
Simulink.sdi.setStorageLocation(storage);Simulink.sdi.setRecordData(true);
cleanup=onCleanup(@()restore_storage(storage,previousStorage,previousRecord));
evidence=qualify_saved_model(acf_open);
end

function restore_storage(owned,previous,record)
if strcmp(Simulink.sdi.getStorageLocation,owned),Simulink.sdi.clear;end
Simulink.sdi.setStorageLocation(previous);Simulink.sdi.setRecordData(record);
end
