function prepare_and_build_cell_modal_v6(bankFolder,buildFolder,phaseFile)
prepare_cell_modal_bank_v6(bankFolder,phaseFile);
build_upper_force_native_modal_v6(buildFolder,fullfile(bankFolder,'configs.mat'));
end
