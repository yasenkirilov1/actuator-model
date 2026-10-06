function prepare_and_build_cell_modal_lead_v7(bankFolder,buildFolder)
prepare_cell_modal_lead_bank_v7(bankFolder);
build_upper_force_native_modal_lead_v7(buildFolder,fullfile(bankFolder,'configs.mat'));
end
