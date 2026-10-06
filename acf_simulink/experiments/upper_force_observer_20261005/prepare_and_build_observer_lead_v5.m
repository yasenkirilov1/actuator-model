function prepare_and_build_observer_lead_v5(bankFolder,buildFolder)
prepare_observer_lead_bank_v5(bankFolder);
build_upper_force_native_lead_v5(buildFolder,fullfile(bankFolder,'configs.mat'));
end
