function prepare_and_build(bankFolder,buildFolder)
prepare_controller_gain_bank(bankFolder);
build_upper_force_native_controller_gain_v9(buildFolder,fullfile(bankFolder,'configs.mat'));
end
