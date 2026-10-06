function prepare_and_build_observer_noise_v4(bankFolder,buildFolder)
% One private batch process; the guard runs only numerical wiring harnesses.
prepare_observer_noise_bank_v4(bankFolder);
build_upper_force_native_noise_v4(buildFolder,fullfile(bankFolder,'configs.mat'));
end
