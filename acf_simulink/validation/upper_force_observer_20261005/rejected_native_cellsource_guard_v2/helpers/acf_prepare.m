function p=acf_prepare(p)
% Private dispatcher; retain the native frozen physical/acquisition preparer.
p=acf_adc_prepare(p);
p=acf_prepare_upper_observer(p);
end
