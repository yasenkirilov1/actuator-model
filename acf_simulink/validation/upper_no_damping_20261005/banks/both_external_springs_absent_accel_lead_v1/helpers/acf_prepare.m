function p=acf_prepare(p)
p=acf_adc_prepare(p);
p=acf_prepare_upper_observer(p);
p=acf_prepare_accel_lead(p);
end
