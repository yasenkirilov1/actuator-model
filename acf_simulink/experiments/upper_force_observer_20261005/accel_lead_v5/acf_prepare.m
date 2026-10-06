function p=acf_prepare(p)
% The physical preparer and observer are unchanged; add only current-FF lead.
p=acf_adc_prepare(p);
p=acf_prepare_upper_observer(p);
p=acf_prepare_accel_lead(p);
end
