function p=acf_prepare(p)
% Exact physical/observer prep plus frozen-identified modal and FF lead.
p=acf_adc_prepare(p);
p=acf_prepare_upper_observer(p);
p=acf_prepare_cell_modal(p);
p=acf_prepare_accel_lead(p);
end
