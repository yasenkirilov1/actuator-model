function p=acf_fixed_prepare(p)
% Reuse exact physical/observer prep; add only modal-controller coefficients.
p=acf_fixed_adc_prepare(p);
p=acf_fixed_prepare_upper_observer(p);
p=acf_fixed_prepare_cell_modal(p);
end
