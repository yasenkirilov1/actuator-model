function p=acf_prepare(p)
% Reuse exact physical/observer prep; add only modal-controller coefficients.
p=acf_adc_prepare(p);
p=acf_prepare_upper_observer(p);
p=acf_prepare_cell_modal(p);
end
