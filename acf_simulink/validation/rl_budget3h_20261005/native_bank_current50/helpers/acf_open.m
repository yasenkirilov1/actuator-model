function mdl = acf_open
%ACF_OPEN Open the authoritative saved diagram, preserving loaded block edits.
folder=fileparts(mfilename('fullpath'));addpath(folder);
mdl='acf_force_control_simple';
filename=fullfile(folder,'generated',[mdl '.slx']);
if ~bdIsLoaded(mdl)
    assert(isfile(filename),'ACF:MissingModel', ...
        'The saved Simulink model is missing: %s. Restore the SLX file.',filename);
    load_system(filename);
end
if acf_desktop_available
    open_system(mdl);set_param(mdl,'ZoomFactor','FitSystem');
end
end
