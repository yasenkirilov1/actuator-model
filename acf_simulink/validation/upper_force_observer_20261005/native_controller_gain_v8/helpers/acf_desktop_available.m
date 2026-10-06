function tf = acf_desktop_available
%ACF_DESKTOP_AVAILABLE Detect the desktop without requiring Java in R2026b.
% The new desktop can be running while usejava('desktop') returns false.
if isMATLABReleaseOlderThan('R2026b')
    tf=usejava('desktop');
else
    tf=desktop('inuse');
end
end
