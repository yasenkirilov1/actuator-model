function contract=install_modal_guide_smoothing_v7(mdl,p)
% Only controller Coulomb feedforward smoothing changes; plant stays fixed.
assert(p.coil.coulombForce==.2&&p.coil.viscousFriction==2&&p.coil.frictionSpeed==.0002);
sys=[mdl '/Force controller/Coil PI and PID/Measured motion feedforward'];
block=[sys '/Guide friction velocity scale'];
assert(strcmp(get_param(block,'Gain'),'1/p.coil.frictionSpeed'));
set_param(block,'Gain','1/p.cellModalLead.guideFrictionFeedforwardSpeed_m_s');
contract=struct('controllerOnly',true,'coulombCompensationScale',1, ...
 'smoothingSpeed_m_s',p.cellModalLead.guideFrictionFeedforwardSpeed_m_s, ...
 'physicalGuideFrictionSpeed_m_s',p.coil.frictionSpeed,'physicalFrictionUnchanged',true, ...
 'observerAndForceReconstructionUnchanged',true,'soleBlock','Measured motion feedforward/Guide friction velocity scale');
end
