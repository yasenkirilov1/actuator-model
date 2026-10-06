function p=acf_prepare_upper_observer(p)
% Bounded Q/R design sweep; physical sensors and timing remain unchanged.
if ~isfield(p,'upperObserver') || ~p.upperObserver.enabled,return;end
o=p.upperObserver;dt=p.force.sampleTime;
m=o.identifiedMassA_kg;k=o.identifiedStiffness_N_m;c=o.identifiedDamping_Ns_m;
assert(m>0&&k>0&&c>=0&&~p.mechanics.upperRigid);
assert(o.forceProcessDensity_N_sqrtHz>0&&o.accelerationProcessDensity_m_s2_sqrtHz>0&&o.positionMeasurementRms_m>0);
% Cache only the deterministic coefficient design, never a physical state.
% All design inputs are in this key; per-case equilibrium and derivative
% coefficients are recomputed below. This leaves scalar arithmetic identical
% to the frozen v3 design for matching Q/R inputs.
persistent previousKey previousDesign
key=[dt,m,k,c,o.forceProcessDensity_N_sqrtHz,o.accelerationProcessDensity_m_s2_sqrtHz,o.positionMeasurementRms_m,o.identifiedCurrentResponse_s];
if isempty(previousKey)||~isequal(key,previousKey)
 F=[0,1,0,0;-k/m,-c/m,0,0;0,0,0,1;-k/m,-c/m,0,0];
 G=[0,0,0;1/m,-1,0;0,0,0;1/m,0,-1];H=[0,0,1,0];
 z=expm([F,G;zeros(3,7)]*dt);Ad=z(1:4,1:4);Bd=z(1:4,5:7);
 Ad(1:2,3:4)=0;Bd(1:2,3)=0;
 ob=zeros(4);for j=1:4,ob(j,:)=H*Ad^(j-1);end
 scale=diag([1e-3,.1,1e-3,.1]);scaledOb=ob*scale;
 observabilityRank=rank(scaledOb,1e-12);assert(observabilityRank==4);
 % These densities and R are algorithm design weights, not new sensor specs.
 W=[0,0;1/m,0;0,0;1/m,-1];
 Qc=W*diag([o.forceProcessDensity_N_sqrtHz^2,o.accelerationProcessDensity_m_s2_sqrtHz^2])*W';
 v=expm([F,Qc;zeros(4),-F']*dt);Qd=v(1:4,5:8)*Ad';Qd=(Qd+Qd')/2;
 R=o.positionMeasurementRms_m^2;P=diag([1e-6,.01,1e-6,.01]);
 for iteration=1:200000
  predicted=Ad*P*Ad'+Qd;L=predicted*H'/(H*predicted*H'+R);
  J=eye(4)-L*H;next=J*predicted*J'+L*R*L';next=(next+next')/2;
  if norm(next-P,'fro')<1e-14*max(1,norm(P,'fro')),P=next;break;end
  P=next;
 end
 assert(iteration<200000,'Upper observer Riccati iteration did not settle.');
 predicted=Ad*P*Ad'+Qd;L=predicted*H'/(H*predicted*H'+R);J=eye(4)-L*H;
 A=Ad*J;B=[Ad*L,Bd];C=J;D=[L,zeros(4,3)];poles=eig(A);
 assert(max(abs(poles))<1&&all(isfinite([A(:);B(:);C(:);D(:)])));
 previousDesign=struct('A',A,'B',B,'C',C,'D',D,'gain',L, ...
  'continuousF',F,'continuousG',G,'measurementC',H,'discreteF',Ad,'discreteG',Bd, ...
  'processCovariance',Qd,'measurementVariance',R,'posteriorCovariance',P, ...
  'estimatorPolesReal',real(poles),'estimatorPolesImag',imag(poles), ...
  'maximumPoleMagnitude',max(abs(poles)),'observabilityRank',observabilityRank, ...
  'scaledObservabilityCondition',cond(scaledOb),'riccatiIterations',iteration, ...
  'stateNames',{{'A_minus_R_m','vA_minus_vR_m_s','A_minus_S_m','vA_minus_vS_m_s'}}, ...
  'inputNames',{{'measured_A_minus_S_m','aligned_delta_actuator_force_N','aligned_robot_acceleration_m_s2','measured_S_acceleration_m_s2'}}, ...
  'forcePredictionA',-1/o.identifiedCurrentResponse_s,'forcePredictionB',1/o.identifiedCurrentResponse_s, ...
  'forcePredictionC',1,'forcePredictionD',0, ...
  'accelerationFromStateGain',[-k/m,-c/m,0,0],'accelerationFromForceGain',1/m);
 previousKey=key;
end
p.d.upperMotionObserver=previousDesign;
p.d.upperMotionObserver.forcePredictionInitial_A=(p.force.reference-(p.uncertainty.massB_model_kg+p.sensor.distalMass_model_kg)*p.d.g)/p.uncertainty.forceConstant_model_N_A;
assert(strcmp(o.forceEstimator,'local_matched_cell_and_holder_force_balance'));
assert(p.mechanics.c1==0);
mb=p.uncertainty.massB_model_kg;k1=p.inertial.lowerLoadInverse.identifiedStiffness_N_m;
assert(abs(mb-p.inertial.lowerLoadInverse.identifiedMassB_kg)<1e-12&&mb>0&&k1>0);
wc=2*pi*o.forceDerivativeCutoff_Hz;
p.d.upperMotionObserver.measuredForceDerivativeA=[0,1;-wc^2,-2*wc];
p.d.upperMotionObserver.measuredForceDerivativeB=[0;wc^2];
p.d.upperMotionObserver.measuredForceDerivativeC=[-wc^2,-2*wc];
p.d.upperMotionObserver.measuredForceDerivativeD=wc^2;
p.d.upperMotionObserver.measuredForceDerivativeScale=mb/k1;
p.d.upperMotionObserver.identifiedHolderMass_kg=mb;
p.d.upperMotionObserver.identifiedCellStiffness_N_m=k1;
end
