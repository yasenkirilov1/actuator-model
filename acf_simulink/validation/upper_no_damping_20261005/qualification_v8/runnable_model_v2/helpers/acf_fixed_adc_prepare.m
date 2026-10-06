function p=acf_fixed_adc_prepare(p)
%ACF_ADC_PREPARE Private continuous-front-end acquisition study coefficients.
% The physical plant/controller are the frozen V8 implementation. Sensor
% noise is fixed one-sided PSD at an explicitly fixed integration clock;
% converter output noise and quantization are independent of that PSD.
if ~isfield(p,'adcStudy'),p.adcStudy=struct;end
defaults=struct('version','continuous_analog_native_adc_20261002', ...
 'fineClock_s',1/300000,'outputRate_Hz',100000,'converter','sar', ...
 'modulatorRate_Hz',3840000,'analogCutoff_Hz',5100, ...
 'converterAnalogOrder',[],'converterAnalogCutoff_Hz',16000, ...
 'identifiedAnalogCutoff_Hz',[],'analogDamping',1/sqrt(2), ...
 'emaTime_s',50e-6,'identifiedEmaTime_s',[], ...
 'pipelineDelay_s',[],'identifiedPipelineDelay_s',[],'accelerationExtraDelay_s',0, ...
 'forceQuantum_N',320/2^16,'accelerationQuantum_m_s2',5/2^16/.04*9.80665, ...
 'forceAdcNoiseRms_N',0,'accelerationAdcNoiseRms_m_s2',0,'adcNoiseCorrelationTime_s',0, ...
 'forceAdcRange_N',160,'accelerationAdcRange_g',62.5, ...
 'forceAnalogToneAmplitude_N',0,'lateralAnalogToneAmplitude_N',0, ...
 'accelerationZAnalogToneAmplitude_m_s2',0,'accelerationXAnalogToneAmplitude_m_s2',0, ...
 'analogToneFrequency_Hz',10000,'analogTonePhase_rad',0);
for name=fieldnames(defaults)',if ~isfield(p.adcStudy,name{1}),p.adcStudy.(name{1})=defaults.(name{1});end,end
s=p.adcStudy;h=s.fineClock_s;assert(isfinite(h)&&h>0&&p.sim.step<=h);
assert(all(isfinite([s.outputRate_Hz,s.modulatorRate_Hz,s.analogCutoff_Hz,s.analogDamping,s.emaTime_s])));
assert(s.outputRate_Hz>0&&s.analogCutoff_Hz>0&&s.analogDamping>0&&s.emaTime_s>=0);
assert(any(strcmp(s.converter,{'sar','sinc3'})),'Use sar or sinc3.');
assert(abs(1/s.outputRate_Hz/h-round(1/s.outputRate_Hz/h))<1e-8, ...
 'The exact ADC output period must divide the fine clock; do not silently round 32k to 30k.');
assert(abs(p.force.sampleTime/h-round(p.force.sampleTime/h))<1e-8);
assert(abs(h/p.sim.step-round(h/p.sim.step))<1e-8);
if isempty(s.identifiedAnalogCutoff_Hz),s.identifiedAnalogCutoff_Hz=s.analogCutoff_Hz;end
if isempty(s.identifiedEmaTime_s),s.identifiedEmaTime_s=s.emaTime_s;end
if isempty(s.converterAnalogOrder),s.converterAnalogOrder=3*strcmp(s.converter,'sar');end
assert(any(s.converterAnalogOrder==[0,3]));
actualPipeline=p.acquisition.pipelineDelay_s;
identifiedPipeline=p.acquisition.identifiedPipelineDelay_s;
if ~isempty(s.pipelineDelay_s),actualPipeline=s.pipelineDelay_s;end
if ~isempty(s.identifiedPipelineDelay_s),identifiedPipeline=s.identifiedPipelineDelay_s;end
actualOffsets=[p.acquisition.forceDelayOffset_s,p.acquisition.lateralDelayOffset_s, ...
 p.acquisition.accelerationDelayOffset_s,p.acquisition.positionDelayOffset_s];
% The frozen preparer only computes legacy integer delays. Those blocks are
% bypassed in this fork; the active physical pipeline uses Transport Delay.
p.acquisition.rawSampleTime_s=h;p.acquisition.pipelineDelay_s=h;
p.acquisition.identifiedPipelineDelay_s=h;
p.acquisition.forceDelayOffset_s=0;p.acquisition.lateralDelayOffset_s=0;
p.acquisition.accelerationDelayOffset_s=0;p.acquisition.positionDelayOffset_s=0;
p=acf_fixed_prepare_baseline_v8(p);
p.acquisition.pipelineDelay_s=actualPipeline;p.acquisition.identifiedPipelineDelay_s=identifiedPipeline;
p.acquisition.forceDelayOffset_s=actualOffsets(1);p.acquisition.lateralDelayOffset_s=actualOffsets(2);
p.acquisition.accelerationDelayOffset_s=actualOffsets(3);p.acquisition.positionDelayOffset_s=actualOffsets(4);
s.outputSampleTime_s=1/s.outputRate_Hz;
assert(isfinite(s.adcNoiseCorrelationTime_s)&&s.adcNoiseCorrelationTime_s>=0);
if s.adcNoiseCorrelationTime_s==0,noiseAlpha=0;
else,noiseAlpha=exp(-s.outputSampleTime_s/s.adcNoiseCorrelationTime_s);end
s.d.converterNoise=struct('numerator',[sqrt(1-noiseAlpha^2),0], ...
 'denominator',[1,-noiseAlpha],'unitSteadyStateOutputRMS',1,'correlationTime_s',s.adcNoiseCorrelationTime_s);
assert(all(isfinite([s.forceQuantum_N,s.accelerationQuantum_m_s2,s.forceAdcNoiseRms_N,s.accelerationAdcNoiseRms_m_s2])));
assert(all([s.forceQuantum_N,s.accelerationQuantum_m_s2,s.forceAdcNoiseRms_N,s.accelerationAdcNoiseRms_m_s2]>=0));
p.accel.quantum=max(s.accelerationQuantum_m_s2,1e-15);
if s.converterAnalogOrder==3
 w=2*pi*s.converterAnalogCutoff_Hz;
 s.d.converterAnalog=struct('A',[0,1,0;0,0,1;-w^3,-2*w^2,-2*w], ...
  'B',[0;0;w^3],'C',[1,0,0],'D',0,'steadyStatePerUnitInput',[1;0;0], ...
  'order',3,'cutoff_Hz',s.converterAnalogCutoff_Hz,'DCGroupDelay_s',2/w);
else
 s.d.converterAnalog=struct('A',-1,'B',0,'C',0,'D',1, ...
  'steadyStatePerUnitInput',0,'order',0,'cutoff_Hz',Inf,'DCGroupDelay_s',0);
end
% Baseline h0=10us, response energy E0, and its fixed ENBW are immutable.
referenceEnergy=.06354253123515036;referenceClock=10e-6;
referenceENBW=referenceEnergy/(2*referenceClock);
s.d.referenceNoiseClock_s=referenceClock;
noise=struct('referenceNoiseClock_s',referenceClock,'referenceNoiseImpulseEnergy',referenceEnergy, ...
 'referenceNoiseENBW_Hz',referenceENBW,'forceNoiseDensity_N_sqrtHz',p.sensor.noiseRms/sqrt(referenceENBW), ...
 'lateralNoiseDensity_N_sqrtHz',p.contact.lateralNoiseRms_N/sqrt(referenceENBW), ...
 'accelerationNoiseDensity_m_s2_sqrtHz',p.accel.noiseRms/sqrt(referenceENBW), ...
 'positionNoiseDensity_m_sqrtHz',p.sensor.strokeNoiseRms/sqrt(referenceENBW));
for name=fieldnames(noise)',s.d.(name{1})=noise.(name{1});end
s.d.forceNoiseInputRms_N=s.d.forceNoiseDensity_N_sqrtHz/sqrt(2*h);
s.d.lateralNoiseInputRms_N=s.d.lateralNoiseDensity_N_sqrtHz/sqrt(2*h);
s.d.accelerationNoiseInputRms_m_s2=s.d.accelerationNoiseDensity_m_s2_sqrtHz/sqrt(2*h);
s.d.positionNoiseInputRms_m=s.d.positionNoiseDensity_m_sqrtHz/sqrt(2*h);
if strcmp(s.converter,'sinc3')
 osr=s.modulatorRate_Hz/s.outputRate_Hz;
 assert(abs(osr-round(osr))<1e-9&&osr>=2,'SINC3 OSR must be an actual integer at the stated modulator clock.');
 osr=round(osr);box=ones(1,osr)/osr;bMod=conv(conv(box,box),box);
 fineBins=round((0:numel(bMod)-1)/(s.modulatorRate_Hz*h));
 numerator=accumarray(fineBins(:)+1,bMod(:),[],@sum)';
 exactDelay=3*(osr-1)/(2*s.modulatorRate_Hz);
else
 % An explicit cancelled z=0 state avoids the block's empty-initial-state
 % ambiguity while retaining the exact identity response for a SAR.
 osr=1;bMod=1;numerator=[1,0];exactDelay=0;
end
denominator=[1,zeros(1,numel(numerator)-1)];
s.d.converter=struct('numerator',numerator,'denominator',denominator, ...
 'steadyStatePerUnitInput',ones(numel(numerator)-1,1),'oversamplingRatio',osr, ...
 'exactModulatorImpulseLength',numel(bMod),'fineImpulseLength',numel(numerator), ...
 'exactGroupDelay_s',exactDelay,'fineGroupDelay_s',sum((0:numel(numerator)-1).*numerator)*h);
% Audit the polyphase delay-bin approximation against the manufacturer's
% exact modulator-clock kernel. It is not a fictitious 30k converter.
frequencies=linspace(0,min(15000,.49*s.outputRate_Hz),1001);
exact=exp(-2i*pi*frequencies(:)*(0:numel(bMod)-1)/s.modulatorRate_Hz)*bMod(:);
fine=exp(-2i*pi*frequencies(:)*(0:numel(numerator)-1)*h)*numerator(:);
band=frequencies<=5000;
s.d.converter.responseApproximation=struct('evaluatedFrequency_Hz',frequencies, ...
 'exactReal',real(exact)','exactImag',imag(exact)', ...
 'fineReal',real(fine)','fineImag',imag(fine)', ...
 'maximumComplexErrorThrough5k',max(abs(fine(band)-exact(band))), ...
 'maximumRelativeGainErrorThrough5k',max(abs(abs(fine(band))./max(abs(exact(band)),eps)-1)), ...
 'groupDelayError_s',s.d.converter.fineGroupDelay_s-exactDelay, ...
 'interpretation','Exact CIC/SINC3 coefficients aggregated into causal fine-clock delay bins. Refine selected native cases and this response audit.');
channels={'force','lateral','acceleration','position','frame'};
for k=1:numel(channels)
 channel=channels{k};
 if strcmp(channel,'frame')
  cutoff=s.identifiedAnalogCutoff_Hz*p.acquisition.identifiedAntiAliasCutoff_Hz/5100;
  damping=1/sqrt(2);ema=s.identifiedEmaTime_s*p.acquisition.identifiedEmaTime_s/50e-6;
  pipeline=identifiedPipeline;
 else
  cutoff=s.analogCutoff_Hz*p.acquisition.antiAliasCutoff_Hz/5100*p.acquisition.([channel 'AntiAliasScale']);
  damping=s.analogDamping;ema=s.emaTime_s*p.acquisition.emaTime_s/50e-6;
  pipeline=actualPipeline+actualOffsets(k);
  if strcmp(channel,'acceleration'),pipeline=pipeline+s.accelerationExtraDelay_s;end
 end
 assert(cutoff>0&&cutoff<.4/h&&pipeline>=0&&ema>=0);
 w=2*pi*cutoff;A=[0,1;-w*w,-2*damping*w];B=[0;w*w];C=[1,0];D=0;
 if ema==0,alpha=0;else,alpha=exp(-s.outputSampleTime_s/ema);end
 s.d.channels.(channel)=struct('analogA',A,'analogB',B,'analogC',C,'analogD',D, ...
  'analogSteadyStatePerUnitInput',-A\B,'cutoff_Hz',cutoff,'damping',damping, ...
  'emaNumerator',[1-alpha,0],'emaDenominator',[1,-alpha], ...
  'emaSteadyStatePerUnitInput',1/(1-alpha),'emaTime_s',ema,'pipelineDelay_s',pipeline, ...
  'analogDCGroupDelay_s',2*damping/w,'emaDCGroupDelay_s',alpha/(1-alpha)*s.outputSampleTime_s);
end
s.d.forceGravityOffset_N=p.sensor.distalMass_kg*p.d.g;
s.d.lateralGravityOffset_N=p.sensor.distalMass_kg*p.mechanics.gravity*sin(p.mechanics.angle);
s.d.scope='Private acquisition fork: native continuous analog conditioning before ADC sampling; fixed sensor PSD; independent ADC output noise/quantization; causal identified response for issued commands; unchanged physical V8 plant/controller. No physical hardware qualification.';
p.adcStudy=s;
end
