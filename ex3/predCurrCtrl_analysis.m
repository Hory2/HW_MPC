% Analysis of the predictions and the minimization step of a predictive 
% current controller for a three-phase grid system with a prediction 
% horizon of one step without (or with) delay compensation
%
% Model Predictive Control of Power Electronic Systems, TAU, Tampere
% Petros Karamanakos, September 2025

%% generate one problem instance at time step k

% omega * time
wt = Par.Base.w * 0.005;

% current reference
phi = 10*pi/180;
ikp1_ref_abc = Par.Base.I * [sin(wt+phi); sin(wt+phi-2*pi/3); sin(wt+phi-4*pi/3)]; 

% current measurement
ik_abc = Par.Base.I * [0.9; -0.5; -0.9+0.5];

% grid voltage measurement
wt = Par.Base.w * 0.005;
vgk_abc = Par.Base.V * [sin(wt); sin(wt-2*pi/3); sin(wt-4*pi/3)]; 

% previous switch position
ukm1_abc = [1; 1; 0];   	


%% call the predictive controller

inVec = [ikp1_ref_abc; ik_abc; vgk_abc; ukm1_abc];
figure(1); clf; 

uk_abc = currCtrl(inVec);