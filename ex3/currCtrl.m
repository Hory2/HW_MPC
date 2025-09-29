function outVec = currCtrl(inVec)

% Predictive current controller for a three-phase grid system with a 
% prediction horizon of one step
%
% Model Predictive Control of Power Electronic Systems, TAU, Tampere
% Petros Karamanakos, September 2025

global Par

% interface
ikp1_ref_abc = inVec(1:3);	% three-phase current reference at time step k+1
ik_abc = inVec(4:6);        % three-phase current measurement at time step k
vgk_abc = inVec(7:9);    	% three-phase grid voltage measurement at time step k
ukm1_abc = inVec(10:12);   	% previous three-phase switch position applied at time step k-1

% turn the inputs into the per unit system and the stationary orthogonal 
% reference frame
% **** to be added ****

% implement the predictive current controller
% **** to be added ****
    
% for the chosen switch position: 
% compute the predicted current in pu and alpha/beta at the next time step
ikp1 = [0; 0];  % **** to be added ****

% output interface
outVec = [uk_abc; ikp1];
    
return