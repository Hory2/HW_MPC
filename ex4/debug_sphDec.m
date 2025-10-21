% Debugging of the sphere decoder 

% Model Predictive Control of Power Electronic Systems, TAU, Tampere
% Petros Karamanakos, September 2025

% set the general system and control parameter
driveIni

% input parameter
xk = [0.4; 0.96; 0.8895; -0.1933];      % machine state
u_km1 = zeros(3,1);                     % previous switch position
Np = Par.Ctr.Np;                        % prediction horizon

% current reference
if Np == 2
    % current reference at k+1 and k+2 in alpha/beta
    Yref = [0.3591; 0.9390; 0.3295; 0.9498];
elseif Np == 3
    % current reference at k+1, k+2 and k+3 in alpha/beta
    Yref = [0.3591; 0.9390; 0.3295; 0.9498; 0.2995; 0.9597]; 
else
    error('not available')
end
    
% load the problem matrices
Gamma = Par.Prob.Gamma;
Ypsilon = Par.Prob.Ypsilon;
Qtilde = Par.Prob.Qtilde;
E = Par.Prob.E;
S = Par.Prob.S;
Hinv = Par.Prob.Hinv;
V = Par.Prob.V;

% Theta matrix
Theta_a = -(Yref-Gamma*xk)'*Qtilde*Ypsilon;     % (5.13b)
Theta_b = -Par.Ctr.lambda*(E*u_km1)'*S;        	% (5.13b)
Theta = (Theta_a + Theta_b)';                   % 3Np x 1, (5.13b)

% unconstrained solution for Uk over k,...,k+Np-1
Uk_unc = -Hinv * Theta;                         % 3Np x 1, (5.16)
Ubar_unc = V * Uk_unc;      % unconstrained solution in V-space
                            
% initial solution
Uk_ini = zeros(3*Np,1);  
Ubar_ini = V * Uk_ini;      % initial solution in V-space

% upper bound on the squared distance
rho = norm(Ubar_unc-Ubar_ini,2)^2;    % we use the *squared* distance / 2-norm here
rho = rho * (1+1e-9);       % we do this to avoid numerical problems and not finding any solution

% call the sphere decoding algorithm
[~, Uk, ~, searchFlag] = sphDec(Uk_ini, [], 0, 1, 3*Np, rho, Ubar_unc, V, 0, u_km1);
if searchFlag == 0
    error('Sphere decoder did not find a solution'); 
else
    fprintf('The sphere decoder required %i iterations\n', searchFlag);
end