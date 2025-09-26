% Initialization file for a predictive current controller for a 
% single-phase system with a prediction horizon of one step
%
% Model Predictive Control of Power Electronic Systems, TAU, Tampere
% Petros Karamanakos, September 2025

global Par

% sampling intervals
Par.Ts.control = 25e-6; % sampling interval of controller [s] 
Par.Ts.system = 5e-6;   % sampling interval of power electronic system [s]

% system parameters (in SI)
Par.SI.L = 2e-3;        % load inductance [H]
Par.SI.R = 2;           % load resistance [Ohm]
Par.SI.Vpeak = sqrt(2/3)*3300;  % peak phase voltage [V]
Par.SI.f0 = 50;         % fundamental grid frequency [Hz]
Par.SI.Vdc = 5200;      % total dc-link voltage [V] 

% per unit system
Par.Base.w =2*pi()*Par.SI.f0                % base angular frequency 

Par.Base.V =Par.SI.Vpeak                % base voltage [V]
Par.Base.I =908.8339*sqrt(2)        % base current [A]
Par.Base.Z =Par.Base.V/Par.Base.I                % base load impedance [Ohm]
% ----UNCOMMENT THIS BLOCK ONCE EXERCISE 2.1.5 HAS BEEN COMPLETED----
% continuous-time system model
F = -Par.SI.R/Par.SI.L	
G = 1/Par.SI.L 	

% discrete-time system model (forward Euler)
Par.Ctr.A = 1+F*Par.Ts.control	
Par.Ctr.B = G*Par.Ts.control	

Par.Ctr.lambda_u = 5e-3;         	% tuning parameter

