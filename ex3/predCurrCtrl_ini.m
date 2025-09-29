% Initialization file for a predictive current controller for a 
% three-phase converter system and a prediction horizon of one step
%
% Model Predictive Control of Power Electronic Systems, TAU, Tampere
% Petros Karamanakos, September 2025

global Par

% sampling intervals
Par.Ts.control = 100e-6;	% sampling interval of controller [s] 
Par.Ts.system = 5e-6;       % sampling interval of power electronic system [s]

% rated grid values
Par.Rated.Vll = 3300;       % line-to-line rms voltage [V]
Par.Rated.S = 9e6;          % apparent power [VA]
Par.Rated.I = Par.Rated.S/(sqrt(3)*Par.Rated.Vll);     % rms phase current [A]
Par.Rated.f0 = 50;          % fundamental frequency [Hz]

% per unit system
Par.Base.w = 2*pi*Par.Rated.f0;       	% base angular frequency [rad/s]
Par.Base.V = sqrt(2/3)*Par.Rated.Vll;   % base voltage [V]
Par.Base.I = sqrt(2)*Par.Rated.I;       % base current [A]
Par.Base.Z = Par.Base.V / Par.Base.I;   % base impedance [Ohm]

% grid parameters in per unit
Par.Grid_pu.X = 0.15;       % total grid reactance [pu]
Par.Grid_pu.R = 0.015;     	% total grid resistance [pu]

% grid parameter in SI
Par.Grid_SI.L = Par.Grid_pu.X * Par.Base.Z/Par.Base.w;
Par.Grid_SI.R = Par.Grid_pu.R * Par.Base.Z;
  
% total dc-link voltage
Par.Grid_SI.Vdc = 5200;                     
Par.Grid_pu.Vdc = Par.Grid_SI.Vdc / Par.Base.V;

% reduced Clarke transformation
Par.K = 2/3 * [1 -0.5 -0.5;           	% abc -> alpha/beta
               0 sqrt(3)/2 -sqrt(3)/2];   
Par.Kinv = [1 0;                        % alpha/beta -> abc
           -0.5 sqrt(3)/2; 
           -0.5 -sqrt(3)/2];           
           
% continuous-time system model
% F = **** to be added ****
% G1 = **** to be added ****
% G2 = **** to be added ****

% discrete-time system model (forward Euler)
% Par.Ctr.A = **** to be added **** 
% Par.Ctr.B1 = **** to be added **** 
% Par.Ctr.B2 = **** to be added **** 

% controller settings
Par.Ctr.lambdaU = 10e-3;     	% penalty on switching

% gain in the three-phase grid voltage to simulate grid voltage asymmetries
% (only required for Exercise 3.4)
relPhaseAmpl = [1; 1; 1];