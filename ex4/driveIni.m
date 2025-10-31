% Initialization file for MPC with current reference tracking for an 
% induction motor drive system with prediction horizons exceeding one step
%
% Model Predictive Control of Power Electronic Systems, TAU, Tampere
% Petros Karamanakos, September 2025

global Par
clear functions         % clear all persistent variables

% sampling intervals
Par.Ts.control = 100e-6;% sampling interval of controller [s] 
Par.Ts.system = 2.5e-6; % sampling interval of power electronic system [s]

% rated machine values
Par.Rated.Vll = 3300;   % line-to-line rms voltage [V]
Par.Rated.Is = 356;     % phase current [A]
Par.Rated.f0 = 50;      % fundamental frequency [Hz]
Par.Rated.S = sqrt(3)*Par.Rated.Vll*Par.Rated.Is;   % apparent power [VA]

% machine parameters in per unit
Par.Mach.Xm = 2.3489;   % magnetizing inductance [pu]
Par.Mach.Xls = 0.1493;  % stator leakage inductance [pu]
Par.Mach.Xlr = 0.1104;  % rotor leakage inductance [pu]
Par.Mach.Rs = 0.0108;   % stator resistance [pu]
Par.Mach.Rr = 0.0091;   % rotor resistance [pu]
Par.Mach.pp = 5;        % number of pole pairs
Par.Mach.cosphi = 0.809;% power factor

% deduced machine quantities
Par.Mach.Xs = Par.Mach.Xls + Par.Mach.Xm;
Par.Mach.Xr = Par.Mach.Xlr + Par.Mach.Xm;
Par.Mach.D = Par.Mach.Xs*Par.Mach.Xr - Par.Mach.Xm^2;  % determinant
Par.Mach.kT = 1/Par.Mach.cosphi;    % torque correction factor 

% load parameters
%Par.Load.M = 1;         % inertia of machine and load [s]
%Par.Load.F = 0.05;      % friction factor [pu]
   
% per unit system
Par.Base.w = 2*pi*Par.Rated.f0;       	% base angular frequency 
Par.Base.V = sqrt(2/3)*Par.Rated.Vll;   % base voltage [V]
Par.Base.I = sqrt(2)*Par.Rated.Is;

% total dc-link voltage
Par.Vdc.SI = 5200;                     
Par.Vdc.pu = Par.Vdc.SI / Par.Base.V;

% reduced Clarke transformation
Par.K = 2/3 * [1 -0.5 -0.5;           	% abc -> alpha/beta
               0 sqrt(3)/2 -sqrt(3)/2];     

% controller settings
Par.Ctr.lambda = 5e-3;  	% penalty on switching
Par.Ctr.Np = 2;          	% prediction horizon (in time steps)

% initial conditions for the induction machine
psiS0 = [1; 0];
psiR0 = [0.91; 0];

% operating point (angular electrical rotor speed)
omegaR = 1;

% discrete-time machine model
Par = machModel(Par, omegaR, Par.Ts.control*Par.Base.w);

% derive the matrices of the optimization problem
Par = problMatrices(Par);