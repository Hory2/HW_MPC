function Par = machModel(Par, wr, Ts)

% State-space model of an induction motor drive system with the following
% characteristics:
% * state variables: stator current and rotor flux linkages in alpha/beta
% * constant speed wr
% * per unit
%
% Model Predictive Control of Power Electronic Systems, TAU, Tampere
% Petros Karamanakos, September 2025

% input interface
Rs = Par.Mach.Rs;
Rr = Par.Mach.Rr;
Xs = Par.Mach.Xs;
Xr = Par.Mach.Xr;
Xm = Par.Mach.Xm;
D = Par.Mach.D;
Vdc = Par.Vdc.pu;

tauS = Xr*D / (Rs*Xr^2 + Rr*Xm^2);
tauR = Xr/Rr;

% continuous-time model, see (2.59) and (2.60)
% dx = F*x + G*uk;
% x = [isa, isb, Psira, Psirb]      with a=alpha, b=beta
% uk = [uka, ukb, ukc]
F = [-1/tauS 0 Xm/(tauR*D) wr*Xm/D;
    0 -1/tauS -wr*Xm/D Xm/(tauR*D);
    Xm/tauR 0 -1/tauR -wr;
    0 Xm/tauR wr -1/tauR];
G = Vdc/2*Xr/D*[eye(2); zeros(2,2)]*Par.K;

% discrete-time state-space matrices using exact (Euler) discretization
% x_kp1 = A*xk + B*uk
% uk = [uka, ukb, ukc]
A = expm(F*Ts);
B = -inv(F) * (eye(4)-A) * G;
C = [1 0 0 0; 0 1 0 0];

% output interface
Par.Sys.A = A; 
Par.Sys.B = B; 
Par.Sys.C = C;