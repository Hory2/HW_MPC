function Par = problMatrices(Par)

% MPC with current reference tracking for an induction motor drive 
% system with prediction horizons exceeding one step:
% computation of the problem matrices
%
% Model Predictive Control of Power Electronic Systems, TAU, Tampere
% Petros Karamanakos, September 2025

% parameters
lambda = Par.Ctr.lambda;% scalar penalty (tuning parameter) on switching effort
Np = Par.Ctr.Np;        % prediction horizon
Q = eye(2);             % penalty matrix on outputs y

% get the discrete-time system model
A = Par.Sys.A;   	% 4x4 (dimensions relate to the machine model)
B = Par.Sys.B;      % 4x3
C = Par.Sys.C;    	% 2x4
% -> y(k+1) = C*A*x(k) + C*B*u_abc
%    x = [is; psiR]
%    y = is

if Np == 2
    % Y = Gamma*x(k) + Ypsilon*U      	% (5.38)
    % with Y = [y(k+1), ..., y(k+N)]
    %      U = [u(k), ... u(k+N-1)]
    Gamma = [C*A; C*A^2];
    Ypsilon = [C*B      zeros(2,3)
               C*A*B    C*B     ];
    
    % sum_{n=k}^{k+N-1} ||u(n)-u(n-1)||^2 = ||S*U - E*u(k-1)||^2
    S = [eye(3) zeros(3);       	% (5.43)
        -eye(3) eye(3)];
    E = [eye(3); zeros(3)];         % (5.43)
    
    Qtilde = [Q         zeros(2);
              zeros(2)  Q];
    
elseif Np == 3
    % Y = Gamma*x(k) + Ypsilon*U      	% (5.38)
    % with Y = [y(k+1), ..., y(k+N)]
    %      U = [u(k), ... u(k+N-1)]
    Gamma = [C*A; C*A^2; C*A^3];
    Ypsilon = [C*B      zeros(2,6)
               C*A*B    C*B     zeros(2,3)
               C*A^2*B  C*A*B    C*B];
    
    % sum_{n=k}^{k+N-1} ||u(n)-u(n-1)||^2 = ||S*U - E*u(k-1)||^2
    S = [eye(3) zeros(3,6);       	% (5.43)
        -eye(3) eye(3) zeros(3);
        zeros(3) -eye(3) eye(3)];
    E = [eye(3); zeros(6,3)];         % (5.43)
    
    Qtilde = [Q         zeros(2);
              zeros(2)  Q];
    Qtilde(5:6,5:6)=Q;
else
    error('not available');
end
    

% Hessian matrix
H = Ypsilon'*Qtilde*Ypsilon + lambda*(S'*S);   	% 3Np x 3Np
Hinv = inv(H);

% generator matrix:
V = inv(chol(Hinv))';        % V is lower triangular and V'*V = H

% store the problem matrices
Par.Prob.Gamma = Gamma;
Par.Prob.Ypsilon = Ypsilon;
Par.Prob.Qtilde = Qtilde;
Par.Prob.E = E;
Par.Prob.S = S;
Par.Prob.Hinv = Hinv;
Par.Prob.V = V;