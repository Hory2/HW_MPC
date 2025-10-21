function [U, Uopt, rho, searchFlag] = sphDec(U, Uopt, dist, i, iMax, rho, Ubar_unc, V, searchFlag, u_km1)

% basic sphere decoder
%     
% U:        switching sequence with -1,0,1 elements
% Uopt:     optimal switching sequence with -1,0,1 elements
% dist:     squared distance d^2
% i:        level
% iMax:     maximal level number (equal to 3*Np)
% rho:      squared radius of the sphere
% Ubar_unc:	unconstrained solution in the V-space
% V:        generator matrix
% searchflag: 0:no solution found, >0:at least one solution found
% u_km1:    three-phase switch position at k-1
%
% Model Predictive Control of Power Electronic Systems, TAU, Tampere
% Petros Karamanakos, September 2025

for u = -1:1:1
    U(i) = u;         % to be added
    d = norm(Ubar_unc(1:i)- V(1:i,1:i)*U(1:i),2)^2 + dist;     % to be added
    
    % display node information:
    for j = 1:i-1, fprintf('   '); end;
    fprintf('i=%i: U=', i); 
    fprintf('%i',U(1:i));
    fprintf(' dist/rho=%1.2f  ...', d/rho);
    
    if d<=rho % to be added
        if i < iMax
            fprintf('branch further\n');
            [U, Uopt, rho, searchFlag] = sphDec(U, Uopt, d, i+1, iMax, rho, Ubar_unc, V, searchFlag, u_km1); % to be added
        else
            fprintf('a (better) switching sequence found - tightening by %1.2f percent\n',(rho-d)/d*100);
            Uopt = U;
            rho = d;
            searchFlag = searchFlag + 1;            
        end
    else
      	fprintf('terminate (cut) branch\n');
    end
end

return