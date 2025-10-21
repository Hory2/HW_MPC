function [U, Uopt, rho, searchFlag] = sphDec(U, Uopt, dist, i, iMax, rho, Ubar_unc, V, searchFlag, u_km1)
print =0;
switchConstarinCheck=0;
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
    d = norm(Ubar_unc(i)- V(i,1:i)*U(1:i),2)^2 + dist;     % to be added
   
    
    % check if transfer is legal
    valid_transefer=1;
    if i>3 && switchConstarinCheck
        if 1<(U(i)-U(i-3))
            valid_transefer=0;
            if print
                fprintf("ivalid transfer detected \n")
            end
        end
    end

    % display node information:
    if print
        for j = 1:i-1, fprintf('   '); end;
        fprintf('i=%i: U=', i); 
        fprintf('%i',U(1:i));
        fprintf(' dist/rho=%1.2f  ...', d/rho);
        fprintf('ro= %1.2f ... ',rho )
    end

    if d<=rho && valid_transefer % to be added
        if i < iMax
            if print
                fprintf('branch further\n');
            end
            [U, Uopt, rho, searchFlag] = sphDec(U, Uopt, d, i+1, iMax, rho, Ubar_unc, V, searchFlag, u_km1); % to be added
        else
            if print
                fprintf('a (better) switching sequence found - tightening by %1.2f percent\n',(rho-d)/d*100);
            end
            Uopt = U;
            rho = d;
            searchFlag = searchFlag + 1;            
        end
    else
        if print
      	    fprintf('terminate (cut) branch\n');
        end
    end
end

return