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
ab_ikp1_ref = Par.K* ikp1_ref_abc/Par.Base.I; 	% three-phase current reference at time step k+1
ab_ik =Par.K*ik_abc/Par.Base.I  ;        % three-phase current measurement at time step k
ab_vgk =Par.K* vgk_abc/Par.Base.V;    	% three-phase grid voltage measurement at time step k


% implement the predictive current controller
% **** to be added ****
sm=sum(abs(ukm1_abc))+1;% 0 1 2 3 +1
pole_pocet=[3*3*3, 3*3*2, 3*2*2, 2*2*2];
n=pole_pocet(sm);

iter=1;
pos_u=zeros(n,3);

for i=-1:1
    for j=-1:1
        for k=-1:1
            tmp=[i,j,k];
            if(max(abs(tmp-ukm1_abc'))<2)
                pos_u(iter,:)=tmp;
                iter=iter+1;
            end
            
        end
    end
end

J=ones(size(pos_u,1),1);
ik1_pred=zeros(2,n);
for i=1:n
    ik1_pred(:,i)=Par.Ctr.A*ab_ik+Par.Ctr.B1*pos_u(i,:)'+Par.Ctr.B2*ab_vgk;
    J(i)=norm(ik1_pred(:,i)-ab_ikp1_ref,2)^2+Par.Ctr.lambdaU *norm(ukm1_abc-pos_u(i,:)',1);
end
[err,index]=min(J);
err
uk_abc=pos_u(index,:);

% for the chosen switch position: 
% compute the predicted current in pu and alpha/beta at the next time step
% ikp1 = ik1_pred(:,index)  % **** to be added ****
ikp1= ik1_pred(:,index);

% output interface
outVec = [uk_abc'; ikp1];
    
return