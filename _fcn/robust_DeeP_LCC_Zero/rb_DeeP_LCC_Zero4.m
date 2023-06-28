% Copyright 2021 ETH Zurich, Linbin Huang, Jeremy Coulson, John Lygeros, Florian Dorfler
% 
% Licensed under the Apache License, Version 2.0 (the "License");
% you may not use this file except in compliance with the License.
% You may obtain a copy of the License at
% 
%     http://www.apache.org/licenses/LICENSE-2.0
% 
% Unless required by applicable law or agreed to in writing, software
% distributed under the License is distributed on an "AS IS" BASIS,
% WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
% See the License for the specific language governing permissions and
% limitations under the License.

function [uout, problem_status] = rb_DeeP_LCC_Zero4(UP,YP,UF,YF,WP,WF,...
    uini,yini,eini,weight_v, weight_s, weight_u, r,lambda_g,lambda_y,u_limit,s_limit, time)
% This function will solve the DeePC optimization problem by using OSQP.
% The minimax problem will be transformed it to a conic
% program and then solved by YALMIP + MOSEK.

% The coding language can refer to:
% https://yalmip.github.io/example/robustmpc/ and
% https://yalmip.github.io/command/optimizer/ 
persistent controller4

ub = u_limit(2);
lb = u_limit(1);
ym = s_limit(2);
yl = s_limit(1);



m1       = size(uini,1);               % dimension of control input
m2       = size(eini,1);               % dimension of disturbance
p        = size(yini,1);                % dimension of output
Tini     = size(UP,1)/m1 ;               % horizon of past data
N        = size(UF,1)/m1 ;               % horizon of future data
T        = size(UP,2) + Tini + N - 1;   % time length of pre-collected data

% reshape past data into one single trajectory
% uini = col(u(-Tini),u(-Tini+1),...,u(-1)) (similarly for yini and eini)
uini_col = reshape(uini,[m1*Tini,1]);
yini_col = reshape(yini,[p*Tini,1]);
eini_col = reshape(eini,[m2*Tini,1]);

Xini = [uini_col; eini_col; yini_col;];
Xini = Xini';
if time == 0
    
    I = eye(T-Tini-N+1);
    
    H = [UP;WP;YP;UF;WF];
    invH1 = pinv(H);
    invH = [invH1 invH1(:,(m1+m2)*Tini+1:(m1+m2)*Tini+p*Tini)];

    Sf = [zeros(m1,p-m1),eye(m1)];
    Sf_blk = Sf;
    for i = 2:N
        Sf_blk = blkdiag(Sf_blk,Sf); 
    end
  
    
    num = N;
    dimDis = floor(N/num) + 1;
    
    yalmip('clear');
    
    u1 = sdpvar(m1*N,1);
    w1 = zeros(m2*dimDis,1);
    xini1 = sdpvar(1,(m1+m2+p)*Tini);
    t1 = sdpvar(1,1);
    tauy1 = sdpvar(p*Tini,1);
    
    Uc = [lb <= u1 <= ub];
    
    %Noting:we have some magic number in the K implementation!
    %May need further change
    K = zeros(N, 2);
    for i = 1:N
        K(i, 1) = 1 - 1 / N * (i - 1);
        K(i, 2) = 1 / N * (i - 1);
    end
%     K = ones(m2*N, 1);
    Q_v         = weight_v*eye(p-m1);          % penalty for velocity error
    Q_s         = weight_s*eye(m1);    % penalty for spacing error
    Q           = blkdiag(Q_v,Q_s);  
    
    Q_blk    = zeros(p*N);
    for i = 1:N
    Q_blk((i-1)*p+1:i*p,(i-1)*p+1:i*p) = Q; 
    end

    A = blkdiag(eye((m1+m2+p)*Tini + m1*N),K',eye(p*Tini)) * invH'*(I*lambda_g + YF'*Q_blk*YF)*invH * blkdiag(eye((m1+m2+p)*Tini + m1*N),K,eye(p*Tini)) + ...
        weight_u * blkdiag(zeros((m1+m2+p)*Tini),eye(m1*N),zeros(m2*2),zeros(p*Tini)) + lambda_y * blkdiag(zeros((m1+m2+p)*Tini + m1*N + m2*2),eye(p*Tini));
  
    Wc = [cone([t1;chol(nearestSPD(A))*[xini1';u1;w1;tauy1]]), yl <= Sf_blk*YF*invH*[xini1';u1;K*w1;tauy1] <= ym];
    
    objective1 = t1;
    
    controller4  = optimizer([Uc,Wc],objective1,[],{xini1},u1);
    [uout, problem_status] = controller4(Xini);
else
    [uout, problem_status] = controller4(Xini);
end

end

