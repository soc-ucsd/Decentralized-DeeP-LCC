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

function uout = minmax_DeeP_LCC(Up,Yp,Uf,Yf,Ep,Ef,...
    uini,yini,eini,Q,R,r,lambda_g,lambda_y,u_limit,s_limit, time)
% This function will solve the DeePC optimization problem by using OSQP.
% The minimax problem will be transformed it to a conic
% program and then solved by YALMIP + MOSEK.

% The coding language can refer to:
% https://yalmip.github.io/example/robustmpc/ and
% https://yalmip.github.io/command/optimizer/ 
persistent controller1

UP = Up;
YP = Yp;
WP = Ep;
UF = Uf;
YF = Yf;
WF = Ef;
ub = u_limit(2);
lb = u_limit(1);



m1       = size(uini,1);               % dimension of control input
m2       = size(eini,1);               % dimension of disturbance
p        = size(yini,1);                % dimension of output
Tini     = size(Up,1)/m1 ;               % horizon of past data
N        = size(Uf,1)/m1 ;               % horizon of future data
T        = size(Up,2) + Tini + N - 1;   % time length of pre-collected data

% reshape past data into one single trajectory
% uini = col(u(-Tini),u(-Tini+1),...,u(-1)) (similarly for yini and eini)
uini_col = reshape(uini,[m1*Tini,1]);
yini_col = reshape(yini,[p*Tini,1]);
eini_col = reshape(eini,[m2*Tini,1]);

Xini = [uini_col; eini_col; yini_col;];
Xini = Xini';
if time == Tini
    
    I = eye(T-Tini-N+1);
    
    H = [UP;WP;YP;UF;WF];
    invH1 = pinv(H);
    invH = [invH1 invH1(:,(m1+m2)*Tini+1:(m1+m2)*Tini+p*Tini)];

    K1 = YF*invH;
    Kw = K1(:,m1*Tini+1:(m1+m2)*Tini);
    
    ml = min(eini_col);
    mp = max(eini_col);
    
    num = N;
    
    yalmip('clear');
    
    u1 = sdpvar(m1*N,1);
    w1 = sdpvar(m2*(N/num+1),1);
    xini1 = sdpvar(1,(m1+m2+p)*Tini);
    t1 = sdpvar(1,1);
    tauy1 = sdpvar(p*Tini,1);
    
    Uc = [lb(1) <= u1 <= ub(1)];
    
    KT = zeros(m2*N,m2*(N/num+1));
    for i = 1:m2*N/num
        for j = 1:m2*num
            KT((i-1)*num+j,i) = 1 - 1 / num * (j - 1);
            KT((i-1)*num+j,i+1) = 1 / num * (j - 1);
        end
    end
    K = KT;

    
    A = blkdiag(eye((m1+m2+p)*Tini + m1*N),K',eye(p*Tini)) * invH'*(I*lambda_g + YF'*YF*Q)*invH * blkdiag(eye((m1+m2+p)*Tini + m1*N),K,eye(p*Tini)) + ...
        R * blkdiag(zeros((m1+m2+p)*Tini),eye(m1*N),zeros(m2*(N/num+1)),zeros(p*Tini)) + lambda_y * blkdiag(zeros((m1+m2+p)*Tini + m1*N + m2*(N/num+1)),eye(p*Tini));
  
    Wc = [-ml <= w1 <= mp, uncertain(w1), cone([t1;chol(nearestSPD(A))*[xini1';u1;w1;tauy1]])];
    
    objective1 = t1;
    
    controller1  = optimizer([Uc,Wc],objective1,[],{xini1},u1);
    uout = controller1(Xini);
else
    uout = controller1(Xini);
end

end

