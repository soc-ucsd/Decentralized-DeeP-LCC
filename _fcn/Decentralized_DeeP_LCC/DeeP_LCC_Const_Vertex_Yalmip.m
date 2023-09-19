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

function [uout, problem_status] = DeeP_LCC_Const_Vertex_V1(UP,YP,UF,YF,WP,WF,...
    uini,yini,eini,weight_v, weight_s, weight_u, lambda_g,lambda_y,u_limit,s_limit)
% This function will solve the DeePC optimization problem by using OSQP.
% The minimax problem will be transformed it to a conic
% program and then solved by YALMIP + MOSEK.

% The coding language can refer to:
% https://yalmip.github.io/example/robustmpc/ and
% https://yalmip.github.io/command/optimizer/ 
persistent controller flag

%Define dimension for control system
m1 = size(uini,1); % dimension of control input
m2 = size(eini,1); % dimension of disturbance
p  = size(yini,1); % dimension of output
Tini = size(UP,1)/m1 ; % horizon of past data
N = size(UF,1)/m1 ; % horizon of future data
T = size(UP,2) + Tini + N - 1; % time length of pre-collected data

% Define initial condition
uini_col = reshape(uini,[m1*Tini,1]);
yini_col = reshape(yini,[p*Tini,1]);
eini_col = reshape(eini,[m2*Tini,1]);

%Define parameters
ub = u_limit(2);
lb = u_limit(1);
ym = s_limit(2);
yl = s_limit(1);
%Constant disturbance bound
dis_current = eini_col(end);
dis_mean = mean(eini_col);
d_low = dis_mean - min(eini_col);
d_up = max(eini_col) - dis_mean;
dis_min = dis_current -  d_low;
dis_max = dis_current + d_up;
if dis_min == dis_max
    dis_min = dis_min - 0.1;
end


%Update the initial condition and the disturbance bound
Xini = [uini_col; eini_col; yini_col;];
Xini = Xini';
Ml = dis_min;
Mp = dis_max;

%Initialize the optimization problem once
if isempty(flag)
    flag = 1;
    I = eye(T-Tini-N+1);
    
    H = [UP;WP;YP;UF;WF];
    invH1 = pinv(H);
    invH = [invH1 invH1(:,(m1+m2)*Tini+1:(m1+m2)*Tini+p*Tini)];

    Sf = [zeros(m1,p-m1),eye(m1)];
    Sf_blk = Sf;
    for i = 2:N
        Sf_blk = blkdiag(Sf_blk,Sf); 
    end
    
    interval = N; %Define the downsampling interval
    num_points = floor((N-2)/interval) + 2; %Define the downsampling data points
    dimDis = m2 * num_points;

    yalmip('clear');
    
    u1 = sdpvar(m1*N,1);
    ml = sdpvar(1, 1);
    mp = sdpvar(1, 1);
    wnorm = sdpvar(m2*dimDis, 1);
    w1 = wnorm * (mp - ml)/2 + (ml + mp)/2;
    xini1 = sdpvar(1,(m1+m2+p)*Tini);
    t1 = sdpvar(1,1);
    tauy1 = sdpvar(p*Tini,1);
    
    %Input limitation
    Uc = [lb <= u1 <= ub];
    
    %Define matrix which transfers downsampling disturbance variable to the
    %whole disturbance trajectory (Only support with one-dimension disturbance)
    K = zeros(N, num_points);
    if num_points >2
        for i = 1:(floor((N-2)/interval))*interval
            temp1 = floor((i-1)/interval)+2;
            temp2 = floor((i-1)/interval)+1;
            K(i, temp2) = 1 - mod(i-1, interval)/interval;
            K(i, temp1) = mod(i-1, interval)/interval;
        end
    end
    for i = (floor((N-2)/interval)*interval+1):N-1
        temp3 = N-(floor((N-2)/interval)*interval+1);
        K(i, num_points-1) = 1 - mod(i-1, interval)/temp3;
        K(i, num_points) = mod(i-1, interval)/temp3;
    end
    K(N, num_points) = 1;

    Q_v         = weight_v*eye(p-m1);  % penalty for velocity error
    Q_s         = weight_s*eye(m1);    % penalty for spacing error
    Q           = blkdiag(Q_v,Q_s);  
    
    Q_blk    = zeros(p*N);
    for i = 1:N
    Q_blk((i-1)*p+1:i*p,(i-1)*p+1:i*p) = Q; 
    end

    A = blkdiag(eye((m1+m2+p)*Tini + m1*N),K',eye(p*Tini)) * invH'*(I*lambda_g + YF'*Q_blk*YF)*invH * blkdiag(eye((m1+m2+p)*Tini + m1*N),K,eye(p*Tini)) + ...
        weight_u * blkdiag(zeros((m1+m2+p)*Tini),eye(m1*N),zeros(m2*dimDis),zeros(p*Tini)) + lambda_y * blkdiag(zeros((m1+m2+p)*Tini + m1*N + m2*dimDis),eye(p*Tini));
  
    Wc = [-1 <= wnorm <= 1, uncertain(wnorm), cone([t1;sqrtm(A)*[xini1';u1;w1;tauy1]]), yl <= Sf_blk*YF*invH*[xini1';u1;K*w1;tauy1] <= ym];

    objective1 = t1;
    options = sdpsettings('solver','mosek','verbose',2);
    controller  = optimizer([Uc,Wc],objective1,options,{xini1, ml, mp},u1);
    [uout, problem_status] = controller(Xini, Ml, Mp);
else
    'Vertex_V1'
    ts = tic;
    [uout, problem_status] = controller(Xini, Ml, Mp);
    tEnd = toc(ts)
end
end

