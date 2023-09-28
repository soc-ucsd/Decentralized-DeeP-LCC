%The code for Yalmip is developed and adapted from the following paper: 
% Decentralized Data-Enabled Predictive Control for Power System Oscillation Damping
function [uout, problem_status, time_form, time_solver] = DeeP_LCC_TimeV_Vertex_Yalmip(UP,YP,UF,YF,WP,WF,...
    uini,yini,eini,weight_v, weight_s, weight_u, lambda_g,lambda_y,u_limit,s_limit, Tstep)
%Define dimension for control system
m1 = size(uini,1); % dimension of control input
m2 = size(eini,1); % dimension of disturbance
p  = size(yini,1); % dimension of output
Tini = size(UP,1)/m1 ; % horizon of past data
N = size(UF,1)/m1 ; % horizon of future data
T = size(UP,2) + Tini + N - 1; % time length of pre-collected data
interval = 8; %Define the downsampling interval
num_points = floor((N-2)/interval) + 2; %Define the downsampling data points
dim_dis = m2 * num_points;

% Define initial condition
uini_col = reshape(uini,[m1*Tini,1]);
yini_col = reshape(yini,[p*Tini,1]);
eini_col = reshape(eini,[m2*Tini,1]);

%Define parameters
ub = u_limit(2);
lb = u_limit(1);
ym = s_limit(2);
yl = s_limit(1);
%Time-varying disturbance bound
d_base = eini_col(end);
a_all = zeros(size(eini_col, 1)-1, 1);
for i = 2:size(eini_col, 1)
    a_all(i-1) = (eini_col(i) - eini_col(i-1))/Tstep;
end
a_current = a_all(end);
a_low = mean(a_all) - min(a_all);
a_up = max(a_all) - mean(a_all);
d_min = zeros(dim_dis, 1);
d_max = zeros(dim_dis, 1);
for i = 1:dim_dis
    if i == dim_dis
        d_min(i) = d_base + (a_current - a_low) * Tstep * N;
        d_max(i) = d_base + (a_current + a_up) * Tstep * N;
    else
        d_min(i) = d_base + (a_current - a_low) * ((i-1) * interval + 1) * Tstep;
        d_max(i) = d_base + (a_current + a_up) * ((i-1) * interval + 1) * Tstep;
    end
end


%Update the initial condition and the disturbance bound
Xini = [uini_col; eini_col; yini_col;];
Xini = Xini';

%Initialize the optimization problem once
    I = eye(T-Tini-N+1);
    
    H = [UP;WP;YP;UF;WF];
    invH1 = pinv(H);
    invH = [invH1 invH1(:,(m1+m2)*Tini+1:(m1+m2)*Tini+p*Tini)];

    Sf = [zeros(m1,p-m1),eye(m1)];
    Sf_blk = Sf;
    for i = 2:N
        Sf_blk = blkdiag(Sf_blk,Sf); 
    end
%     interval = 12; %Define the downsampling interval
%     num_points = floor((N-2)/interval) + 2; %Define the downsampling data points
%     dim_dis = m2 * num_points;

    yalmip('clear');
    
    u1 = sdpvar(m1*N,1);
    ml = d_min;
    mp = d_max;
    w1 = sdpvar(dim_dis, 1);
    xini1 = Xini;
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
        weight_u * blkdiag(zeros((m1+m2+p)*Tini),eye(m1*N),zeros(m2*dim_dis),zeros(p*Tini)) + lambda_y * blkdiag(zeros((m1+m2+p)*Tini + m1*N + m2*dim_dis),eye(p*Tini));
  
    Wc = [ml <= w1 <= mp, uncertain(w1), cone([t1;sqrtm(A)*[xini1';u1;w1;tauy1]]), yl <= Sf_blk*YF*invH*[xini1';u1;K*w1;tauy1] <= ym];

    objective1 = t1;
    t_form = tic;
    [model,recoverymodel] = export([Uc, Wc],objective1,sdpsettings('solver','sedumi'));
    pars.fid = 0;
%     t_solve = tic;
%     [x,y,info] = sedumi(model.A,model.b,model.C,model.K,pars);
%     time_solver = toc(t_solve);
%     assign(recover(recoverymodel.used_variables),y);
    Kone.l = model.K.l;
    Kone.q = model.K.q;
    time_form = toc(t_form);
    t_m = tic;
    [res1, mosektime]= SolveMosek((model.A)', model.b, model.C, Kone);
    time_m = toc(t_m);
    time_form = time_form + time_m - mosektime;
    time_solver = mosektime;
%     uout = res1.sol.itr.xx(1+1:51);
    assign(recover(recoverymodel.used_variables),res1.sol.itr.y);
    uout = value(u1);
    problem_status = 0;
%     if (info.numerr == 2)
%         problem_status = 1;
%     end
    
end

