%The code for Yalmip is developed and adapted from the following paper: 
% Decentralized Data-Enabled Predictive Control for Power System Oscillation Damping
function [uout, problem_status, time_form, time_solver] = DeeP_LCC_Zero_Yalmip(UP,YP,UF,YF,WP,WF,...
    uini,yini,eini,weight_v, weight_s, weight_u, lambda_g,lambda_y,u_limit,s_limit)

%Define dimension for control system
m1 = size(uini,1); % dimension of control input
m2 = size(eini,1); % dimension of disturbance
p  = size(yini,1); % dimension of output
Tini = size(UP,1)/m1 ; % horizon of past data
N = size(UF,1)/m1 ; % horizon of future data
T = size(UP,2) + Tini + N - 1; % time length of pre-collected data
num_points = 1; %Define the downsampling data points
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

yalmip('clear');
    
u1 = sdpvar(m1*N,1);
xini1 = Xini;
t1 = sdpvar(1,1);
tauy1 = sdpvar(p*Tini,1);
w1 = zeros(m2, 1);
K = zeros(m2*N, 1);
    
%Input limitation
Uc = [lb <= u1 <= ub];
    
%Define matrix which transfers downsampling disturbance variable to the
%whole disturbance trajectory (Only support with one-dimension disturbance)

Q_v         = weight_v*eye(p-m1);  % penalty for velocity error
Q_s         = weight_s*eye(m1);    % penalty for spacing error
Q           = blkdiag(Q_v,Q_s);  
    
Q_blk    = zeros(p*N);
for i = 1:N
    Q_blk((i-1)*p+1:i*p,(i-1)*p+1:i*p) = Q; 
end

A = blkdiag(eye((m1+m2+p)*Tini + m1*N),K',eye(p*Tini)) * invH'*(I*lambda_g + YF'*Q_blk*YF)*invH * blkdiag(eye((m1+m2+p)*Tini + m1*N),K,eye(p*Tini)) + ...
    weight_u * blkdiag(zeros((m1+m2+p)*Tini),eye(m1*N),zeros(m2*dim_dis),zeros(p*Tini)) + lambda_y * blkdiag(zeros((m1+m2+p)*Tini + m1*N + m2*dim_dis),eye(p*Tini));
  
Wc = [cone([t1;sqrtm(A)*[xini1';u1;w1;tauy1]]), yl <= Sf_blk*YF*invH*[xini1';u1;K*w1;tauy1] <= ym];

objective1 = t1;
t_form = tic;
[model,recoverymodel] = export([Uc, Wc],objective1,sdpsettings('solver','sedumi'));
pars.fid = 0;
Kone.l = model.K.l;
Kone.q = model.K.q;
time_form = toc(t_form);
t_m = tic;
[res1, mosektime]= SolveMosek((model.A)', model.b, model.C, Kone);
time_m = toc(t_m);
time_form = time_form + time_m - mosektime;
time_solver = mosektime;
assign(recover(recoverymodel.used_variables),res1.sol.itr.y);
uout = value(u1);
problem_status = 0;
end

