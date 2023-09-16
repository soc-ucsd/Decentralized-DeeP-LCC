%Calculate the input with zero disturbance bound estimation method
%Solve it via dual form
%Implict x = [t1; u1; tauy1; lambda1; lambda2]
%Dimension: t1: 1, u1: m1*N, tauy1: p*Tini, safe constraints: m1*N (Consider
% m1 as the number of CAVs in the system), disturbance: m2*dimDis, 
%initial condition: (m1+m2+p)*Tini, lambda1, lambda2: 2*dim_safe*dim_dis
%Constant Constrain: [u_min, u_max] vectors, [s_min, s_max] scalar, 
%[d_min, d_max] vectors

function [uout, problem_status, time_comp] = DeeP_LCC_Zero_Dual(UP,YP,UF,YF,WP,WF,...
    uini,yini,eini,weight_v,weight_s,weight_u,lambda_g,lambda_y,u_limit,s_limit)
%Define dimension for control system
m1 = size(uini, 1); % dimension of control input
m2 = size(eini,  1); % dimension of disturbance
p = size(yini,1); % dimension of output
Tini = size(UP, 1) / m1 ; % horizon of past data
N = size(UF, 1) / m1 ; % horizon of future data
T = size(UP, 2) + Tini + N - 1; % time length of pre-collected data

%Define dimension for optimization problem
num_points = 1; %Define the downsampling data points
dim_ini = (m1 + m2 + p) * Tini;
dim_u1 = m1 * N;
dim_safe = m1 * N;
dim_t1 = 1;
dim_tauy1 = p * Tini;
dim_dis = m2 * num_points;
dim_lambda_i = 2 * dim_dis;
dim_lambda = dim_lambda_i * dim_safe;

% Define initial condition
uini_col = reshape(uini,[m1*Tini,1]);
yini_col = reshape(yini,[p*Tini,1]);
eini_col = reshape(eini,[m2*Tini,1]);
xini = [uini_col; eini_col; yini_col;];
xini = xini';


%Define parameters
u_min = u_limit(1) * ones(dim_u1, 1);
u_max = u_limit(2) * ones(dim_u1, 1);
s_min = s_limit(1);
s_max = s_limit(2);
% For zero case
d_min = 0 * ones(dim_dis, 1); 
d_max = 0 * ones(dim_dis, 1);
% d_min = mean(eini_col); 
% d_max = mean(eini_col); 
Aep = [eye(dim_dis); -eye(dim_dis)];
bep = [d_max; -d_min];

%Initialize Sedumi parameters 
A = [];
c = [];

%Define pesudo matrix for representing g and y
H = [UP;WP;YP;UF;WF];
invH1 = pinv(H);
invH = [invH1 invH1(:,(m1+m2)*Tini+1:(m1+m2)*Tini+p*Tini)];

%Define matrix which selects CAVs' space information from y
Sf = [zeros(m1,p-m1),eye(m1)];
Sf_blk = Sf;
for i = 2:N
    Sf_blk = blkdiag(Sf_blk,Sf); 
end

%Define matrix which transfers downsampling disturbance variable to the
%whole disturbance trajectory (Only support with one-dimension disturbance)
K = ones(N, dim_dis);

%Define performance matrix
Q_v = weight_v*eye(p-m1); % penalty for velocity error
Q_s = weight_s*eye(m1); % penalty for spacing error
Q = blkdiag(Q_v,Q_s);  
Q_blk = zeros(p*N);
for i = 1:N
Q_blk((i-1)*p+1:i*p,(i-1)*p+1:i*p) = Q; 
end

%Define the origin cost function, i.e., v^T Gamma v
I = eye(T-Tini-N+1);
Gamma = blkdiag(eye((m1+m2+p)*Tini + m1*N), K', eye(p*Tini))*invH'*...
        (I*lambda_g+YF'*Q_blk*YF)*invH*...
        blkdiag(eye((m1+m2+p)*Tini + m1*N),K,eye(p*Tini))+weight_u*...
        blkdiag(zeros((m1+m2+p)*Tini), eye(m1*N), zeros(m2*num_points), zeros(p*Tini))...
        +lambda_y*blkdiag(zeros((m1+m2+p)*Tini+m1*N+m2*num_points),eye(p*Tini));


%Represent the safe contraints
M_safe = Sf_blk*YF*invH*blkdiag(eye(dim_ini+dim_u1), K, eye(dim_tauy1));
%Represent the equality constraints for dual variable
for i = 1:dim_safe
    m_temp = M_safe(i, :);
    m_ep = m_temp(dim_ini+dim_u1+1:dim_ini+dim_u1+dim_dis);
    %Get the ith lambda_i_1
    M_temp1 = zeros(dim_lambda_i, (dim_t1+dim_u1+dim_tauy1+2*dim_lambda));
    pos_start_temp1 = dim_t1+dim_u1+dim_tauy1+dim_lambda_i*(i-1)+1;
    pos_end_temp1 = dim_t1+dim_u1+dim_tauy1+dim_lambda_i*i;
    M_temp1(:, pos_start_temp1:pos_end_temp1) = eye(dim_lambda_i); 
    A = [A; Aep'*M_temp1];
    c = [c; m_ep'];
    %Get the ith lambda_i_2
    M_temp2 = zeros(dim_lambda_i, (dim_t1+dim_u1+dim_tauy1+2*dim_lambda));
    pos_start_temp2 = dim_t1+dim_u1+dim_tauy1+dim_lambda+dim_lambda_i*(i-1)+1;
    pos_end_temp2 = dim_t1+dim_u1+dim_tauy1++dim_lambda+dim_lambda_i*i;
    M_temp2(:, pos_start_temp2:pos_end_temp2) = eye(dim_lambda_i); 
    A = [A; Aep'*M_temp2];
    c = [c; -m_ep'];
end
%Represent the inequality constraints for upper and lower bound
for i = 1:dim_safe
    m_temp = M_safe(i, :);
    m_c = m_temp(1:dim_ini);
    m_d = [m_temp(dim_ini+1:dim_ini+dim_u1), m_temp(dim_ini+dim_u1+dim_dis+1:end)];
    %Inequality constraints for the upper bound
    M_temp1 = zeros(1, (dim_t1+dim_u1+dim_tauy1+2*dim_lambda));
    M_temp1(dim_t1+1:dim_t1+dim_u1+dim_tauy1) = m_d;
    pos_start_temp1 = dim_t1+dim_u1+dim_tauy1+dim_lambda_i*(i-1)+1;
    pos_end_temp1 = dim_t1+dim_u1+dim_tauy1+dim_lambda_i*i;
    M_temp1(pos_start_temp1:pos_end_temp1) = bep';
    A = [A; M_temp1];
    c = [c; s_max-m_c*xini'];
    %Inequality constraints for the lower bound
    M_temp2 = zeros(1, (dim_t1+dim_u1+dim_tauy1+2*dim_lambda));
    M_temp2(dim_t1+1:dim_t1+dim_u1+dim_tauy1) = -m_d;
    pos_start_temp2 = dim_t1+dim_u1+dim_tauy1+dim_lambda+dim_lambda_i*(i-1)+1;
    pos_end_temp2 = dim_t1+dim_u1+dim_tauy1+dim_lambda+dim_lambda_i*i;
    M_temp2(pos_start_temp2:pos_end_temp2) = bep';
    A = [A; M_temp2];
    c = [c; -s_min+m_c*xini'];
end
%Represent the inequality constraints for dual variable
M_temp3 = zeros(2*dim_lambda, dim_t1+dim_u1+dim_tauy1+2*dim_lambda);
M_temp3(:, dim_t1+dim_u1+dim_tauy1+1:end) = eye(2*dim_lambda);
A = [A; -M_temp3];
c = [c; zeros(2*dim_lambda, 1)];

%Represent the input limitation constraint
%Get u from implicit x
Asel = [zeros(dim_u1, dim_t1), eye(dim_u1), zeros(dim_u1, dim_tauy1+2*dim_lambda)];
A = [A; Asel; -Asel];
c = [c; u_max; -u_min];

%Represent the epi-graph form of previous objective function
M_obj = sqrtm(Gamma);
% M_obj = sqrtm(Gamma);
%Get the block for constant
M_obj_c = [M_obj(:, 1:dim_ini), ...
           M_obj(:, dim_ini+dim_u1+1:dim_ini+dim_u1+dim_dis)];
%Get the block for decision variable
M_obj_d = [M_obj(:, dim_ini+1:dim_ini+dim_u1), ...
           M_obj(:, dim_ini+dim_u1+dim_dis+1:end)];
M_temp4 = [eye(dim_t1), zeros(dim_t1, size(M_obj_d, 2)), zeros(dim_t1, 2*dim_lambda);
           zeros(size(M_obj_d, 1), dim_t1), M_obj_d, zeros(size(M_obj_d, 1), 2*dim_lambda)];
w1 = 0;
vc = [xini'; w1]; %Constant
A = sparse([A; -M_temp4]);
c = [c; zeros(dim_t1); M_obj_c*vc];


Kone.f = 2 * dim_dis * dim_safe;
Kone.l = 2 * dim_safe + 2 * dim_lambda + 2 * dim_u1;
Kone.q = (dim_t1 + size(M_obj_c, 1)) * ones(1, 1);
%Define the current objective function
b = -[ones(dim_t1, 1); zeros(dim_u1+dim_tauy1, 1); zeros(2 * dim_lambda, 1)];
% [x, y, info] = sedumi(A, b, c, Kone);
%Transfer to Mosek form
[res1,mosektime]= SolveMosek(A', b, c, Kone);
problem_status = 1;
time_comp = mosektime;
uout = res1.sol.itr.y(1+dim_t1:dim_t1+dim_u1);
if (all(res1.sol.itr.solsta == 'OPTIMAL'))
    problem_status = 0;
end
end

