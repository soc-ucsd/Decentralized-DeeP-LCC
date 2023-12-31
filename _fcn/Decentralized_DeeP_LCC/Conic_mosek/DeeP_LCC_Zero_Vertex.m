%Calculate the input with zero disturbance bound estimation method
%Solve it via vertex form
%Implict x = [t1; u1; tauy1] (Noting that here x represent decision variables)
%Dimension: t1: 1, u1: m1*N, tauy1: p*Tini, safe constraints: m1*N (Consider
% m1 as the number of CAVs in the system), disturbance: m2*dimDis, 
%initial condition: (m1+m2+p)*Tini
%Constant Constrain: [u_min, u_max], [s_min, s_max], [d_min, d_max]all vectors

function [uout, problem_status, time_comp] = DeeP_LCC_Zero_Vertex(UP,YP,UF,YF,WP,WF,...
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

% Define initial condition
uini_col = reshape(uini,[m1*Tini,1]);
yini_col = reshape(yini,[p*Tini,1]);
eini_col = reshape(eini,[m2*Tini,1]);
xini = [uini_col; eini_col; yini_col;];
xini = xini';

%Define parameters
u_min = u_limit(1) * ones(dim_u1, 1);
u_max = u_limit(2) * ones(dim_u1, 1);
s_min = s_limit(1) * ones(dim_safe, 1);
s_max = s_limit(2) * ones(dim_safe, 1);

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
K = zeros(N, dim_dis);


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

%Represent the input limitation constraint
%Get u from implicit x
Asel = [zeros(dim_u1, dim_t1), eye(dim_u1), zeros(dim_u1, dim_tauy1)];
A = [Asel; -Asel];
c = [u_max; -u_min];

%Represent the safe constraints
M_safe = Sf_blk*YF*invH*blkdiag(eye(dim_ini+dim_u1), K, eye(dim_tauy1));
%Get the block for constant
M_safe_c = [M_safe(:, 1:dim_ini), ...
            M_safe(:, dim_ini+dim_u1+1:dim_ini+dim_u1+dim_dis)];
%Get the block for decision variable
M_safe_d = [M_safe(:, dim_ini+1:dim_ini+dim_u1), ...
            M_safe(:, dim_ini+dim_u1+dim_dis+1:end)];
M_temp1 = [zeros(size(M_safe_d, 1), dim_t1), M_safe_d]; 
w1 = 0;
vc = [xini'; w1]; %Constant
A = [A; M_temp1; -M_temp1];
c = [c; s_max-M_safe_c*vc; -s_min+M_safe_c*vc];

%Represent the epi-graph form of previous objective function
M_obj = sqrtm(Gamma);
%Get the block for constant
M_obj_c = [M_obj(:, 1:dim_ini), ...
           M_obj(:, dim_ini+dim_u1+1:dim_ini+dim_u1+dim_dis)];
%Get the block for decision variable
M_obj_d = [M_obj(:, dim_ini+1:dim_ini+dim_u1), ...
           M_obj(:, dim_ini+dim_u1+dim_dis+1:end)];
M_temp2 = [eye(dim_t1), zeros(dim_t1, size(M_obj_d, 2));
           zeros(size(M_obj_d, 1), dim_t1), M_obj_d];
w1 = 0;
vc = [xini'; w1]; %Constant
A = [A; -M_temp2];
c = [c; zeros(dim_t1); M_obj_c*vc];

%The input limitation and the safe constraints are in non-negative orthant
Kone.l = 2 * dim_u1 + 2 * dim_safe;

%The contraint for objective function is a second-order cone
Kone.q = (dim_t1 + size(M_obj_c, 1)) * ones(1, 1);

%Define the current objective function
b = -[ones(dim_t1, 1); zeros(dim_u1+dim_tauy1, 1)];
% [x, y, info] = sedumi(A, b, c, Kone);
%Transfer to Mosek form
[res1, mosektime]= SolveMosek(A', b, c, Kone);
time_comp = mosektime;
uout = res1.sol.itr.y(1+dim_t1:dim_t1+dim_u1);
if (all(res1.sol.itr.solsta == 'OPTIMAL'))
    problem_status = 0;
end
end

    
