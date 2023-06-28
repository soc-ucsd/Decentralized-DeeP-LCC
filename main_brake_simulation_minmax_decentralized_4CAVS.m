% =========================================================================
% %                       Safety-critical Simulation
% Scenario:
%       The head vehicle takes a sudden brake
%
% See Section V of the following paper for details
%   Title : Data-Driven Predicted Control for Connected and Autonomous
%           Vehicles in Mixed Traffic
%   Author: Jiawei Wang, Yang Zheng, Qing Xu and Keqiang Li
% =========================================================================

clc; close all; clear;
addpath('_fcn');
warning off;

% -------------------------------------------------------------------------
%   Parameter setup
% -------------------------------------------------------------------------

% ----------------
% Scenario Setup
% ----------------
% whether traffic flow is mixed
mix                 = 1;                    % 0. all HDVs; 1. there exist CAVs
ID                  = [0,0,1,0,0,1,0,0,0,1,0,0,1,0,0,0];    % ID of vehicle types
                                            % 1: CAV  0: HDV
pos_cav             = find(ID==1);          % position of CAVs
n_vehicle           = length(ID);           % number of vehicles
n_cav               = length(pos_cav);      % number of CAVs
n_hdv               = n_vehicle-n_cav;      % number of HDVs

% perturbation on the head vehicle
per_type            = 2;    % 1. sinuoid perturbation 2. brake perturbation 
sine_amp            = 5;    % amplitidue of sinuoid perturbation
brake_amp           = 10;   % brake amplitude of brake perturbation

% time
total_time          = 80;              % Total Simulation Time
Tstep               = 0.05;            % Time Step
total_time_step     = total_time/Tstep;

% ----------------
% HDV setup
% ----------------
% Type for HDV car-following model
hdv_type            = 1;    % 1. OVM   2. IDM
% Parameter setup for HDV 
data_str            = '2';  % 1. random ovm  2. manual heterogeneous ovm  3. homogeneous ovm
switch hdv_type
    case 1
        load(['_data/hdv_ovm4CAVS_',data_str,'.mat']);
    case 2
        load('_data/hdv_idm.mat');      
end
% Uncertainty for HDV acceleration
acel_noise          = 0.1;  % A white noise signal on HDV's acceleration

% -------------------------------------------------------------------------
%   Formulation for DeeP-LCC
% ------------------------------------------------------------------------- 
% ----------------
% Parameter setup
% ----------------
% Type of the controller
controller_type     = 1;    % 1. DeeP-LCC  2. MPC 
% Initialize Equilibrium Setup (they might be updated in the control process)
v_star              = 15;   % Equilibrium velocity
s_star              = 20;   % Equilibrium spacing for CAV
% Horizon setup 
Tini                = 20;   % length of past data in control process
N                   = 50;   % length of future data in control process
% Performance cost
weight_v            = 1;    % weight coefficient for velocity error
weight_s            = 0.5;  % weight coefficient for spacing error   
weight_u            = 0.1;  % weight coefficient for control input
% Setup in DeeP-LCC
T                   = 2000; % length of data samples
lambda_g            = 100;  % penalty on ||g||_2^2 in objective
lambda_y            = 1e4;  % penalty on ||sigma_y||_2^2 in objective
% Constraints
constraint_bool     = 1;    % whether there exist constraints
acel_max            = 2;    % maximum acceleration
dcel_max            = -5;   % minimum acceleration (maximum deceleration)
spacing_max         = 40;   % maximum spacing
spacing_min         = 5;    % minimum spacing
u_limit             = [dcel_max,acel_max];
s_limit             = [spacing_min,spacing_max]-s_star;
% what signals are measurable (for output definition)
measure_type        = 3;    % 1. Only the velocity errors of all the vehicles are measurable;
                            % 2. All the states, including velocity error and spacing error are measurable;
                            % 3. Velocity error and spacing error of the CAVs are measurable, 
                            %    and the velocity error of the HDVs are measurable.
% equilibrium setup
fixed_spacing_bool      = 0;    % wheter fix equilibrium spacing
                                % 0. the equilibrium spacing will be updated via an OVM-type spacing policy
                                % 1. fix the equilibrium spacing


% ----------------
% Process parameters
% ----------------
n_ctr = 2*n_vehicle;    % number of state variables
m_ctr = n_cav;          % number of input variables
switch measure_type     % number of output variables
    case 1
        p_ctr = n_vehicle;
    case 2
        p_ctr = 2*n_vehicle;
    case 3
        p_ctr = n_vehicle + n_cav;
end

Q_v         = weight_v*eye(n_vehicle);          % penalty for velocity error
Q_s         = weight_s*eye(p_ctr-n_vehicle);    % penalty for spacing error
Q           = blkdiag(Q_v,Q_s);                 % penalty for trajectory error
R           = weight_u*eye(m_ctr);              % penalty for control input

u           = zeros(m_ctr,total_time_step);     % control input
x           = zeros(n_ctr,total_time_step);     % state variables
y           = zeros(p_ctr,total_time_step);     % output variables
pr_status   = zeros(total_time_step,1);         % problem status
e           = zeros(1,total_time_step);         % external input

% ----------------
% Pre-collected data
% ----------------
% load pre-collected data for DeeP-LCC
i_data              = 1;    % id of the pre-collected data
load(['_data\trajectory_data_collection\data_4CAVS_',data_str,'_',num2str(i_data),'_noiseLevel_',num2str(acel_noise),'.mat']);

%Seperate the pre-collected data for subsystems
SU1 = [1, 0, 0, 0];
SUp1_blk = SU1;
SUf1_blk = SU1;

SU2 = [0, 1, 0, 0];
SUp2_blk = SU2;
SUf2_blk = SU2;

SU3 = [0, 0, 1, 0];
SUp3_blk = SU3;
SUf3_blk = SU3;

SU4 = [0, 0, 0, 1];
SUp4_blk = SU4;
SUf4_blk = SU4;

SY1 = zeros(4, 20);
SY1(1, 3) = 1;
SY1(2, 4) = 1;
SY1(3, 5) = 1;
SY1(4, 17) = 1;
SYp1_blk = SY1;
SYf1_blk = SY1;
                    
SY2 = zeros(5, 20);
SY2(1, 6) = 1;
SY2(2, 7) = 1;
SY2(3, 8) = 1;
SY2(4, 9) = 1;
SY2(5, 18) = 1;
SYp2_blk = SY2;
SYf2_blk = SY2;

SY3 = zeros(4, 20);
SY3(1, 10) = 1;
SY3(2, 11) = 1;
SY3(3, 12) = 1;
SY3(4, 19) = 1;
SYp3_blk = SY3;
SYf3_blk = SY3;

SY4 = zeros(5, 20);
SY4(1, 13) = 1;
SY4(2, 14) = 1;
SY4(3, 15) = 1;
SY4(4, 16) = 1;
SY4(5, 20) = 1;
SYp4_blk = SY4;
SYf4_blk = SY4;

SE1 = zeros(1, 20);
SE1(1, 2) = 1;
SEp1_blk = SE1;
SEf1_blk = SE1;

SE2 = zeros(1, 20);
SE2(1, 5) = 1;
SEp2_blk = SE2;
SEf2_blk = SE2;

SE3 = zeros(1, 20);
SE3(1, 9) = 1;
SEp3_blk = SE3;
SEf3_blk = SE3;

SE4 = zeros(1, 20);
SE4(1, 12) = 1;
SEp4_blk = SE4;
SEf4_blk = SE4;

for i = 2:Tini
    SUp1_blk = blkdiag(SUp1_blk,SU1);
    SUp2_blk = blkdiag(SUp2_blk,SU2);
    SUp3_blk = blkdiag(SUp3_blk,SU3);
    SUp4_blk = blkdiag(SUp4_blk,SU4);

    SYp1_blk = blkdiag(SYp1_blk,SY1);
    SYp2_blk = blkdiag(SYp2_blk,SY2);
    SYp3_blk = blkdiag(SYp3_blk,SY3);
    SYp4_blk = blkdiag(SYp4_blk,SY4);
    
    SEp1_blk = blkdiag(SEp1_blk,SE1);
    SEp2_blk = blkdiag(SEp2_blk,SE2);
    SEp3_blk = blkdiag(SEp3_blk,SE3);
    SEp4_blk = blkdiag(SEp4_blk,SE4);
end
for i = 2:N
    SUf1_blk = blkdiag(SUf1_blk,SU1);
    SUf2_blk = blkdiag(SUf2_blk,SU2);
    SUf3_blk = blkdiag(SUf3_blk,SU3);
    SUf4_blk = blkdiag(SUf4_blk,SU4);

    SYf1_blk = blkdiag(SYf1_blk,SY1);
    SYf2_blk = blkdiag(SYf2_blk,SY2);
    SYf3_blk = blkdiag(SYf3_blk,SY3);
    SYf4_blk = blkdiag(SYf4_blk,SY4);
    
    SEf1_blk = blkdiag(SEf1_blk,SE1);
    SEf2_blk = blkdiag(SEf2_blk,SE2);
    SEf3_blk = blkdiag(SEf3_blk,SE3);
    SEf4_blk = blkdiag(SEf4_blk,SE4);
end
Up1 = SUp1_blk * Up;
Up2 = SUp2_blk * Up;
Up3 = SUp3_blk * Up;
Up4 = SUp4_blk * Up;


Yp1 = SYp1_blk * Yp;
Yp2 = SYp2_blk * Yp;
Yp3 = SYp3_blk * Yp;
Yp4 = SYp4_blk * Yp;

Uf1 = SUf1_blk * Uf;
Uf2 = SUf2_blk * Uf;
Uf3 = SUf3_blk * Uf;
Uf4 = SUf4_blk * Uf;

Yf1 = SYf1_blk * Yf;
Yf2 = SYf2_blk * Yf;
Yf3 = SYf3_blk * Yf;
Yf4 = SYf4_blk * Yf;

Ep1 = SEp1_blk * Yp;
Ep2 = SEp2_blk * Yp;
Ep3 = SEp3_blk * Yp;
Ep4 = SEp4_blk * Yp;

Ef1 = SEf1_blk * Yf;
Ef2 = SEf2_blk * Yf;
Ef3 = SEf3_blk * Yf;
Ef4 = SEf4_blk * Yf;


% -------------------------------------------------------------------------
%   Simulation
%--------------------------------------------------------------------------
% Mixed traffic states
% S(time,vehicle id,state variable), in state variable: 1. position; 2. velocity; 3. acceleration
S               = zeros(total_time_step,n_vehicle+1,3); 
S(1,1,1)        = 0;
for i = 2 : n_vehicle+1
    S(1,i,1)    = S(1,i-1,1) - hdv_parameter.s_star(i-1);
end
S(1,:,2)        = v_star * ones(n_vehicle+1,1);

%  reference trajectory is all zeros: stabilize the system to equilibrium
r               = zeros(p_ctr,total_time_step+N); 

% -------------------------------------------------------------------------
%   Experiment starts here
%--------------------------------------------------------------------------
tic

% ------------------
%  Initialization: the CAVs and the head vehicle have zero control input
% ------------------
% initial past data in control process
uini = zeros(m_ctr,Tini);
eini = zeros(1,Tini);
yini = zeros(p_ctr,Tini);

for k = 1:Tini-1
    % calculate acceleration for the HDVs
    acel                =  HDV_dynamics(S(k,:,:),hdv_parameter) ...
                                -acel_noise + 2*acel_noise*rand(n_vehicle,1);
    
    S(k,1,3)           = 0;               % the head vehicle
    S(k,2:end,3)       = acel;            % all the vehicles use HDV model
    S(k,pos_cav+1,3)   = uini(:,k);       % the CAV
    
    % update traffic states
    S(k+1,:,2)          = S(k,:,2) + Tstep*S(k,:,3);
    S(k+1,1,2)          = eini(k) + v_star;          % the velocity of the head vehicle
    S(k+1,:,1)          = S(k,:,1) + Tstep*S(k,:,2);
    
    % update past output data
    yini(:,k)           = measure_mixed_traffic(S(k,2:end,2),S(k,:,1),ID,v_star,s_star,measure_type);
    
end

k_end = k+1;
yini(:,k_end) = measure_mixed_traffic(S(k_end,2:end,2),S(k_end,:,1),ID,v_star,s_star,measure_type);

% update data in u,e,y
u(:,1:Tini) = uini;
e(:,1:Tini) = eini;
y(:,1:Tini) = yini;

% For MPC, which might have infeasible cases
previous_u_opt = 0; 

% ------------------
%  Continue the simulation
% ------------------
for k = Tini:total_time_step-1
    % calculate acceleration for the HDVs
    acel         =  HDV_dynamics(S(k,:,:),hdv_parameter) ...
                    -acel_noise + 2*acel_noise*rand(n_vehicle,1);
    S(k,2:end,3) = acel;     % all the vehicles use HDV model

    
    if mix
        switch controller_type
            case 1  % calculate control input via DeeP-LCC     
                if constraint_bool
                    %Get initial conditions for subsystems
                    uini1 = SU1 * uini;
                    uini2 = SU2 * uini;
                    uini3 = SU3 * uini;
                    uini4 = SU4 * uini;
                    yini1 = SY1 * yini;
                    yini2 = SY2 * yini;
                    yini3 = SY3 * yini;
                    yini4 = SY4 * yini;
                    eini1 = SE1 * yini;
                    eini2 = SE2 * yini; 
                    eini3 = SE3 * yini;
                    eini4 = SE4 * yini; 
                    [u_opt1, pr1] = rb_DeeP_LCC_Bound1(Up1,Yp1,Uf1,Yf1,Ep1,Ef1,uini1,yini1,eini1,weight_v, weight_s, weight_u, r(:,k:k+N-1),...
                        lambda_g,lambda_y,u_limit,s_limit, k-Tini);
                    [u_opt2, pr2] = rb_DeeP_LCC_Bound2(Up2,Yp2,Uf2,Yf2,Ep2,Ef2,uini2,yini2,eini2,weight_v, weight_s, weight_u, r(:,k:k+N-1),...
                        lambda_g,lambda_y,u_limit,s_limit, k-Tini);
                    [u_opt3, pr3] = rb_DeeP_LCC_Bound3(Up3,Yp3,Uf3,Yf3,Ep3,Ef3,uini3,yini3,eini3,weight_v, weight_s, weight_u, r(:,k:k+N-1),...
                        lambda_g,lambda_y,u_limit,s_limit, k-Tini);
                    [u_opt4, pr4] = rb_DeeP_LCC_Bound4(Up4,Yp4,Uf4,Yf4,Ep4,Ef4,uini4,yini4,eini4,weight_v, weight_s, weight_u, r(:,k:k+N-1),...
                        lambda_g,lambda_y,u_limit,s_limit, k-Tini);
%                     flagClip = 1;
%                     [u_opt1, pr1] = rb_DeeP_LCC_Prop1(Up1,Yp1,Uf1,Yf1,Ep1,Ef1,uini1,yini1,eini1,weight_v, weight_s, weight_u, r(:,k:k+N-1),...
%                         lambda_g,lambda_y,u_limit,s_limit, k, dcel_max, acel_max, Tstep, flagClip);
%                     [u_opt2, pr2] = rb_DeeP_LCC_Prop2(Up2,Yp2,Uf2,Yf2,Ep2,Ef2,uini2,yini2,eini2,weight_v, weight_s, weight_u, r(:,k:k+N-1),...
%                         lambda_g,lambda_y,u_limit,s_limit, k, dcel_max, acel_max, Tstep, flagClip);
%                     [u_opt3, pr3] = rb_DeeP_LCC_Prop3(Up3,Yp3,Uf3,Yf3,Ep3,Ef3,uini3,yini3,eini3,weight_v, weight_s, weight_u, r(:,k:k+N-1),...
%                         lambda_g,lambda_y,u_limit,s_limit, k, dcel_max, acel_max, Tstep, flagClip);
%                     [u_opt4, pr4] = rb_DeeP_LCC_Prop4(Up4,Yp4,Uf4,Yf4,Ep4,Ef4,uini4,yini4,eini4,weight_v, weight_s, weight_u, r(:,k:k+N-1),...
%                         lambda_g,lambda_y,u_limit,s_limit, k, dcel_max, acel_max, Tstep, flagClip);
                    u_opt = [u_opt1(1); u_opt2(1); u_opt3(1); u_opt4(1)];
                    if (pr1 || pr2 || pr3 || pr4)
                        pr = 1;
                    else
                        pr = 0;
                    end
%                     [u_opt1,pr] = qp_DeeP_LCC(Up1,Yp1,Uf1,Yf1,Ep1,Ef1,uini1,yini1,eini1,weight_v, weight_s, weight_u,r(:,k:k+N-1),...
%                         lambda_g,lambda_y, u_limit,s_limit);
%                     [u_opt2,pr] = qp_DeeP_LCC(Up2,Yp2,Uf2,Yf2,Ep2,Ef2,uini2,yini2,eini2,weight_v, weight_s, weight_u,r(:,k:k+N-1),...
%                         lambda_g,lambda_y, u_limit,s_limit);
%                     [u_opt3,pr] = qp_DeeP_LCC(Up3,Yp3,Uf3,Yf3,Ep3,Ef3,uini3,yini3,eini3,weight_v, weight_s, weight_u,r(:,k:k+N-1),...
%                         lambda_g,lambda_y, u_limit,s_limit);
%                     [u_opt4,pr] = qp_DeeP_LCC(Up4,Yp4,Uf4,Yf4,Ep4,Ef4,uini4,yini4,eini4,weight_v, weight_s, weight_u,r(:,k:k+N-1),...
%                         lambda_g,lambda_y, u_limit,s_limit);
%                     u_opt = [u_opt1(1); u_opt2(1); u_opt3(1); u_opt4(1)];
                else
                    [u_opt,y_opt,pr] = qp_DeeP_LCC(Up,Yp,Uf,Yf,Ep,Ef,uini,yini,eini,Q,R,r(:,k:k+N-1),...
                        lambda_g,lambda_y);
                end
            case 2  % calculate control input via MPC
                if constraint_bool
                    [u_opt,y_opt,pr] = qp_MPC(ID,Tstep,hdv_type,measure_type,v_star,uini,yini,N,Q,R,r(:,k:k+N-1),u_limit,s_limit,previous_u_opt);
                    previous_u_opt   = u_opt;
                else
                    [u_opt,y_opt,pr] = qp_MPC(ID,Tstep,hdv_type,measure_type,v_star,uini,yini,N,Q,R,r(:,k:k+N-1));                    
                end
         end
        % one-step implementation in receding horizon manner
        u(:,k) = u_opt(1:m_ctr,1);
        % update accleration for the CAV
        S(k,pos_cav+1,3)   = u(:,k);
        % judge whether AEB (automatic emergency braking, which is implemented in the function of 'HDV_dynamics') commands to brake
        brake_vehicle_ID = find(acel==dcel_max);                % the vehicles that need to brake
        brake_cav_ID     = intersect(brake_vehicle_ID,pos_cav); % the CAVs that need to brake
        if ~isempty(brake_cav_ID)
            S(k,brake_cav_ID+1,3) = dcel_max;
        end
        % record problem status
        pr_status(k) = pr;
    end
  
    % update traffic states
    S(k+1,:,2) = S(k,:,2) + Tstep*S(k,:,3);
    % perturbation for the head vehicle
    switch per_type
        case 1
            S(k+1,1,2) = 15 + sine_amp*sin(2*pi/(10/Tstep)*(k-Tini));
            S(k+1,:,1) = S(k,:,1) + Tstep*S(k,:,2);
        case 2
            if (k-Tini)*Tstep < brake_amp/5
                S(k+1,1,3) = -5;
            elseif (k-Tini)*Tstep < brake_amp/5+5
                S(k+1,1,3) = 0;
            elseif (k-Tini)*Tstep < brake_amp/5+5+5
                S(k+1,1,3) = 2;
            else
                S(k+1,1,3) = 0;
            end
            S(k+1,:,2) = S(k,:,2) + Tstep*S(k,:,3);
            S(k+1,:,1) = S(k,:,1) + Tstep*S(k,:,2);
    end
    
    % update equilibrium setup for the CAVs
    v_star = mean(S(k-Tini+1:k,1,2));               % update v_star
    if ~fixed_spacing_bool        
        s_star = acos(1-v_star/30*2)/pi*(35-5) + 5; % update s_star
        
    end
    
    % update past data in control process
    uini = u(:,k-Tini+1:k);
    % the output needs to be re-calculated since the equilibrium might have been updated
    for k_past = k-Tini+1:k
    % Record output
        y(:,k_past) = measure_mixed_traffic(S(k_past,2:end,2),S(k_past,:,1),ID,v_star,s_star,measure_type);
        e(k_past)   = S(k_past,1,2) - v_star;
    end 
    yini = y(:,k-Tini+1:k);
    eini = S(k-Tini+1:k,1,2) - v_star;
    eini = eini';
  
    fprintf('Simulation number: %d  |  process... %2.2f%% \n',i_data,(k-Tini)/total_time_step*100);
    %fprintf('Fixed Spacing: %d',fixed_spacing_bool);
    %fprintf('Current spacing of the first CAV: %4.2f \n',S(k,3,1)-S(k,4,1));
  
end
k_end = k+1;
y(:,k_end) = measure_mixed_traffic(S(k_end,2:end,2),S(k_end,:,1),ID,v_star,s_star,measure_type);

tsim = toc;

fprintf('Simulation ends at %6.4f seconds \n', tsim);

% -------------------------------------------------------------------------
%   Results output
%--------------------------------------------------------------------------
if mix
switch controller_type
    case 1
%         save(['_data\simulation_data\Decentralized_DeeP_LCC\simulation6',data_str,'_',num2str(i_data),'_perType_',num2str(per_type),'_noiseLevel_',num2str(acel_noise),...
%             '_fixSpacing_',num2str(fixed_spacing_bool),...
%             '_hdvType_',num2str(hdv_type),'_lambdaG_',num2str(lambda_g),'_lambdaY_',num2str(lambda_y),'.mat'],...
%             'hdv_type','acel_noise','S','T','Tini','N','ID','Tstep','v_star','pr_status');
        save(['_data\simulation_data\Decentralized_DeeP_LCC\Brake_Simulation_Decentralized.mat'],...
            'hdv_type','acel_noise','S','T','Tini','N','ID','Tstep','v_star','pr_status');

    case 2
        save(['_data\simulation_data\MPC\constrained_simulation\simulation_data',data_str,'_',num2str(i_data),'_perType_',num2str(per_type),'_noiseLevel_',num2str(acel_noise),...
            '_fixSpacing_',num2str(fixed_spacing_bool),...
            '_hdvType_',num2str(hdv_type),'.mat'],...
            'hdv_type','acel_noise','S','T','Tini','N','ID','Tstep','v_star','pr_status');
end
else
    save(['_data\simulation_data\HDV\constrained_simulation\simulation_data',data_str,'_',num2str(i_data),'_perType_',num2str(per_type),'_noiseLevel_',num2str(acel_noise),...
            '_hdvType_',num2str(hdv_type),'.mat'],...
            'hdv_type','acel_noise','S','T','Tini','N','ID','Tstep','v_star');
end


