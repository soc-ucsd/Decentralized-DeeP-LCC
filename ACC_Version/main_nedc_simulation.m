% =========================================================================
%                               NEDC Simulation
% Scenario:
%       The head vehicle follows a trajectory modified from the Extra-Urban
%       Driving Cycle (EUDC) in New European Driving Circle (NEDC)
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
mix                 = 0;                    % 0. all HDVs; 1. there exist CAVs
ID                  = [0,0,0,1,0,0,0,0];    % ID of vehicle types
                                            % 1: CAV  0: HDV
pos_cav             = find(ID==1);          % position of CAVs
n_vehicle           = length(ID);           % number of vehicles
n_cav               = length(pos_cav);      % number of CAVs
n_hdv               = n_vehicle-n_cav;      % number of HDVs

% Definition for Head vehicle trajectory
head_vehicle_trajectory     = load('_data/nedc.mat');
end_time                    = head_vehicle_trajectory.time(end);    % end time for the head vehicle trajectory
head_vehicle_trajectory.vel = head_vehicle_trajectory.vel/3.6;

% Initialization Time
initialization_time         = 30;               % Time for the original HDV-all system to stabilize
adaption_time               = 20;               % Time for the CAVs to adjust to their desired state
% Total Simulation Time
total_time                  = initialization_time + adaption_time + end_time;  
Tstep                       = 0.05;             % Time Step
total_time_step             = round(total_time/Tstep);
time_cpu_max = 0;
% ----------------
% HDV setup
% ----------------
% Type for HDV car-following model
hdv_type            = 1;    % 1. OVM   2. IDM
% Parameter setup for HDV 
data_str            = '2';  % 1. random ovm  2. manual heterogeneous ovm  3. homogeneous ovm
switch hdv_type
    case 1
        load(['_data/hdv_ovm_',data_str,'.mat']);
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
controller_type     = 2;    % 1. cDeeP-LCC  2. dDeeP-LCC(Zero) 3. dDeeP-LCC(Const) 4. dDeeP-LCC(Time-vary) 
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
T                   = 1500; % length of data samples
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
time_all = zeros(total_time_step-initialization_time/Tstep, 1);
% ----------------
% Process parameters
% ----------------
n_ctr           = 2*n_vehicle;    % number of state variables
m_ctr           = n_cav;          % number of input variables
switch measure_type               
    case 1
        p_ctr   = n_vehicle;      % number of output variables
    case 2
        p_ctr   = 2*n_vehicle;
    case 3
        p_ctr   = n_vehicle + n_cav;
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
load(['_data\trajectory_data_collection\data_','T=',num2str(T),'_',data_str,'_',num2str(i_data),'_noiseLevel_',num2str(acel_noise),'.mat']);

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
%  Initialization: all the vehicles use the HDV model
% ------------------
for k = 1:initialization_time/Tstep-1
    % calculate acceleration
    acel            =  HDV_dynamics(S(k,:,:),hdv_parameter) ...
                            -acel_noise + 2*acel_noise*rand(n_vehicle,1);
    S(k,1,3)        = 0;               % the head vehicle
    S(k,2:end,3)    = acel;            % all the vehicles use HDV model
    
    % update traffic states
    S(k+1,:,2)      = S(k,:,2) + Tstep*S(k,:,3);
    S(k+1,1,2)      = head_vehicle_trajectory.vel(1);    % the velocity of the head vehicle
    S(k+1,:,1)      = S(k,:,1) + Tstep*S(k,:,2);
    
    % update equilibrium velocity
    v_star          = head_vehicle_trajectory.vel(1);
    % update states in DeeP-LCC
    y(:,k)          = measure_mixed_traffic(S(k,2:end,2),S(k,:,1),ID,v_star,s_star,measure_type);
    e(k)            = S(k,1,2) - v_star;
    u(:,k)          = S(k,pos_cav+1,3);
        
end

% update past data in control process
uini                = u(:,k-Tini+1:k);
yini                = y(:,k-Tini+1:k);
eini                = S(k-Tini+1:k,1,2) - v_star;

% ------------------
%  The CAVs start to use DeeP-LCC
% ------------------
for k = initialization_time/Tstep:total_time_step-1
    % calculate acceleration for the HDVs
    acel            =  HDV_dynamics(S(k,:,:),hdv_parameter) ...
                    -acel_noise + 2*acel_noise*rand(n_vehicle,1);
    S(k,2:end,3)    = acel;     
    
    if mix
        [ui_ini, ei_ini, yi_ini] = select_traj_subsys(uini, eini, yini, ID);
        % one-step implementation in receding horizon manner
        u_opt = zeros(m_ctr, 1);
        time_comp = zeros(m_ctr, 1);
        pr = 0;
        switch controller_type
            case 1 %cDeeP-LCC
                controller_str = 'centralized';
                [u_temp, pr_temp, time_comp_temp] = DeeP_LCC_Zero_Dual(Up,Yp,Uf,Yf,Ep,Ef,...
                                                                         uini,yini,eini,weight_v, weight_s, weight_u,...
                                                                         lambda_g,lambda_y,u_limit,s_limit);
                u_opt = u_temp(1:m_ctr, 1);
                time_all(k-Tini+1) = time_comp_temp;
                if (time_comp_temp > time_cpu_max)
                    time_cpu_max = time_comp_temp;
                end
                if pr_temp == 1
                   pr = 1;
                end
            case 2 %dDeeP-LCC(Zero)
                controller_str = 'decen_Zero';
                for i = 1:m_ctr
                    [u_temp, pr_temp, time_comp_temp] = DeeP_LCC_Zero_Dual(Uip{i},Yip{i},Uif{i},Yif{i},Eip{i},Eif{i},...
                                                                         ui_ini{i},yi_ini{i},ei_ini{i},weight_v, weight_s, weight_u,...
                                                                         lambda_g,lambda_y,u_limit,s_limit);
                    u_opt(i) = u_temp(1);
                    time_comp(i) = time_comp_temp;
                    if pr_temp == 1
                        pr = 1;
                    end
                end
                time_temp1 = max(time_comp);
                time_all(k-Tini+1) = time_temp1;
                if (time_temp1 > time_cpu_max)
                    time_cpu_max = time_temp1;
                end
            case 3 %dDeeP-LCC(Constant)
                controller_str = 'decen_Const';
                for i = 1:m_ctr
                    [u_temp, pr_temp, time_comp_temp] = DeeP_LCC_Const_Dual(Uip{i},Yip{i},Uif{i},Yif{i},Eip{i},Eif{i},...
                                                                         ui_ini{i},yi_ini{i},ei_ini{i},weight_v, weight_s, weight_u,...
                                                                         lambda_g,lambda_y,u_limit,s_limit);
                    u_opt(i) = u_temp(1);
                    time_comp(i) = time_comp_temp;
                    if pr_temp == 1
                        pr = 1;
                    end
                end
                time_temp1 = max(time_comp);
                time_all(k-Tini+1) = time_temp1;
                if (time_temp1 > time_cpu_max)
                    time_cpu_max = time_temp1;
                end
            case 4 %dDeeP-LCC(Time-vary)
                controller_str = 'decen_TimeV';
                for i = 1:m_ctr
                    [u_temp, pr_temp, time_comp_temp] = DeeP_LCC_TimeV_Dual(Uip{i},Yip{i},Uif{i},Yif{i},Eip{i},Eif{i},...
                                                                         ui_ini{i},yi_ini{i},ei_ini{i},weight_v, weight_s, weight_u,...
                                                                         lambda_g,lambda_y,u_limit,s_limit,Tstep);
                    u_opt(i) = u_temp(1);
                    time_comp(i) = time_comp_temp;
                    if pr_temp == 1
                        pr = 1;
                    end
                end
                time_temp1 = max(time_comp);
                time_all(k-Tini+1) = time_temp1;
                if (time_temp1 > time_cpu_max)
                    time_cpu_max = time_temp1;
                end
        end
        % one-step implementation in receding horizon manner
        u(:,k) = u_opt;
        % update accleration for the CAV
        S(k,pos_cav+1,3)            = u(:,k);
        % judge whether AEB (automatic emergency braking, which is implemented in the function of 'HDV_dynamics') commands to brake
        brake_vehicle_ID            = find(acel==dcel_max);                % the vehicles that need to brake
        brake_cav_ID                = intersect(brake_vehicle_ID,pos_cav); % the CAVs that need to brake
        if ~isempty(brake_cav_ID)
            S(k,brake_cav_ID+1,3)   = dcel_max;
        end
        % record problem status
        pr_status(k)                = pr;
    end
  
    % update traffic states
    S(k+1,:,2)      = S(k,:,2) + Tstep*S(k,:,3);
    % trajectory for the head vehicle
    % before adaption_time, the head vehicle maintains its velocity and the CAVs first stabilize the traffic system
    if k*Tstep < initialization_time + adaption_time 
        S(k+1,1,2)  = head_vehicle_trajectory.vel(1);
    else
        S(k+1,1,2)  = head_vehicle_trajectory.vel(k-(initialization_time+adaption_time)/Tstep+1);
    end
    S(k+1,:,1)      = S(k,:,1) + Tstep*S(k,:,2);
    
    % update equilibrium setup for the CAVs
    v_star          = mean(S(k-Tini+1:k,1,2));              % average velocity of the head vehicle among the past Tini time
    s_star          = acos(1-v_star/30*2)/pi*(35-5) + 5;    % use the OVM-type spacing policy to calculate the equilibrium spacing of the CAVs
    s_limit = [spacing_min,spacing_max]-s_star;

    % update past data in control process
    uini = u(:,k-Tini+1:k);
    % the output needs to be re-calculated since the equilibrium has been updated
    for k_past = k-Tini+1:k
        y(:,k_past) = measure_mixed_traffic(S(k_past,2:end,2),S(k_past,:,1),ID,v_star,s_star,measure_type);
        e(k_past)   = S(k_past,1,2) - v_star;
    end 
    yini = y(:,k-Tini+1:k);
    eini = S(k-Tini+1:k,1,2) - v_star;
    eini = eini';
    
    fprintf('Current simulation time: %.2f seconds (%.2f%%) \n',k*Tstep,(k*Tstep-initialization_time)/(total_time-initialization_time)*100);
  
end
k_end = k+1;
y(:,k_end) = measure_mixed_traffic(S(k_end,2:end,2),S(k_end,:,1),ID,v_star,s_star,measure_type);

tsim = toc;

fprintf('Simulation ends at %6.4f seconds \n', tsim);

% -------------------------------------------------------------------------
%   Results output
%--------------------------------------------------------------------------
trajectory_id = 1;
if mix
    save(['_data\simulation_data\Controllers\NEDC_',controller_str,'_T=',num2str(T),'.mat'],...
          'hdv_type','acel_noise','S','T','Tini','N','ID','Tstep','v_star','pr_status','time_cpu_max', 'time_all');           
else
    save('_data\simulation_data\HDV\NEDC_HDV.mat',...
         'hdv_type','acel_noise','S','T','Tini','N','ID','Tstep','v_star','pr_status','time_cpu_max', 'time_all');    
end



