% =========================================================================
%               Analysis for simulation results under same data sample               
% =========================================================================

clc; clear; %close all;

% Data set
data_str        = '2';  % 1. random ovm  2. manual ovm  3. homogeneous ovm
% Type for HDV car-following model
hdv_type        = 1;    % 1. OVM   2. IDM
% Uncertainty for HDV behavior
acel_noise      = 0.1;  % A white noise signal on HDV's original acceleration

% Head vehicle trajectory
trajectory_id = '1';
% head_vehicle_trajectory = load(['./_data/nedc_modified_v',num2str(trajectory_id),'.mat']);
head_vehicle_trajectory = load(['_data/nedc.mat']);
end_time = head_vehicle_trajectory.time(end);

initialization_time = 30;               % Time for the original HDV-all system to stabilize
adaption_time       = 20;               % Time for the CAVs to adjust to their desired state

% Total simulation results
% total_time          = initialization_time + adaption_time + end_time;  % Total Simulation Time
% begin_time          = initialization_time + adaption_time;

% Seperate different phases in the simulation
% time_point = [60,88,121,176,206];  % For Trajectory V1
time_point = [60,88,121,166,196];   % For Trajectory V2
time_i     = 4;
begin_time = time_point(time_i);
total_time = time_point(time_i+1);


weight_v     = 1;        % weight coefficient for velocity error
weight_s     = 0.5;      % weight coefficient for spacing error   
weight_u     = 0.1;      % weight coefficient for control input

lambda_g     = 100;      % penalty on ||g||_2^2 in objective
lambda_y     = 1e4;      % penalty on ||sigma_y||_2^2 in objective

i_data          = 2;


% load(['_data\simulation_data\MPC\nedc_simulation\simulation_data',data_str,'_modified_v',num2str(trajectory_id),'_noiseLevel_',num2str(acel_noise),...
%         '_hdvType_',num2str(hdv_type),'.mat']);
% S_MPC      = S;       
% load(['_data\simulation_data\DeePC\nedc_simulation\simulation_data',data_str,'_',num2str(i_data),'_modified_v',num2str(trajectory_id),'_noiseLevel_',num2str(acel_noise),...
%         '_hdvType_',num2str(hdv_type),'_lambdaG_',num2str(lambda_g),'_lambdaY_',num2str(lambda_y),'.mat']);
% S_DeePC    = S; 
% load(['_data\simulation_data\HDV\nedc_simulation\simulation_data',data_str,'_modified_v',num2str(trajectory_id),'_noiseLevel_',num2str(acel_noise),...
%         '_hdvType_',num2str(hdv_type),'.mat']);
% S_HDV      = S;  
 load(['_data\simulation_data\HDV\NEDC_HDV.mat']);
 S_HDV = S;
 load(['_data\simulation_data\Decentralized_DeeP_LCC\NNEDC_decen_Zero_T=1500.mat']);
 S_DeePC = S;
 load(['_data\simulation_data\Decentralized_DeeP_LCC\NNEDC_decen_TimeV_T=1500.mat']);
 S_rDeePC = S;


n_vehicle   = length(ID);           % number of vehicles
% Smooth acceleration signal
smooth_window = 10;
for i = 2:n_vehicle+1
   S_rDeePC(:,i,3)   = smooth(S_rDeePC(:,i,3),smooth_window); 
   S_DeePC(:,i,3)     = smooth(S_DeePC(:,i,3),smooth_window); 
   S_HDV(:,i,3)     = smooth(S_HDV(:,i,3),smooth_window); 
end

FuelConsumption = zeros(1,3);
VelocityError   = zeros(1,3);



for i=begin_time/Tstep:total_time/Tstep
    R_rDeePC  = 0.333 + 0.00108*S_rDeePC(i,5:end,2).^2 + 1.2*S_rDeePC(i,5:end,3);
    F_rDeePC  = 0.444 + 0.09*R_rDeePC.*S_rDeePC(i,5:end,2) + 0.054 * max(0,S_rDeePC(i,5:end,3)).^2.*S_rDeePC(i,5:end,2);
    F_rDeePC(R_rDeePC <= 0) = 0.444;
    FuelConsumption(1) = FuelConsumption(1) + sum(F_rDeePC)*Tstep;

    R_DeePC  = 0.333 + 0.00108*S_DeePC(i,5:end,2).^2 + 1.2*S_DeePC(i,5:end,3);
    F_DeePC  = 0.444 + 0.09*R_DeePC.*S_DeePC(i,5:end,2) + 0.054 * max(0,S_DeePC(i,5:end,3)).^2.*S_DeePC(i,5:end,2);
    F_DeePC(R_DeePC <= 0) = 0.444;
    FuelConsumption(2) = FuelConsumption(2) + sum(F_DeePC)*Tstep;
    
    R_HDV  = 0.333 + 0.00108*S_HDV(i,5:end,2).^2 + 1.2*S_HDV(i,5:end,3);
    F_HDV  = 0.444 + 0.09*R_HDV.*S_HDV(i,5:end,2) + 0.054 * max(0,S_HDV(i,5:end,3)).^2.*S_HDV(i,5:end,2);
    F_HDV(R_HDV <= 0) = 0.444;
    FuelConsumption(3) = FuelConsumption(3) + sum(F_HDV)*Tstep;      
end

VelocityError = VelocityError/n_vehicle/((total_time-begin_time)/Tstep);

fprintf('Fuel Consumption:   rDeePC  |    DeePC    |   HDVs  \n');
fprintf('                  %4.2f  |  %4.2f  |   %4.2f \n',FuelConsumption);
fprintf('                  %4.2f%%  |  %4.2f%%  |   %4.2f%% \n',(FuelConsumption(3)-FuelConsumption)/FuelConsumption(3)*100);
