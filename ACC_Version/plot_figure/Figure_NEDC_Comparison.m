% =========================================================================
%               Analysis for simulation results under same data sample
% =========================================================================

clc; clear; close all;

addpath('arrow.m');

% Data set
data_str        = '2';  % 1. random ovm  2. manual ovm  3. homogeneous ovm
% Mix or not
mix             = 1;    % 0. all HDVs; 1. mix
% Type of the controller
controller_type = 1;    % 1. DeeP-LCC  2. MPC  
% Type for HDV car-following model
hdv_type        = 1;    % 1. OVM   2. IDM
% Uncertainty for HDV behavior
acel_noise      = 0.1;  % A white noise signal on HDV's original acceleration

% Head vehicle trajectory
trajectory_id = '3';
% head_vehicle_trajectory = load(['_data/nedc_modified_v',num2str(trajectory_id),'.mat']);
head_vehicle_trajectory = load(['_data/nedc.mat']);
end_time = head_vehicle_trajectory.time(end);

initialization_time = 30;               % Time for the original HDV-all system to stabilize
adaption_time       = 20;               % Time for the CAVs to adjust to their desired state
total_time          = initialization_time + adaption_time + end_time;  % Total Simulation Time
begin_time          = initialization_time + adaption_time;

weight_v     = 1;        % weight coefficient for velocity error
weight_s     = 0.5;      % weight coefficient for spacing error
weight_u     = 0.1;      % weight coefficient for control input

lambda_g     = 100;      % penalty on ||g||_2^2 in objective
lambda_y     = 1e4;      % penalty on ||sigma_y||_2^2 in objective

i_data          = 1;

%Load data sets
load(['_data\simulation_data\Controllers\NEDC_decen_TimeV_T=1500.mat']);
S_large = S;
load(['_data\simulation_data\Controllers\NEDC_decen_TimeV_T=500.mat']);
S_small = S;

n_vehicle   = length(ID);           % number of vehicles


% -------------------------------------------------------------------------
%   Plot Results
%--------------------------------------------------------------------------
color_gray  = [170 170 170]/255;
color_red   = [244, 53, 124]/255;
color_blue  = [67, 121, 227]/255;
color_black = [0 0 0];
color_blue_2 = [61, 90, 128]/255;
color_red_2  = [238, 108, 77]/255;
label_size  = 18;
total_size  = 14;
line_width  = 1.5;

% Head vehicle trajectory
% time_point = [60,88,121,176,206];  % For Trajectory V1
time_point = [60,88,121,166,196];   % For Trajectory V2
% time_point = [60,108,161,226,276];  % For Trajectory V3
time_scale = (begin_time:Tstep:total_time)-(initialization_time + adaption_time);

figure;
plot(time_scale,S(begin_time/Tstep:round(total_time/Tstep),1,2),'Color',color_black,'linewidth',line_width); hold on;
grid on;
set(gca,'TickLabelInterpreter','latex','fontsize',total_size);
set(gca,'XLim',[begin_time total_time]-(initialization_time + adaption_time));
set(gca,'YLim',[10,30]);


for time_i = 1:4
    plot(time_point(time_i)*ones(20,1)-(initialization_time + adaption_time),linspace(10,S(time_point(time_i)/Tstep,1,2),20),'--','Color',color_blue,'linewidth',line_width); hold on;
    text_phase = text((time_point(time_i)+time_point(time_i+1))/2-(initialization_time + adaption_time),11.5,['phase ',num2str(time_i)],...
        'Interpreter','latex','FontSize',label_size,'HorizontalAlignment','center','Color',color_red);
end




xl = xlabel('$t$ [$\mathrm{s}$]','fontsize',label_size,'Interpreter','latex','Color','k');
yl = ylabel('Velocity [$\mathrm{m/s}$]','fontsize',label_size,'Interpreter','latex','Color','k');

set(gcf,'Position',[250 550 850 300]);
fig = gcf;
fig.PaperPositionMode = 'auto';


total_time          = initialization_time + adaption_time + end_time;  % Total Simulation Time
begin_time          = initialization_time + adaption_time;

figure;
% h = axes('position',[0 0 1 1]);
% axis(h);
plot(time_scale,S(begin_time/Tstep:round(total_time/Tstep),1,2),'Color',color_black,'linewidth',line_width); hold on;
% for i = 1:n_vehicle
%     if ID(i) == 0
%         plot(time_scale,S(begin_time/Tstep:round(total_time/Tstep),i+1,2),'Color',color_gray,'linewidth',line_width/2); hold on; % line for velocity of HDVs
%     end
% end
id_cav = 1;
S = S_large;
for i = 1:n_vehicle
    if ID(i) == 1
            plot(time_scale,S(begin_time/Tstep:round(total_time/Tstep),i+1,2),'Color',color_red,'linewidth',line_width); hold on; % line for velocity of CAVs
            id_cav = id_cav+1;
    end
end
S = S_small;
for i = 1:n_vehicle
    if ID(i) == 1
            plot(time_scale,S(begin_time/Tstep:round(total_time/Tstep),i+1,2),'Color',color_blue,'linewidth',line_width); hold on; % line for velocity of CAVs
            id_cav = id_cav+1;
    end
end

for time_i = 1:4
    plot(time_point(time_i)*ones(20,1)-(initialization_time + adaption_time),linspace(10,S(time_point(time_i)/Tstep,1,2),20),'--','Color',color_blue_2,'linewidth',line_width/2); hold on;
    text_phase = text((time_point(time_i)+time_point(time_i+1))/2-(initialization_time + adaption_time),12.5,['Phase ',num2str(time_i)],...
        'Interpreter','latex','FontSize',label_size,'HorizontalAlignment','center','Color',color_red_2);
end

grid on;
set(gca,'TickLabelInterpreter','latex','fontsize',total_size);
set(gca,'XLim',[time_scale(1) time_scale(end)]);

xl = xlabel('$t$ [$\mathrm{s}$]','fontsize',label_size,'Interpreter','latex','Color','k');
yl = ylabel('Velocity [$\mathrm{m/s}$]','fontsize',label_size,'Interpreter','latex','Color','k');
legend({'T=1500','T=500'})
set(gca,'YLim',[11.5,28.5]);
hLegend = legend({'1', '$T=1500$', '$T=500$'}, 'Interpreter','latex',...
          'Position',[0.16,0.72,0.13,0.15]);
hLegend.AutoUpdate = 'off';
hLegend.PlotChildren = hLegend.PlotChildren([2,3]);
hLegend.EdgeColor = 'none';

set(gcf,'Position',[250 150 850 300]);
fig = gcf;
fig.PaperPositionMode = 'auto';
