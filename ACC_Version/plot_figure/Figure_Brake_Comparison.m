% =========================================================================
%               Analysis for simulation results under same data sample               
% =========================================================================

% clc; clear; close all;

% Data set
data_str         = '2';  % 1. random ovm  2. manual ovm  3. homogeneous ovm
% Mix or not
mix              = 1;    % 0. all HDVs; 1. mix
% Type of the controller
controller_type  = 1;    % 1. DeeP-LCC  2. MPC 
% Type for HDV car-following model
hdv_type         = 1;    % 1. OVM   2. IDM
% Uncertainty for HDV behavior
acel_noise       = 0.1;  % A white noise signal on HDV's original acceleration
% Perturbation amplitude
per_type         = 1;   % 1. sinuoid perturbation 2. brake perturbation 3. ngsim simulation
per_amp          = 5;
% whether there exists constraints
constraint_bool  = 1;
% wheter fix equilibrium spacing
fixed_spacing_bool = 0;

% Simulation Time
begin_time       = 0.05;
end_time         = 50;              

t_start_step = 20;
t_step = 0.05;
t_end_step = end_time / t_step;
flag_update = 0;
v_star = 15;
s_star = 20;
% ID = [0,0,0,1,0,0,0,0];

i_data           = 1;     % Data set number

weight_v     = 1;        % weight coefficient for velocity error
weight_s     = 0.5;      % weight coefficient for spacing error   
weight_u     = 0.1;      % weight coefficient for control input

lambda_g     = 100;      % penalty on ||g||_2^2 in objective
lambda_y     = 1e4;      % penalty on ||sigma_y||_2^2 in objective



if mix
switch controller_type
    case 1
        load('_data\simulation_data\Controllers\Brake_decen_Zero_T=1500.mat');
        S_zero = S;
        load('_data\simulation_data\Controllers\Brake_decen_TimeV_T=1500.mat');
        S_timeV = S;
    case 2
        load(['..\_data\simulation_data\MPC\constrained_simulation\simulation_data',data_str,'_',num2str(i_data),'_perType_',num2str(per_type),'_noiseLevel_',num2str(acel_noise),...
            '_fixSpacing_',num2str(fixed_spacing_bool),...
            '_hdvType_',num2str(hdv_type),'.mat']);
        controller_str = 'MPC';    
end
else
        if constraint_bool
            load(['..\_data\simulation_data\HDV\constrained_simulation\simulation_data',data_str,'_',num2str(i_data),'_perType_',num2str(per_type),'_noiseLevel_',num2str(acel_noise),...
                '_hdvType_',num2str(hdv_type),'.mat']);
            controller_str = 'DeeP-LCC';
        else
            load(['..\_data\simulation_data\HDV\simulation_data',data_str,'_',num2str(i_data),'_perType_',num2str(per_type),'_noiseLevel_',num2str(acel_noise),...
                '_hdvType_',num2str(hdv_type),'_lambdaG_',num2str(lambda_g),'_lambdaY_',num2str(lambda_y),'.mat']);
            controller_str = 'DeeP-LCC';
        end
end


n_vehicle   = length(ID);           % number of vehicles



% -------------------------------------------------------------------------
%   Plot Results
%--------------------------------------------------------------------------
color_gray  = [190 190 190]/255;
color_red   = [244, 53, 124]/255;
color_blue  = [67, 121, 227]/255;
color_black = [0 0 0];
color_orange = [255,132,31]/255;
color_green = [119, 172, 48]/255;
label_size  = 18;
total_size  = 14;
line_width  = 2;

% Velocity
figure;
id_cav = 1;
plot(begin_time:Tstep:end_time,S(begin_time/Tstep:end_time/Tstep,1,2),'Color',color_black,'linewidth',line_width-0.5); hold on;
S = S_zero;
for i = 1:n_vehicle
    if ID(i) == 1
            plot(begin_time:Tstep:end_time,S(begin_time/Tstep:end_time/Tstep,i+1,2),'Color',color_orange,'linewidth',line_width); hold on; % line for velocity of CAVs
            id_cav  = id_cav+1;
    end 
end
S = S_timeV;
for i = 1:n_vehicle
    if ID(i) == 1
            plot(begin_time:Tstep:end_time,S(begin_time/Tstep:end_time/Tstep,i+1,2),'Color',color_green,'linewidth',line_width); hold on; % line for velocity of CAVs
            id_cav  = id_cav+1;
    end 
end
grid on;
set(gca,'TickLabelInterpreter','latex','fontsize',total_size);
set(gca,'YLim',[3 23]);
set(gca,'XLim',[0 50]);

xl = xlabel('$t$ [$\mathrm{s}$]','fontsize',label_size,'Interpreter','latex','Color','k');
yl = ylabel('Velocity [$\mathrm{m/s}$]','fontsize',label_size,'Interpreter','latex','Color','k');

hLegend = legend({'NA','$\mathtt{DeeP-LCC}$','Robust $\mathtt{DeeP-LCC}$'}, 'Interpreter','latex');
hLegend.AutoUpdate = 'off';
hLegend.PlotChildren = hLegend.PlotChildren([2,3]);

set(gcf,'Position',[250 150 400 300]);
fig = gcf;
fig.PaperPositionMode = 'auto';


% Spacing
fig=figure;
id_cav = 1;
S = S_zero;
for i = 1:n_vehicle
   if ID(i) == 1
        plot(begin_time:Tstep:end_time,S(begin_time/Tstep:end_time/Tstep,i,1)-S(begin_time/Tstep:end_time/Tstep,i+1,1),'Color',color_orange,'linewidth',line_width); hold on; % line for velocity of CAVs
        id_cav = id_cav + 1;
   end 
end
S = S_timeV;
for i = 1:n_vehicle
   if ID(i) == 1
        plot(begin_time:Tstep:end_time,S(begin_time/Tstep:end_time/Tstep,i,1)-S(begin_time/Tstep:end_time/Tstep,i+1,1),'Color',color_green,'linewidth',line_width); hold on; % line for velocity of CAVs
        id_cav = id_cav + 1;
   end 
end

plot([0,50], [5, 5], 'r--','LineWidth',1.5)
annotation(fig,'textbox',...
    [0.61,0.29,0.33,0.09],'Color',[1 0 0],...
    'String','Safe Bound',...
    'FontSize',13,...
    'FontName','Times New Roman',...
    'FitBoxToText','off',...
    'EdgeColor','none');
grid on;
set(gca,'TickLabelInterpreter','latex','fontsize',total_size);
set(gca,'YLim',[-5 35]);
set(gca,'XLim',[0 50]);
set(gca,'YTick',-5:10:35);

xl = xlabel('$t$ [$\mathrm{s}$]','fontsize',label_size,'Interpreter','latex','Color','k');
yl = ylabel('Spacing [$\mathrm{m}$]','fontsize',label_size,'Interpreter','latex','Color','k');

legend({'$\mathtt{DeeP-LCC}$', 'Robust $\mathtt{DeeP-LCC}$'}, 'Interpreter','latex',...
          'Position',[0.40,0.75,0.48,0.15]);
set(gcf,'Position',[550 150 400 300]);
fig = gcf;
fig.PaperPositionMode = 'auto';


% Problem status
if mix
figure;
plot(begin_time:Tstep:end_time,pr_status);
set(gcf,'Position',[850 150 400 300]);
fig = gcf;
fig.PaperPositionMode = 'auto';
end

