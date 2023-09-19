% =========================================================================
%               Analysis for simulation results under same data sample               
% =========================================================================

clc; clear; close all;

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
end_time         = 40;              

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


%Load data set
% load('_data\simulation_data\Controllers\Brake_decen_Zero_T=1500.mat');
load('_data\simulation_data\Controllers\data_set2\Brake_decen_Zero_T=1500_i_data=21.mat');
% load('_data\Test\Brake_decen_Zero_T=500.mat');
n_vehicle   = length(ID);           % number of vehicles



% -------------------------------------------------------------------------
%   Plot Results
%--------------------------------------------------------------------------
color_gray  = [190 190 190]/255;
color_red   = [244, 53, 124]/255;
color_blue  = [67, 121, 227]/255;
color_black = [0 0 0];
color_orange = [255,132,31]/255;
label_size  = 18;
total_size  = 14;
line_width  = 2;

% Velocity
figure;
id_cav = 1;
plot(begin_time:Tstep:end_time,S(begin_time/Tstep:end_time/Tstep,1,2),'Color',color_black,'linewidth',line_width-0.5); hold on;
for i = 1:n_vehicle
    if ID(i) == 0
        plot(begin_time:Tstep:end_time,S(begin_time/Tstep:end_time/Tstep,i+1,2),'Color',color_gray,'linewidth',line_width-0.5); hold on; % line for velocity of HDVs
    end
end
for i = 1:n_vehicle
    if ID(i) == 1
        if id_cav == 1
            plot(begin_time:Tstep:end_time,S(begin_time/Tstep:end_time/Tstep,i+1,2),'Color',color_red,'linewidth',line_width); hold on; % line for velocity of CAVs
            id_cav  = id_cav+1;
        elseif id_cav == 2
            plot(begin_time:Tstep:end_time,S(begin_time/Tstep:end_time/Tstep,i+1,2),'Color',color_blue,'linewidth',line_width); hold on;
            id_cav  = id_cav+1;
        elseif id_cav == 3
            plot(begin_time:Tstep:end_time,S(begin_time/Tstep:end_time/Tstep,i+1,2),'Color',color_orange,'linewidth',line_width); hold on;
            id_cav  = id_cav+1;
        elseif id_cav == 4
            plot(begin_time:Tstep:end_time,S(begin_time/Tstep:end_time/Tstep,i+1,2),'Color',"#77AC30",'linewidth',line_width); hold on;
        end
    end 
end
grid on;
set(gca,'TickLabelInterpreter','latex','fontsize',total_size);
set(gca,'YLim',[0 20]);
set(gca,'XLim',[0 50]);

xl = xlabel('$t$ [$\mathrm{s}$]','fontsize',label_size,'Interpreter','latex','Color','k');
yl = ylabel('Velocity [$\mathrm{m/s}$]','fontsize',label_size,'Interpreter','latex','Color','k');

set(gcf,'Position',[250 150 400 300]);
fig = gcf;
fig.PaperPositionMode = 'auto';

% if mix
%     print(gcf,['.\figs\BrakePerturbation_VelocityProfile_Controller_',num2str(controller_type)],'-painters','-depsc2','-r300');
% else
%     print(gcf,'.\figs\BrakePerturbation_VelocityProfile_AllHDVs','-painters','-depsc2','-r300');
% end

% Spacing
fig=figure;
id_cav = 1;
for i = 1:n_vehicle
    if ID(i) == 0
        plot(begin_time:Tstep:end_time,S(begin_time/Tstep:end_time/Tstep,i,1)-S(begin_time/Tstep:end_time/Tstep,i+1,1),'Color',color_gray,'linewidth',line_width-0.5); hold on; % line for velocity of HDVs
    end
end
for i = 1:n_vehicle
   if ID(i) == 1
        if id_cav ==1
        plot(begin_time:Tstep:end_time,S(begin_time/Tstep:end_time/Tstep,i,1)-S(begin_time/Tstep:end_time/Tstep,i+1,1),'Color',color_red,'linewidth',line_width); hold on; % line for velocity of CAVs
        id_cav = id_cav + 1;
        elseif id_cav == 2
        plot(begin_time:Tstep:end_time,S(begin_time/Tstep:end_time/Tstep,i,1)-S(begin_time/Tstep:end_time/Tstep,i+1,1),'Color',color_blue,'linewidth',line_width); hold on; % line for velocity of CAVs
        id_cav = id_cav + 1;
        elseif id_cav == 3
        plot(begin_time:Tstep:end_time,S(begin_time/Tstep:end_time/Tstep,i,1)-S(begin_time/Tstep:end_time/Tstep,i+1,1),'Color',color_orange,'linewidth',line_width); hold on; % line for velocity of CAVs
        id_cav = id_cav + 1;
        elseif id_cav == 4
        plot(begin_time:Tstep:end_time,S(begin_time/Tstep:end_time/Tstep,i,1)-S(begin_time/Tstep:end_time/Tstep,i+1,1),'Color',"#77AC30",'linewidth',line_width); hold on; % line for velocity of CAVs
        end
   end 
end
% if mix
%     plot(begin_time:Tstep:end_time,5*ones(round((end_time-begin_time)/Tstep)+1),'--k','linewidth',line_width);
%     text(11,6.2,'$s_\mathrm{min}=5\,\mathrm{m}$','Interpreter','latex','FontSize',label_size);
% end
plot([0,50], [5, 5], 'r--','LineWidth',1.5)
annotation(fig,'textbox',...
    [0.1455 0.280999998664856 0.334 0.0893333346684774],'Color',[1 0 0],...
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

set(gcf,'Position',[550 150 400 300]);
fig = gcf;
fig.PaperPositionMode = 'auto';

% if mix
%     print(gcf,['.\figs\BrakePerturbation_SpacingProfile_Controller_',num2str(controller_type)],'-painters','-depsc2','-r300');
% else
%     print(gcf,'.\figs\BrakePerturbation_SpacingProfile_AllHDVs','-painters','-depsc2','-r300');
% end

% Problem status
if mix
figure;
plot(begin_time:Tstep:end_time,pr_status);
set(gcf,'Position',[850 150 400 300]);
fig = gcf;
fig.PaperPositionMode = 'auto';
end

% Acceleration
figure;
id_cav = 1;
plot(begin_time:Tstep:end_time,S(begin_time/Tstep:end_time/Tstep,1,3),'Color',color_black,'linewidth',line_width-0.5); hold on;
for i = 1:n_vehicle
    if ID(i) == 1
        if id_cav == 1
            plot(begin_time:Tstep:end_time,S(begin_time/Tstep:end_time/Tstep,i+1,3),'Color',color_red,'linewidth',line_width); hold on; % line for velocity of CAVs
            id_cav  = id_cav+1;
        elseif id_cav == 2
            plot(begin_time:Tstep:end_time,S(begin_time/Tstep:end_time/Tstep,i+1,3),'Color',color_blue,'linewidth',line_width); hold on; % line for velocity of CAVs
            id_cav  = id_cav+1;
        elseif id_cav == 3
            plot(begin_time:Tstep:end_time,S(begin_time/Tstep:end_time/Tstep,i+1,3),'Color',color_orange,'linewidth',line_width); hold on; % line for velocity of CAVs
            id_cav  = id_cav+1;
        elseif id_cav == 4
            plot(begin_time:Tstep:end_time,S(begin_time/Tstep:end_time/Tstep,i+1,3),'Color',"#77AC30",'linewidth',line_width); hold on; % line for velocity of CAVs
        end
    end 
end
grid on;
set(gca,'TickLabelInterpreter','latex','fontsize',total_size);
set(gca,'YLim',[-6 4]);
set(gca,'XLim',[0 50]);

xl = xlabel('$t$ [$\mathrm{s}$]','fontsize',label_size,'Interpreter','latex','Color','k');
yl = ylabel('Acceleration [$\mathrm{m/s^2}$]','fontsize',label_size,'Interpreter','latex','Color','k');

set(gcf,'Position',[250 450 400 300]);
fig = gcf;
fig.PaperPositionMode = 'auto';
% if mix
%     print(gcf,['.\figs\BrakePerturbation_AccelerationProfile_Controller_',num2str(controller_type)],'-painters','-depsc2','-r300');
% else
%     print(gcf,'.\figs\BrakePerturbation_AccelerationProfile_AllHDVs','-painters','-depsc2','-r300');
% end

% -------------------------------------------------------------------------
%   Calculate Performance Indexes
%--------------------------------------------------------------------------

smooth_window = 10;
for i = 2:n_vehicle+1
   S(:,i,3)   = smooth(S(:,i,3),smooth_window); 
end

FuelConsumption = 0;
VelocityError   = 0;
for i=begin_time/Tstep:end_time/Tstep
    R  = 0.333 + 0.00108*S(i,5:end,2).^2 + 1.2*S(i,5:end,3);
    Fuel  = 0.444 + 0.09*R.*S(i,5:end,2) + 0.054 * max(0,S(i,5:end,3)).^2.*S(i,5:end,2);
    Fuel(R <= 0) = 0.444;
    FuelConsumption = FuelConsumption + sum(Fuel)*Tstep;
    
%     VelocityError = VelocityError + sum(abs(S(i,4:end,2)-S(i,1,2))/S(i,1,2));
    VelocityError = VelocityError + sum((S(i,5:end,2)-15).^2);
end

VelocityError = VelocityError/n_vehicle/((end_time-begin_time)/Tstep);

fprintf('Fuel comsumption:   %4.2f \n',FuelConsumption);
fprintf('Velocity error:   %4.2f \n',VelocityError);