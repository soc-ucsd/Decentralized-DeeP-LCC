% =========================================================================
%               Generate heterogeneous HDV paramters
% =========================================================================

clc; close all; clear;

hdv_type = 1;
data_str = 2;
v_star   = 15;

% -------------------
% ID          = [0,0,1,0,0,1,0,0,0,1,0,0,1,0,0,0];    % ID of vehicle types
                                    % 1: CAV  0: HDV
% -------------------

% % Homegeneous setup for OVM
% alpha       = 0.6*ones(8,1);
% beta        = 0.9*ones(8,1);
% s_go        = 25*ones(8,1);
% s_st        = 5;
% v_max       = 30;
% % Equilibrium spacing
% s_star  = acos(1-v_star/v_max*2)./pi*(s_go-s_st) + s_st;
% 
% hdv_parameter = struct('type',hdv_type,...
%             'alpha',alpha,'beta',beta,'s_st',s_st,'s_go',s_go,'v_max',v_max,'s_star',s_star);
% save('_data/hdv_ovm_homogeneous.mat','hdv_parameter');

switch hdv_type
    case 1
        s_st        = 5;
        v_max       = 30;
        switch data_str
            case 1
                % Driver Model: OVM
                alpha       = 0.4 + 0.4*rand(16,1);
                beta        = 0.7 + 0.4*rand(16,1);
                s_go        = 30 + 10*rand(16,1);
            case 2        
                % Manual set for parameters 
% This data set makes zero method diverge, 4 vehicles ahead, correpsonding
% to data1
%                 alpha   = [0.60;0.70;0.70;0.45;0.60;0.40;0.80;...
%                            0.45;0.75];
%                 beta    = [0.90;0.95;0.95;0.60;0.90;0.80;1.00;...
%                            0.60;0.95];
%This data set makes zero method breaks the safety constraints, 3 vehicles
%ahead, corresponding to data2
                alpha   = [0.60;0.70;0.45;0.60;0.40;0.80;...
                           0.45;0.75];
                beta    = [0.90;0.95;0.60;0.90;0.80;1.00;...
                           0.60;0.95];
                s_go    = [35 ; 33; 38; 35; 39; 34 ;...
                            38 ; 31] ;

%                 alpha   = [0.70;0.45;0.60;0.40;0.80;...
%                            0.45;0.75];
%                 beta    = [0.95;0.60;0.90;0.80;1.00;...
%                            0.60;0.95];
%                 s_go    = [33; 38; 35; 39; 34 ;...
%                             38 ; 31] ;

%                 alpha   = [0.60;0.40;0.80;0.45;0.75];
%                 beta    = [0.90;0.80;1.00;0.60;0.95];
%                 s_go    = [35; 39; 34 ;...
%                             38 ; 31] ;
        end
        
        % Consider nominal parameter for the CAV position, which only works
        % in comparison for all the vehicles are HDVs
%         alpha(1)    = 0.6;
%         beta(1)     = 0.9;
%         s_go(1)     = 35;
        

        % Equilibrium spacing
        s_star  = acos(1-v_star/v_max*2)./pi*(s_go-s_st) + s_st;
        
    case 2
        % Driver Model: IDM
        v_max       = 30;
        T_gap       = ones(8,1);
        a           = 1;
        b           = 1.5;
        delta       = 4;
        s_st        = 5;
        
        % Equilibrium spacing
        s_star  = (s_st+T_gap.*v_star)./sqrt(1-(v_star/v_max)^delta);
end

switch hdv_type
    case 1
        hdv_parameter = struct('type',hdv_type,...
            'alpha',alpha,'beta',beta,'s_st',s_st,'s_go',s_go,'v_max',v_max,'s_star',s_star);
        save(['_data/hdv_ovm_',num2str(data_str),'.mat'],'hdv_parameter');
    case 2
        hdv_parameter = struct('type',hdv_type,...
            'v_max',v_max,'T_gap',T_gap,'a',a,'b',b,'delta',delta,'s_st',s_st,'s_star',s_star);
        save('_data/hdv_idm.mat','hdv_parameter');
end