%Get trajectories for each subsystem from the trajectory of the whole
%system and the structure of the mixed traffic system
function [ui, ei, yi] = select_traj_subsys(u, e, y, ID)
T = size(u, 2);
pos_cav = find(ID==1);
n_vehicle = length(ID);
n_cav = length(pos_cav);
if ID(1) == 1
    flag_first_cav = 1;
else
    flag_first_cav = 0;
end

%Decide the size of mixed traffic system
ni_vehicle = zeros(1,n_cav);% number of vehicles in each LCC subsystem
for i_cav = 1:n_cav-1
    ni_vehicle(i_cav) = pos_cav(i_cav+1) - pos_cav(i_cav);
end
ni_vehicle(n_cav) = n_vehicle - pos_cav(end) + 1;

ui = cell(n_cav, 1);
ei = cell(n_cav, 1);
yi = cell(n_cav, 1);

%Seperate the trajectory for each subsytem
for i = 1:n_cav
    ui{i} = u(i, :);
    yi{i} = zeros(ni_vehicle(i)+1, T);
    if i ~= n_cav
        yi{i}(1:end-1, :) = y(pos_cav(i):pos_cav(i+1)-1, :);
    else
        yi{i}(1:end-1, :) = y(pos_cav(i):n_vehicle, :);
    end
    yi{i}(end, :) = y(n_vehicle+i, :);
    
    if i == 1 && flag_first_cav == 1
        ei{i} = e;
    else
        ei{i} = y(pos_cav(i)-1,:);
    end
end

end
