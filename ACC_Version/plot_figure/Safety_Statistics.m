num_Violation = 0;
num_Emergency = 0;
cav_id = 4;
for i = 1:100
    load(['_data\simulation_data\Controllers\data_set2\Brake_decen_TimeV_T=1500_i_data=',num2str(i),'.mat']);
    spacing_preced = S(:,cav_id,1)-S(:,cav_id+1,1);
    spacing_min = min(spacing_preced);
    spacing_max = max(spacing_preced);
    if (spacing_min < 0 || spacing_max > 45)
        num_Emergency = num_Emergency + 1;
    end
    if (spacing_min < 4 || spacing_max > 41)
        num_break = num_break + 1;
    end
end
