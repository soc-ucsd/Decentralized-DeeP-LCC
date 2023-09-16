%Calculate the closed-loop cost in the form of \sum u^T R u + (y-r)^T R y
function cost = calculateCLCost(S, weight_v, weight_s, weight_u, t_start_step,...
                         v_star, s_star, flag_update, t_end_step, ID)
cost = 0;
pos_cav = find(ID == 1);
v_temp = v_star;
s_temp = s_star;
T_ini = 20;

for i = t_start_step:t_end_step
    if flag_update
        v_temp = mean(S(i-T_ini+1:i,1,2));
        s_temp = acos(1-v_star/30*2)/pi*(35-5) + 5;
    end
    ucost_temp = weight_u * sum(S(i, pos_cav+1 ,3).^2);
    vcost_temp = weight_v * sum((S(i, 4:end, 2)-v_temp).^2);
    scost_temp = weight_s * sum((S(i, pos_cav+1, 1)-s_temp).^2);
    cost_temp = ucost_temp + vcost_temp + scost_temp;
    cost = cost + cost_temp;
end
end