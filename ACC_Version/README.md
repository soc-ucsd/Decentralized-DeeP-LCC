The "main_nedc_simulation" can be used to reproduce the result for experiment A and Table II
with controller type 2 (DeeP-LCC), controller type 4 (robust DeeP-LCC) and 
data sets in "_data\trajectory_data_collection" which are "data_T=500_2_1_noise..." and 
"data_T=1500_2_1_noise...".

The results for reproducing Fig. 4 is in "_data\simulation_data\Controllers"
which are "NEDC_decen_TimeV_T=500...", "NEDC_decen_TimeV_T=1500...", "NEDC_decen_Zero_T=500...",
"NEDC_decen_Zero_T=1500...". The figure is plotted by the "Figure_NEDC_Comparison" in "plot_figure". 

The "main_sin_brake_simulation" can be used to reproduce the result for experiment B
with controller type 2 (DeeP-LCC), controller type 4 (robust DeeP-LCC) and 
data sets in "_data\trajectory_data_collection" which are "data_T=500_2_1_noise..." and 
"data_T=1500_2_2_noise...".

The results for reproducing Fig. 5 is in "_data\simulation_data\Controllers"
which are "Brake_decen_TimeV_T=500...", "Brake_decen_TimeV_T=1500...", "Brake_decen_Zero_T=500...",
"Brake_decen_Zero_T=1500...". The figure is plotted by the "Figure_Brake_Comparison" in "plot_figure".

We note that DeeP-LCC will not always cause collision but it has a high probability 
to cause an emergency. We provide the "main_brake_safe_simulation" for run a set of 
trajectories and store the results. For both T=500 and T=1500, we generate 10 trajectories 
to test whether the CAV breaks the safe bound for different controllers. The result 
shows robust DeeP-LCC does not break the safe bound for all test cases while the 
DeeP-LCC breaks the safe bound 7 times for T=500 and 7 times for T=1500 with 2 of 
them are slit violation. Trajectories used for simulation are in 
"_data\trajectory_data_collection\data_set" and results are in 
"_data\simulation_data\Controllers\data_set".
