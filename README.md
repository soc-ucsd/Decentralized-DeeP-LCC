Decentralized DeeP-LCC
In this project, we present a decentralized DeeP-LCC for control of the Connected and Autonomous Vehicles(CAVs) in mixed traffic. This repository contains the MATLAB scripts for reproducing the experiments in our paper and for further development: (Add the paper)

## Decentralized Data-EnablEd Predictive Leading Cruise Control(Decentralized DeeP-LCC)
DeeP-LCC is a data-driven predictive control strategy for CAVs in mixed traffic, where human-driven vehicles(HDVs) also exist. It aims to deal with unknown nonlinear car-following behaviors of HDVs. Instead of assuming a parametric car-following model, DeeP-LCC uses measurable driving data to construct a data representation of the system and integrates it with MPC to realize achieve safe and optimal control for CAVs. It is adapted from the standard [Data-EnablEd Predictive Control (DeePC)](https://ieeexplore.ieee.org/abstract/document/8795639/) method considering the characteristics of mixed traffic. However, due to the centralized setting, it is non-trivial for a central unit to collect data and compute control inputs within the sampling period when the scale of the mixed traffic system becomes large. Some efforts have been made to transfer the centralized optimization problem into a distributed optimization problem using the Alternating Direction Method of Multipliers(ADMM) but these methods suffer from frequent communication which is also hard to complete during the sampling period. 

To address this problem, we developed the decentralized DeeP − LCC for CAVs in mixed traffic flow. The entire mixed traffic system could be naturally divided into n subsystems, each representing a small mixed traffic system with one CAV *i* and its following *m<sub>i* HDVs. Each CAV’s decentralized controller only uses information from its corresponding subsystem, the preceding vehicle and the head vehicle to compute its individual input and mitigate the traffic waves. Coupling constraints between subsystems are modeled as disturbance and handled through robust optimization by estimating the bound of the disturbance. Thus, it avoids frequent communication for using ADMM to get accurate disturbance values
from neighboring systems. (Need to change the figure)

<img src="docs/img/system_schematic.png" align="center" width="100%"/>

Related projects: 

1. [DeeP-LCC](https://github.com/soc-ucsd/DeeP-LCC)
2. [Distributed DeeP-LCC](https://github.com/wangjw18/Distributed-DeeP-LCC)
3. [Mixed-traffic](https://github.com/soc-ucsd/mixed-traffic)
4. [Leading Cruise Control (LCC)](https://github.com/soc-ucsd/LCC)

## Optimization Formulation

The decentralized DeeP-LCC is formed as a robust optimization problem and is implemented in a receding horizon manner.

<img src="docs/img/problem_formulation.png" align="center" width="55%"/>

## Dependency
The code requires the installation of [Mosek](https://www.mosek.com/)

## Instruction
Files `main_sin_brake_simulation.m` and `main_nedc_simulation.m` are used to simulate three different scenarios(sinusoidal pertubation, NEDC, Braking) and controllers with different formulation and disturbance estimation methods are in the folder `_fcn/Decentralized_DeeP_LCC`.
Currently, `main_sin_brake_simulation.m` and `main_nedc_simulation.m` are ready to run for reproducing results in our paper(add paper link).To use it for the mixed traffic system with different formulations, please:

Note: _Check the variable `ID` in each file used to make sure it corresponds to your formulation_
1. Define and generate parameters for HDVs via `data_generation/data_hdvParameter`;
2. Collect trajectory data for the mixed traffic system from `data_generation/data_trajectoryDataCollection`;
3. Choose proper simulation files, perturbation type and controllers to run the simulation 

# Contact us
To contact us about decentralized DeeP-LCC, email either [Xu Shang](mailto:x3shang@ucsd.edu?Subject=Decentralized DeeP-LCC) or [Yang Zheng](mailto:zhengy@eng.ucsd.edu?Subject=Decentralized DeeP-LCC).


