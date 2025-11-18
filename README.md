# AFSMC
Adaptive Fuzzy Sliding Mode Trajectory Tracking Control Using Hexagonal Fuzzy Numbers


AFSMC / HFN-AFSMC Trajectory Tracking Simulator
Version: 1.3
Author: Dr. Amir Ali Mokhtarzadeh
Affiliation: Robotics & AI Laboratory — Huaiyin Institute of Technology
===========================================================================================
** License **

This project is licensed under the Apache License 2.0.
Copyright © 2025 Amir Ali Mokhtarzadeh.
You may use, modify, and distribute this software under the terms of the
Apache License, Version 2.0. A copy of the license is provided in the `LICENSE`
file included with this repository.


** Disclaimer **
This software package and its modules are provided for academic research and educational purposes only.
Not for deployment on real robotic systems without further validation and safety testing.
No warranties are expressed or implied. Use at your own risk.
© 2025 Huaiyin Institute of Technology — Robotics & AI Laboratory





** Overview**
This software reproduces the simulation results and figures from the accompanying paper on AFSMC and HFN-AFSMC control strategies.
It provides a graphical control panel (Tkinter-based GUI) for parameter tuning, running experiments, and generating publication-quality graphs.

**New in Version 1.4 **
1- updating controller.py to unified_controller.py and including all algorithms
2- bringing SMC, AFSMC, and comparisions, all under one gui and all using unified_controller


The package includes:Kinematic/d dynamic robot model (4-wheel omnidirectional).
Unified controller (SMC/AFSMC modes with HFN adaptation).
GUI app for parameter tuning and visualization.
Batch analysis for robustness metrics (RMSE, overshoot, chattering, energy).

Features:
1- Simulation Modes: Single (SMC/AFSMC) or comparison for Case 1 (straight line) and Case 2 (circular trajectory).
2- Parameter Tuning: Interactive GUI for controller (HFN breakpoints, gains), robot/motor params, cases.
3- Visualizations: Trajectory, velocities (linear/angular), errors (e_x/e_y/e_θ individual/combined), wheel velocities.
4- Batch Analysis: Robustness under scenarios (nominal, noise, payload, disturbance); generates tables/CSV (e.g., Table 2).
5- Outputs: High-res PNG graphs, CSV datasets, performance summaries.
6- Extensible: Easy to add disturbances or ablate fuzzy types (e.g., triangular).


Requirements:
Python 3.8+ (tested on 3.12).
Libraries: numpy, matplotlib, pandas, tkinter (standard), dataclasses (3.7+).
No external installs needed (uses stdlib + listed).

Installing libraries via pip:
pip install numpy matplotlib pandas


Installation & Setup:

1- Clone/Download: Place files in a dir (e.g., AFSMC/):afsmc_app_x.py (main GUI).
	afsmc_simulation.py (sim core).
	unified_controller.py (controller logic).
	afsmc_plots.py (visualization).
	gui_frames/ (subdirs with frames: controller_frame.py, etc.).
	utility_metric.py (metrics for tables; optional).

Run: 

cd AFSMC-Simulation
python3 afsmc_app_x.py

1. Launch GUI:
Open app Tabs:
	Controller (HFN/gains),
	Robot & Motor (geometry/inertia),
	Case (initial poses/v_cmd),
	Graphs (select plots),
	Plot Options (limits/colors).
Modes:
	"AFSMC",
	"SMC",
	"Comparison (Case 1)".
	
Case:
	"Case 1 (line)"
	"Case 2 (circle)"—auto-fills params from paper.


2. Single Mode Simulation
	Select mode (e.g., "AFSMC"),
	case,
	check graphs (e.g., "Errors", "Velocities").

Hit Generate graphs Popups PNGs (e.g., single_err_AFSMC_case_1.png) + summary (RMSE ~0.07 rad e_θ).
Outputs: Individual plots (trajectory, v/ω split, e_x/e_y/e_θ split).

3. Comparison Mode
	Select "Comparison (Case 1)" → Pop-up for graph selection (overrides main tab).
	Generate → Individual PNGs (e.g., comparison_e_theta_case_1.png for heading error; comparison_omega_case_1.png for angular velocity).
	Combined optional (e.g., comparison_err_combined_case_1.png).

4. Save/Load Params
	Save Params: JSON of current settings (controller/robot/case).
	Load Params: Restore from JSON.

5- generating comparison bar graphs:
python plot_comparison_err.py (require generate two csv files before with graphs:
	comparision-12-13.csv
	comparison-afsmc-smc-err.csv)

6- Comparison table values:
python3 utility_metric.py
