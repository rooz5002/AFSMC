#!/usr/bin/env python3
'''
plot_comparison.py

Note: this simulation module is part of the AFSMC / HFN-AFSMC research package and is intended
for academic and educational use only. Please cite the corresponding paper when you use or
reproduce these results.
© [2024-2025] Robotics & AI Laboratory — Huaiyin Institute of Technology.
Developed under the supervision of Dr. Amir Ali Mokhtarzadeh for research on 
Adaptive Fuzzy Sliding Mode Control (AFSMC) and trajectory tracking simulation.
 '''
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


# -----------------------------------------------------------
# Helper: RMSE
# -----------------------------------------------------------
def rmse(e):
    return float(np.sqrt(np.mean(e**2)))


# ===========================================================
#   PART 1: RMSE BAR CHART FROM TRAJECTORY CSV
# ===========================================================
def generate_rmse_bar():
    csv_path = "comparision-afsmc-smc-err.csv"
    df = pd.read_csv(csv_path)

    x_ref = df["x_ref"].to_numpy()
    y_ref = df["y_ref"].to_numpy()

    x_smc = df["x_SMC"].to_numpy()
    y_smc = df["y_SMC"].to_numpy()

    x_af = df["x_AFSMC"].to_numpy()
    y_af = df["y_AFSMC"].to_numpy()

    # Compute errors
    ex_smc = x_smc - x_ref
    ey_smc = y_smc - y_ref

    ex_af = x_af - x_ref
    ey_af = y_af - y_ref

    # Compute RMSE
    rmse_x_smc = rmse(ex_smc)
    rmse_y_smc = rmse(ey_smc)

    rmse_x_af = rmse(ex_af)
    rmse_y_af = rmse(ey_af)

    print("\n=== RMSE computed from trajectory CSV ===")
    print(f"SMC   : RMSE_x={rmse_x_smc:.4f}, RMSE_y={rmse_y_smc:.4f}")
    print(f"AFSMC : RMSE_x={rmse_x_af:.4f}, RMSE_y={rmse_y_af:.4f}")

    # Plot bar chart
    labels = ["RMSE_x", "RMSE_y"]
    smc_vals = [rmse_x_smc, rmse_y_smc]
    af_vals = [rmse_x_af, rmse_y_af]

    x = np.arange(len(labels))
    width = 0.35

    fig, ax = plt.subplots(figsize=(6, 4))
    ax.bar(x - width/2, smc_vals, width, label="SMC")
    ax.bar(x + width/2, af_vals, width, label="AFSMC")

    ax.set_xticks(x)
    ax.set_xticklabels(labels)
    ax.set_ylabel("RMSE (m)")
    ax.set_title("RMSE Comparison")
    ax.legend()
    ax.grid(True, axis="y", linestyle=":", linewidth=0.5)

    fig.tight_layout()
    fig.savefig("rmse_bar_from_csv.png", dpi=300)
    print("Saved: rmse_bar_from_csv.png")



# ===========================================================
#   PART 2: ENERGY BAR CHART FROM OMEGA CSV
# ===========================================================
def generate_energy_bar():
    csv_path = "comparision-12-13.csv"
    df = pd.read_csv(csv_path)

    t = df["t"].to_numpy()
    dt = np.mean(np.diff(t))

    omega_smc = df["omega_SMC"].to_numpy()
    omega_af = df["omega_AFSMC"].to_numpy()

    # Energy = ∫ u(t)^2 dt
    energy_smc = np.trapz(omega_smc**2, t)
    energy_af = np.trapz(omega_af**2, t)

    print("\n=== Control Energy computed from OMEGA CSV ===")
    print(f"SMC   : Energy={energy_smc:.6f}")
    print(f"AFSMC : Energy={energy_af:.6f}")

    # Plot bar chart
    labels = ["Control Energy"]
    smc_vals = [energy_smc]
    af_vals = [energy_af]

    x = np.arange(len(labels))
    width = 0.35

    fig, ax = plt.subplots(figsize=(5, 4))
    ax.bar(x - width/2, smc_vals, width, label="SMC")
    ax.bar(x + width/2, af_vals, width, label="AFSMC")

    ax.set_xticks(x)
    ax.set_xticklabels(labels)
    ax.set_ylabel("Energy (integral of ω² dt)")
    ax.set_title("Control Energy Comparison")
    ax.legend()
    ax.grid(True, axis="y", linestyle=":", linewidth=0.5)

    fig.tight_layout()
    fig.savefig("energy_bar_from_omega_csv.png", dpi=300)
    print("Saved: energy_bar_from_omega_csv.png")



# ===========================================================
#   MAIN
# ===========================================================
if __name__ == "__main__":
    generate_rmse_bar()
    generate_energy_bar()
    print("\nAll useful graphs generated successfully.\n")

