'''
utility_metric.py: Standalone test for Table 2 generation

Note: this simulation module is part of the AFSMC / HFN-AFSMC research package and is intended
for academic and educational use only. Please cite the corresponding paper when you use or
reproduce these results.
© [2024-2025] Robotics & AI Laboratory — Huaiyin Institute of Technology.
Developed under the supervision of Dr. Amir Ali Mokhtarzadeh for research on 
Adaptive Fuzzy Sliding Mode Control (AFSMC) and trajectory tracking simulation.
 '''
 
 
import numpy as np
import pandas as pd
from afsmc_simulation import RobotParams, MotorParams, CaseStudyParams, simulate_case1
from unified_controller import DEFAULT_PARAMS, ControllerParams


# Metric definitions
def compute_rmse(e: np.ndarray) -> float:
    return float(np.sqrt(np.mean(e ** 2)))


def compute_energy_avg(u: np.ndarray, t: np.ndarray) -> float:
    T = t[-1] - t[0]
    return float(np.trapz(u ** 2, t) / T)


def compute_overshoot(e: np.ndarray, steady_fraction: float = 0.2) -> float:
    N = len(e)
    steady_window = max(1, int(N * steady_fraction))
    e_ss = float(np.mean(e[-steady_window:]))
    e_max = float(np.max(e))
    if abs(e_ss) < 1e-6:
        # Fall back to absolute value if steady-state is ~0
        return float(abs(e_max) * 100.0)
    return float(100.0 * (e_max - e_ss) / abs(e_ss))


# Metric evaluation for one run
def evaluate_controller_metrics(res: dict):
    t = res["t"]

    # RMSE metrics
    rmse_x = compute_rmse(res["e_x"])
    rmse_y = compute_rmse(res["e_y"])
    rmse_th = compute_rmse(res["e_theta"])

    # Normalised energy on the angular velocity command
    E = compute_energy_avg(res["omega"], t)

    # Overshoot on position error norm
    e_norm = np.hypot(res["e_x"], res["e_y"])
    overshoot = compute_overshoot(e_norm)

    return {
        "rmse_x": rmse_x,
        "rmse_y": rmse_y,
        "rmse_theta": rmse_th,
        "energy": E,
        "overshoot": overshoot,
    }



# Fallback simulation wrapper
def simulate_case1_fallback(mode, ctrl, case, robot, motor, scenario="nominal"):

    res = simulate_case1(mode, ctrl, case, robot, motor)
    res["scenario"] = scenario
    return res



# Batch run over labelled scenarios
def run_batch_test(ctrl, robot, motor, case):
    scenarios = ["nominal", "sensor_noise", "payload_20", "external_dist"]
    np.random.seed(42)

    af_data, smc_data = [], []

    for scen in scenarios:
        print(f"Running {scen} (nominal fallback)...")
        res_af = simulate_case1_fallback("AFSMC", ctrl, case, robot, motor, scenario=scen)
        res_smc = simulate_case1_fallback("SMC", ctrl, case, robot, motor, scenario=scen)

        af_metrics = evaluate_controller_metrics(res_af)
        smc_metrics = evaluate_controller_metrics(res_smc)

        af_data.append({**af_metrics, "scenario": scen})
        smc_data.append({**smc_metrics, "scenario": scen})

    # Average across scenarios (currently just a nominal repeat)
    metric_keys = [k for k in af_data[0].keys() if k != "scenario"]
    af_avg = {k: float(np.mean([m[k] for m in af_data])) for k in metric_keys}
    smc_avg = {k: float(np.mean([m[k] for m in smc_data])) for k in metric_keys}

    # Save CSV for Table 2
    df_avg = pd.DataFrame(
        {"AFSMC": [af_avg[k] for k in metric_keys],
         "SMC": [smc_avg[k] for k in metric_keys]},
        index=metric_keys,
    )
    df_avg.to_csv("table2_test.csv")
    print("Metrics computed! Saved to table2_test.csv")

    print("\nAFSMC Avg:")
    for k in metric_keys:
        print(f"{k}: {af_avg[k]:.4f}")

    print("\nSMC Avg:")
    for k in metric_keys:
        print(f"{k}: {smc_avg[k]:.4f}")



if __name__ == "__main__":
    ctrl = ControllerParams(**DEFAULT_PARAMS)
    robot = RobotParams()
    motor = MotorParams()
    case = CaseStudyParams()
    run_batch_test(ctrl, robot, motor, case)
