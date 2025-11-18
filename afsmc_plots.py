'''
 afsmc_plots.py
Note: this simulation module is part of the AFSMC / HFN-AFSMC research package and is intended
for academic and educational use only. Please cite the corresponding paper when you use or
reproduce these results.
© [2024-2025] Robotics & AI Laboratory — Huaiyin Institute of Technology.
Developed under the supervision of Dr. Amir Ali Mokhtarzadeh for research on 
Adaptive Fuzzy Sliding Mode Control (AFSMC) and trajectory tracking simulation.
 '''

from dataclasses import dataclass
import numpy as np
import matplotlib.pyplot as plt

from afsmc_simulation import compute_rmse


@dataclass
class PlotConfig:
    xlim: tuple | None = None
    ylim: tuple | None = None
    colors: dict | None = None


def _apply_limits(cfg: PlotConfig, ax=None):
    if ax is None:
        ax = plt.gca()
    if cfg.xlim is not None:
        ax.set_xlim(*cfg.xlim)
    if cfg.ylim is not None:
        ax.set_ylim(*cfg.ylim)


def _get_color(cfg: PlotConfig, key: str, default: str):
    if cfg.colors is None:
        return default
    c = cfg.colors.get(key, "").strip()
    return c if c else default


def plot_single_mode(res: dict, mode: str, selected: dict, cfg: PlotConfig, case_label: str = "Case 1") -> dict:
    t = res["t"]
    datasets = {}

    # Trajectory
    if selected.get("traj", False):
        c_ref = _get_color(cfg, "traj_ref", "k")
        c_robot = _get_color(cfg, "traj_robot", "b")

        plt.figure()
        plt.plot(res["x_ref"], res["y_ref"], label="Reference", color=c_ref)
        plt.plot(res["x"], res["y"], label=mode, color=c_robot)
        plt.xlabel("x [m]")
        plt.ylabel("y [m]")
        if "Case 2" in case_label:
            plt.title("Circular trajectory tracking")
        else:
            plt.title("Linear trajectory tracking (Case 1)")
        plt.legend()
        plt.axis("equal")
        _apply_limits(cfg)  # FIXED: Apply GUI limits after equal aspect
        # FIXED: Save for manuscript, but don't close (let app plt.show() display)
        plt.savefig(f'single_traj_{mode}_{case_label.lower().replace(" ", "_")}.png', dpi=300, bbox_inches='tight')
        datasets[f"Trajectory ({mode})"] = (["t", "x_ref", "y_ref", "x", "y"], np.column_stack([t, res["x_ref"], res["y_ref"], res["x"], res["y"]]))

    # Velocities
    if selected.get("vel", False):
        c_v = _get_color(cfg, "v", "r")
        c_omega = _get_color(cfg, "omega", "b")

        plt.figure()
        plt.plot(t, res["v"], label="v [m/s]", color=c_v)
        plt.plot(t, res["omega"], label="ω [rad/s]", color=c_omega)
        plt.xlabel("Time [s]")
        plt.ylabel("Velocity")
        plt.title("Linear and angular velocity")
        plt.legend()
        _apply_limits(cfg)
        # FIXED: Save, no close
        plt.savefig(f'single_vel_{mode}_{case_label.lower().replace(" ", "_")}.png', dpi=300, bbox_inches='tight')

        datasets[f"Velocities ({mode})"] = (["t", "v", "omega"], np.column_stack([t, res["v"], res["omega"]]))

    # Errors
    if selected.get("err", False):
        c_ex = _get_color(cfg, "e_x", "r")  # NEW: Red for e_x
        c_ey = _get_color(cfg, "e_y", "g")
        c_eth = _get_color(cfg, "e_theta", "m")

        plt.figure()
        plt.plot(t, res["e_x"], label="e_x [m]", color=c_ex)  # NEW
        plt.plot(t, res["e_y"], label="e_y [m]", color=c_ey)
        plt.plot(t, res["e_theta"], label="e_θ [rad]", color=c_eth)
        plt.xlabel("Time [s]")
        plt.ylabel("Error")
        if "Case 2" in case_label:
            plt.title("Circular trajectory tracking error")
        else:
            plt.title("Linear trajectory tracking error")  # Matches paper Fig. 7
        plt.legend()
        _apply_limits(cfg)
        # FIXED: Save, no close
        plt.savefig(f'single_err_{mode}_{case_label.lower().replace(" ", "_")}.png', dpi=300, bbox_inches='tight')

        cols = ["t", "e_x", "e_y", "e_theta"]  # NEW
        data = np.column_stack([t, res["e_x"], res["e_y"], res["e_theta"]])
        datasets[f"Errors ({mode})"] = (cols, data)

    # Wheel velocities
    if selected.get("wheels", False):
        c_wl = _get_color(cfg, "w_left", "c")
        c_wr = _get_color(cfg, "w_right", "y")

        plt.figure()
        plt.plot(t, res["w_left"], label="Left wheels (w1, w3)", color=c_wl)
        plt.plot(t, res["w_right"], label="Right wheels (w2, w4)", color=c_wr)
        plt.xlabel("Time [s]")
        plt.ylabel("Wheel velocity [rad/s]")
        if "Case 2" in case_label:
            plt.title("Wheel velocities (circular tracking)")
        else:
            plt.title("Wheel velocities (straight tracking)")
        plt.legend()
        _apply_limits(cfg)
        # FIXED: Save, no close
        plt.savefig(f'single_wheels_{mode}_{case_label.lower().replace(" ", "_")}.png', dpi=300, bbox_inches='tight')

        datasets[f"Wheel velocities ({mode})"] = (["t", "w_left", "w_right"], np.column_stack([t, res["w_left"], res["w_right"]]))

    return datasets


def plot_comparison(
    res_smc: dict,
    res_af: dict,
    selected: dict,
    cfg: PlotConfig,
    case_label: str = "Case 1",
    generate_extras: bool = True  # For images 14-17
) -> dict:
    t = res_smc["t"]
    datasets = {}

    # Multi-panel subplot if >1 selected
    do_subplots = sum(selected.values()) > 1
    if do_subplots:
        fig, axs = plt.subplots(2, 2, figsize=(12, 10))
        axs = axs.flatten()
        subplot_idx = 0

        # 0: Trajectory
        if selected.get("traj", False):
            c_ref = _get_color(cfg, "traj_ref", "k")
            c_smc = _get_color(cfg, "traj_smc", "b")
            c_af = _get_color(cfg, "traj_af", "r")
            axs[subplot_idx].plot(res_smc["x_ref"], res_smc["y_ref"], label="Reference", color=c_ref, linewidth=2)
            axs[subplot_idx].plot(res_smc["x"], res_smc["y"], label="SMC", color=c_smc, linestyle="-", linewidth=1.5)
            axs[subplot_idx].plot(res_af["x"], res_af["y"], label="AFSMC", color=c_af, linestyle="--", linewidth=1.5)
            axs[subplot_idx].set_xlabel("x [m]")
            axs[subplot_idx].set_ylabel("y [m]")
            axs[subplot_idx].set_title(f"{case_label} Trajectory Comparison")
            axs[subplot_idx].legend()
            axs[subplot_idx].axis("equal")
            _apply_limits(cfg, axs[subplot_idx])
            datasets["Trajectory (SMC vs AFSMC)"] = (
                ["x_ref", "y_ref", "x_SMC", "y_SMC", "x_AFSMC", "y_AFSMC"],
                np.column_stack([res_smc["x_ref"], res_smc["y_ref"], res_smc["x"], res_smc["y"], res_af["x"], res_af["y"]])
            )
            subplot_idx += 1

        # 1: Velocities
        if selected.get("vel", False):
            c_v = _get_color(cfg, "v", "r")
            c_omega = _get_color(cfg, "omega", "g")
            axs[subplot_idx].plot(t, res_smc["v"], label="v SMC", color=c_v, linestyle="-")
            axs[subplot_idx].plot(t, res_smc["omega"], label="ω SMC", color=c_omega, linestyle="-")
            axs[subplot_idx].plot(t, res_af["v"], label="v AFSMC", color=c_v, linestyle="--")
            axs[subplot_idx].plot(t, res_af["omega"], label="ω AFSMC", color=c_omega, linestyle="--")
            axs[subplot_idx].set_xlabel("Time [s]")
            axs[subplot_idx].set_ylabel("Velocity")
            axs[subplot_idx].set_title(f"{case_label} Velocity Comparison")
            axs[subplot_idx].legend()
            _apply_limits(cfg, axs[subplot_idx])
            datasets["Velocities (SMC vs AFSMC)"] = (
                ["t", "v_SMC", "omega_SMC", "v_AFSMC", "omega_AFSMC"],
                np.column_stack([t, res_smc["v"], res_smc["omega"], res_af["v"], res_af["omega"]])
            )
            subplot_idx += 1
            
            # Individual angular ω (Heading Velocity)
            plt.figure(figsize=(8, 6))
            plt.plot(t, res_smc["omega"], label="SMC ω (rad/s)", color=c_omega, linestyle="-")
            plt.plot(t, res_af["omega"], label="AFSMC ω (rad/s)", color=c_omega, linestyle="--")
            plt.xlabel("Time [s]")
            plt.ylabel("Angular Velocity ω [rad/s]")
            plt.title("Angular velocity comparison between AFSMC and SMC for linear trajectory tracking")
            plt.legend()
            _apply_limits(cfg)
            plt.savefig(f'comparison_omega_{case_label.lower().replace(" ", "_")}.png', dpi=300, bbox_inches='tight')
            cols_omega = ["t", "omega_SMC", "omega_AFSMC"]
            data_omega = np.column_stack([t, res_smc["omega"], res_af["omega"]])
            datasets["omega (SMC vs AFSMC)"] = (cols_omega, data_omega)
            
            # Wheel Velocities comparison
            c_wl = _get_color(cfg, "w_left", "c")
            c_wr = _get_color(cfg, "w_right", "y")
            plt.plot(t, res_smc["w_left"], label="Left SMC (w1,w3)", color=c_wl, linestyle="-")
            plt.plot(t, res_smc["w_right"], label="Right SMC (w2,w4)", color=c_wr, linestyle="-")
            plt.plot(t, res_af["w_left"], label="Left AFSMC", color=c_wl, linestyle="--")
            plt.plot(t, res_af["w_right"], label="Right AFSMC", color=c_wr, linestyle="--")
            plt.xlabel("Time [s]")
            plt.ylabel("Wheel Velocity [rad/s]")
            plt.title(f"{case_label} Wheel Velocity Comparison")
            plt.legend()
            _apply_limits(cfg)
            datasets["Wheel velocities (SMC vs AFSMC)"] = (
                ["t", "w_left_SMC", "w_right_SMC", "w_left_AFSMC", "w_right_AFSMC"],
                np.column_stack([t, res_smc["w_left"], res_smc["w_right"], res_af["w_left"], res_af["w_right"]])
            )


        # 2: Errors
        if selected.get("err", False):
            c_ex = _get_color(cfg, "e_x", "orange")
            c_ey = _get_color(cfg, "e_y", "g")
            c_eth = _get_color(cfg, "e_theta", "m")
            axs[subplot_idx].plot(t, res_smc["e_x"], label="e_x SMC", color=c_ex, linestyle="-")
            axs[subplot_idx].plot(t, res_smc["e_y"], label="e_y SMC", color=c_ey, linestyle="-")
            axs[subplot_idx].plot(t, res_smc["e_theta"], label="e_θ SMC", color=c_eth, linestyle="-")
            axs[subplot_idx].plot(t, res_af["e_x"], label="e_x AFSMC", color=c_ex, linestyle="--")
            axs[subplot_idx].plot(t, res_af["e_y"], label="e_y AFSMC", color=c_ey, linestyle="--")
            axs[subplot_idx].plot(t, res_af["e_theta"], label="e_θ AFSMC", color=c_eth, linestyle="--")
            axs[subplot_idx].set_xlabel("Time [s]")
            axs[subplot_idx].set_ylabel("Error")
            axs[subplot_idx].set_title(f"{case_label} Error Comparison")
            axs[subplot_idx].legend()
            _apply_limits(cfg, axs[subplot_idx])
            cols = ["t", "e_x_SMC", "e_y_SMC", "e_theta_SMC", "e_x_AFSMC", "e_y_AFSMC", "e_theta_AFSMC"]
            data = np.column_stack([t, res_smc["e_x"], res_smc["e_y"], res_smc["e_theta"], res_af["e_x"], res_af["e_y"], res_af["e_theta"]])
            datasets["Errors (SMC vs AFSMC)"] = (cols, data)
            subplot_idx += 1
            
            # Individual e_θ (Heading)
            plt.figure(figsize=(8, 6))
            plt.plot(t, res_smc["e_theta"], label="SMC e_θ (rad)", color=c_eth, linestyle="--")
            plt.plot(t, res_af["e_theta"], label="AFSMC e_θ (rad)", color=c_eth, linestyle="-")
            plt.xlabel("Time [s]")
            plt.ylabel("Heading Error e_θ [rad]")
            plt.title("Heading error comparison between AFSMC and SMC for linear trajectory track")
            plt.legend()
            _apply_limits(cfg)
            plt.savefig(f'comparison_e_theta_{case_label.lower().replace(" ", "_")}.png', dpi=300, bbox_inches='tight')
            cols_th = ["t", "e_theta_SMC", "e_theta_AFSMC"]
            data_th = np.column_stack([t, res_smc["e_theta"], res_af["e_theta"]])
            datasets["e_theta (SMC vs AFSMC)"] = (cols_th, data_th)

        # 3: Wheels
        if selected.get("wheels", False):
            c_wl = _get_color(cfg, "w_left", "c")
            c_wr = _get_color(cfg, "w_right", "y")
            axs[subplot_idx].plot(t, res_smc["w_left"], label="Left SMC (w1,w3)", color=c_wl, linestyle="-")
            axs[subplot_idx].plot(t, res_smc["w_right"], label="Right SMC (w2,w4)", color=c_wr, linestyle="-")
            axs[subplot_idx].plot(t, res_af["w_left"], label="Left AFSMC", color=c_wl, linestyle="--")
            axs[subplot_idx].plot(t, res_af["w_right"], label="Right AFSMC", color=c_wr, linestyle="--")
            axs[subplot_idx].set_xlabel("Time [s]")
            axs[subplot_idx].set_ylabel("Wheel Velocity [rad/s]")
            axs[subplot_idx].set_title(f"{case_label} Wheel Velocity Comparison")
            axs[subplot_idx].legend()
            _apply_limits(cfg, axs[subplot_idx])
            datasets["Wheel velocities (SMC vs AFSMC)"] = (
                ["t", "w_left_SMC", "w_right_SMC", "w_left_AFSMC", "w_right_AFSMC"],
                np.column_stack([t, res_smc["w_left"], res_smc["w_right"], res_af["w_left"], res_af["w_right"]])
            )
            subplot_idx += 1

        # Hide unused panels
        for i in range(subplot_idx, 4):
            axs[i].set_visible(False)
        plt.tight_layout()
        # FIXED: Save, but no close
        plt.savefig(f'comparison_subplots_{case_label.lower().replace(" ", "_")}.png', dpi=300, bbox_inches='tight')
        # plt.close()  # REMOVED: Allow display via app's plt.show()

    else:
        # Single-panel fallback (generate individual PNGs)
        for plot_type, enabled in selected.items():
            if not enabled:
                continue
            plt.figure(figsize=(8, 6))
            if plot_type == "traj":
                c_ref = _get_color(cfg, "traj_ref", "k")
                c_smc = _get_color(cfg, "traj_smc", "b")
                c_af = _get_color(cfg, "traj_af", "r")
                plt.plot(res_smc["x_ref"], res_smc["y_ref"], label="Reference", color=c_ref)
                plt.plot(res_smc["x"], res_smc["y"], label="SMC", color=c_smc, linestyle="-")
                plt.plot(res_af["x"], res_af["y"], label="AFSMC", color=c_af, linestyle="--")
                plt.xlabel("x [m]")
                plt.ylabel("y [m]")
                plt.title(f"{case_label} Trajectory Comparison")
                plt.legend()
                plt.axis("equal")
                _apply_limits(cfg)
                datasets["Trajectory (SMC vs AFSMC)"] = (
                    ["x_ref", "y_ref", "x_SMC", "y_SMC", "x_AFSMC", "y_AFSMC"],
                    np.column_stack([res_smc["x_ref"], res_smc["y_ref"], res_smc["x"], res_smc["y"], res_af["x"], res_af["y"]])
                )
                # FIXED: Save, no close
                plt.savefig(f'comparison_traj_{case_label.lower().replace(" ", "_")}.png', dpi=300, bbox_inches='tight')
                # plt.close()  # REMOVED
            elif plot_type == "vel":
                c_v = _get_color(cfg, "v", "r")
                c_omega = _get_color(cfg, "omega", "g")
                plt.plot(t, res_smc["v"], label="v SMC", color=c_v, linestyle="-")
                plt.plot(t, res_smc["omega"], label="ω SMC", color=c_omega, linestyle="-")
                plt.plot(t, res_af["v"], label="v AFSMC", color=c_v, linestyle="--")
                plt.plot(t, res_af["omega"], label="ω AFSMC", color=c_omega, linestyle="--")
                plt.xlabel("Time [s]")
                plt.ylabel("Velocity")
                plt.title(f"{case_label} Velocity Comparison")
                plt.legend()
                _apply_limits(cfg)
                datasets["Velocities (SMC vs AFSMC)"] = (
                    ["t", "v_SMC", "omega_SMC", "v_AFSMC", "omega_AFSMC"],
                    np.column_stack([t, res_smc["v"], res_smc["omega"], res_af["v"], res_af["omega"]])
                )
                # FIXED: Save, no close
                plt.savefig(f'comparison_vel_{case_label.lower().replace(" ", "_")}.png', dpi=300, bbox_inches='tight')
                # plt.close()  # REMOVED
                
                # Individual angular ω (Heading Velocity)
                plt.figure(figsize=(8, 6))
                plt.plot(t, res_smc["omega"], label="SMC ω (rad/s)", color=c_omega, linestyle="-")
                plt.plot(t, res_af["omega"], label="AFSMC ω (rad/s)", color=c_omega, linestyle="--")
                plt.xlabel("Time [s]")
                plt.ylabel("Angular Velocity ω [rad/s]")
                plt.title("Angular velocity comparison between AFSMC and SMC for linear trajectory tracking")
                plt.legend()
                _apply_limits(cfg)
                plt.savefig(f'comparison_omega_{case_label.lower().replace(" ", "_")}.png', dpi=300, bbox_inches='tight')
                cols_omega = ["t", "omega_SMC", "omega_AFSMC"]
                data_omega = np.column_stack([t, res_smc["omega"], res_af["omega"]])
                datasets["omega (SMC vs AFSMC)"] = (cols_omega, data_omega)
                
            elif plot_type == "err":
                c_ex = _get_color(cfg, "e_x", "orange")
                c_ey = _get_color(cfg, "e_y", "g")
                c_eth = _get_color(cfg, "e_theta", "m")
                plt.plot(t, res_smc["e_x"], label="e_x SMC", color=c_ex, linestyle="-")
                plt.plot(t, res_smc["e_y"], label="e_y SMC", color=c_ey, linestyle="-")
                plt.plot(t, res_smc["e_theta"], label="e_θ SMC", color=c_eth, linestyle="-")
                plt.plot(t, res_af["e_x"], label="e_x AFSMC", color=c_ex, linestyle="--")
                plt.plot(t, res_af["e_y"], label="e_y AFSMC", color=c_ey, linestyle="--")
                plt.plot(t, res_af["e_theta"], label="e_θ AFSMC", color=c_eth, linestyle="--")
                plt.xlabel("Time [s]")
                plt.ylabel("Error")
                plt.title(f"{case_label} Error Comparison")
                plt.legend()
                _apply_limits(cfg)
                cols = ["t", "e_x_SMC", "e_y_SMC", "e_theta_SMC", "e_x_AFSMC", "e_y_AFSMC", "e_theta_AFSMC"]
                data = np.column_stack([t, res_smc["e_x"], res_smc["e_y"], res_smc["e_theta"], res_af["e_x"], res_af["e_y"], res_af["e_theta"]])
                datasets["Errors (SMC vs AFSMC)"] = (cols, data)
                # FIXED: Save, no close
                plt.savefig(f'comparison_err_{case_label.lower().replace(" ", "_")}.png', dpi=300, bbox_inches='tight')
                # plt.close()  # REMOVED
            elif plot_type == "wheels":
                c_wl = _get_color(cfg, "w_left", "c")
                c_wr = _get_color(cfg, "w_right", "y")
                plt.plot(t, res_smc["w_left"], label="Left SMC (w1,w3)", color=c_wl, linestyle="-")
                plt.plot(t, res_smc["w_right"], label="Right SMC (w2,w4)", color=c_wr, linestyle="-")
                plt.plot(t, res_af["w_left"], label="Left AFSMC", color=c_wl, linestyle="--")
                plt.plot(t, res_af["w_right"], label="Right AFSMC", color=c_wr, linestyle="--")
                plt.xlabel("Time [s]")
                plt.ylabel("Wheel Velocity [rad/s]")
                plt.title(f"{case_label} Wheel Velocity Comparison")
                plt.legend()
                _apply_limits(cfg)
                datasets["Wheel velocities (SMC vs AFSMC)"] = (
                    ["t", "w_left_SMC", "w_right_SMC", "w_left_AFSMC", "w_right_AFSMC"],
                    np.column_stack([t, res_smc["w_left"], res_smc["w_right"], res_af["w_left"], res_af["w_right"]])
                )
                # FIXED: Save, no close
                plt.savefig(f'comparison_wheels_{case_label.lower().replace(" ", "_")}.png', dpi=300, bbox_inches='tight')
                # plt.close()  # REMOVED

    # Extras for images 14-17 (generate if flag set)
    if generate_extras:
        # Image 14: RMSE Bar Chart
        rmse_metrics = ['e_x', 'e_y', 'e_theta']
        rmse_smc = [compute_rmse(res_smc[m]) for m in rmse_metrics]
        rmse_af = [compute_rmse(res_af[m]) for m in rmse_metrics]
        x = np.arange(len(rmse_metrics))
        width = 0.35
        fig, ax = plt.subplots()
        ax.bar(x - width/2, rmse_smc, width, label='SMC', color='b', alpha=0.8)
        ax.bar(x + width/2, rmse_af, width, label='AFSMC', color='r', alpha=0.8)
        ax.set_xlabel('Error Metric')
        ax.set_ylabel('RMSE')
        ax.set_title(f'{case_label} RMSE Comparison')
        ax.set_xticks(x)
        ax.set_xticklabels(rmse_metrics)
        ax.legend()
        _apply_limits(cfg, ax)
        # FIXED: Save, no close
        plt.savefig('image14_rmse_bar.png', dpi=300)
        # plt.close()  # REMOVED

        # Image 15: Chattering (Switching Surface s over time)
        fig, ax = plt.subplots()
        ax.plot(t, res_smc["s"], label='SMC s(t)', color='b', linestyle='-')
        ax.plot(t, res_af["s"], label='AFSMC s(t)', color='r', linestyle='--')
        ax.set_xlabel('Time [s]')
        ax.set_ylabel('Switching Surface s')
        ax.set_title(f'{case_label} Chattering Analysis (Switching Surface)')
        ax.legend()
        _apply_limits(cfg, ax)
        # FIXED: Save, no close
        plt.savefig('image15_chattering.png', dpi=300)
        # plt.close()  # REMOVED
        datasets["Chattering (s)"] = (["t", "s_SMC", "s_AFSMC"], np.column_stack([t, res_smc["s"], res_af["s"]]))

        # Image 16: Gain Adaptation (beta for AFSMC)
        fig, ax = plt.subplots()
        ax.plot(t, res_af["beta"], label='AFSMC β(t)', color='g', linewidth=2)
        ax.set_xlabel('Time [s]')
        ax.set_ylabel('Gain β')
        ax.set_title(f'{case_label} Fuzzy Gain Adaptation')
        ax.legend()
        _apply_limits(cfg, ax)
        # FIXED: Save, no close
        plt.savefig('image16_gain_adapt.png', dpi=300)
        # plt.close()  # REMOVED
        datasets["Gain Adaptation"] = (["t", "beta_AFSMC"], np.column_stack([t, res_af["beta"]]))

        # Image 17: RMSE Table (as text summary; render as fig if needed)
        rmse_x_smc, rmse_y_smc, rmse_th_smc = [compute_rmse(res_smc[m]) for m in rmse_metrics]
        rmse_x_af, rmse_y_af, rmse_th_af = [compute_rmse(res_af[m]) for m in rmse_metrics]
        summary_text = (
            f"{case_label} RMSE Summary\n"
            f"{'Metric':<10} {'SMC':<8} {'AFSMC':<8}\n"
            f"{'e_x (m)':<10} {rmse_x_smc:<8.4f} {rmse_x_af:<8.4f}\n"
            f"{'e_y (m)':<10} {rmse_y_smc:<8.4f} {rmse_y_af:<8.4f}\n"
            f"{'e_θ (rad)':<10} {rmse_th_smc:<8.4f} {rmse_th_af:<8.4f}\n"
        )
        print(summary_text)  # For console; or use plt.text for fig
        fig, ax = plt.subplots()
        ax.text(0.1, 0.5, summary_text, fontsize=12, va='center')
        ax.set_xlim(0, 1); ax.set_ylim(0, 1)
        ax.axis('off')
        # FIXED: Save, no close
        plt.savefig('image17_rmse_table.png', dpi=300)
        # plt.close()  # REMOVED

    return datasets


def summary_single(res: dict, mode: str) -> str:
    rmse_y = compute_rmse(res["e_y"])
    rmse_theta = compute_rmse(res["e_theta"])
    rmse_x = compute_rmse(res["e_x"])
    msg = (
        f"Mode: {mode}\n\n"
        f"RMSE(e_x)      = {rmse_x:.6f} m\n"  # NEW
        f"RMSE(e_y)      = {rmse_y:.6f} m\n"
        f"RMSE(e_theta)  = {rmse_theta:.6f} rad\n"
        f"Final e_x      = {res['e_x'][-1]:.6f} m\n"  # NEW
        f"Final e_y      = {res['e_y'][-1]:.6f} m\n"
        f"Final e_theta  = {res['e_theta'][-1]:.6f} rad\n"
    )
    return msg


def summary_comparison(res_smc: dict, res_af: dict) -> str:
    rmse_x_smc = compute_rmse(res_smc["e_x"])
    rmse_y_smc = compute_rmse(res_smc["e_y"])
    rmse_th_smc = compute_rmse(res_smc["e_theta"])
    rmse_x_af = compute_rmse(res_af["e_x"])
    rmse_y_af = compute_rmse(res_af["e_y"])
    rmse_th_af = compute_rmse(res_af["e_theta"])

    msg = (
        "Case 1 comparison (same parameters)\n\n"
        f"SMC:    RMSE(e_x) = {rmse_x_smc:.6f} m, RMSE(e_y) = {rmse_y_smc:.6f} m, "
        f"RMSE(e_theta) = {rmse_th_smc:.6f} rad\n"
        f"AFSMC:  RMSE(e_x) = {rmse_x_af:.6f} m, RMSE(e_y) = {rmse_y_af:.6f} m, "
        f"RMSE(e_theta) = {rmse_th_af:.6f} rad\n"
    )
    return msg

# Patch for afsmc_plots.py: Metrics for batch comparison

def compute_settling_time(e: np.ndarray, t: np.ndarray, tol: float = 0.05) -> float:
    """Time when |e| stays < tol thereafter."""
    for i in range(len(e) - 1, 0, -1):
        if np.all(np.abs(e[i:]) < tol):
            return t[i]
    return t[-1]  # If never settles

def compute_chattering_index(s: np.ndarray) -> float:
    """Std of 2nd derivative (high-freq content)."""
    if len(s) < 3:
        return 0.0
    s_diff2 = np.diff(np.diff(s))
    return np.std(s_diff2)

def compute_overshoot(e: np.ndarray) -> float:
    """Max absolute error (overshoot proxy)."""
    return float(np.max(np.abs(e)))

def compute_energy(omega: np.ndarray, t: np.ndarray) -> float:
    """Integral of ω^2 dt (control energy)."""
    return float(np.trapz(omega**2, t))

def batch_metrics(res: dict) -> dict:
    """Compute all metrics from res dict."""
    t = res["t"]
    return {
        "rmse_x": compute_rmse(res["e_x"]),
        "rmse_y": compute_rmse(res["e_y"]),
        "settling": compute_settling_time(np.hypot(res["e_x"], res["e_y"]), t),
        "chattering": compute_chattering_index(res["s"]),
        "overshoot": compute_overshoot(np.hypot(res["e_x"], res["e_y"])),  # Combined for simplicity
        "energy": compute_energy(res["omega"], t),
    }
    
    
