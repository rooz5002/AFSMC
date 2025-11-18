'''
afsmc_simulation.py
Note: this simulation module is part of the AFSMC / HFN-AFSMC research package and is intended
for academic and educational use only. Please cite the corresponding paper when you use or
reproduce these results.
© [2024-2025] Robotics & AI Laboratory — Huaiyin Institute of Technology.
Developed under the supervision of Dr. Amir Ali Mokhtarzadeh for research on 
Adaptive Fuzzy Sliding Mode Control (AFSMC) and trajectory tracking simulation.
 '''

from dataclasses import dataclass
import numpy as np

from unified_controller import ControllerParams, compute_omega
from unified_controller import compute_omega, ControllerParams  # Ensure imported


def wrap_angle(angle: float) -> float:
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


# -------------------------------
# Robot and motor parameter sets
# -------------------------------

@dataclass
class RobotParams:
    # Your basic robot parameters from the manuscript
    mass: float = 1.8            # kg, total weight
    length: float = 0.306        # m, 306 mm
    width: float = 0.281         # m, 281 mm
    wheel_spacing: float = 0.287 # m, 287 mm (distance between left/right wheels)
    wheel_radius: float = 0.066  # m, 66 mm
    inertia: float = 1.35        # kg·m^2, moment of inertia J


@dataclass
class MotorParams:
    # Simple 2nd-order velocity model with limits.
    v_max: float = 3.5           # m/s, maximum no-load speed
    a_max: float = 5           # m/s^2, maximum |dv/dt|
    zeta: float = 0.4            # damping ratio (tune for overshoot)
    omega_n: float = 3.0         # rad/s, natural frequency (tune for settling time)


@dataclass
class CaseStudyParams:
    # Case 1 (straight line), as in the manuscript
    x_ref0: float = 1.0          # ideal initial x
    y_ref0: float = 0.0          # ideal initial y
    theta_ref0: float = np.pi/3  # ideal initial heading

    x0: float = 0.5              # actual initial x
    y0: float = 0.0              # actual initial y
    theta0: float = np.pi/3      # actual initial heading

    v_cmd: float = 2.5           # m/s, commanded linear speed
    t_end: float = 20.0          # s
    dt:   float = 0.01           # s


# -------------------------------
# Simulation with motor dynamics
# -------------------------------

def simulate_case1(
    mode: str,
    ctrl_params: ControllerParams,
    case: CaseStudyParams,
    robot: RobotParams,
    motor: MotorParams,
) -> dict:

    n_steps = int(case.t_end / case.dt) + 1
    t = np.linspace(0.0, case.t_end, n_steps)

    # State and reference
    x = np.zeros(n_steps)
    y = np.zeros(n_steps)
    theta = np.zeros(n_steps)

    x_ref = np.zeros(n_steps)
    y_ref = np.zeros(n_steps)
    theta_ref = np.zeros(n_steps)

    # Errors and control histories (NEW: added e_x)
    e_x = np.zeros(n_steps)      # NEW: Longitudinal error
    e_y = np.zeros(n_steps)
    e_theta = np.zeros(n_steps)
    v = np.zeros(n_steps)
    omega_cmd = np.zeros(n_steps)
    s_hist = np.zeros(n_steps)
    beta_hist = np.zeros(n_steps)

    # Initial conditions (actual and reference)
    x[0] = case.x0
    y[0] = case.y0
    theta[0] = case.theta0

    x_ref[0] = case.x_ref0
    y_ref[0] = case.y_ref0
    theta_ref[0] = case.theta_ref0

    # Integral and derivative states
    int_e_theta = 0.0
    prev_y_e = 0.0
    prev_e_theta = 0.0

    # Motor dynamic states
    v_state = 0.0   # current linear speed
    v_dot = 0.0     # its derivative (dv/dt)

    # Reference trajectory: straight line at constant heading theta_ref0
    v_g = case.v_cmd
    omega_ref_base = 0.0  # Straight line: constant heading, no turn
    for k in range(n_steps):
        alpha = omega_ref_base * t[k]  # alpha=0 for straight
        x_ref[k] = case.x_ref0 + v_g * t[k] * np.cos(theta_ref[0])
        y_ref[k] = case.y_ref0 + v_g * t[k] * np.sin(theta_ref[0])
        theta_ref[k] = theta_ref[0]  # Constant heading

    for k in range(1, n_steps):
        dt = case.dt

        # --- Motor dynamics: 2nd-order with limited dv/dt ---
        v_ref = v_g

        v_ddot = -2.0 * motor.zeta * motor.omega_n * v_dot \
                 - (motor.omega_n ** 2) * (v_state - v_ref)

        v_dot += v_ddot * dt

        if v_dot > motor.a_max:
            v_dot = motor.a_max
        elif v_dot < -motor.a_max:
            v_dot = -motor.a_max

        v_state += v_dot * dt

        if v_state > motor.v_max:
            v_state = motor.v_max
        elif v_state < 0.0:
            v_state = 0.0

        v[k] = v_state


        # --- Position error in global frame ---
        dx = x[k - 1] - x_ref[k - 1]
        dy = y[k - 1] - y_ref[k - 1]

        # Heading error (wrapped)
        e_theta[k - 1] = wrap_angle(theta[k - 1] - theta_ref[k - 1])

        # Longitudinal (along-track) and lateral (cross-track) errors (FIXED: Add e_x)
        e_x[k - 1] = dx * np.cos(theta_ref[k - 1]) + dy * np.sin(theta_ref[k - 1])
        e_y[k - 1] = -dx * np.sin(theta_ref[k - 1]) + dy * np.cos(theta_ref[k - 1])

        # --- Derivatives (backward difference) ---
        y_e_dot = (e_y[k - 1] - prev_y_e) / dt if dt > 0 else 0.0
        e_theta_dot = (e_theta[k - 1] - prev_e_theta) / dt if dt > 0 else 0.0
        prev_y_e = e_y[k - 1]
        prev_e_theta = e_theta[k - 1]

        # --- Integral term for e_theta ---
        int_e_theta += e_theta[k - 1] * dt

        # --- Equivalent angular velocity: nominal for straight line ---
        omega_eq = omega_ref_base

        # --- Unified controller call ---
        omega, s_val, beta_val = compute_omega(
            mode=mode,
            y_e=e_y[k - 1],
            e_theta=e_theta[k - 1],
            int_e_theta=int_e_theta,
            y_e_dot=y_e_dot,
            e_theta_dot=e_theta_dot,
            omega_eq=omega_eq,
            params=ctrl_params,
            gamma_hfn=1.0,
        )

        omega_cmd[k - 1] = omega
        s_hist[k - 1] = s_val
        beta_hist[k - 1] = beta_val

        # --- Kinematic update using actual v_state ---
        x[k] = x[k - 1] + v_state * np.cos(theta[k - 1]) * dt
        y[k] = y[k - 1] + v_state * np.sin(theta[k - 1]) * dt
        theta[k] = theta[k - 1] + omega * dt
        theta[k] = wrap_angle(theta[k])  # Wrap

    # --- Final error (FIXED: Add e_x[-1]) ---
    e_theta[-1] = wrap_angle(theta[-1] - theta_ref[-1])
    dx_last = x[-1] - x_ref[-1]
    dy_last = y[-1] - y_ref[-1]
    e_x[-1] = dx_last * np.cos(theta_ref[-1]) + dy_last * np.sin(theta_ref[-1])
    e_y[-1] = -dx_last * np.sin(theta_ref[-1]) + dy_last * np.cos(theta_ref[-1])

    # --- Wheel velocities from robot geometry ---
    d = robot.wheel_spacing / 2.0
    w_right = (v + omega_cmd * d) / robot.wheel_radius
    w_left = (v - omega_cmd * d) / robot.wheel_radius

    return {
        "t": t,
        "x": x,
        "y": y,
        "theta": theta,
        "x_ref": x_ref,
        "y_ref": y_ref,
        "theta_ref": theta_ref,
        "v": v,
        "omega": omega_cmd,
        "e_x": e_x,     # NEW
        "e_y": e_y,
        "e_theta": e_theta,
        "w_left": w_left,
        "w_right": w_right,
        "s": s_hist,
        "beta": beta_hist,
    }


def simulate_case2(
    mode: str,
    ctrl_params: ControllerParams,
    case: CaseStudyParams,
    robot: RobotParams,
    motor: MotorParams,
) -> dict:
    """
    Case 2 – Circular trajectory tracking (paper: x=4 cos θ, y=4 sin θ, R=4 m).
    """
    # --- Time base (same as Case 1) ---
    n_steps = int(case.t_end / case.dt) + 1
    t = np.linspace(0.0, case.t_end, n_steps)

    # --- Allocate state and reference arrays (NEW: added e_x) ---
    x = np.zeros(n_steps)
    y = np.zeros(n_steps)
    theta = np.zeros(n_steps)

    x_ref = np.zeros(n_steps)
    y_ref = np.zeros(n_steps)
    theta_ref = np.zeros(n_steps)

    e_x = np.zeros(n_steps)      # NEW: Longitudinal error
    e_y = np.zeros(n_steps)
    e_theta = np.zeros(n_steps)
    v = np.zeros(n_steps)
    omega_cmd = np.zeros(n_steps)
    s_hist = np.zeros(n_steps)
    beta_hist = np.zeros(n_steps)

    # --- Circular reference: match paper x=4 cos θ, y=4 sin θ (center 0,0, CCW) ---
    R_curve = 4.0  # Paper radius [m]
    v_g = 2.5  # Paper speed [m/s] (typo fix: not cm/s)
    omega_ref_base = v_g / R_curve  # ~0.625 rad/s

    # Initial ref position from case params; project to circle if off
    x_ref0 = case.x_ref0
    y_ref0 = case.y_ref0
    pos_norm = np.sqrt(x_ref0**2 + y_ref0**2)
    if abs(pos_norm - R_curve) > 1e-3:
        print(f"Warning: Ref initial dist {pos_norm:.3f} m != R={R_curve} m. Projecting direction to circle.")
        scale = R_curve / pos_norm if pos_norm > 0 else 1.0
        x_ref0 *= scale
        y_ref0 *= scale
    phi0 = np.arctan2(y_ref0, x_ref0)  # Initial parameter angle

    alpha = omega_ref_base * t + phi0  # Cumulative parameter (θ in paper)
    x_ref[:] = R_curve * np.cos(alpha)
    y_ref[:] = R_curve * np.sin(alpha)
    theta_ref[:] = (alpha + np.pi / 2) % (2 * np.pi)  # Tangent heading (CCW), wrapped to [-π, π]

    # --- Initial conditions (actual robot: use case params) ---
    x[0] = case.x0
    y[0] = case.y0
    theta[0] = wrap_angle(case.theta0)  # Wrap initial heading

    # --- Integral and derivative states (same as Case 1) ---
    int_e_theta = 0.0
    prev_y_e = 0.0
    prev_e_theta = 0.0

    # --- Motor dynamic states (identical to Case 1) ---
    v_state = 0.0
    v_dot = 0.0

    # ===================== MAIN SIMULATION LOOP =====================
    for k in range(1, n_steps):
        dt = case.dt

        # --- Motor dynamics: 2nd-order with limited dv/dt (copy of Case 1) ---
        v_ref = v_g  # Constant ref speed for circle

        v_ddot = -2.0 * motor.zeta * motor.omega_n * v_dot \
                 - (motor.omega_n ** 2) * (v_state - v_ref)

        v_dot += v_ddot * dt

        if v_dot > motor.a_max:
            v_dot = motor.a_max
        elif v_dot < -motor.a_max:
            v_dot = -motor.a_max

        v_state += v_dot * dt

        if v_state > motor.v_max:
            v_state = motor.v_max
        elif v_state < 0.0:
            v_state = 0.0

        v[k] = v_state


        # --- Position error in global frame ---
        dx = x[k - 1] - x_ref[k - 1]
        dy = y[k - 1] - y_ref[k - 1]

        # Heading error (wrapped)
        e_theta[k - 1] = wrap_angle(theta[k - 1] - theta_ref[k - 1])

        # Longitudinal (along-track) and lateral (cross-track) errors (FIXED: Add e_x)
        e_x[k - 1] = dx * np.cos(theta_ref[k - 1]) + dy * np.sin(theta_ref[k - 1])
        e_y[k - 1] = -dx * np.sin(theta_ref[k - 1]) + dy * np.cos(theta_ref[k - 1])

        # --- Derivatives (backward difference, same as Case 1) ---
        y_e_dot = (e_y[k - 1] - prev_y_e) / dt if dt > 0 else 0.0
        e_theta_dot = (e_theta[k - 1] - prev_e_theta) / dt if dt > 0 else 0.0
        prev_y_e = e_y[k - 1]
        prev_e_theta = e_theta[k - 1]

        # --- Integral term for e_theta ---
        int_e_theta += e_theta[k - 1] * dt

        # --- Equivalent angular velocity: nominal for circle ---
        omega_eq = omega_ref_base

        # --- Unified controller call (identical signature to Case 1) ---
        omega, s_val, beta_val = compute_omega(
            mode=mode,
            y_e=e_y[k - 1],
            e_theta=e_theta[k - 1],
            int_e_theta=int_e_theta,
            y_e_dot=y_e_dot,
            e_theta_dot=e_theta_dot,
            omega_eq=omega_eq,
            params=ctrl_params,
            gamma_hfn=1.0,
        )

        omega_cmd[k - 1] = omega
        s_hist[k - 1] = s_val
        beta_hist[k - 1] = beta_val

        # --- Kinematic update using actual v_state (same as Case 1) ---
        x[k] = x[k - 1] + v_state * np.cos(theta[k - 1]) * dt
        y[k] = y[k - 1] + v_state * np.sin(theta[k - 1]) * dt
        theta[k] = theta[k - 1] + omega * dt
        theta[k] = wrap_angle(theta[k])  # Keep wrapped

    # --- Final error (same pattern as Case 1) ---
    e_theta[-1] = wrap_angle(theta[-1] - theta_ref[-1])
    dx_last = x[-1] - x_ref[-1]
    dy_last = y[-1] - y_ref[-1]
    e_x[-1] = dx_last * np.cos(theta_ref[-1]) + dy_last * np.sin(theta_ref[-1])  # FIXED
    e_y[-1] = -dx_last * np.sin(theta_ref[-1]) + dy_last * np.cos(theta_ref[-1])

    # --- Wheel velocities from robot geometry (same as Case 1) ---
    d = robot.wheel_spacing / 2.0
    w_right = (v + omega_cmd * d) / robot.wheel_radius
    w_left = (v - omega_cmd * d) / robot.wheel_radius

    return {
        "t": t,
        "x": x,
        "y": y,
        "theta": theta,
        "x_ref": x_ref,
        "y_ref": y_ref,
        "theta_ref": theta_ref,
        "v": v,
        "omega": omega_cmd,
        "e_x": e_x,     # NEW
        "e_y": e_y,  # Lateral (matches y_e in paper)
        "e_theta": e_theta,
        "w_left": w_left,
        "w_right": w_right,
        "s": s_hist,
        "beta": beta_hist,
    }

# Patch for afsmc_simulation.py: Add scenario support to simulate_case1

# Full simulate_case1_with_scenario (extends original)
from unified_controller import compute_omega, ControllerParams  # Ensure imported

def simulate_case1_with_scenario(
    mode: str,
    ctrl_params: ControllerParams,
    case: CaseStudyParams,
    robot: RobotParams,
    motor: MotorParams,
    scenario: str = "nominal",  # "nominal", "sensor_noise", "payload_20", "external_dist"
) -> dict:
   
    n_steps = int(case.t_end / case.dt) + 1
    t = np.linspace(0.0, case.t_end, n_steps)

    # State and reference
    x = np.zeros(n_steps)
    y = np.zeros(n_steps)
    theta = np.zeros(n_steps)

    x_ref = np.zeros(n_steps)
    y_ref = np.zeros(n_steps)
    theta_ref = np.zeros(n_steps)

    # Errors and control histories
    e_x = np.zeros(n_steps)
    e_y = np.zeros(n_steps)
    e_theta = np.zeros(n_steps)
    v = np.zeros(n_steps)
    omega_cmd = np.zeros(n_steps)
    s_hist = np.zeros(n_steps)
    beta_hist = np.zeros(n_steps)

    # Scenario-specific
    noise_std_pos = 0.01 if scenario == "sensor_noise" else 0.0
    noise_std_vel = 0.05 if scenario == "sensor_noise" else 0.0
    mass_mult = 1.2 if scenario == "payload_20" else 1.0  # Slows acceleration (inertia proxy)
    dist_amp = 0.1 if scenario == "external_dist" else 0.0

    # Initial conditions with noise
    x[0] = case.x0 + np.random.normal(0, noise_std_pos) if noise_std_pos > 0 else case.x0
    y[0] = case.y0 + np.random.normal(0, noise_std_pos) if noise_std_pos > 0 else case.y0
    theta[0] = wrap_angle(case.theta0 + np.random.normal(0, noise_std_pos/10) if noise_std_pos > 0 else case.theta0)

    # Integral and derivative states
    int_e_theta = 0.0
    prev_y_e = 0.0
    prev_e_theta = 0.0

    # Motor dynamic states
    v_state = 0.0
    v_dot = 0.0

    # MAIN LOOP
    for k in range(1, n_steps):
        dt = case.dt

        # Reference trajectory (straight line)
        alpha = case.theta_ref0
        x_ref[k] = case.x_ref0 + case.v_cmd * t[k] * np.cos(alpha)
        y_ref[k] = case.y_ref0 + case.v_cmd * t[k] * np.sin(alpha)
        theta_ref[k] = alpha

        # Motor dynamics with disturbance and payload effect
        v_ref = case.v_cmd
        v_ddot = -2.0 * motor.zeta * motor.omega_n * v_dot - (motor.omega_n ** 2) * (v_state - v_ref)
        if dist_amp > 0:
            v_ddot += dist_amp * np.sin(2 * np.pi * t[k] / 5.0)  # 5s period disturbance
        v_dot += v_ddot * dt / mass_mult  # Payload slows accel
        v_dot = np.clip(v_dot, -motor.a_max, motor.a_max)
        v_state += v_dot * dt
        v_state = np.clip(v_state, 0.0, motor.v_max)
        v[k] = v_state

        # Position error in global frame with noise
        dx = x[k - 1] + np.random.normal(0, noise_std_pos) - x_ref[k - 1]
        dy = y[k - 1] + np.random.normal(0, noise_std_pos) - y_ref[k - 1]
        e_theta[k - 1] = wrap_angle(theta[k - 1] - theta_ref[k - 1] + np.random.normal(0, noise_std_pos/10))
        e_x[k - 1] = dx * np.cos(theta_ref[k - 1]) + dy * np.sin(theta_ref[k - 1])
        e_y[k - 1] = -dx * np.sin(theta_ref[k - 1]) + dy * np.cos(theta_ref[k - 1])

        # Derivatives with noise
        y_e_dot = (e_y[k - 1] - prev_y_e) / dt + np.random.normal(0, noise_std_vel)
        e_theta_dot = (e_theta[k - 1] - prev_e_theta) / dt + np.random.normal(0, noise_std_vel/10)
        prev_y_e = e_y[k - 1]
        prev_e_theta = e_theta[k - 1]

        # Integral term
        int_e_theta += e_theta[k - 1] * dt

        # Equivalent angular velocity (for straight: tangent correction)
        omega_eq = case.v_cmd * np.tan(e_y[k - 1]) / (case.v_cmd + 1e-6)  # Approx preview

        # Controller call (your unified algo)
        omega, s_val, beta_val = compute_omega(
            mode=mode,
            y_e=e_y[k - 1],
            e_theta=e_theta[k - 1],
            int_e_theta=int_e_theta,
            y_e_dot=y_e_dot,
            e_theta_dot=e_theta_dot,
            omega_eq=omega_eq,
            params=ctrl_params,
            gamma_hfn=1.0,
        )
        omega_cmd[k - 1] = omega
        s_hist[k - 1] = s_val
        beta_hist[k - 1] = beta_val

        # Kinematic update
        x[k] = x[k - 1] + v_state * np.cos(theta[k - 1]) * dt
        y[k] = y[k - 1] + v_state * np.sin(theta[k - 1]) * dt
        theta[k] = wrap_angle(theta[k - 1] + omega * dt)

    # Final errors
    e_theta[-1] = wrap_angle(theta[-1] - theta_ref[-1])
    dx_last = x[-1] - x_ref[-1]
    dy_last = y[-1] - y_ref[-1]
    e_x[-1] = dx_last * np.cos(theta_ref[-1]) + dy_last * np.sin(theta_ref[-1])
    e_y[-1] = -dx_last * np.sin(theta_ref[-1]) + dy_last * np.cos(theta_ref[-1])

    # Wheel velocities
    d = robot.wheel_spacing / 2.0
    w_right = (v + omega_cmd * d) / robot.wheel_radius
    w_left = (v - omega_cmd * d) / robot.wheel_radius

    return {
        "t": t,
        "x": x,
        "y": y,
        "theta": theta,
        "x_ref": x_ref,
        "y_ref": y_ref,
        "theta_ref": theta_ref,
        "v": v,
        "omega": omega_cmd,
        "e_x": e_x,
        "e_y": e_y,
        "e_theta": e_theta,
        "w_left": w_left,
        "w_right": w_right,
        "s": s_hist,
        "beta": beta_hist,
        "scenario": scenario,
    }
    

def compute_rmse(e: np.ndarray) -> float:
    return float(np.sqrt(np.mean(e ** 2)))
