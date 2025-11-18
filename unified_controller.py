'''
unified_controller.py

Note: this simulation module is part of the AFSMC / HFN-AFSMC research package and is intended
for academic and educational use only. Please cite the corresponding paper when you use or
reproduce these results.
© [2024-2025] Robotics & AI Laboratory — Huaiyin Institute of Technology.
Developed under the supervision of Dr. Amir Ali Mokhtarzadeh for research on 
Adaptive Fuzzy Sliding Mode Control (AFSMC) and trajectory tracking simulation.
 '''

from dataclasses import dataclass
import numpy as np

# -----------------------------
# Defaults (to log/save)
# -----------------------------
DEFAULT_PARAMS = {
    "lambda_": 1.0,          # λ in Eq. (11)
    "l2": 0.5,               # l2 in Eq. (11)
    "k_I": 0.0,              # integral gain for e_θ
    "phi1": 1.0,             # φ1 scaling of y_e
    "phi2": 1.0,             # φ2 scaling of e_θ
    "delta": 0.5,            # Δ in Eq. (13)
    "beta_min": 0.5,         # β_min
    "beta_max": 3.0,         # β_max
    "hfn_breakpoints": [0.0, 0.05, 0.10, 0.15, 0.20, 0.25],
}


@dataclass
class ControllerParams:
    lambda_: float = 1.0
    l2: float = 0.5
    k_I: float = 0.0
    phi1: float = 1.0
    phi2: float = 1.0
    delta: float = 0.5
    beta_min: float = 0.5
    beta_max: float = 3.0
    hfn_breakpoints: tuple = (0.0, 0.05, 0.10, 0.15, 0.20, 0.25)


# -----------------------------
# HFN membership (|e|,|ė|) -> μ
# -----------------------------
def hfn_mu(x, xi, gamma=1.0):
    """
    Hexagonal fuzzy number membership μ(x) as used in the paper.
    xi: (xi1..xi6), see Section 3.1 (HFN).
    """
    xi1, xi2, xi3, xi4, xi5, xi6 = xi

    if x <= xi1 or x >= xi6:
        return 0.0
    elif xi1 < x <= xi2:
        return ((x - xi1) / (xi2 - xi1)) ** gamma
    elif xi2 < x <= xi3:
        return 1.0
    elif xi3 < x <= xi4:
        return 1.0
    elif xi4 < x <= xi5:
        return ((xi5 - x) / (xi5 - xi4)) ** gamma
    else:  # xi5 < x < xi6
        return ((xi6 - x) / (xi6 - xi5)) ** gamma


def hfn_beta(e, e_dot, params: ControllerParams, gamma=1.0):
    """
    Map error magnitude (|e| + 0.5|ė|) to β(t) using HFN membership
    and a simple linear mapping in [β_min, β_max].
    This is the AFSMC adaptation stage in Section 3.1.
    """
    x = abs(e) + 0.5 * abs(e_dot)
    mu = hfn_mu(x, params.hfn_breakpoints, gamma)
    return params.beta_min + (params.beta_max - params.beta_min) * mu


# -----------------------------
# Unified control computation
# -----------------------------
def compute_omega(
    mode: str,
    y_e: float,
    e_theta: float,
    int_e_theta: float,
    y_e_dot: float,
    e_theta_dot: float,
    omega_eq: float,
    params: ControllerParams,
    gamma_hfn: float = 1.0,
):
    """
    Compute angular velocity ω_cmd from sliding surface and switching law.

    mode:
        "SMC"    – constant β (classical SMC)
        "AFSMC"  – β(e, ė) from HFN

    Returns:
        omega_cmd, s, beta_eff
    """

    # Sliding surface: s = λ φ1 y_e + l2 φ2 e_θ + k_I ∫ e_θ dt
    s = (
        params.lambda_ * params.phi1 * y_e
        + params.l2 * params.phi2 * e_theta
        + params.k_I * int_e_theta
    )

    # Choose β
    if mode.upper() == "SMC":
        # classical SMC: fixed β
        beta_eff = params.beta_max

    elif mode.upper() == "AFSMC":
        # AFSMC: HFN-based adaptive β(e, ė)
        e_comb = np.hypot(y_e, e_theta)
        e_comb_dot = np.hypot(y_e_dot, e_theta_dot)
        beta_eff = hfn_beta(e_comb, e_comb_dot, params, gamma=gamma_hfn)

    else:
        raise ValueError(f"Unknown mode: {mode}")

    # switching term with tanh(s/Δ) as smooth sign(·)
    sw = beta_eff * np.tanh(s / params.delta)

    # ω = ω_eq − sw
    omega_cmd = omega_eq - sw

    return omega_cmd, s, beta_eff
