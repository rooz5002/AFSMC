'''
gui_frames/case_frame.py

Note: this simulation module is part of the AFSMC / HFN-AFSMC research package and is intended
for academic and educational use only. Please cite the corresponding paper when you use or
reproduce these results.
© [2024-2025] Robotics & AI Laboratory — Huaiyin Institute of Technology.
Developed under the supervision of Dr. Amir Ali Mokhtarzadeh for research on 
Adaptive Fuzzy Sliding Mode Control (AFSMC) and trajectory tracking simulation.
 '''

import tkinter as tk
from tkinter import ttk
import numpy as np

from afsmc_simulation import CaseStudyParams


class CaseFrame(ttk.Frame):
    def __init__(self, parent):
        super().__init__(parent)
        self._build_ui()

    def _build_ui(self):
        # Labels and entries for each param
        params = [
            ("x_ref0", "Ref x0 [m]"),
            ("y_ref0", "Ref y0 [m]"),
            ("theta_ref0", "Ref θ0 [rad]"),
            ("x0", "Actual x0 [m]"),
            ("y0", "Actual y0 [m]"),
            ("theta0", "Actual θ0 [rad]"),
            ("v_cmd", "v_cmd [m/s]"),
            ("t_end", "t_end [s]"),
            ("dt", "dt [s]"),
        ]

        self.entries = {}
        for i, (key, label) in enumerate(params):
            ttk.Label(self, text=label).grid(row=i, column=0, sticky="e", padx=5, pady=2)
            entry = ttk.Entry(self, width=12)
            entry.grid(row=i, column=1, sticky="w", padx=5, pady=2)
            self.entries[key] = entry

        # Default buttons
        btn_frame = ttk.Frame(self)
        btn_frame.grid(row=len(params), column=0, columnspan=2, pady=10)
        ttk.Button(btn_frame, text="Case 1 Defaults", command=self._load_case1_defaults).grid(row=0, column=0, padx=5)
        ttk.Button(btn_frame, text="Case 2 Defaults", command=self._load_case2_defaults).grid(row=0, column=1, padx=5)

        # Set defaults (Case 1)
        self.set_initials(
            x_ref0=1.0, y_ref0=0.0, theta_ref0=np.pi/3,
            x0=0.5, y0=0.0, theta0=np.pi/3,
            v_cmd=2.5, t_end=20.0, dt=0.01
        )

    def _load_case1_defaults(self):
        """Load Case 1 (straight line) defaults."""
        self.set_initials(
            x_ref0=1.0, y_ref0=0.0, theta_ref0=np.pi/3,
            x0=0.5, y0=0.0, theta0=np.pi/3,
            v_cmd=2.5, t_end=20.0, dt=0.01
        )

    def _load_case2_defaults(self):
        """Load Case 2 (circle) defaults from paper."""
        self.set_initials(
            x_ref0=4.0, y_ref0=0.0, theta_ref0=np.pi/2,
            x0=4.0, y0=2.0, theta0=5*np.pi/6,
            v_cmd=2.5, t_end=20.0, dt=0.01
        )

    def get_params(self) -> CaseStudyParams:
        """Parse entries to CaseStudyParams; raise ValueError on invalid input."""
        try:
            return CaseStudyParams(
                x_ref0=float(self.entries["x_ref0"].get()),
                y_ref0=float(self.entries["y_ref0"].get()),
                theta_ref0=float(self.entries["theta_ref0"].get()),
                x0=float(self.entries["x0"].get()),
                y0=float(self.entries["y0"].get()),
                theta0=float(self.entries["theta0"].get()),
                v_cmd=float(self.entries["v_cmd"].get()),
                t_end=float(self.entries["t_end"].get()),
                dt=float(self.entries["dt"].get()),
            )
        except ValueError as e:
            raise ValueError(f"Invalid parameter in Case frame: {e}")

    def set_initials(self, **kwargs):
        """Set entry values from keyword args (e.g., for defaults)."""
        for key, value in kwargs.items():
            if key in self.entries:
                self.entries[key].delete(0, tk.END)
                self.entries[key].insert(0, f"{value:.6g}")
            else:
                print(f"Warning: Unknown param '{key}' for set_initials; skipping.")

    def set_params(self, param_dict: dict):
        """Set entries from loaded JSON dict (for app-level load)."""
        try:
            self.set_initials(**param_dict)  # Reuse set_initials for simplicity
        except Exception as e:
            print(f"Warning: Failed to set Case params: {e}")
