# gui_frames/case_frame.py
#
# Case parameter input frame for AFSMC app.
# gui_frames/case_frame.py
#
# Case parameter input frame for AFSMC app.

# gui_frames/case_frame.py

# gui_frames/case_frame.py

import tkinter as tk
from tkinter import ttk
import numpy as np

from afsmc_simulation import CaseStudyParams


class CaseFrame(ttk.Frame):
    def __init__(self, parent):
        super().__init__(parent)
        self.vars = {}           # StringVar dictionary
        self._build_ui()

    def _build_ui(self):
        params = [
            ("x_ref0",      "Ref x0 [m]"),
            ("y_ref0",      "Ref y0 [m]"),
            ("theta_ref0",  "Ref θ0 [rad]"),
            ("x0",          "Actual x0 [m]"),
            ("y0",          "Actual y0 [m]"),
            ("theta0",      "Actual θ0 [rad]"),
            ("v_cmd",       "v_cmd [m/s]"),
            ("t_end",       "t_end [s]"),
            ("dt",          "dt [s]"),
        ]

        for i, (key, label) in enumerate(params):
            ttk.Label(self, text=label).grid(row=i, column=0, sticky="e", padx=5, pady=2)
            self.vars[key] = tk.StringVar()
            entry = ttk.Entry(self, textvariable=self.vars[key], width=15)
            entry.grid(row=i, column=1, sticky="w", padx=5, pady=2)

        # Buttons
        btn_frame = ttk.Frame(self)
        btn_frame.grid(row=len(params), column=0, columnspan=2, pady=10)
        ttk.Button(btn_frame, text="Case 1 Defaults", command=self._load_case1_defaults).grid(row=0, column=0, padx=5)
        ttk.Button(btn_frame, text="Case 2 Defaults", command=self._load_case2_defaults).grid(row=0, column=1, padx=5)

        # Initial load
        self._load_case1_defaults()

    def _load_case1_defaults(self):
        values = {
            "x_ref0": 1.0, "y_ref0": 0.0, "theta_ref0": np.pi/3,
            "x0": 0.5, "y0": 0.0, "theta0": np.pi/3,
            "v_cmd": 2.5, "t_end": 20.0, "dt": 0.01
        }
        for k, v in values.items():
            self.vars[k].set(f"{v:.6g}")

    def _load_case2_defaults(self):
        values = {
            "x_ref0": 4.0, "y_ref0": 0.0, "theta_ref0": np.pi/2,
            "x0": 4.0, "y0": 2.0, "theta0": 5*np.pi/6,
            "v_cmd": 2.5, "t_end": 20.0, "dt": 0.01
        }
        for k, v in values.items():
            self.vars[k].set(f"{v:.6g}")

    def get_params(self) -> CaseStudyParams:
        try:
            return CaseStudyParams(
                x_ref0=float(self.vars["x_ref0"].get()),
                y_ref0=float(self.vars["y_ref0"].get()),
                theta_ref0=float(self.vars["theta_ref0"].get()),
                x0=float(self.vars["x0"].get()),
                y0=float(self.vars["y0"].get()),
                theta0=float(self.vars["theta0"].get()),
                v_cmd=float(self.vars["v_cmd"].get()),
                t_end=float(self.vars["t_end"].get()),
                dt=float(self.vars["dt"].get()),
            )
        except ValueError as e:
            raise ValueError(f"Invalid parameter in Case frame: {e}")

    def set_params(self, param_dict: dict):
        """Used when loading JSON file."""
        try:
            for key in self.vars:
                if key in param_dict:
                    val = param_dict[key]
                    if hasattr(val, "item"):  # numpy → python scalar
                        val = val.item()
                    self.vars[key].set(f"{val:.6g}")
        except Exception as e:
            print(f"Warning: Failed to set Case params: {e}")

    # ←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←←
    # This method is required by your on_case_change() callback
    def set_initials(self, **kwargs):
        """Kept for backward compatibility with existing app code."""
        for key, value in kwargs.items():
            if key in self.vars:
                self.vars[key].set(f"{value:.6g}")
