'''
gui_frames/controller_frame.py

Note: this simulation module is part of the AFSMC / HFN-AFSMC research package and is intended
for academic and educational use only. Please cite the corresponding paper when you use or
reproduce these results.
© [2024-2025] Robotics & AI Laboratory — Huaiyin Institute of Technology.
Developed under the supervision of Dr. Amir Ali Mokhtarzadeh for research on 
Adaptive Fuzzy Sliding Mode Control (AFSMC) and trajectory tracking simulation.
 '''

import tkinter as tk
from tkinter import ttk

from unified_controller import ControllerParams, DEFAULT_PARAMS


class ControllerFrame(ttk.LabelFrame):
    def __init__(self, parent):
        super().__init__(parent, text="Controller parameters")
        self.entries = {}
        self._build()
        self._load_defaults()

    def _build(self):
        fields = [
            ("lambda_",         "λ"),
            ("l2",              "l2"),
            ("k_I",             "k_I"),
            ("phi1",            "φ1"),
            ("phi2",            "φ2"),
            ("delta",           "Δ"),
            ("beta_min",        "β_min"),
            ("beta_max",        "β_max"),
            ("hfn_breakpoints", "HFN breakpoints"),
        ]

        for i, (key, label) in enumerate(fields):
            ttk.Label(self, text=label + ":").grid(
                row=i, column=0, sticky="e", padx=5, pady=2
            )
            e = ttk.Entry(self, width=28)
            e.grid(row=i, column=1, sticky="w", padx=5, pady=2)
            self.entries[key] = e

        # Optional hint row for breakpoints
        ttk.Label(
            self,
            text="HFN breakpoints (comma-separated), e.g. 0,0.05,0.1,0.15,0.2,0.25",
            font=("TkDefaultFont", 8),
        ).grid(row=len(fields), column=0, columnspan=2, sticky="w", padx=5, pady=(2, 0))

    
    # In gui_frames/controller_frame.py (add this method)
    def set_params(self, param_dict: dict):
        """Set entries from loaded JSON dict."""
        try:
            # Assuming entries like self.entries['l'] = Entry for l, etc.
            for key, value in param_dict.items():
                if key in self.entries:  # Or specific attr like self.l_entry
                    self.entries[key].delete(0, tk.END)
                    self.entries[key].insert(0, f"{value:.6g}")
        except Exception as e:
            print(f"Warning: Failed to set Controller params: {e}")
    
    def _load_defaults(self):
        # Clear first
        for e in self.entries.values():
            e.delete(0, tk.END)

        # Scalar parameters from DEFAULT_PARAMS dict
        self.entries["lambda_"].insert(0, str(DEFAULT_PARAMS["lambda_"]))
        self.entries["l2"].insert(0, str(DEFAULT_PARAMS["l2"]))
        self.entries["k_I"].insert(0, str(DEFAULT_PARAMS["k_I"]))
        self.entries["phi1"].insert(0, str(DEFAULT_PARAMS["phi1"]))
        self.entries["phi2"].insert(0, str(DEFAULT_PARAMS["phi2"]))
        self.entries["delta"].insert(0, str(DEFAULT_PARAMS["delta"]))
        self.entries["beta_min"].insert(0, str(DEFAULT_PARAMS["beta_min"]))
        self.entries["beta_max"].insert(0, str(DEFAULT_PARAMS["beta_max"]))

        # HFN breakpoints list → comma-separated string
        hfn = DEFAULT_PARAMS.get("hfn_breakpoints", ())
        self.entries["hfn_breakpoints"].insert(
            0, ",".join(str(x) for x in hfn)
        )

    def get_params(self) -> ControllerParams:
        try:
            lambda_ = float(self.entries["lambda_"].get())
            l2 = float(self.entries["l2"].get())
            k_I = float(self.entries["k_I"].get())
            phi1 = float(self.entries["phi1"].get())
            phi2 = float(self.entries["phi2"].get())
            delta = float(self.entries["delta"].get())
            beta_min = float(self.entries["beta_min"].get())
            beta_max = float(self.entries["beta_max"].get())

            hfn_str = self.entries["hfn_breakpoints"].get().strip()
            if not hfn_str:
                hfn_breakpoints = ()
            else:
                hfn_breakpoints = tuple(
                    float(s.strip()) for s in hfn_str.split(",") if s.strip()
                )

        except ValueError as e:
            raise ValueError(f"Invalid controller parameter: {e}")

        return ControllerParams(
            lambda_=lambda_,
            l2=l2,
            k_I=k_I,
            phi1=phi1,
            phi2=phi2,
            delta=delta,
            beta_min=beta_min,
            beta_max=beta_max,
            hfn_breakpoints=hfn_breakpoints,
        )
