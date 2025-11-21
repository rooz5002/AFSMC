# gui_frames/controller_frame.py

import tkinter as tk
from tkinter import ttk
from unified_controller import ControllerParams, DEFAULT_PARAMS


class ControllerFrame(ttk.LabelFrame):
    def __init__(self, parent):
        super().__init__(parent, text="Controller parameters")
        self.vars = {}           # Will hold StringVar for all fields
        self.hfn_var = None      # Special StringVar for HFN breakpoints
        self._build()
        self._load_defaults()

    def _build(self):
        fields = [
            ("lambda_",     "λ"),
            ("l2",          "l2"),
            ("k_I",         "k_I"),
            ("phi1",        "φ1"),
            ("phi2",        "φ2"),
            ("delta",       "Δ"),
            ("beta_min",    "β_min"),
            ("beta_max",    "β_max"),
        ]

        # Create StringVar + Entry for scalar parameters
        for i, (key, label) in enumerate(fields):
            ttk.Label(self, text=label + ":").grid(row=i, column=0, sticky="e", padx=5, pady=2)
            self.vars[key] = tk.StringVar()
            e = ttk.Entry(self, textvariable=self.vars[key], width=28)
            e.grid(row=i, column=1, sticky="w", padx=5, pady=2)

        # HFN breakpoints — special handling with dedicated StringVar
        row = len(fields)
        ttk.Label(self, text="HFN breakpoints (comma-separated):").grid(
            row=row, column=0, columnspan=2, sticky="w", padx=5, pady=(10, 2)
        )
        self.hfn_var = tk.StringVar(value="0.0,0.05,0.1,0.15,0.2,0.25")
        ttk.Entry(self, textvariable=self.hfn_var, width=50).grid(
            row=row+1, column=0, columnspan=2, sticky="we", padx=5, pady=2
        )

        # Optional hint
        ttk.Label(
            self,
            text="e.g. 0.0,0.05,0.1,0.15,0.2,0.25",
            font=("TkDefaultFont", 8),
            foreground="gray"
        ).grid(row=row+2, column=0, columnspan=2, sticky="w", padx=5)

    def _load_defaults(self):
        # Load scalar defaults
        defaults = {
            "lambda_": DEFAULT_PARAMS["lambda_"],
            "l2": DEFAULT_PARAMS["l2"],
            "k_I": DEFAULT_PARAMS["k_I"],
            "phi1": DEFAULT_PARAMS["phi1"],
            "phi2": DEFAULT_PARAMS["phi2"],
            "delta": DEFAULT_PARAMS["delta"],
            "beta_min": DEFAULT_PARAMS["beta_min"],
            "beta_max": DEFAULT_PARAMS["beta_max"],
        }
        for key, val in defaults.items():
            self.vars[key].set(str(val))

        # Load HFN breakpoints
        bp = DEFAULT_PARAMS.get("hfn_breakpoints", [0.0, 0.05, 0.1, 0.15, 0.2, 0.25])
        self.hfn_var.set(",".join(map(str, bp)))

    def set_params(self, data: dict):
        """Called when loading a JSON file."""
        try:
            # Scalar parameters
            for key in ["lambda_", "l2", "k_I", "phi1", "phi2", "delta", "beta_min", "beta_max"]:
                if key in data and key in self.vars:
                    self.vars[key].set(str(data[key]))

            # HFN breakpoints — convert list → comma-separated string
            if "hfn_breakpoints" in data:
                bp = data["hfn_breakpoints"]
                if isinstance(bp, (list, tuple)):
                    text = ",".join(map(str, bp))
                else:
                    text = str(bp)
                self.hfn_var.set(text)

        except Exception as e:
            print(f"Warning: Failed to set Controller params: {e}")

    def get_params(self) -> ControllerParams:
        """Return a proper ControllerParams instance."""
        try:
            # Parse scalar values
            params = {
                "lambda_": float(self.vars["lambda_"].get()),
                "l2": float(self.vars["l2"].get()),
                "k_I": float(self.vars["k_I"].get()),
                "phi1": float(self.vars["phi1"].get()),
                "phi2": float(self.vars["phi2"].get()),
                "delta": float(self.vars["delta"].get()),
                "beta_min": float(self.vars["beta_min"].get()),
                "beta_max": float(self.vars["beta_max"].get()),
            }

            # Parse HFN breakpoints string → real list of floats
            bp_text = self.hfn_var.get().strip()
            if bp_text:
                hfn_breakpoints = [float(x) for x in bp_text.replace(" ", "").split(",") if x]
            else:
                hfn_breakpoints = [0.0, 0.05, 0.1, 0.15, 0.2, 0.25]

            return ControllerParams(
                **params,
                hfn_breakpoints=hfn_breakpoints
            )

        except ValueError as e:
            raise ValueError(f"Invalid controller parameter: {e}")
