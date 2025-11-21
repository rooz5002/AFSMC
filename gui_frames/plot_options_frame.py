'''
gui_frames/plot_options_frame.py

Note: this simulation module is part of the AFSMC / HFN-AFSMC research package and is intended
for academic and educational use only. Please cite the corresponding paper when you use or
reproduce these results.
© [2024-2025] Robotics & AI Laboratory — Huaiyin Institute of Technology.
Developed under the supervision of Dr. Amir Ali Mokhtarzadeh for research on 
Adaptive Fuzzy Sliding Mode Control (AFSMC) and trajectory tracking simulation.
 '''
 
 
 
 
 
 
import tkinter as tk
from tkinter import ttk


class PlotOptionsFrame(ttk.LabelFrame):
    def __init__(self, parent):
        super().__init__(parent, text="Plot options (axes & colours)")
        self.entries = {}
        self.color_entries = {}
        self.show_data_var = tk.BooleanVar(value=True)
        self._build()
        self._load_default_colors()

    def _build(self):
        # Axis limits
        fields = [
            ("x_min", "x_min"),
            ("x_max", "x_max"),
            ("y_min", "y_min"),
            ("y_max", "y_max"),
        ]

        for i, (key, label) in enumerate(fields):
            ttk.Label(self, text=label + ":").grid(row=i // 2, column=(i % 2) * 2, sticky="e", padx=5, pady=2)
            e = ttk.Entry(self, width=7)
            e.grid(row=i // 2, column=(i % 2) * 2 + 1, sticky="w", padx=5, pady=2)
            self.entries[key] = e

        ttk.Checkbutton(
            self,
            text="Open raw data window after plotting",
            variable=self.show_data_var
        ).grid(row=0, column=4, rowspan=2, sticky="w", padx=10, pady=2)

        # Colours
        row = 2
        ttk.Label(self, text="Colours:").grid(row=row, column=0, columnspan=5, sticky="w", padx=5, pady=(4, 2))

        color_fields = [
            ("traj_ref",   "traj_ref"),
            ("traj_robot", "traj_robot"),
            ("v",          "v(t)"),
            ("omega",      "ω(t)"),
            ("e_y",        "e_y(t)"),
            ("e_theta",    "e_θ(t)"),
            ("w_left",     "w_left"),
            ("w_right",    "w_right"),
        ]

        row += 1
        col = 0
        for key, label in color_fields:
            ttk.Label(self, text=label + ":").grid(row=row, column=col, sticky="e", padx=5, pady=2)
            e = ttk.Entry(self, width=7)
            e.grid(row=row, column=col + 1, sticky="w", padx=5, pady=2)
            self.color_entries[key] = e

            col += 2
            if col > 4:
                col = 0
                row += 1

    def _load_default_colors(self):
        defaults = {
            "traj_ref": "k",
            "traj_robot": "b",
            "v": "r",
            "omega": "b",
            "e_y": "g",
            "e_theta": "m",
            "w_left": "c",
            "w_right": "y",
        }
        for key, val in defaults.items():
            self.color_entries[key].insert(0, val)

    def get_plot_config(self):
        # Axis limits
        def parse(val):
            s = self.entries[val].get().strip()
            return float(s) if s else None

        try:
            x_min = parse("x_min")
            x_max = parse("x_max")
            y_min = parse("y_min")
            y_max = parse("y_max")
        except ValueError as e:
            raise ValueError(f"Invalid axis limit: {e}")

        xlim = (x_min, x_max) if (x_min is not None or x_max is not None) else None
        ylim = (y_min, y_max) if (y_min is not None or y_max is not None) else None

        # Colours
        colors = {k: e.get().strip() for k, e in self.color_entries.items()}
        return xlim, ylim, colors, self.show_data_var.get()
