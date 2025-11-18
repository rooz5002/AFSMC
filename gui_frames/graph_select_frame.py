'''
gui_frames/graph_select_frame.py

Note: this simulation module is part of the AFSMC / HFN-AFSMC research package and is intended
for academic and educational use only. Please cite the corresponding paper when you use or
reproduce these results.
© [2024-2025] Robotics & AI Laboratory — Huaiyin Institute of Technology.
Developed under the supervision of Dr. Amir Ali Mokhtarzadeh for research on 
Adaptive Fuzzy Sliding Mode Control (AFSMC) and trajectory tracking simulation.
 '''
 
 
 
import tkinter as tk
from tkinter import ttk


class GraphSelectFrame(ttk.LabelFrame):
    def __init__(self, parent):
        super().__init__(parent, text="Graphs to generate")
        self.vars = {}
        self._build()

    def _build(self):
        self.vars["traj"]   = tk.BooleanVar(value=True)
        self.vars["vel"]    = tk.BooleanVar(value=True)
        self.vars["err"]    = tk.BooleanVar(value=True)
        self.vars["wheels"] = tk.BooleanVar(value=True)

        ttk.Checkbutton(self, text="Trajectory (x–y)", variable=self.vars["traj"]) \
            .grid(row=0, column=0, sticky="w", padx=5, pady=3)

        ttk.Checkbutton(self, text="Linear / angular velocities", variable=self.vars["vel"]) \
            .grid(row=0, column=1, sticky="w", padx=5, pady=3)

        ttk.Checkbutton(self, text="Tracking errors", variable=self.vars["err"]) \
            .grid(row=1, column=0, sticky="w", padx=5, pady=3)

        ttk.Checkbutton(self, text="Wheel velocities", variable=self.vars["wheels"]) \
            .grid(row=1, column=1, sticky="w", padx=5, pady=3)

    def get_selected_graphs(self):
        return {k: v.get() for k, v in self.vars.items()}
