'''
gui_frames/robot_motor_frame.py

Note: this simulation module is part of the AFSMC / HFN-AFSMC research package and is intended
for academic and educational use only. Please cite the corresponding paper when you use or
reproduce these results.
© [2024-2025] Robotics & AI Laboratory — Huaiyin Institute of Technology.
Developed under the supervision of Dr. Amir Ali Mokhtarzadeh for research on 
Adaptive Fuzzy Sliding Mode Control (AFSMC) and trajectory tracking simulation.
 '''
 
 
 
 
 
 
import tkinter as tk
from tkinter import ttk
from afsmc_simulation import RobotParams, MotorParams


class RobotMotorFrame(ttk.LabelFrame):
    def __init__(self, parent):
        super().__init__(parent, text="Robot & motor parameters")
        self.robot_entries = {}
        self.motor_entries = {}
        self._build()
        self._load_defaults()

    def _build(self):
        # Robot section
        robot_fields = [
            ("mass",          "Total weight [kg]"),
            ("length",        "Length [m]"),
            ("width",         "Width [m]"),
            ("wheel_spacing", "Wheel spacing [m]"),
            ("wheel_radius",  "Wheel radius [m]"),
            ("inertia",       "Inertia J [kg·m²]"),
        ]

        for i, (key, label) in enumerate(robot_fields):
            ttk.Label(self, text=label + ":").grid(row=i, column=0, sticky="e", padx=5, pady=2)
            e = ttk.Entry(self, width=12)
            e.grid(row=i, column=1, sticky="w", padx=5, pady=2)
            self.robot_entries[key] = e

        # Motor section
        motor_fields = [
            ("v_max",  "Max no-load speed [m/s]"),
            ("a_max",  "Max acceleration [m/s²]"),
            ("zeta",   "Damping ratio ζ"),
            ("omega_n","Natural freq ωₙ [rad/s]"),
        ]

        for j, (key, label) in enumerate(motor_fields):
            row = j
            ttk.Label(self, text=label + ":").grid(row=row, column=2, sticky="e", padx=5, pady=2)
            e = ttk.Entry(self, width=12)
            e.grid(row=row, column=3, sticky="w", padx=5, pady=2)
            self.motor_entries[key] = e

    def _load_defaults(self):
        r_def = RobotParams()
        m_def = MotorParams()

        for key, entry in self.robot_entries.items():
            entry.insert(0, str(getattr(r_def, key)))

        for key, entry in self.motor_entries.items():
            entry.insert(0, str(getattr(m_def, key)))

    def get_params(self):
        try:
            robot = RobotParams(
                mass=float(self.robot_entries["mass"].get()),
                length=float(self.robot_entries["length"].get()),
                width=float(self.robot_entries["width"].get()),
                wheel_spacing=float(self.robot_entries["wheel_spacing"].get()),
                wheel_radius=float(self.robot_entries["wheel_radius"].get()),
                inertia=float(self.robot_entries["inertia"].get()),
            )

            motor = MotorParams(
                v_max=float(self.motor_entries["v_max"].get()),
                a_max=float(self.motor_entries["a_max"].get()),
                zeta=float(self.motor_entries["zeta"].get()),
                omega_n=float(self.motor_entries["omega_n"].get()),
            )

            return robot, motor

        except ValueError as e:
            raise ValueError(f"Invalid robot/motor parameter: {e}")
