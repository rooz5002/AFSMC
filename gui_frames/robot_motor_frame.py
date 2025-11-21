# gui_frames/robot_motor_frame.py

import tkinter as tk
from tkinter import ttk
from afsmc_simulation import RobotParams, MotorParams


class RobotMotorFrame(ttk.LabelFrame):
    def __init__(self, parent):
        super().__init__(parent, text="Robot & motor parameters")
        self.robot_vars = {}
        self.motor_vars = {}
        self._build()
        self._load_defaults()

    def set_robot_params(self, data: dict):
        for key, var in self.robot_vars.items():
            if key in data:
                var.set(data[key])

    def set_motor_params(self, data: dict):
        """Fill motor parameter entries from dict."""
        for key, var in self.motor_vars.items():
            if key in data:
                var.set(data[key])
                
                
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

        self.robot_vars = {}
        for i, (key, label) in enumerate(robot_fields):
            ttk.Label(self, text=label + ":").grid(row=i, column=0, sticky="e", padx=5, pady=2)
            self.robot_vars[key] = tk.StringVar()
            e = ttk.Entry(self, textvariable=self.robot_vars[key], width=12)
            e.grid(row=i, column=1, sticky="w", padx=5, pady=2)

        # Motor section
        motor_fields = [
            ("v_max",  "Max no-load speed [m/s]"),
            ("a_max",  "Max acceleration [m/s²]"),
            ("zeta",   "Damping ratio ζ"),
            ("omega_n","Natural freq ωₙ [rad/s]"),
        ]

        self.motor_vars = {}
        for j, (key, label) in enumerate(motor_fields):
            row = j
            ttk.Label(self, text=label + ":").grid(row=row, column=2, sticky="e", padx=5, pady=2)
            self.motor_vars[key] = tk.StringVar()
            e = ttk.Entry(self, textvariable=self.motor_vars[key], width=12)
            e.grid(row=row, column=3, sticky="w", padx=5, pady=2)


    def _load_defaults(self):
        r_def = RobotParams()
        m_def = MotorParams()

        for key in self.robot_vars:
            self.robot_vars[key].set(str(getattr(r_def, key)))
        for key in self.motor_vars:
            self.motor_vars[key].set(str(getattr(m_def, key)))

    def get_params(self):
        try:
            robot = RobotParams(
                mass=float(self.robot_vars["mass"].get()),
                length=float(self.robot_vars["length"].get()),
                width=float(self.robot_vars["width"].get()),
                wheel_spacing=float(self.robot_vars["wheel_spacing"].get()),
                wheel_radius=float(self.robot_vars["wheel_radius"].get()),
                inertia=float(self.robot_vars["inertia"].get()),
            )

            motor = MotorParams(
                v_max=float(self.motor_vars["v_max"].get()),
                a_max=float(self.motor_vars["a_max"].get()),
                zeta=float(self.motor_vars["zeta"].get()),
                omega_n=float(self.motor_vars["omega_n"].get()),
            )

            return robot, motor
        except ValueError as e:
            raise ValueError(f"Invalid robot/motor parameter: {e}")
