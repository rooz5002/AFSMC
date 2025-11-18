
'''
afsmc_app.py version: 1.4

Main application shell: wires frames + simulation + plotting together.
Note: this simulation module is part of the AFSMC / HFN-AFSMC research package and is intended
for academic and educational use only. Please cite the corresponding paper when you use or
reproduce these results.
© [2024-2025] Robotics & AI Laboratory — Huaiyin Institute of Technology.
Developed under the supervision of Dr. Amir Ali Mokhtarzadeh for research on 
Adaptive Fuzzy Sliding Mode Control (AFSMC) and trajectory tracking simulation.
 '''


import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import json
import numpy as np

# NEW: Set matplotlib backend for Tkinter integration BEFORE importing pyplot
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt  # Now safe to import

from afsmc_simulation import simulate_case1, simulate_case2, CaseStudyParams
from gui_frames.controller_frame import ControllerFrame
from gui_frames.robot_motor_frame import RobotMotorFrame
from gui_frames.case_frame import CaseFrame
from gui_frames.graph_select_frame import GraphSelectFrame
from gui_frames.plot_options_frame import PlotOptionsFrame

from afsmc_plots import (
    PlotConfig,
    plot_single_mode,
    plot_comparison,
    summary_single,
    summary_comparison,
)

from afsmc_data_window import DataWindow


class AFSMCApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("AFSMC / SMC Case Study Panel")

        # Styles for light backgrounds and components (ttk requires styles)
        style = ttk.Style()
        style.theme_use('clam')  # Theme that supports bg styling

        # Frame styles (applied via configure on frames)
        style.configure("Ctrl.TFrame", background="#E6F3FF")   # Light blue
        style.configure("Robot.TFrame", background="#E6FFE6")  # Light green
        style.configure("Case.TFrame", background="#FFFFE6")   # Light yellow
        style.configure("Graph.TFrame", background="#FFE6E6")  # Light pink
        style.configure("Plot.TFrame", background="#F0F0F0")   # Light gray

        # Label styles to match (use in frames: ttk.Label(style="Ctrl.Label"))
        style.configure("Ctrl.Label", background="#E6F3FF", foreground="black")
        style.configure("Robot.Label", background="#E6FFE6", foreground="black")
        style.configure("Case.Label", background="#FFFFE6", foreground="black")
        style.configure("Graph.Label", background="#FFE6E6", foreground="black")
        style.configure("Plot.Label", background="#F0F0F0", foreground="black")

        # Tab styles (uniform for notebook; per-tab hard in ttk, so base + map for states)
        style.configure("Custom.TNotebook.Tab", background="#E6F3FF", foreground="black")
        style.map("Custom.TNotebook.Tab",
                  background=[('selected', '#B3D9FF'), ('active', '#CCE7FF'),  # Blue theme as base
                              ('!selected', '#F0F0F0'), ('!active', '#E0E0E0')])

        # Button styles with different colors
        style.configure("Generate.TButton", background="#4CAF50", foreground="black")  # Green
        style.configure("Save.TButton", background="#2196F3", foreground="black")      # Blue
        style.configure("Load.TButton", background="#FF9800", foreground="black")      # Orange
        style.configure("Exit.TButton", background="#F44336", foreground="black")      # Red
        

        # Create notebook for tabs
        self.notebook = ttk.Notebook(self, style="Custom.TNotebook")
        self.notebook.grid(row=0, column=0, sticky="nsew", padx=10, pady=5)

        # Create frames and add as tabs with styles (no extra arg; apply style here)
        self.ctrl_frame = ControllerFrame(self.notebook)
        self.ctrl_frame.configure(style="Ctrl.TFrame")
        self.notebook.add(self.ctrl_frame, text="Controller")

        self.robot_frame = RobotMotorFrame(self.notebook)
        self.robot_frame.configure(style="Robot.TFrame")
        self.notebook.add(self.robot_frame, text="Robot & Motor")

        self.case_frame = CaseFrame(self.notebook)
        self.case_frame.configure(style="Case.TFrame")
        self.notebook.add(self.case_frame, text="Case")

        self.graph_frame = GraphSelectFrame(self.notebook)
        self.graph_frame.configure(style="Graph.TFrame")
        self.notebook.add(self.graph_frame, text="Graphs")

        self.plot_frame = PlotOptionsFrame(self.notebook)
        self.plot_frame.configure(style="Plot.TFrame")
        self.notebook.add(self.plot_frame, text="Plot Options")

        # Mode + buttons (below notebook)
        self._build_mode_and_buttons()

        # to hold last datasets
        self.datasets = {}

        # Layout: notebook expands, buttons fixed
        self.rowconfigure(0, weight=1)
        self.columnconfigure(0, weight=1)
        self.rowconfigure(1, weight=0)

    # disclaimer
    def show_disclaimer(self):
        text = (
            "AFSMC / HFN-AFSMC Simulation Package\n\n"
            "This software package and its accompanying modules are provided for academic research "
            "and educational purposes only. All algorithms, models, and visualizations are implemented "
            "to demonstrate control strategies described in the associated scientific work, and are not "
            "intended for deployment on real robotic systems without additional validation, safety testing, "
            "and certification.\n\n"
            "No warranties or representations are made regarding the correctness, completeness, or fitness "
            "of this software for any particular purpose. Use is entirely at your own risk. The authors and "
            "contributors shall not be liable for any direct or indirect damages arising from its use.\n\n"
            "Users are encouraged to cite the original paper(s) associated with the Adaptive Fuzzy Sliding "
            "Mode Control (AFSMC) and the HFN-AFSMC control methodologies when employing or referencing this "
            "work in academic publications.\n\n"
            "© Huaiyin Institute of Technology — Robotics & AI research use."
        )
        messagebox.showinfo("Disclaimer", text)
    
    def _build_mode_and_buttons(self):
        frame = ttk.Frame(self)
        frame.grid(row=1, column=0, padx=10, pady=10, sticky="ew")

        ttk.Label(frame, text="Mode:").grid(row=0, column=0, padx=5, pady=2, sticky="e")
        self.mode_var = tk.StringVar()
        self.mode_combo = ttk.Combobox(
            frame,
            textvariable=self.mode_var,
            state="readonly",
            values=[
                "AFSMC (switching φ)",
                "SMC",
                "Comparison (Case 1)",
            ],
            width=25,
        )
        self.mode_combo.grid(row=0, column=1, padx=5, pady=2, sticky="w")
        self.mode_combo.current(0)
        
        # Case selector
        ttk.Label(frame, text="Case:").grid(row=0, column=2, padx=5, pady=2, sticky="e")
        self.case_var = tk.StringVar()
        self.case_combo = ttk.Combobox(
            frame,
            textvariable=self.case_var,
            state="readonly",
            values=["Case 1 (line)", "Case 2 (circle)"],
            width=18,
        )
        self.case_combo.grid(row=0, column=3, padx=5, pady=2, sticky="w")
        self.case_combo.current(0)

        # Bind case change to pre-fill params (assumes CaseFrame has set_initials)
        def on_case_change(*args):
            case_label = self.case_var.get()
            if "Case 2" in case_label:
                # Pre-fill Case 2 params from paper
                self.case_frame.set_initials(
                    x_ref0=4.0, y_ref0=0.0, theta_ref0=np.pi/2,
                    x0=4.0, y0=2.0, theta0=5*np.pi/6,
                    v_cmd=2.5, t_end=20.0
                )
            else:
                # Reset to Case 1 defaults if needed
                self.case_frame.set_initials(
                    x_ref0=1.0, y_ref0=0.0, theta_ref0=np.pi/3,
                    x0=0.5, y0=0.0, theta0=np.pi/3,
                    v_cmd=2.5, t_end=20.0
                )
        self.case_combo.bind("<<ComboboxSelected>>", on_case_change)

        # Bind mode change to reset case to 1 for comparisons
        def on_mode_change(*args):
            if "Comparison" in self.mode_var.get():
                self.case_var.set("Case 1 (line)")
                on_case_change()  # Trigger Case 1 defaults
        self.mode_combo.bind("<<ComboboxSelected>>", on_mode_change)

        btn_frame = ttk.Frame(frame)
        btn_frame.grid(row=1, column=0, columnspan=4, pady=10)

        ttk.Button(btn_frame, text="Generate graphs", style="Generate.TButton", command=self.generate_graphs).grid(
            row=0, column=0, padx=5, pady=2
        )
        ttk.Button(btn_frame, text="Save Params", style="Save.TButton", command=self._save_params).grid(
            row=0, column=1, padx=5, pady=2
        )
        ttk.Button(btn_frame, text="Load Params", style="Load.TButton", command=self._load_params).grid(
            row=0, column=2, padx=5, pady=2
        )
        ttk.Button(btn_frame, text="Exit", style="Exit.TButton", command=self.destroy).grid(
            row=0, column=3, padx=5, pady=2
        )
        ttk.Button(btn_frame, text="Disclaimer", style="Exit.TButton", command=self.show_disclaimer).grid(row=0, column=4, padx=5, pady=2)
      
       

    def _save_params(self):
        try:
            # Collect params
            params = {
                "controller": self.ctrl_frame.get_params().__dict__,  # Assumes dataclass
                "robot": self.robot_frame.get_params()[0].__dict__,  # robot part
                "motor": self.robot_frame.get_params()[1].__dict__,  # motor part
                "case": self.case_frame.get_params().__dict__,
                # Skip graph/plot frames as they are selections/configs
            }
            filename = filedialog.asksaveasfilename(
                defaultextension=".json",
                filetypes=[("JSON files", "*.json")],
                initialname="afsmc_params.json"
            )
            if filename:
                with open(filename, "w") as f:
                    json.dump(params, f, indent=4)
                messagebox.showinfo("Saved", f"Parameters saved to {filename}")
        except Exception as e:
            messagebox.showerror("Save Error", str(e))

    def _load_params(self):
        filename = filedialog.askopenfilename(
            filetypes=[("JSON files", "*.json")]
        )
        if not filename:
            return
        try:
            with open(filename, "r") as f:
                params = json.load(f)
            # Update frames (requires set_params in each)
            self.ctrl_frame.set_params(params.get("controller", {}))
            self.robot_frame.set_params(
                params.get("robot", {}),
                params.get("motor", {})
            )
            self.case_frame.set_params(params.get("case", {}))
            messagebox.showinfo("Loaded", f"Parameters loaded from {filename}")
        except (json.JSONDecodeError, KeyError) as e:
            messagebox.showerror("Load Error", f"Invalid file: {str(e)}")

    def _read_all_params(self):
        # Centralised error handling for all frames
        try:
            ctrl = self.ctrl_frame.get_params()
            robot, motor = self.robot_frame.get_params()
            case = self.case_frame.get_params()
            selected = self.graph_frame.get_selected_graphs()
            xlim, ylim, colors, show_data = self.plot_frame.get_plot_config()
        except ValueError as e:
            messagebox.showerror("Error", str(e))
            return None

        if not any(selected.values()):
            messagebox.showwarning("No graphs selected", "Please select at least one graph.")
            return None

        return ctrl, robot, motor, case, selected, xlim, ylim, colors, show_data

    def generate_graphs(self):
        params = self._read_all_params()
        if params is None:
            return

        (
            ctrl,
            robot,
            motor,
            case,
            selected,
            xlim,
            ylim,
            colors,
            show_data,
        ) = params

        cfg = PlotConfig(xlim=xlim, ylim=ylim, colors=colors)
        mode_label = self.mode_var.get()
        case_label = self.case_var.get()

        if mode_label == "Comparison (Case 1)":
            # Force Case 1 params for comparisons (manuscript consistency)
            case1_params = CaseStudyParams(
                x_ref0=1.0, y_ref0=0.0, theta_ref0=np.pi/3,
                x0=0.5, y0=0.0, theta0=np.pi/3,
                v_cmd=2.5, t_end=20.0, dt=0.01
            )
            res_smc = simulate_case1("SMC", ctrl, case1_params, robot, motor)
            res_af  = simulate_case1("AFSMC", ctrl, case1_params, robot, motor)

            # Pass case_label and generate_extras for images 14-17
            datasets = plot_comparison(res_smc, res_af, selected, cfg, case_label="Case 1", generate_extras=True)
            msg = summary_comparison(res_smc, res_af)

        else:
            mode = "AFSMC" if mode_label.startswith("AFSMC") else "SMC"

            if case_label.startswith("Case 1"):
                res = simulate_case1(mode, ctrl, case, robot, motor)
            else:  # Case 2 (circle)
                res = simulate_case2(mode, ctrl, case, robot, motor)

            datasets = plot_single_mode(res, mode, selected, cfg, case_label=case_label)
            msg = summary_single(res, mode)

        self.datasets = datasets

        if msg:
            messagebox.showinfo("Results summary", msg)

        if show_data and self.datasets:
            DataWindow(self, self.datasets)

        # UPDATED: Force interactive and block for display
        plt.ion()  # Enable interactive mode
        plt.show(block=True)  # Block until figures closed


if __name__ == "__main__":
    app = AFSMCApp()
    app.mainloop()
