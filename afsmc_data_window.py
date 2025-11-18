'''
afsmc_data_window.py
Note: this simulation module is part of the AFSMC / HFN-AFSMC research package and is intended
for academic and educational use only. Please cite the corresponding paper when you use or
reproduce these results.
© [2024-2025] Robotics & AI Laboratory — Huaiyin Institute of Technology.
Developed under the supervision of Dr. Amir Ali Mokhtarzadeh for research on 
Adaptive Fuzzy Sliding Mode Control (AFSMC) and trajectory tracking simulation.
 '''

import tkinter as tk
from tkinter import ttk, filedialog, messagebox, scrolledtext
import csv


class DataWindow(tk.Toplevel):
    def __init__(self, parent, datasets: dict):
        super().__init__(parent)
        self.title("Numeric data (rows)")
        self.datasets = datasets

        ttk.Label(self, text="Dataset:").grid(row=0, column=0, padx=5, pady=5, sticky="e")

        self.dataset_names = list(self.datasets.keys())
        self.dataset_var = tk.StringVar(value=self.dataset_names[0])

        self.combo = ttk.Combobox(
            self,
            textvariable=self.dataset_var,
            values=self.dataset_names,
            state="readonly",
        )
        self.combo.grid(row=0, column=1, padx=5, pady=5, sticky="w")
        self.combo.bind("<<ComboboxSelected>>", self._update_text)

        ttk.Button(self, text="Save selected as CSV", command=self._save_csv).grid(
            row=0, column=2, padx=5, pady=5
        )

        self.text = scrolledtext.ScrolledText(self, width=100, height=25)
        self.text.grid(row=1, column=0, columnspan=3, padx=5, pady=5, sticky="nsew")

        self.rowconfigure(1, weight=1)
        self.columnconfigure(1, weight=1)

        self._update_text()

    def _update_text(self, event=None):
        name = self.dataset_var.get()
        cols, data = self.datasets[name]
        self.text.delete("1.0", tk.END)
        header_line = ",".join(cols)
        self.text.insert(tk.END, header_line + "\n")
        for row in data:
            line = ",".join(f"{val:.8f}" for val in row)
            self.text.insert(tk.END, line + "\n")

    def _save_csv(self):
        name = self.dataset_var.get()
        cols, data = self.datasets[name]

        filename = filedialog.asksaveasfilename(
            defaultextension=".csv",
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
        )
        if not filename:
            return

        try:
            with open(filename, "w", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)
                writer.writerow(cols)
                for row in data:
                    writer.writerow(row.tolist())
            messagebox.showinfo("Saved", f"Dataset '{name}' saved to {filename}")
        except Exception as e:
            messagebox.showerror("Error", f"Could not save CSV: {e}")
