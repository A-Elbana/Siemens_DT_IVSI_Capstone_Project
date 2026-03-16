"""
plotter.py

Provides a real-time visualization dashboard using PyQtGraph for 
monitoring robot trajectory tracking errors. Includes functionality to 
compute key performance indicators (KPIs) and export the final data to 
publication-ready Matplotlib plots.
"""

import sys
from collections import deque
import numpy as np
import matplotlib.pyplot as plt

try:
    from PyQt6 import QtWidgets, QtCore
except ModuleNotFoundError:
    from PySide6 import QtWidgets, QtCore

import pyqtgraph as pg

# Configuration Parameters
DEFAULT_BUFFER_SIZE = 2000
DEFAULT_UPDATE_FREQ = 2
WINDOW_WIDTH = 1000
WINDOW_HEIGHT = 600

KPI_LATERAL_TOLERANCE = 2.0
KPI_HEADING_TOLERANCE = 0.05
KPI_TAIL_PERCENTAGE = 0.1

MATPLOTLIB_DPI = 300

pg.setConfigOption('background', 'w')
pg.setConfigOption('foreground', 'k')

class RealTimePlotter:
    """
    Manages the real-time PyQtGraph plotting window and data buffers.
    """
    def __init__(self, buffer_size=DEFAULT_BUFFER_SIZE, update_frequency=DEFAULT_UPDATE_FREQ):
        """
        Initializes the plotting environment and GUI elements.
        
        Args:
            buffer_size (int): The maximum number of historical data points to retain in memory.
            update_frequency (int): The number of simulation steps to wait before redrawing the GUI.
        """
        self._app = QtWidgets.QApplication.instance()
        if self._app is None:
            self._app = QtWidgets.QApplication(sys.argv)

        self.win = pg.GraphicsLayoutWidget(show=True, title="Robot Path Tracking Live")
        self.win.resize(WINDOW_WIDTH, WINDOW_HEIGHT)

        self.plot_lat = self.win.addPlot(title="Lateral Error (units)")
        self.plot_lat.showGrid(x=True, y=True)
        self.plot_lat.setLabel('left', 'Error Distance', units='units')
        self.plot_lat.setLabel('bottom', 'Time', units='s')
        
        self.plot_lat.addLine(y=0, pen=pg.mkPen('k', style=QtCore.Qt.PenStyle.DashLine))
        self.curve_lat = self.plot_lat.plot(pen=pg.mkPen('b', width=2))
        
        self.win.nextRow()

        self.plot_head = self.win.addPlot(title="Heading Error (radians)")
        self.plot_head.showGrid(x=True, y=True)
        self.plot_head.setLabel('left', 'Angle Error', units='rad')
        self.plot_head.setLabel('bottom', 'Time', units='s')
        
        self.plot_head.addLine(y=0, pen=pg.mkPen('k', style=QtCore.Qt.PenStyle.DashLine))
        self.curve_head = self.plot_head.plot(pen=pg.mkPen('r', width=2))

        self.buffer_size = buffer_size
        self.update_frequency = update_frequency
        self._call_count = 0

        self.t_data = deque(maxlen=buffer_size)
        self.lat_data = deque(maxlen=buffer_size)
        self.head_data = deque(maxlen=buffer_size)

    def update_data(self, t, lateral_error, heading_error):
        """
        Appends new error data to the rolling buffers and refreshes the display.
        
        Args:
            t (float): Current simulation time.
            lateral_error (float): Current measured cross-track error.
            heading_error (float): Current measured angular error.
        """
        self.t_data.append(t)
        self.lat_data.append(lateral_error)
        self.head_data.append(heading_error)

        self._call_count += 1
        if self._call_count % self.update_frequency == 0:
            t_arr = np.array(self.t_data)
            self.curve_lat.setData(t_arr, np.array(self.lat_data))
            self.curve_head.setData(t_arr, np.array(self.head_data))
            
            self._app.processEvents()

    def calculate_kpis(self, error_data, tolerance):
        """
        Computes the overshoot, settling time, and steady-state error metrics.
        
        Args:
            error_data (iterable): The historical array of error values to process.
            tolerance (float): The threshold within which the system is considered settled.
            
        Returns:
            tuple: Contains (overshoot, settling_time, steady_state_error).
        """
        if not error_data or len(error_data) < 2:
            return 0.0, 0.0, 0.0

        errors = np.array(error_data)
        times = np.array(self.t_data)
        
        tail_length = max(1, int(len(errors) * KPI_TAIL_PERCENTAGE))
        steady_state_error = float(np.mean(np.abs(errors[-tail_length:])))

        overshoot = 0.0
        initial_sign = np.sign(errors[0]) if errors[0] != 0 else 1
        crossed_zero = False
        
        for e in errors:
            if not crossed_zero and np.sign(e) != initial_sign and e != 0:
                crossed_zero = True
            if crossed_zero and abs(e) > overshoot:
                overshoot = abs(e)

        settling_time = times[-1]
        for i in range(len(errors)-1, -1, -1):
            if abs(errors[i]) > tolerance:
                if i + 1 < len(times):
                    settling_time = times[i + 1]
                else:
                    settling_time = times[-1]
                break
        else:
            settling_time = 0.0 

        return float(overshoot), float(settling_time), steady_state_error

    def print_final_kpis(self):
        """
        Calculates and formats the final Key Performance Indicators to standard output.
        """
        if not self.t_data:
            return

        lat_os, lat_st, lat_sse = self.calculate_kpis(self.lat_data, tolerance=KPI_LATERAL_TOLERANCE)
        head_os, head_st, head_sse = self.calculate_kpis(self.head_data, tolerance=KPI_HEADING_TOLERANCE)

        print(f"\n{'='*40}")
        print("          FINAL RUN KPIs")
        print(f"{'='*40}")
        print("--- Lateral Error ---")
        print(f"Overshoot:          {lat_os:.2f} units")
        print(f"Settling Time:      {lat_st:.2f} s")
        print(f"Steady-State Error: {lat_sse:.2f} units\n")

        print("--- Heading Error ---")
        print(f"Overshoot:          {head_os:.4f} rad")
        print(f"Settling Time:      {head_st:.2f} s")
        print(f"Steady-State Error: {head_sse:.4f} rad")
        print(f"{'='*40}\n")

    def save_matplotlib_plots(self, filename="plot_kpis.png"):
        """Generates and saves a Matplotlib figure of the errors."""
        if not self.t_data:
            return

        # Fetch KPIs to display them directly on the plots
        lat_os, lat_st, lat_sse = self.calculate_kpis(self.lat_data, tolerance=2.0)
        head_os, head_st, head_sse = self.calculate_kpis(self.head_data, tolerance=0.05)

        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
        fig.suptitle("Robot Path Tracking Errors Over Time", fontsize=14, fontweight='bold')

        # 1. Lateral (Cross-Track) Error Subplot
        ax1.plot(self.t_data, self.lat_data, color='blue', label='Lateral Error')
        ax1.axhline(0, color='black', linestyle='--', linewidth=1)
        ax1.set_ylabel("Lateral Error (units)")
        ax1.set_title(f"Cross-Track Error (OS: {lat_os:.1f}, ST: {lat_st:.1f}s, SSE: {lat_sse:.1f})")
        ax1.grid(True, linestyle=':', alpha=0.7)
        ax1.legend(loc="upper right")

        # 2. Heading Error Subplot
        ax2.plot(self.t_data, self.head_data, color='red', label='Heading Error')
        ax2.axhline(0, color='black', linestyle='--', linewidth=1)
        ax2.set_xlabel("Time (s)")
        ax2.set_ylabel("Heading Error (rad)")
        ax2.set_title(f"Heading Error (OS: {head_os:.2f}, ST: {head_st:.2f}s, SSE: {head_sse:.3f})")
        ax2.grid(True, linestyle=':', alpha=0.7)
        ax2.legend(loc="upper right")

        # Adjust layout and save
        plt.tight_layout()
        plt.savefig(filename, dpi=300, bbox_inches='tight')
        plt.close(fig)
        print(f"[INFO] High-resolution plots saved to: {filename}")


    def close(self):
        """
        Executes final operations including KPI calculation, image saving, and GUI teardown.
        """
        self.print_final_kpis()
        self.save_matplotlib_plots()
        self.win.close()
        if self._app is not None:
            self._app.quit()