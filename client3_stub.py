"""
Client 3: Plotter/Logger
---------------------------------------------------
 HOW TO USE
----------------
1. Run vsiBuild -f vsiBuildCommands
2. Use copy_components.bash to copy controller.py into line_follower/src/client3/
3. Open the auto-generated file line_follower/src/client3/client3.py
4. Paste each Code Block below into its matching custom code region
"""


# Code Block 1 - Global Variables & Definitions region
import os
import sys

# Parameters
PLOTTER_BUFFER_SIZE = 3000
PLOTTER_UPDATE_FREQ = 20
KPI_PRINT_INTERVAL = 5

current_dir = os.getcwd()
if current_dir not in sys.path:
	sys.path.append(current_dir)
from plotter import RealTimePlotter

# ------------------------------------------------------------


# Code Block 2 - Constructor region
self.plotter = RealTimePlotter(buffer_size=PLOTTER_BUFFER_SIZE, update_frequency=PLOTTER_UPDATE_FREQ)

# ------------------------------------------------------------

# Code Block 3 - Main simulation loop region after updating internal variables
current_time = vsiCommonPythonApi.getSimulationTimeInNs() / 1_000_000_000.0

self.plotter.update_data(
    current_time, 
    self.mySignals.cross_track_error, 
    self.mySignals.heading_error
)
if int(current_time) % KPI_PRINT_INTERVAL == 0:
    self.plotter.print_final_kpis()


# Adjust this line
# nextExpectedTime += self.simulationStep  ->  nextExpectedTime += self.simulationStep * 2