"""
Client 3: Plotter/Logger
---------------------------------------------------
 STEPS
----------------
1. Run vsiBuild -f vsiBuildCommands
2. Copy controller.py into line_follower/src/client3/
3. Open the auto-generated file line_follower/src/client3/client3.py
4. Paste each Code Block below into its matching custom code region
"""


# Code Block 1 - Global Variables & Definitions region
import os
current_dir = os.getcwd()
if current_dir not in sys.path:
	sys.path.append(current_dir)
from plotter import RealTimePlotter

# ------------------------------------------------------------


# Code Block 2 - Constructor region
self.plotter = RealTimePlotter(buffer_size=3000, update_frequency=20)

# ------------------------------------------------------------

# Code Block 3 - Main simulation loop region
dt = self.simulationStep / 1_000_000_000.0
if dt > 0:
    lat_err = self.mySignals.cross_track_error
    head_err = self.mySignals.heading_error
    
    cross_track_correction = math.atan2(lat_err, self.V_base)
    
    total_error = head_err - cross_track_correction
    
    steering_correction = self.pid.compute(error=total_error, dt=dt)
    
    self.mySignals.steering_correction = float(steering_correction)


# Adjust this line
# nextExpectedTime += self.simulationStep  ->  nextExpectedTime += self.simulationStep * 2