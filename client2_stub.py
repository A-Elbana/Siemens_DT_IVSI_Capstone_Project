"""
Client 2: PID Controller
---------------------------------------------------
 HOW TO USE
----------------
1. Run vsiBuild -f vsiBuildCommands
2. Use copy_components.bash to copy controller.py into line_follower/src/client2/
3. Open the auto-generated file line_follower/src/client2/client2.py
4. Paste each Code Block below into its matching custom code region
"""


# Code Block 1 - Global Variables & Definitions region
from controller import PIDController
import numpy as np

# Parameters
PID_KP = 4.6
PID_KI = 1.47
PID_KD = 9.8
V_BASE = 20.0

# ------------------------------------------------------------


# Code Block 2 - Constructor region
self.pid = PIDController(kp=PID_KP, ki=PID_KI, kd=PID_KD)
self.V_base = V_BASE

# ------------------------------------------------------------

# Code Block 3 - Main simulation loop region
dt = self.simulationStep / 1_000_000_000.0
if dt > 0:
    lat_err = self.mySignals.cross_track_error
    head_err = self.mySignals.heading_error
    
    cross_track_correction = np.arctan2(lat_err, self.V_base)
    
    total_error = head_err - cross_track_correction
    
    steering_correction = self.pid.compute(error=total_error, dt=dt)
    
    self.mySignals.steering_correction = float(steering_correction)

if self.mySignals.target_reached == True:
    vsiCommonPythonApi.terminate()