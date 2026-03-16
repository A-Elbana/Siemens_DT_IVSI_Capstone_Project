"""
Client 1: Environment/Plant Simulator
---------------------------------------------------
 HOW TO USE
----------------
1. Run vsiBuild -f vsiBuildCommands
2. Use copy_components.bash to copy robot_model.py into line_follower/src/client1/
3. Open the auto-generated file line_follower/src/client1/client1.py
4. Paste each Code Block below into its matching custom code region
"""


# Code Block 1 - Global Variables & Definitions region
from robot_model import DiffDriveRobot, ReferencePath

# Parameters
START_X = 100.0
START_Y = 400.0
START_YAW = 0.0
ROBOT_L = 25.0
PATH_TYPE = "straight" # curved or straight
V_BASE = 20.0 # Constant forward speed

# ------------------------------------------------------------


# Code Block 2 - Constructor region
self.robot = DiffDriveRobot(x=START_X, y=START_Y, yaw=START_YAW, L=ROBOT_L)
self.path = ReferencePath(path_type=PATH_TYPE) 
self.V_base = V_BASE 

# ------------------------------------------------------------

# Code Block 3 - Main simulation loop region

if self.mySignals.target_reached == True:
    vsiCommonPythonApi.terminate()

dt = self.simulationStep / 1_000_000_000.0

if dt > 0:
    steering_correction = self.mySignals.steering_correction

    vr = self.V_base + steering_correction
    vl = self.V_base - steering_correction

    self.robot.model(vr, vl, dt)

    lat_err, head_err = self.path.get_errors(self.robot.state.x, self.robot.state.y, self.robot.state.yaw)

    self.mySignals.cross_track_error = float(lat_err)
    self.mySignals.heading_error = float(head_err)

final_waypoint = self.path.waypoints[-1]

dist_to_target = math.hypot(self.robot.state.x - final_waypoint[0], 
                            self.robot.state.y - final_waypoint[1])

if dist_to_target < 25.0:
    print(f"\n[SUCCESS] Target reached! Distance to target: {dist_to_target:.2f}")
    print("Terminating the simulation...")
    self.mySignals.target_reached = True
