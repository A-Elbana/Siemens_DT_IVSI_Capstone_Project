"""
Client 1: Environment/Plant Simulator
---------------------------------------------------
 STEPS
----------------
1. Run vsiBuild -f vsiBuildCommands
2. Copy robot_model.py into line_follower/src/client1/
3. Open the auto-generated file line_follower/src/client1/client1.py
4. Paste each Code Block below into its matching custom code region
"""


# Code Block 1 - Global Variables & Definitions region
from robot_model import DiffDriveRobot, ReferencePath

# ------------------------------------------------------------


# Code Block 2 - Constructor region
self.robot = DiffDriveRobot(x=100.0, y=400.0, yaw=0.0, L=25.0)
self.path = ReferencePath(path_type="curved") # curved or straight
self.V_base = 20.0 # Constant forward speed

# ------------------------------------------------------------

# Code Block 3 - Main simulation loop region


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

if dist_to_target < 10.0:
    print(f"\n[SUCCESS] Target reached! Distance to target: {dist_to_target:.2f}")
    print("Terminating the simulation...")
    
    vsiCommonPythonApi.requestStop()
