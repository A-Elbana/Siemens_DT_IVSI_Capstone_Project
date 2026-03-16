"""
robot_model.py

Defines the mathematical kinematics and state representation of a 
Differential Drive Robot. Includes an ideal simulation environment 
equipped with internal process noise (slip) and measurement noise.
Also generates path arrays based on cubic Bezier curves.
"""

from dataclasses import dataclass
import numpy as np

# Configuration Parameters
DEFAULT_MOTOR_TAU = 0.05
MIN_SLIP = 0.25
MAX_SLIP = 0.55
DIV_EPSILON = 1e-6
BEZIER_RESOLUTION = 50
CROSS_TRACK_ERROR_STD = 3.0
HEADING_ERROR_STD = 0.1

@dataclass
class State:
    """
    Represents the pose of the robot in a 2D plane.
    """
    x: float
    y: float
    yaw: float

class DiffDriveRobot:
    """
    Simulates the kinematics of a differential drive mechanism.
    """
    def __init__(self, x, y, yaw, L, motor_tau=DEFAULT_MOTOR_TAU):
        """
        Initializes the robot parameters.
        
        Args:
            x (float): Initial X coordinate.
            y (float): Initial Y coordinate.
            yaw (float): Initial heading in radians.
            L (float): The track width (distance between wheels).
            motor_tau (float): Time constant for the motor dynamics model.
        """
        self.L = L
        self.state: State = State(x, y, yaw)
        
        self.motor_tau = motor_tau 
        self.constant_slip = np.random.uniform(MIN_SLIP, MAX_SLIP)

        self.actual_vr = 0.0
        self.actual_vl = 0.0

    def angular_velocity(self, vr, vl):
        """
        Calculates the instantaneous angular velocity of the chassis.
        
        Args:
            vr (float): Right wheel velocity.
            vl (float): Left wheel velocity.
            
        Returns:
            float: The chassis rotational velocity in rad/s.
        """
        return (vr - vl) / self.L

    def R(self, vr, vl):
        """
        Calculates the signed distance from the Instantaneous Center of Curvature (ICC).
        
        Args:
            vr (float): Right wheel velocity.
            vl (float): Left wheel velocity.
            
        Returns:
            float: The radius of curvature. Inf if movement is purely translational.
        """
        sum_v = vl + vr
        diff_v = vr - vl
        
        if abs(diff_v) < DIV_EPSILON:
            return float('inf') 
        return (self.L / 2) * (sum_v / diff_v)

    def model(self, target_vr, target_vl, dt):
        """
        Updates the pose of the robot based on input velocities and integration timestep.
        
        Args:
            target_vr (float): Desired right wheel velocity.
            target_vl (float): Desired left wheel velocity.
            dt (float): Timestep delta for integration.
        """
        alpha = dt / (self.motor_tau + dt)
        self.actual_vr = self.actual_vr + alpha * (target_vr - self.actual_vr)
        self.actual_vl = self.actual_vl + alpha * (target_vl - self.actual_vl)

        noisy_vr = self.actual_vr * (1.0 + self.constant_slip)
        noisy_vl = self.actual_vl

        W = self.angular_velocity(noisy_vr, noisy_vl)
        
        if abs(W) < DIV_EPSILON:
            self.state.x = self.state.x + (noisy_vl * np.cos(self.state.yaw) * dt)
            self.state.y = self.state.y + (noisy_vl * np.sin(self.state.yaw) * dt)
            return
            
        radius = self.R(noisy_vr, noisy_vl)
        M1 = np.zeros((3,3), float)
        V1 = np.zeros(3, float)
        V2 = np.zeros(3, float)
        
        ICCX = self.state.x - (radius * np.sin(self.state.yaw))
        ICCY = self.state.y + (radius * np.cos(self.state.yaw))

        M1[0][0] = np.cos(W*dt)
        M1[0][1] = -np.sin(W*dt)
        M1[1][0] = np.sin(W*dt)
        M1[1][1] = np.cos(W*dt)
        M1[2][2] = 1

        V1[0] = self.state.x - ICCX
        V1[1] = self.state.y - ICCY
        V1[2] = self.state.yaw

        V2[0] = ICCX
        V2[1] = ICCY
        V2[2] = W * dt

        M = M1.dot(V1) + V2
        self.state.x = M[0]
        self.state.y = M[1]
        self.state.yaw = (M[2] + np.pi) % (2 * np.pi) - np.pi


class ReferencePath:
    """
    Generates and evaluates waypoints defining the trajectory.
    """
    def __init__(self, path_type="straight"):
        """
        Initializes the path points based on the selected configuration.
        
        Args:
            path_type (str): Type of path to instantiate. Default is "straight".
        """
        self.waypoints = []
        if path_type == "straight":
            self.waypoints = np.array([[x, 300.0] for x in range(100, 700, 10)])
        elif path_type == "curved":
            waypoints = []
            p0 = np.array([100.0, 300.0])
            p1 = np.array([300.0, 100.0])
            p2 = np.array([500.0, 500.0])
            p3 = np.array([700.0, 300.0])
            p4 = np.array([900.0, 100.0])
            p5 = np.array([1000.0, 500.0])
            p6 = np.array([1200.0, 300.0])
            
            curve1 = self._cubic_bezier(p0, p1, p2, p3, num_points=BEZIER_RESOLUTION)
            curve2 = self._cubic_bezier(p3, p4, p5, p6, num_points=BEZIER_RESOLUTION)[1:]
            
            waypoints.extend(curve1)
            waypoints.extend(curve2)
        
            self.waypoints = np.array(waypoints)

    def _cubic_bezier(self, p0, p1, p2, p3, num_points=BEZIER_RESOLUTION):
        """
        Computes a cubic Bezier curve.
        
        Args:
            p0-p3 (np.array): Control points.
            num_points (int): The number of output points to generate.
            
        Returns:
            list: List of calculated 2D coordinates.
        """
        points = []
        for t in np.linspace(0, 1, num_points):
            pt = ((1-t)**3)*p0 + 3*((1-t)**2)*t*p1 + 3*(1-t)*(t**2)*p2 + (t**3)*p3
            points.append(pt)
        return points

    def get_errors(self, robot_x, robot_y, robot_yaw):
        """
        Calculates the error metrics of the robot relative to the reference path,
        with optional Gaussian noise simulating sensor inaccuracies.
        
        Args:
            robot_x (float): Current global X coordinate of the robot.
            robot_y (float): Current global Y coordinate of the robot.
            robot_yaw (float): Current heading of the robot.
            noise_std (float): Standard deviation of the Gaussian noise added to errors.
            
        Returns:
            tuple: Contains (cross_track_error, heading_error).
        """
        robot_pos = np.array([robot_x, robot_y])
        
        distances = np.linalg.norm(self.waypoints - robot_pos, axis=1)
        closest_idx = int(np.argmin(distances))
        
        if closest_idx == len(self.waypoints) - 1:
            closest_idx -= 1 

        p1 = self.waypoints[closest_idx]
        p2 = self.waypoints[closest_idx + 1]
        
        path_dx = p2[0] - p1[0]
        path_dy = p2[1] - p1[1]
        target_yaw = np.arctan2(path_dy, path_dx)
        
        heading_error = target_yaw - robot_yaw
        heading_error = (heading_error + np.pi) % (2 * np.pi) - np.pi
        
        path_vec = p2 - p1
        robot_vec = robot_pos - p1
        cross_track_error = np.cross(path_vec, robot_vec) / np.linalg.norm(path_vec)
        
        cross_track_error += np.random.normal(0, CROSS_TRACK_ERROR_STD)
        heading_error += np.random.normal(0, HEADING_ERROR_STD)
            
        return float(cross_track_error), float(heading_error)