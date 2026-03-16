"""
controller.py

Implements a Proportional-Integral-Derivative (PID) controller.
Calculates control efforts based on cumulative and temporal error margins.
"""

import numpy as np

class PIDController:
    """
    A standard PID controller for closed-loop error correction.
    """
    def __init__(self, kp, ki, kd) -> None:
        """
        Initializes the PID controller with given gains.
        
        Args:
            kp (float): Proportional gain.
            ki (float): Integral gain.
            kd (float): Derivative gain.
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0.0
        self.cumulative_error = 0.0
        
    def compute(self, error, dt):
        """
        Computes the PID control output based on the current error.
        
        Args:
            error (float): The current calculated error from the setpoint.
            dt (float): The time delta since the last computation step.
            
        Returns:
            float: The calculated control effort.
        """
        self.cumulative_error += error * dt
        p = self.kp * error
        i = self.ki * self.cumulative_error
        d = self.kd * ((error - self.prev_error) / dt)
        output = p + i + d

        self.prev_error = error
        return output