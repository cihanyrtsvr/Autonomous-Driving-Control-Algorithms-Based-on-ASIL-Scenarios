"""Utility document for the model that has been used for future prediction in MPC_Control.py"""
import numpy as np

# from scipy.optimize import minimize, differential_evolution


class Inputs:
    """Inputs for the model function"""

    def __init__(self, steer_angle=0, acceleration=0):
        self.steer_angle = steer_angle
        self.acceleration = acceleration

    def set_steer_angle(self, steer_angle):
        """Set the steering angle"""
        self.steer_angle = steer_angle

    def set_acceleration(self, acceleration):
        """Set the acceleration"""
        self.acceleration = acceleration


class State:
    """State representation for model"""

    def __init__(self):
        self.pose_x = 0
        self.pose_y = 0
        self.theta_angle = 0
        self.velocity = 0
        self.cross_track_error = 0
        self.error_theta = 0  # heading error

    def set_pose(self, pose_x, pose_y):
        """Set the pose"""
        self.pose_x = pose_x
        self.pose_y = pose_y

    def set_theta_angle(self, theta_angle):
        """Set the theta angle"""
        self.theta_angle = theta_angle

    def set_velocity(self, velocity):
        """Set the velocity"""
        self.velocity = velocity

    def set_cross_track_error(self, cross_track_error):
        """Set the cross track error"""
        self.cross_track_error = cross_track_error

    def set_error_theta(self, error_theta):
        """Set the error theta"""
        self.error_theta = error_theta


def model(inputs, init_state, coff, dtime, vehicle_long_diameter):
    """Model function that used to predict the future condition"""
    final_state = State()

    ## find the final satte after dt of time ###
    final_state.pose_x = (
        init_state.pose_x + init_state.velocity * np.cos(init_state.theta_angle) * dtime
    )
    final_state.pose_y = (
        init_state.pose_y + init_state.velocity * np.sin(init_state.theta_angle) * dtime
    )
    final_state.theta_angle = (
        init_state.theta_angle
        + (init_state.velocity / vehicle_long_diameter) * inputs.steer_angle * dtime
    )
    final_state.velocity = init_state.velocity + inputs.acceleration * dtime

    theta_destination = np.arctan(
        coff[2] + 2 * coff[1] * init_state.pose_x + 3 * coff[0] * init_state.pose_x**2
    )
    final_state.cross_track_error = (
        np.polyval(coff, init_state.pose_x)
        - init_state.pose_y
        + (init_state.velocity * np.sin(init_state.error_theta) * dtime)
    )
    final_state.error_theta = (
        init_state.theta_angle
        - theta_destination
        + ((init_state.velocity / vehicle_long_diameter) * inputs.steer_angle * dtime)
    )

    return final_state
