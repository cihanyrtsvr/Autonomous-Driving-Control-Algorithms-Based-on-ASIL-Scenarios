"""

The module is created to inclue common 
constants and helper/utility functions

"""
# pylint: disable=I1101

from queue import Queue
import carla
import numpy as np


HAS_DUMMY_CAR = False  # used for emergency breaking mechanism
SERVER_DELTA_SECOND = 0.05  # in Hertz
SPEED = 30  # in Km/h - target speed  when no obstacle detected
THRESHOLD_SPEED = 40  # in Km/h
ANGLE_TO_RADIAN = 0.0174533  # rad/ang
MS_TO_KMH = 3.6


WEATHER = carla.WeatherParameters(
    cloudiness=0.0,
    precipitation=0.0,
    sun_altitude_angle=10.0,
    sun_azimuth_angle=70.0,
    precipitation_deposits=0.0,
    wind_intensity=0.0,
    fog_density=0.0,
    wetness=0.0,
)
DISTANCE_THRESHOLD = 2  # in meters
CRITICAL_DISTANCE = 7  # in meters
DECELERATION = 1.5  # in meters/sec^2
DECELERATION = 1.5  # in meters/sec^2
CAR_SPAWN_POS = [0, 0]  # X and Y coordinates
DUMMY_SPAWN_POS = [-0.5, 20]


control_emergency_br = carla.VehicleControl(throttle=0.0, brake=1.0)
obstacle_queue = Queue()

# MPC Control Parameters

# Define step-time
STEP_TIME = 0.01
STEP_TIME = 0.015
## prediction horizen ##
PREDICT_HORIZON = 10
ACCELERATION_OFFSET = PREDICT_HORIZON
## init geuss ##
x0_pose = np.zeros(2 * PREDICT_HORIZON)
# interpolation values
INTERPOL_INCREMENT_ITERATION = 0.01

## cost function weights ##

CROSS_TRACK_ERROR_WEIGHT = 400
ERROR_THETA_WEIGHT = 40
VELOCITY_WEIGHT = 30
STEERING_RATE_WEIGHT = 1000
ACCELERATION_RATE_WEIGHT = 2
STEERING_WEIGHT = 7
ACCELERATION_WEIGHT = 1
## input bounds ##
BOUND1 = (-1.22, 1.22)
BOUND2 = (0, 1)
BOUNDS = [BOUND1] * 10 + [BOUND2] * 10
conv_rad_to_steer = 180.0 / 70.0 / np.pi

THROTTLE_OUTPUT = 0
STEER_OUTPUT = 0
BRAKE_OUTPUT = 0
TOTAL_DISTANCE_AHEAD = 5
