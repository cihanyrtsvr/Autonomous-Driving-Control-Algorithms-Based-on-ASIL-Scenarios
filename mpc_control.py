#!/usr/bin/env python3

"""
2D Controller Class to be used for the CARLA waypoint follower demo.
"""

import csv
import dataclasses
import numpy as np
from scipy.optimize import minimize
from mpc_utils import State, Inputs, model
import constant

# import matplotlib.pyplot as plt
import cutils

init_state = State()


@dataclasses.dataclass
class Current:
    """Current class for convention created."""

    speed: int
    desired_speed: int
    time: int
    pose_x: int
    pose_y: int
    yaw: int
    timestamp: int


@dataclasses.dataclass
class SetControl:
    """Current class for convention created."""

    throttle: int
    brake: int
    steer: int


class Controller2D:
    """The 2 Dimentional MPC controller class."""

    def __init__(self, waypoints):
        self.vars = cutils.CUtils()
        self._current = Current(
            speed=0, desired_speed=0, time=0, pose_x=0, pose_y=0, yaw=0, timestamp=0
        )

        self._frame = 0
        self._start_control_loop = False
        self._set_control = SetControl(throttle=0, brake=0, steer=0)
        self._waypoints = waypoints

    def update_values(self, vehicle_parameters, timestamp, frame):
        """Updating the MPC parameters depend on
        current parameters of ego_vehicle

        Args:
            x_pose (int):
            y_pose (int):
            yaw (int):
            speed (int):
            timestamp (int):
            frame (boolean):
        """
        (
            self._current.pose_x,
            self._current.pose_y,
            self._current.yaw,
            self._current.speed,
        ) = vehicle_parameters
        self._current.timestamp = timestamp
        self._frame = frame
        if self._frame:
            self._start_control_loop = True

    def update_desired_speed(self, speed_limit):
        """Updating the speed depend on the waypoint passed speed parameters"""

        min_distance = float("inf")
        desired_speed = 0

        # Iterate directly over _waypoints
        for waypoint in self._waypoints:
            distance = np.linalg.norm(
                np.array(
                    [
                        waypoint[0] - self._current.pose_x,
                        waypoint[1] - self._current.pose_y,
                    ]
                )
            )
            if distance < min_distance:
                min_distance = distance
                min_waypoint = (
                    waypoint  # Keep track of the waypoint with minimum distance
                )

        # Find the desired speed based on the closest waypoint
        if min_waypoint != self._waypoints[-1]:
            if 0 <= speed_limit < constant.SPEED / 3.6:
                desired_speed = speed_limit
            else:
                desired_speed = min_waypoint[2]
        else:
            desired_speed = self._waypoints[-1][2]

        self._current.desired_speed = desired_speed

    def update_waypoints(self, new_waypoints):
        """When there is a new waypoints

        Args:
            new_waypoints (matrix): row -> (x, y, velocity)
        """
        self._waypoints = new_waypoints

    def get_commands(self):
        """Used to get control commands from MPC library

        Returns:
            object: tuple of throttle, brake, and steering
        """
        return (
            self._set_control.throttle,
            self._set_control.steer,
            self._set_control.brake,
        )

    def set_throttle(self, input_throttle):
        """sets the throttle depend on the produced MPC response

        Args:
            input_throttle (int):
        """
        # Clamp the throttle command to valid constant.bounds
        throttle = np.fmax(np.fmin(input_throttle, 1.0), 0.0)
        self._set_control.throttle = throttle

    def set_steer(self, input_steer_in_rad):
        """sets the steering depend on the produced MPC response

        Args:
            input_steer_in_rad (int):
        """
        # Covnert radians to [-1, 1]
        input_steer = constant.conv_rad_to_steer * input_steer_in_rad

        # Clamp the steering command to valid constant.bounds
        steer = np.fmax(np.fmin(input_steer, 1.22), -1.22)

        self._set_control.steer = steer

    def set_brake(self, input_brake):
        """sets the braking depend on the produced MPC response

        Args:
            input_brake (int):
        """
        # Clamp the steering command to valid constant.bounds
        brake = np.fmax(np.fmin(input_brake, 1.0), 0.0)
        self._set_control.brake = brake

    def map_to_car(self, x_pose, y_pose, yaw, waypoints):
        """finds the distance from waypoint to ego_vehicle

        Args:
            x_pose (int):
            y_pose (int):
            yaw (int):
            waypoints (matrix): x and y pose

        Returns:
            cars : the distance in  ax and y coordinate
        """
        waypoints = np.array(waypoints)
        shift_x = waypoints[0] - x_pose
        shift_y = waypoints[1] - y_pose

        car_x_pose = shift_x * np.cos(-yaw) - shift_y * np.sin(-yaw)
        car_y_pose = shift_x * np.sin(-yaw) + shift_y * np.cos(-yaw)

        return car_x_pose, car_y_pose

    def map_coordinate_to_car_coord(self, x_pose, y_pose, yaw, waypoints):
        """Mapping th ego_vehicle pose to waypoint coordinate

        Args:
            x_pose (int):
            y_pose (int):
            yaw (int):
            waypoints (tuple):

        Returns:
            referance_waypoint: Which waypoint is aimed to arrive
        """
        waypoints = np.squeeze(waypoints)
        waypoints_x_pose = waypoints[:, 0]
        waypoints_y_pose = waypoints[:, 1]

        num_waypoints = waypoints.shape[0]

        ## create the Matrix with 3 vectors for the waypoint x and y coordinates w.r.t. car
        waypoint_vehicle_referance = np.zeros(shape=(3, num_waypoints))
        cos_yaw = np.cos(-yaw)
        sin_yaw = np.sin(-yaw)

        waypoint_vehicle_referance[0, :] = cos_yaw * (
            waypoints_x_pose - x_pose
        ) - sin_yaw * (waypoints_y_pose - y_pose)
        waypoint_vehicle_referance[1, :] = sin_yaw * (
            waypoints_x_pose - x_pose
        ) + cos_yaw * (waypoints_y_pose - y_pose)

        return waypoint_vehicle_referance

    def update_controls(self, acc_speed):
        """
        This is the part where simulator feedback is taken in to account.
        """

        self.update_desired_speed(acc_speed)

        self.vars.create_var("x_previous", 0.0)
        self.vars.create_var("y_previous", 0.0)
        self.vars.create_var("th_previous", 0.0)
        self.vars.create_var("v_previous", 0.0)
        self.vars.create_var("cte_previous", 0.0)
        self.vars.create_var("eth_previous", 0.0)
        self.vars.create_var("t_previous", 0.0)

        self.control_prediction_loop()

        self.vars.t_previous = (
            self._current.timestamp
        )  # Store timestamp  to be used in next step
        self.vars.v_previous = self._current.speed

    def control_prediction_loop(self):
        """This part is used for the optimizing the controller
        and calculate the best weight(smallest)

        """
        waypoints_vehicle_referance = self.map_coordinate_to_car_coord(
            self._current.pose_x,
            self._current.pose_y,
            self._current.yaw,
            self._waypoints,
        )
        waypoints_vehicle_referance_x = waypoints_vehicle_referance[0, :]
        waypoints_vehicle_referance_y = waypoints_vehicle_referance[1, :]

        ## find COFF of the polynomial ##
        coff = np.polyfit(
            waypoints_vehicle_referance_x, waypoints_vehicle_referance_y, 3
        )
        velocity_referance = self._current.desired_speed
        # Skip the first frame to store previous values properly
        array_error = []
        if self._start_control_loop:
            # cost fun or objective fun req to minimaize ##
            def objective(x_state):
                u_state = Inputs()
                error = 0
                init_state_1 = init_state
                for i in range(constant.PREDICT_HORIZON):
                    u_state.steer_angle = x_state[i - 1]
                    u_state.acceleration = x_state[i + constant.ACCELERATION_OFFSET]
                    next_state = model(
                        u_state,
                        init_state_1,
                        coff,
                        dtime=constant.STEP_TIME,
                        vehicle_long_diameter=2.2073,
                    )
                    if i == 0:
                        error += (
                            constant.CROSS_TRACK_ERROR_WEIGHT
                            * np.absolute(next_state.cross_track_error) ** 2
                            + constant.ERROR_THETA_WEIGHT
                            * np.absolute(next_state.error_theta) ** 2
                            + constant.VELOCITY_WEIGHT
                            * np.absolute(next_state.velocity - velocity_referance) ** 2
                            + constant.STEERING_WEIGHT
                            * np.absolute(u_state.steer_angle) ** 2
                            + constant.ACCELERATION_WEIGHT
                            * np.absolute(u_state.acceleration) ** 2
                        )
                    else:
                        error += (
                            constant.CROSS_TRACK_ERROR_WEIGHT
                            * np.absolute(next_state.cross_track_error) ** 2
                            + constant.ERROR_THETA_WEIGHT
                            * np.absolute(next_state.error_theta) ** 2
                            + constant.VELOCITY_WEIGHT
                            * np.absolute(next_state.velocity - velocity_referance) ** 2
                            + constant.STEERING_RATE_WEIGHT
                            * np.absolute(u_state.steer_angle - x_state[i - 1]) ** 2
                            + constant.ACCELERATION_RATE_WEIGHT
                            * np.absolute(
                                u_state.acceleration
                                - x_state[i + constant.ACCELERATION_OFFSET - 1]
                            )
                            ** 2
                            + constant.STEERING_WEIGHT
                            * np.absolute(u_state.steer_angle) ** 2
                            + constant.ACCELERATION_WEIGHT
                            * np.absolute(u_state.acceleration) ** 2
                        )
                    init_state_1 = next_state
                    array_error.append(error)

                return error

            car_referance_x = car_referance_y = car_reference_yaw = 0.0
            constant.STEER_OUTPUT = 0
            constant.BRAKE_OUTPUT = 0
            constant.THROTTLE_OUTPUT = 0
            cross_track_error = np.polyval(coff, car_referance_x) - car_referance_y

            # get orientation error from fit
            # ( Since we are trying a 3rd order poly, then, f' = a1 + 2*a2*x + 3*a3*x2)
            yaw_err = car_reference_yaw - np.arctan(coff[1])

            # I can send the ACTUAL state to the MPC or I can try to compensate
            # for the latency by "predicting" what would be the state after the latency period.
            latency = 0.005  # Server_Delta/10 Hz

            # # Let's predict the state. Rembember that px, py and psi wrt car are all 0.
            init_state.pose_x = self._current.speed * latency
            init_state.pose_y = 0
            init_state.theta_angle = (
                -1 * self._current.speed * self._set_control.steer * latency / 3
            )
            init_state.velocity = (
                self._current.speed
                + (self._current.speed - self.vars.v_previous)
                / constant.STEP_TIME
                * latency
            )
            init_state.cross_track_error = (
                cross_track_error + self._current.speed * np.sin(yaw_err) * latency
            )
            init_state.error_theta = yaw_err + init_state.theta_angle

            solution = minimize(
                objective, constant.x0_pose, method="SLSQP", bounds=constant.BOUNDS
            )
            u_state = solution.x

            constant.STEER_OUTPUT = u_state[0]
            if u_state[constant.ACCELERATION_OFFSET] < 0:
                constant.BRAKE_OUTPUT = u_state[constant.ACCELERATION_OFFSET]
                print("BRAKE!")
            else:
                constant.THROTTLE_OUTPUT = u_state[constant.ACCELERATION_OFFSET]

            print("[INFO] throttle_output: " + str(constant.THROTTLE_OUTPUT))
            print("[INFO] brake_output: " + str(constant.BRAKE_OUTPUT))
            print("[INFO] steer_output: " + str(constant.STEER_OUTPUT))
            # print("[INFO] X: "+ str(init_state.x))
            # print("[INFO] Y: "+ str(init_state.y))
            # print("[INFO] TH: " + str(init_state.th))
            print("[INFO] V: " + str(init_state.velocity * constant.MS_TO_KMH))
            # print("[INFO] CTE: " + str(init_state.cross_track_error))
            # print("[INFO] ETH: " + str(init_state.error_theta))
            # print("[INFO] COFF: " + str(coff))
            medium_array = np.array(array_error)
            mean_error = np.mean(medium_array)
            print("______________________________________________")

            self.write_to_document(
                self._current.timestamp,
                init_state.cross_track_error,
                init_state.error_theta*180/np.pi,
                mean_error,
            )
            ######################################################
            # SET CONTROLS OUTPUT
            ######################################################
            self.set_throttle(constant.THROTTLE_OUTPUT)  # in percent (0 to 1)
            self.set_steer(constant.STEER_OUTPUT)  # in rad (-1.22 to 1.22)
            self.set_brake(constant.BRAKE_OUTPUT)  # in percent (0 to 1)

    def write_to_document(self, timestamp, cross_track_error, error_theta, error):
        """Function is used to log the values of
        cross track error and heading error(error_theta)
        with respect to timestamp
        Args:
            timestamp (float): time in miliseconds
            cross_track_error (float): lateral distance to waypoint
            error_theta (float): heading error
        """
        fieldnames = ["timestamp", "cross_track_error", "error_theta", "velocity_error"]
        with open(
            "./luxad_docs/MPC_Errors.csv", "a", newline="", encoding="utf-8"
        ) as file:
            writer = csv.DictWriter(file, fieldnames=fieldnames)

            # Check if file is empty, if so write header
            file.seek(0, 2)
            if file.tell() == 0:
                writer.writeheader()
            
            velocity_error = constant.SPEED - self._current.speed
            writer.writerow(
                {
                    "timestamp": timestamp,
                    "cross_track_error": cross_track_error,
                    "error_theta": error_theta,
                    "velocity_error": velocity_error
                }
            )
