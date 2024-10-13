"""Setup environment that includes worlds waypoints and sensors"""
# pylint: disable=I1101

import math
import time
import logging
import dataclasses
import numpy as np
import carla  # type: ignore
import mpc_control
import constant


logging.basicConfig(
    filename="lpcontrol.log",
    level=logging.WARNING,
    format="%(asctime)s - %(levelname)s - %(message)s",
)


@dataclasses.dataclass
class CurrentVehicleParameters:
    """Current Vehicle class for convention created."""

    speed: int
    time: int
    pose_x: int
    pose_y: int
    yaw: int
    speed_limit: int


@dataclasses.dataclass
class HistoryVehicleParameters:
    """Current Vehicle class for convention created."""

    speed: np.ndarray
    time: np.ndarray
    pose_x: np.ndarray
    pose_y: np.ndarray
    yaw: np.ndarray


@dataclasses.dataclass
class Waypoint:
    """The mpc waypoints convention for init."""

    mpc: np.ndarray
    index: int
    distance: np.ndarray
    interpol: np.ndarray  # interpolated values -> (rows = waypoints, columns = [x, y, v])
    hash_interpol: np.ndarray  # self.waypoints_mpc to the index of the waypoint in interpol


class ControllerMPCLuxad:
    """The class has controller objects for the client application."""

    def __init__(self):
        """The initial function for Controller_Luxad"""
        self.controller_speed = constant.SPEED
        self.following_obstacle_queue = constant.obstacle_queue
        # self.graph_history = []
        self.history_vehicle = HistoryVehicleParameters(
            speed=[0], time=[0], pose_x=[0], pose_y=[0], yaw=[0]
        )
        self.closest_index = 0  # closest index to ego_vehicle
        self.closest_distance = 0  # in clossest index distance to ego_vehicle
        self.current_vehicle = CurrentVehicleParameters(
            speed=0, time=0, pose_x=0, pose_y=0, yaw=0, speed_limit=-1
        )
        self.mpc_waypoints = Waypoint(
            mpc=[], index=0, distance=[], interpol=[], hash_interpol=[]
        )

    def drive_through_plan(self, planned_route, ego_vehicle, sense_obj):
        """controls the vehicle on the waypoints,
        the obstacle queue is filled in the obstacle callback

        Args:
            planned_route (matrix): waypoints that needed to be passed
            ego_vehicle (object): a car that needed to be controlled on path
            sense_obj (matrix): Sensors that will be used
        """

        spec, cam, world_obj = sense_obj

        controller = self.controller_defining(planned_route)

        self.waypoint_interpolation(planned_route)

        self.history_vehicle.pose_x[0] = ego_vehicle.get_location().x
        self.history_vehicle.pose_y[0] = ego_vehicle.get_location().y
        self.history_vehicle.yaw[0] = (
            ego_vehicle.get_transform().rotation.yaw * 0.0174533
        )

        logging.debug("The MPC controller created")

        while self.mpc_waypoints.index != (len(planned_route) - 1):
            world_obj.tick()

            # make spectator go with the camera of the car
            spec.set_transform(cam.get_transform())

            ego_vehicle_pose = ego_vehicle.get_location()

            self.current_vehicle.pose_x = ego_vehicle_pose.x
            self.current_vehicle.pose_y = ego_vehicle_pose.y

            self.current_vehicle.yaw = (
                ego_vehicle.get_transform().rotation.yaw * constant.ANGLE_TO_RADIAN
            )

            self.current_vehicle.speed = math.sqrt(
                ego_vehicle.get_velocity().x ** 2 + ego_vehicle.get_velocity().y ** 2
            )
            self.current_vehicle.time = float(time.time() / 1000.0)

            # Store history
            self.history_vehicle.pose_x.append(self.current_vehicle.pose_x)
            self.history_vehicle.pose_y.append(self.current_vehicle.pose_y)
            self.history_vehicle.yaw.append(self.current_vehicle.yaw)
            self.history_vehicle.speed.append(self.current_vehicle.speed)
            self.history_vehicle.time.append(self.current_vehicle.time)
            ######################
            if not self.following_obstacle_queue.empty():
                # dequeue obstacle distance information from the queue
                obs = self.following_obstacle_queue.get(timeout=0.1)
                front_obstacle_distance = obs.obstacle_distance
                logging.warning("The distance to OBS: %s", front_obstacle_distance)

                if front_obstacle_distance < constant.CRITICAL_DISTANCE:
                    control = constant.control_emergency_br
                else:
                    self.current_vehicle.speed_limit = self.distance_braking_speed(
                        front_obstacle_distance,
                        constant.CRITICAL_DISTANCE,
                        constant.DECELERATION,
                    )
                    ###########################
                    self.update_controller(controller)
                    control = self.controller_assignment(controller)

            else:
                self.current_vehicle.speed_limit = -1
                ###########################
                self.update_controller(controller)
                control = self.controller_assignment(controller)
            # self.graph_history.append(
            #    (self.current_vehicle.time, self.current_vehicle.speed)
            # )
            logging.debug("The applied control is: %s", control)
            ego_vehicle.apply_control(control)
            self.mpc_waypoints.index = +1

        logging.debug("The last waypoint is reached!")
        ego_vehicle.apply_control(control)

    def waypoint_interpolation(self, planned_wayponts):
        """The waypoint updater for the planned_waypoints

        Args:
            planned_wayponts (matrix): x,y coordinates of the waypoints routes
        """
        self.mpc_waypoints.mpc = [
            [
                waypoint.transform.location.x,
                waypoint.transform.location.y,
                self.controller_speed / constant.MS_TO_KMH,
            ]
            for waypoint in planned_wayponts
        ]

        self.mpc_waypoints.mpc = np.array(self.mpc_waypoints.mpc)
        # Linear interpolation computations
        # Compute a list of distances between waypoints
        for i in range(1, self.mpc_waypoints.mpc.shape[0]):
            self.mpc_waypoints.distance.append(
                np.sqrt(
                    (self.mpc_waypoints.mpc[i, 0] - self.mpc_waypoints.mpc[i - 1, 0])
                    ** 2
                    + (self.mpc_waypoints.mpc[i, 1] - self.mpc_waypoints.mpc[i - 1, 1])
                    ** 2
                )
            )
        self.mpc_waypoints.distance.append(
            0
        )  # last distance is 0 because it is the distance
        # from the last waypoint to the last waypoint

        interp_counter = 0  # counter for current interpolated point index

        for i in range(self.mpc_waypoints.mpc.shape[0] - 1):
            # Add original waypoint to interpolated waypoints list (and append
            # it to the hash table)
            self.mpc_waypoints.interpol.append(list(self.mpc_waypoints.mpc[i]))
            self.mpc_waypoints.hash_interpol.append(interp_counter)
            interp_counter += 1

            # Suppself.waypoints_mpcs/hide the warning
            np.seterr(invalid="ignore")
            # Interpolate to the next waypoint. First compute the number of
            # points to interpolate based on the desired self.waypoints_mpcolution and
            # incrementally add interpolated points until the next waypoint
            # is about to be reached.
            num_pts_to_interp = int(
                np.floor(
                    self.mpc_waypoints.distance[i]
                    / float(constant.INTERPOL_INCREMENT_ITERATION)
                )
                - 1
            )
            wp_vector = self.mpc_waypoints.mpc[i + 1] - self.mpc_waypoints.mpc[i]
            wp_uvector = wp_vector / np.linalg.norm(wp_vector)
            for j in range(num_pts_to_interp):
                next_wp_vector = (
                    constant.INTERPOL_INCREMENT_ITERATION * float(j + 1) * wp_uvector
                )
                self.mpc_waypoints.interpol.append(
                    list(self.mpc_waypoints.mpc[i] + next_wp_vector)
                )
                interp_counter += 1
        # add last waypoint at the end
        self.mpc_waypoints.interpol.append(list(self.mpc_waypoints.mpc[-1]))
        self.mpc_waypoints.hash_interpol.append(interp_counter)
        interp_counter += 1

    def new_propper_waypoint_iteration(self, current_x_pose, current_y_pose):
        """Getting the new waypoints to follow by interpolation between waypoints

        Args:
            current_x_pose (int): current x
            current_y_pose (int): current y

        Returns:
            matrix(int): waypoint list of interpolated routes
        """

        ###
        # Controller update (this uses the controller2d.py implementation)
        ###

        # To reduce the amount of waypoints sent to the controller,
        # provide a subset of waypoints that are within some
        # lookahead distance from the closest point to the car. Provide
        # a set of waypoints behind the car as well.

        # Find closest waypoint index to car. First increment the index
        # from the previous index until the new distance calculations
        # are increasing. Apply the same rule decrementing the index.
        # The final index should be the closest point (it is assumed that
        # the car will always break out of instability points where there
        # are two indices with the same minimum distance, as in the
        # center of a circle)
        self.closest_distance = np.linalg.norm(
            np.array(
                [
                    self.mpc_waypoints.mpc[self.closest_index, 0] - current_x_pose,
                    self.mpc_waypoints.mpc[self.closest_index, 1] - current_y_pose,
                ]
            )
        )
        new_distance = self.closest_distance
        new_index = self.closest_index
        while new_distance <= self.closest_distance:
            self.closest_distance = new_distance
            self.closest_index = new_index
            new_index += 1
            if new_index >= self.mpc_waypoints.mpc.shape[0]:  # End of path
                break
            new_distance = np.linalg.norm(
                np.array(
                    [
                        self.mpc_waypoints.mpc[new_index, 0] - current_x_pose,
                        self.mpc_waypoints.mpc[new_index, 1] - current_y_pose,
                    ]
                )
            )
        new_distance = self.closest_distance
        new_index = self.closest_index
        while new_distance <= self.closest_distance:
            self.closest_distance = new_distance
            self.closest_index = new_index
            new_index -= 1
            if new_index < 0:  # Beginning of path
                break
            new_distance = np.linalg.norm(
                np.array(
                    [
                        self.mpc_waypoints.mpc[new_index, 0] - current_x_pose,
                        self.mpc_waypoints.mpc[new_index, 1] - current_y_pose,
                    ]
                )
            )

        # Once the closest index is found, return the path that has 1
        # waypoint behind and X waypoints ahead, where X is the index
        # that has a lookahead distance specified by
        # INTERP_LOOKAHEAD_DISTANCE
        waypoint_subset_first_index = self.closest_index - 1
        waypoint_subset_first_index = max(waypoint_subset_first_index, 0)

        waypoint_subset_last_index = self.closest_index
        total_distance_ahead = 0
        while total_distance_ahead < constant.TOTAL_DISTANCE_AHEAD:
            total_distance_ahead += self.mpc_waypoints.distance[
                waypoint_subset_last_index
            ]
            waypoint_subset_last_index += 1
            if waypoint_subset_last_index >= self.mpc_waypoints.mpc.shape[0]:
                waypoint_subset_last_index = self.mpc_waypoints.mpc.shape[0] - 1
                break

        new_waypoints = self.mpc_waypoints.interpol[
            self.mpc_waypoints.hash_interpol[
                waypoint_subset_first_index
            ] : self.mpc_waypoints.hash_interpol[waypoint_subset_last_index]
            + 1
        ]

        return new_waypoints

    def controller_assignment(self, controller):
        """Controller values inherited

        Args:
            controller (object): object of the MPC controller.

        Returns:
            object: control command carla object.
        """
        control = carla.VehicleControl()
        control.throttle, control.steer, control.brake = controller.get_commands()
        control.hand_brake = False
        control.manual_gear_shift = False
        control.reverse = False

        return control

    def controller_defining(self, routes):
        """Creating the MPC controller by passing converted route_points.

        Args:
            routes (matrix): waypoint matrix -> row(x,y,z)

        Returns:
            object: MPC_controller object.
        """
        waypoints_mpc_list = list(
            (
                waypoint.transform.location.x,
                waypoint.transform.location.y,
                self.controller_speed / 3.6,
            )
            for waypoint in routes
        )

        return mpc_control.Controller2D(waypoints_mpc_list)

    def update_controller(self, controller_mpc):
        """Updates the controller by current values

        Args:
            controller_mpc (list): waypoint lists
        """

        new_waypoints = self.new_propper_waypoint_iteration(
            self.current_vehicle.pose_x, self.current_vehicle.pose_y
        )

        controller_mpc.update_waypoints(new_waypoints)
        current_vehicle_parameters = [
            self.current_vehicle.pose_x,
            self.current_vehicle.pose_y,
            self.current_vehicle.yaw,
            self.current_vehicle.speed,
        ]
        print(self.current_vehicle.speed * constant.MS_TO_KMH)
        # Update the other controller values and controls
        controller_mpc.update_values(
            current_vehicle_parameters,
            self.current_vehicle.time,
            True,
        )
        controller_mpc.update_controls(self.current_vehicle.speed_limit)

    def distance_braking_speed(self, obstacle_dist, critical_dist, decelleration):
        """function returns the (ACC) distance control speed

        Args:
            obstacle_dist (int): distance to obstacle
            critical_dist (int): threashold distance to implement emergency brekaing
            decelleration (int): the safe decceleration value

        Returns:
            int: speed for implement distance control
        """
        speed_br = (obstacle_dist - critical_dist) * 2 * decelleration
        speed_br = math.sqrt(speed_br)
        print(speed_br * constant.MS_TO_KMH)
        return speed_br  # convertio from m/s
