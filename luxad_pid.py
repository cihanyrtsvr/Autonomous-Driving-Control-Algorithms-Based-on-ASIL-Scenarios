"""Setup environment that includes worlds waypoints and sensors"""

import math
import time
import logging
import numpy as np
import csv
from navigation import controller
import constant


logging.basicConfig(
    filename="lpcontrol.log",
    level=logging.WARNING,
    format="%(asctime)s - %(levelname)s - %(message)s",
)


class ControllerLuxad:
    """The class has controller objects for the client application"""

    def __init__(self):
        """The initial function for Controller_Luxad"""
        self.controller_speed = constant.SPEED
        self.following_obstacle_queue = constant.obstacle_queue
        self.speed_history = []
        self.target_waypoint = 0
        self.waypoint_index = 0

    def drive_through_plan(self, planned_route, ego_vehicle, sense_obj):
        """controls the vehicle on the waypoints

        Args:
            planned_route (matrix): waypoints that needed to be passed
            ego_vehicle (object): a car that needed to be controlled on path
            sense_obj (matrix): Sensors that will be used
        """

        spec, cam, world_obj = sense_obj

        pid_value = self.create_pid(ego_vehicle)
        logging.debug("The PID controller created")

        self.target_waypoint = planned_route[0]
        time_first = float(time.time() / 1000.0)
        while self.waypoint_index != (len(planned_route) - 1):
            world_obj.tick()
            # get the location of the car in each iteration
            ego_vehicle_pose = ego_vehicle.get_location()
            ego_vehicle_yaw = ego_vehicle.get_transform().rotation.yaw
            ego_vehicle_velocity = 3.6* math.sqrt(
                   ego_vehicle.get_velocity().x ** 2
                   + ego_vehicle.get_velocity().y ** 2
               )
            ego_vehicle_velocity_error = constant.SPEED - ego_vehicle_velocity

            # make spectator go with the camera of the car
            spec.set_transform(cam.get_transform())
            fra_agnle = 90
            cross_track_error, heading_error = self.check_vehicle_to_waypoint_distance(
                ego_vehicle_pose, planned_route, ego_vehicle_yaw, fra_agnle
            )
            current_time = float(time.time() / 1000.0) - time_first
            self.write_to_document(current_time, cross_track_error, heading_error, ego_vehicle_velocity_error)
            if not self.following_obstacle_queue.empty():
                # dequeue obstacle distance information from the queue
                obs = self.following_obstacle_queue.get(timeout=0.1)
                front_obstacle_distance = obs.obstacle_distance
                logging.warning("The distance to OBS: %s", front_obstacle_distance)

                car_control = self.upper_level_control_acc(
                    front_obstacle_distance, pid_value
                )

            else:
                self.controller_speed = constant.SPEED
                car_control = pid_value.run_step(
                    self.controller_speed, self.target_waypoint
                )

            self.speed_history.append((current_time, self.controller_speed))
            logging.debug("The applied control is: %s", car_control)
            ego_vehicle.apply_control(car_control)

        logging.debug("The last waypoint is reached!")
        car_control = pid_value.run_step(0, planned_route[len(planned_route) - 1])
        ego_vehicle.apply_control(car_control)

    def create_pid(self, vehicle_ego):
        """Function that defines PID controller

        Args:
            vehicle_ego (object): The car parameters

        Returns:
            function: PID that created spesificly for passed car
        """

        args_lateral = {"K_P": 1.5, "K_D": 0.35, "K_I": 0.4, "dt": 1.0 / 5.0}

        args_long = {"K_P": 0.9, "K_D": 0.18, "K_I": 0.3, "dt": 1.0 / 5.0}

        return controller.VehiclePIDController(vehicle_ego, args_lateral, args_long)

    def check_vehicle_to_waypoint_distance(
        self, vehicle_pose, planned_waypoints, vehicle_angle, fra_anle
    ):
        """For each DISTANCE_THRESHOLD m distance the waypoint is updated.

        Args:
            vehicle_pose (list): in x, y, z format position of ego vehicle
            target_waypoint (list): in x, y, z format position of next waypoint
            planned_waypoints (matrix): in x, y, z format positions of the route

        """
        waypoints_x_pose = self.target_waypoint.transform.location.x
        waypoints_y_pose = self.target_waypoint.transform.location.y
        delta_x = math.sqrt(
            (vehicle_pose.x - self.target_waypoint.transform.location.x) ** 2
        )
        # print(math.sqrt(delta_x**2))
        delta_y = math.sqrt(
            (-self.target_waypoint.transform.location.y + vehicle_pose.y) ** 2
        )

        real_target_angle = 180 - self.target_waypoint.transform.rotation.yaw
        if real_target_angle < 0:
            real_target_angle = real_target_angle+360
        
        var_vehicle_angle = 180 - vehicle_angle
        if var_vehicle_angle < 0:
            var_vehicle_angle = var_vehicle_angle+360

        heading_angle = math.sqrt(var_vehicle_angle**2) - math.sqrt(
            real_target_angle**2
        )
        if heading_angle > 180 or heading_angle < -180: 
            heading_angle = 0 
        
        error_x = math.sqrt(
            delta_x * math.sin((180 - vehicle_angle) * math.pi / 180) ** 2
        )
        error_y = math.sqrt(
            delta_y * math.cos((180 - vehicle_angle) * math.pi / 180) ** 2
        )
        error = error_x + error_y


        cos_yaw = np.cos(-vehicle_angle*constant.ANGLE_TO_RADIAN)
        sin_yaw = np.sin(-vehicle_angle*constant.ANGLE_TO_RADIAN)

        e_x_res = cos_yaw * (
            waypoints_x_pose - vehicle_pose.x
        ) - sin_yaw * (waypoints_y_pose - vehicle_pose.y)
        e_y_res = sin_yaw * (
            waypoints_x_pose - vehicle_pose.x
        ) + cos_yaw * (waypoints_y_pose - vehicle_pose.y)
        errrs = math.sqrt(e_x_res**2 + e_y_res**2)
        #print(f"MPC_inherited PID_x = {e_x_res} and the normal error: {error}")
        #print(f"MPC_inherited PID_x = {e_y_res}")
        distance_to_ego_vehicle = math.sqrt(delta_x**2 + delta_y**2)
        logging.debug("Distance to waypoint: %s", distance_to_ego_vehicle)

        if distance_to_ego_vehicle < constant.DISTANCE_THRESHOLD:
            self.waypoint_index = self.waypoint_index + 1
            self.target_waypoint = planned_waypoints[self.waypoint_index]

        return error, heading_angle

    def upper_level_control_acc(self, obstacle_distance, pid_creater):
        """The controller that decides to use velocity or distance control

        Args:
            obstacle_distance (int): obstacle_distance
            pid_creater (obj): The PID controller

        Returns:
            obj: The car controller(throttle, brake)
        """
        if obstacle_distance < constant.CRITICAL_DISTANCE:
            speed_control = constant.control_emergency_br
        else:
            self.controller_speed = self.distance_braking_speed(
                obstacle_distance,
                constant.CRITICAL_DISTANCE,
                constant.DECELERATION,
            )
            speed_control = pid_creater.run_step(
                self.controller_speed, self.target_waypoint
            )
        return speed_control

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
        speed_br = math.sqrt(speed_br) * 3.6
        return speed_br  # convertio from m/s to Km/h

    def write_to_document(self, timestamp, cross_track_error, error_theta, vehicle_velocity_error):
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
            "./luxad_docs/PID_Errors.csv", "a", newline="", encoding="utf-8"
        ) as file:
            writer = csv.DictWriter(file, fieldnames=fieldnames)

            # Check if file is empty, if so write header
            file.seek(0, 2)
            if file.tell() == 0:
                writer.writeheader()

            writer.writerow(
                {
                    "timestamp": timestamp,
                    "cross_track_error": cross_track_error,
                    "error_theta": error_theta,
                    "velocity_error": vehicle_velocity_error
                }
            )
