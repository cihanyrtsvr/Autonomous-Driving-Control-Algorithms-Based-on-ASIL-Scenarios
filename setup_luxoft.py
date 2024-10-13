"""Setup environment that includes worlds wapoints  and sensors"""

# pylint: disable=I1101

import logging
from collections import namedtuple
from navigation.global_route_planner import GlobalRoutePlanner
import carla
import constant

logging.basicConfig(
    filename="./luxad_docs/records.log",
    level=logging.WARNING,
    format="%(asctime)s.%(msecs)04d - %(levelname)s - %(message)s",
    datefmt="%S",
    filemode="w",
)


class SetupLuxad:
    """At the start of the application,
    the class provides settings and object"""

    def __init__(self):
        self.world = self.create_world()
        self.create_synch_settings()
        self.waypoint_list = self.draw_waypoints()
        self.obs_queue = constant.obstacle_queue
        self.distance_detection = self.braking_distance(
            constant.THRESHOLD_SPEED, constant.DECELERATION, constant.CRITICAL_DISTANCE
        )

    def create_world(self):
        """Function that provides communication
        link between client and server

        Returns:
            object: world
        """
        client = carla.Client("localhost", 2000)
        client.set_timeout(15.0)

        world = client.get_world()

        if "Town01" not in world.get_map().name:
            world = client.load_world("Town01")

        # Wheather conditions
        weather = constant.WEATHER
        world.set_weather(weather)
        logging.debug("The worlds is created")
        return world

    def create_synch_settings(self):
        """As a default setting Server and Client are running
        asynchronous mode that leads to uncertain responses
        in every iteration.
        To avoid such behavoir we needed to make server and
        client synchronized.

        In the controller_luxoft loop we provide "world.tick()"
        function that allows in each iteration to make server
        side wait client to reach existing status.
        """
        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = constant.SERVER_DELTA_SECOND
        self.world.apply_settings(settings)

    def delete_objects_and_settings(self, vehicle_obj, dummy_obj):
        """
        When the operations ended the
        synchronization should be disabled and
        settings should return to its defaults
        status
        """
        self.world.apply_settings(carla.WorldSettings(False, False, 0))
        vehicle_obj.destroy()
        if dummy_obj is not None:
            dummy_obj.destroy()

    def draw_waypoints(self):
        """Used to draw waypoints"""

        # Set the spawn points
        spawn_points = self.world.get_map().get_spawn_points()

        start_p = carla.Location(spawn_points[0].location)
        finish_p = carla.Location(spawn_points[100].location)
        # To implify the start and end pose
        self.world.debug.draw_point(
            start_p, color=carla.Color(r=0, g=255, b=0), size=1.6, life_time=120.0
        )
        self.world.debug.draw_point(
            finish_p, color=carla.Color(r=0, g=255, b=0), size=1.6, life_time=120.0
        )

        # Set the environment and get the Waypoints
        amap = self.world.get_map()
        sample_resolution = 2
        wps = []
        grp = GlobalRoutePlanner(amap, sample_resolution)

        waypoints = grp.trace_route(start_p, finish_p)

        for iter_i, way in enumerate(waypoints):
            if iter_i % 10 == 0:
                self.world.debug.draw_string(
                    way[0].transform.location,
                    "O",
                    draw_shadow=False,
                    color=carla.Color(r=255, g=0, b=0),
                    life_time=120.0,
                    persistent_lines=True,
                )

            else:
                self.world.debug.draw_string(
                    way[0].transform.location,
                    "^",
                    draw_shadow=False,
                    color=carla.Color(r=0, g=0, b=255),
                    life_time=1000,
                    persistent_lines=True,
                )

        for waypoint in waypoints:
            wps.append(waypoint[0])

        logging.debug("The wayponts are drawn")

        return wps

    def on_obstacle_callback(self, event):
        """Access the obstacle distance information
        when the object detected

        Args:
            event (object): informations on object
        """
        obstacle_distance = event.distance
        actor_type_str = str(event.other_actor)

        actor_loc_rot = event.other_actor.get_transform()

        ObstacleInfo = namedtuple(
            "ObstacleInfo", ["actor_position", "obstacle_distance"]
        )

        actor_type = actor_type_str.split(", ")[1].split("=")[1].split(".")[0]
        logging.debug("Yes OBS detected its name is: %s", actor_type_str)
        if actor_type == "vehicle" and not actor_type == "static":
            obs_sensor_out = ObstacleInfo(actor_loc_rot, obstacle_distance)
            self.obs_queue.put_nowait(obs_sensor_out)
            logging.debug("Yes OBS detected: %s", obs_sensor_out.actor_position)
        else:
            logging.debug("No OBS detected..")

    def vehicle_spawn(self, offset):
        """Cration of Vehicle object in world

        Args:
            offset (array): offset for location spawn

        Returns:
            object: vehicle object
        """
        # Set the spawn points
        spawn_points = self.world.get_map().get_spawn_points()
        new_pose = carla.Transform(
            carla.Location(
                x=spawn_points[0].location.x + offset[0],
                y=spawn_points[0].location.y + offset[1],
                z=spawn_points[0].location.z,
            ),
            carla.Rotation(
                pitch=spawn_points[0].rotation.pitch,
                yaw=spawn_points[0].rotation.yaw,
                roll=spawn_points[0].rotation.roll,
            ),
        )

        # Update the original spawn_points list with the modified transform
        # get the points to put a car
        blueprint_library = self.world.get_blueprint_library()
        vehicle_bp = blueprint_library.filter("vehicle.micro.microlino")[0]

        logging.debug("The vehicle is created")

        return self.world.spawn_actor(vehicle_bp, new_pose)

    def camera_spawn(self, vehicle_obj):
        """The camera and spectator object creator function

        Args:
            vehicle_obj (object): the vehicle that the camera
            will be attached

        Returns:
            object: camera/spectator object
        """
        camera_bp = self.world.get_blueprint_library().find("sensor.camera.rgb")
        camera_transform = carla.Transform(
            carla.Location(x=-6, z=5), carla.Rotation(pitch=330)
        )
        cam = self.world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle_obj)

        # attach spec with camera
        spec = self.world.get_spectator()
        spec.set_transform(cam.get_transform())
        cam_spec = [cam, spec]
        logging.debug("The camera is created")
        return cam_spec

    def obs_sensor_spawn(self, vehicle_obj):
        """Obstacle sensor creator function

        Args:
            vehicle_obj (object): vehicle that sensor will be attached

        Returns:
            object: obstacle sensor
        """
        # Create the obstacle sensor
        obs_bp = self.world.get_blueprint_library().find("sensor.other.obstacle")
        obs_bp.set_attribute("distance", str(30))
        obs_bp.set_attribute("hit_radius", str(1))
        obs_bp.set_attribute("only_dynamics", str(True))
        obs_bp.set_attribute("sensor_tick", str(0.02))
        obs_location = carla.Location(0, 0, 0.4)
        obs_rotation = carla.Rotation(0, 0, 0)
        obs_transform = carla.Transform(obs_location, obs_rotation)

        logging.debug("The Obstacle sensor is created")
        return self.world.spawn_actor(obs_bp, obs_transform, attach_to=vehicle_obj)

    def braking_distance(self, speed, decceleration, cr_dist):
        """The function that determines min detection distance
        for the obstacle detector sensor

        Args:
            speed (int): Constant speed that will be assigned for ACC
            decceleration (int): Safe(comfortable) decceleration constant
            cr_dist (int): threshold distance to implement emergency breaking

        Returns:
            int: min_obstacle distance
        """
        # Convert initial speed from km/h to m/s
        speed_ms = speed * 1000 / 3600

        # Calculate braking distance using the equation of motion
        dist = speed_ms * speed_ms / (2 * decceleration) + cr_dist

        logging.info("The emergency breaking distance is: %s", dist)
        return dist
