import logging
import constant
from setup_luxoft import SetupLuxad
import csv
import os

# Define the path to the CSV file
csv_file_path = "./luxad_docs/MPC_Errors.csv"

from luxad_mpc import ControllerMPCLuxad
from luxad_pid import ControllerLuxad


class Client:
    """Avoids main class to be initialized more than one time"""

    _instance = None

    def __new__(cls, controller_type, *args, **kwargs):
        if not cls._instance:
            cls._instance = super().__new__(cls, *args, **kwargs)
            cls._instance._controller = cls._get_controller(controller_type)
        return cls._instance

    def __init__(self, controller_type):
        self.setup = SetupLuxad()
        self.control = self._controller()

    @classmethod
    def _get_controller(cls, controller_type):
        if controller_type == "MPC":
            return ControllerMPCLuxad
        elif controller_type == "PID":
            return ControllerLuxad
        else:
            raise ValueError("Invalid controller type. Please specify 'MPC' or 'PID'.")

    def run_simulation(self):
        """The function that responsible from
        running whole simulation

        """
        csv_file_path = f"./luxad_docs/{controller_type}_Errors.csv"
        # Clear the contents of the CSV file by truncating it
        with open(csv_file_path, "w", newline="", encoding="utf-8") as clear_file:
            pass  # This simply opens and closes the file, effectively truncating it

        if constant.HAS_DUMMY_CAR:
            dummy_vehicle = self.setup.vehicle_spawn(constant.DUMMY_SPAWN_POS)
        else:
            dummy_vehicle = None

        try:
            vehicle = self.setup.vehicle_spawn(constant.CAR_SPAWN_POS)

            # Get the camera actor and attach it to vehicle
            camera, spectator = self.setup.camera_spawn(vehicle)

            obstacle_sensor = self.setup.obs_sensor_spawn(vehicle)

            # Spawn and arrange the Obstacle sensor and listen
            obstacle_sensor.listen(self.setup.on_obstacle_callback)

            # Fetch and draw the waypoints
            sensor_objects = [spectator, camera, self.setup.world]

            # Control the car through the waypoints
            self.control.drive_through_plan(
                self.setup.waypoint_list, vehicle, sensor_objects
            )
        except (IndexError, KeyboardInterrupt):
            logging.warning("KEYBOARD INTERRUPT")
        except RuntimeError:
            logging.warning("SPAWN ERROR, retry :)")
        finally:
            # Cleaning the world(server) from the created objects
            self.setup.delete_objects_and_settings(vehicle, dummy_vehicle)
            logging.warning("done.\n")


if __name__ == "__main__":
    import sys

    if len(sys.argv) != 2:
        print("Usage: luxad_run_client <controller_type>")
        sys.exit(1)

    controller_type = sys.argv[1]
    client_instance = Client(controller_type)
    client_instance.run_simulation()
