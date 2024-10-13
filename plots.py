import sys
import os
import pandas as pd
import scienceplots
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

plt.style.use("science")


def animate(i, csv_file):
    if not os.path.exists(csv_file):
        print(f"Error: CSV file '{csv_file}' not found.")
        return

    # Check if the file is empty
    if os.path.getsize(csv_file) == 0:
        print(f"Error: CSV file '{csv_file}' is empty.")
        return

    data = pd.read_csv(csv_file)
    timestamp = data["timestamp"]
    cross_track_error = data["cross_track_error"]
    error_theta = data["error_theta"]
    velocity_error = data["velocity_error"]

    ax1.clear()  # Clear the previous plot from ax1
    ax1.plot(timestamp, cross_track_error, label="Cross Track Error", color="blue")
    ax1.set_xlabel("Timestamp")
    ax1.set_ylabel("Cross Track Error")
    ax1.set_title("Live Plot of Cross Track Error")
    ax1.legend(loc="upper right")

    ax2.clear()  # Clear the previous plot from ax2
    ax2.plot(timestamp, error_theta, label="Error Theta", color="orange")
    ax2.set_xlabel("Timestamp")
    ax2.set_ylabel("Error Theta")
    ax2.set_title("Live Plot of Error Theta")
    ax2.legend(loc="upper right")

    ax3.clear()  # Clear the previous plot from ax2
    ax3.plot(timestamp, velocity_error, label="Velocity", color="red")
    ax3.set_xlabel("Timestamp")
    ax3.set_ylabel("Error")
    ax3.set_title("Live Plot of Velocity Error")
    ax3.legend(loc="upper right")


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python plots.py <controller_type>")
        sys.exit(1)

    controller_type = sys.argv[1]
    if controller_type not in ["MPC", "PID"]:
        print("Invalid controller type. Please specify 'MPC' or 'PID'.")
        sys.exit(1)

    csv_file = f"./luxad_docs/{controller_type}_Errors.csv"

    fig, (ax1, ax2, ax3) = plt.subplots(3, 1)
    ani = FuncAnimation(fig, animate, fargs=(csv_file,), interval=1000)
    plt.tight_layout()
    plt.show()
