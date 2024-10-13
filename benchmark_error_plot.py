import pandas as pd
import matplotlib.pyplot as plt

# Read the CSV files
pid_errors_df = pd.read_csv("./luxad_docs/PID_Errors.csv")
mpc_errors_df = pd.read_csv("./luxad_docs/MPC_Errors.csv")

# Extract required columns
pid_data = pid_errors_df[['timestamp', 'cross_track_error', 'error_theta', 'velocity_error']]
mpc_data = mpc_errors_df[['timestamp', 'cross_track_error', 'error_theta', 'velocity_error']]

# Plotting
plt.figure(figsize=(12, 6))

# Cross Track Error
plt.subplot(3, 1, 1)
plt.plot(pid_data['timestamp'], pid_data['cross_track_error'], label='PID')
plt.plot(mpc_data['timestamp'], mpc_data['cross_track_error'], label='MPC')
plt.xlabel('Timestamp')
plt.ylabel('Cross Track Error')
plt.title('Cross Track Error')
plt.legend()

# Error Theta
plt.subplot(3, 1, 2)
plt.plot(pid_data['timestamp'], pid_data['error_theta'], label='PID')
plt.plot(mpc_data['timestamp'], mpc_data['error_theta'], label='MPC')
plt.xlabel('Timestamp')
plt.ylabel('Error Theta')
plt.title('Error Theta')
plt.legend()

# Velocity Error
plt.subplot(3, 1, 3)
plt.plot(pid_data['timestamp'], pid_data['velocity_error'], label='PID')
plt.plot(mpc_data['timestamp'], mpc_data['velocity_error'], label='MPC')
plt.xlabel('Timestamp')
plt.ylabel('Velocity Error')
plt.title('Velocity Error')
plt.legend()

plt.tight_layout()
plt.show()
