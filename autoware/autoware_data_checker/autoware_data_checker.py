import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped
from copy import deepcopy

# Old format
from autoware_auto_vehicle_msgs.msg import VelocityReport, SteeringReport
from autoware_auto_control_msgs.msg import AckermannControlCommand

# New format
# from autoware_vehicle_msgs.msg import VelocityReport, SteeringReport
# from autoware_control_msgs.msg import AckermannControlCommand

from sensor_msgs.msg import Imu
import rosbag2_py
import matplotlib.pyplot as plt
from math import hypot, atan2
import argparse
import rclpy.serialization
from rosidl_runtime_py.utilities import get_message
import logging
import numpy as np
from scipy import interpolate

def moving_average( data, window_size):
    """
    Apply moving average filter
    Note: with mode='same', the filter is applied for both forward and backward, which does not cause a time delay.
    """
    return np.convolve(data, np.ones(window_size)/window_size, mode='same')

def show_result_messages(is_over_threhold):
    if is_over_threhold:
        print('')
        print('Result -> NG. Needs calibration.')
    else:
        print('')
        print('Result -> OK.')

def calculate_scale_factor(reference_data, target_data):
    """
    Calculates the optimal scale factor that minimizes the difference
    between the reference data and the target data using least squares method.

    Args:
        reference_data (numpy.array): The reference data (e.g., global_pose_yawrate).
        target_data (numpy.array): The target data to scale (e.g., imu_yawrate).

    Returns:
        float: The calculated optimal scale factor.
    """
    return np.sum(reference_data * target_data) / np.sum(target_data ** 2)

def resample_data(*datasets: tuple[list[float], list[float]]) -> tuple[list[float], list[list[float]]]:
    """
    Resample multiple datasets with a common timestamp.
    datasets: Each dataset is passed as (timestamps, data) format.
    Returns: Common timestamps and a list of resampled data.
    """
    # Determine the common time range from all datasets
    min_timestamp = max([min(timestamps) for timestamps, _ in datasets])
    max_timestamp = min([max(timestamps) for timestamps, _ in datasets])

    # Decide the common time axis (number of elements based on the smallest dataset)
    common_timestamps = np.linspace(min_timestamp, max_timestamp, num=min([len(data) for _, data in datasets]))

    # Resample each dataset
    resampled_data_list = []
    for timestamps, data in datasets:
        f_data = interpolate.interp1d(timestamps, data, fill_value="extrapolate")
        resampled_data = f_data(common_timestamps)
        resampled_data_list.append(resampled_data)

    return common_timestamps, resampled_data_list

class ROSBagAnalyzer(Node):
    def __init__(self, rosbag_path, non_interactive_mode):
        super().__init__('rosbag_analyzer')
        self.rosbag_path = rosbag_path
        self.non_interactive_mode = non_interactive_mode

        initial_data_structure = {"timestamp": [], "data": []}

        self.global_pose_data = deepcopy(initial_data_structure)
        self.global_pose_vx = deepcopy(initial_data_structure)
        self.global_pose_vy = deepcopy(initial_data_structure)
        self.global_pose_yawrate = deepcopy(initial_data_structure)
        self.vehicle_velocity_data = deepcopy(initial_data_structure)
        self.vehicle_steering_data = deepcopy(initial_data_structure)
        self.twist_estimator_velocity_data = deepcopy(initial_data_structure)
        self.imu_data = deepcopy(initial_data_structure)
        self.control_cmd_data = deepcopy(initial_data_structure)
        self.global_pose_time_stamps = []
        self.window_size = 20  # Moving average window size
        self.wheelbase = 1.087  # go-kart
        # self.wheelbase = 2.75  # jpntaxi

        self.figure_id = 1

        self.error_threshold = {
            "velocity": 0.2,
            "yawrate": 0.03,
            "steering_control": 0.03
        }

        self.global_pose_topic = "/sensing/gnss/pose_with_covariance"
        # self.global_pose_topic = "/localization/pose_estimator/pose_with_covariance" # "/sensing/gnss/pose_with_covariance"
        self.vehicle_velocity_topic = "/vehicle/status/velocity_status"
        self.vehicle_steering_topic = "/vehicle/status/steering_status"
        # self.twist_estimator_topic = "/sensing/gnss/ublox/fix_velocity" # "/sensing/vehicle_velocity_converter/twist_with_covariance"
        self.twist_estimator_topic = "/sensing/vehicle_velocity_converter/twist_with_covariance"
        self.imu_topic = "/sensing/imu/imu_data"
        self.control_cmd = "/control/command/control_cmd"

        # Define only the necessary topics as filters
        self.required_topics = {
            self.global_pose_topic,
            self.vehicle_velocity_topic,
            self.vehicle_steering_topic,
            self.twist_estimator_topic,
            self.imu_topic,
            self.control_cmd
        }

        print('------------------------------------------------------------------')
        print('Setting:')
        print(f" - wheelbase = {self.wheelbase} [m]")
        print(f" - global_pose_topic (global estimation logic e.g. NDT, GNSS) = {self.global_pose_topic}")
        print(f" - vehicle_velocity_topic (vehicle original status)           = {self.vehicle_velocity_topic}")
        print(f" - vehicle_steering_topic (vehicle original steering)         = {self.vehicle_steering_topic}")
        print(f" - twist_estimator_topic (modified velocity status)           = {self.twist_estimator_topic}")
        print(f" - imu_topic                                                  = {self.imu_topic}")
        print(f" - control_cmd_topic                                          = {self.control_cmd}")
        print('------------------------------------------------------------------')

        self.over_threshold_warning_message = "!!! CAUTION !!! difference is too large, need calibration."

    def add_data(self, topic, msg, timestamp):
        # Do not use elif so that duplicated topic name is acceptable.
            if topic == self.global_pose_topic:
                self.global_pose_data["data"].append(msg)
                self.global_pose_data["timestamp"].append(timestamp)
            if topic == self.vehicle_velocity_topic:
                self.vehicle_velocity_data["data"].append(msg)
                self.vehicle_velocity_data["timestamp"].append(timestamp)
            if topic == self.vehicle_steering_topic:
                self.vehicle_steering_data["data"].append(msg)
                self.vehicle_steering_data["timestamp"].append(timestamp)
            if topic == self.twist_estimator_topic:
                self.twist_estimator_velocity_data["data"].append(msg)
                self.twist_estimator_velocity_data["timestamp"].append(timestamp)
            if topic == self.imu_topic:
                self.imu_data["data"].append(msg)
                self.imu_data["timestamp"].append(timestamp)
            if topic == self.control_cmd:
                self.control_cmd_data["data"].append(msg)
                self.control_cmd_data["timestamp"].append(timestamp)

    def read_rosbag(self):
        storage_options = rosbag2_py.StorageOptions(uri=self.rosbag_path, storage_id='sqlite3')
        converter_options = rosbag2_py.ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')
        reader = rosbag2_py.SequentialReader()
        reader.open(storage_options, converter_options)

        topic_types = reader.get_all_topics_and_types()
        type_dict = {topic.name: topic.type for topic in topic_types}

        while reader.has_next():
            (topic, data, timestamp) = reader.read_next()

            if topic not in self.required_topics:
                continue

            try:
                message_type = get_message(type_dict[topic])
            except (ModuleNotFoundError, ImportError) as e:
                logging.warning(f"Skipping topic {topic} due to missing message module: {e}")
                continue

            msg = rclpy.serialization.deserialize_message(data, message_type)
            timestamp_sec = timestamp * 1e-9  # convert from ms to sec
            self.add_data(topic, msg, timestamp_sec)

        print('------------------------------------------------------------------')
        print('topic data num:')
        print(f' - global_pose      = {len(self.global_pose_data["data"])}')
        print(f' - vehicle_velocity = {len(self.vehicle_velocity_data["data"])}')
        print(f' - vehicle_steering = {len(self.vehicle_steering_data["data"])}')
        print(f' - twist_estimator  = {len(self.twist_estimator_velocity_data["data"])}')
        print(f' - imu              = {len(self.imu_data["data"])}')
        print(f' - control_cmd      = {len(self.control_cmd_data["data"])}')
        print('------------------------------------------------------------------')
        
    def show_plot_and_wait_enter(self):
        plt.grid()
        plt.legend()
        plt.pause(0.001)
        if not self.non_interactive_mode:
            input("Please check the result. Press Enter to continue.")
            plt.close()

    def calculate_global_pose_derivatives(self):
        if len(self.global_pose_data["data"]) < 2:
            return

        for i in range(1, len(self.global_pose_data["data"])):
            prev_pose = self.global_pose_data["data"][i - 1]
            curr_pose = self.global_pose_data["data"][i]
            prev_time = self.global_pose_data["timestamp"][i - 1]
            curr_time = self.global_pose_data["timestamp"][i]

            dt = curr_time - prev_time

            # Calculate the velocity in x, y, and yaw directions
            dx = curr_pose.pose.pose.position.x - prev_pose.pose.pose.position.x
            dy = curr_pose.pose.pose.position.y - prev_pose.pose.pose.position.y
            yaw1 = 2 * atan2(prev_pose.pose.pose.orientation.z, prev_pose.pose.pose.orientation.w)
            yaw2 = 2 * atan2(curr_pose.pose.pose.orientation.z, curr_pose.pose.pose.orientation.w)
            
            # Ensure yaw continuity (mapping between -π and π)
            dyaw = yaw2 - yaw1
            dyaw = (dyaw + np.pi) % (2 * np.pi) - np.pi

            vx = dx / dt
            vy = dy / dt
            yawrate = dyaw / dt

            # Append velocity and timestamps to the list
            self.global_pose_vx["data"].append(vx)
            self.global_pose_vx["timestamp"].append(curr_time)
            self.global_pose_vy["data"].append(vy)
            self.global_pose_vy["timestamp"].append(curr_time)
            self.global_pose_yawrate["data"].append(yawrate)
            self.global_pose_yawrate["timestamp"].append(curr_time)



    def compare_velocity(self):
        # Global pose velocity
        global_pose_v = [hypot(vx, vy) for vx, vy in zip(self.global_pose_vx["data"], self.global_pose_vy["data"])]
        global_pose_timestamps = self.global_pose_vy["timestamp"]
        smoothed_global_pose_v = moving_average(global_pose_v, self.window_size)

        # Vehicle velocity data
        vehicle_velocity = [data.longitudinal_velocity for data in self.vehicle_velocity_data["data"]]
        vehicle_velocity_timestamps = self.vehicle_velocity_data["timestamp"]

        # Twist estimator velocity
        twist_estimator_velocity = [data.twist.twist.linear.x for data in self.twist_estimator_velocity_data["data"]]
        twist_estimator_velocity_timestamps = self.twist_estimator_velocity_data["timestamp"]

        # Resample using linear interpolation
        _, [r_global_pose_v, r_twist_estimator_velocity, r_vehicle_velocity] = \
            resample_data((global_pose_timestamps, smoothed_global_pose_v), \
                               (twist_estimator_velocity_timestamps, twist_estimator_velocity), \
                               (vehicle_velocity_timestamps, vehicle_velocity))

        # Ignore stationary data since it is just a noise
        valid_velocity_threshold = 0.01
        valid_indices_estimator = np.abs(r_twist_estimator_velocity) >= valid_velocity_threshold
        valid_indices_raw = np.abs(r_vehicle_velocity) >= valid_velocity_threshold

        avg_diff_converted = np.mean(np.abs(r_global_pose_v[valid_indices_estimator] - r_twist_estimator_velocity[valid_indices_estimator]))
        avg_diff_vehicle = np.mean(np.abs(r_global_pose_v[valid_indices_raw] - r_vehicle_velocity[valid_indices_raw]))

        # Calculate the scale factor using least squares method
        scale_factor_converted = calculate_scale_factor(r_global_pose_v[valid_indices_estimator], r_twist_estimator_velocity[valid_indices_estimator])
        scale_factor_vehicle = calculate_scale_factor(r_global_pose_v[valid_indices_raw], r_vehicle_velocity[valid_indices_raw])

        print('------------------------------------------------------------------')
        threshold = self.error_threshold["velocity"]
        print('Velocity Analysis:')
        print(f"- Average difference between global_pose_velocity and twist_estimator_velocity: {avg_diff_converted:.5}.")
        if avg_diff_converted > threshold:
            print(f"    - {self.over_threshold_warning_message}")
        print(f"    - Note: This is performed for data with twist_estimator_velocity >= {valid_velocity_threshold:.3} to ignore noise effect when stationary.")
        print(f"    - Note: If this value exceeds {threshold:.3}, vehicle velocity is not accurate -> do velocity calibration (apply scale factor, etc.), or global pose is broken -> change data.")
        print(f"    - Optimal scale factor for twist_estimator_velocity: {scale_factor_converted:.5}")
        print(f"- Average difference between global_pose_velocity and vehicle_velocity: {avg_diff_vehicle:.5}.")
        if avg_diff_vehicle > threshold:
            print(f"    - {self.over_threshold_warning_message}")
        print(f"    - Note: This is performed for data with twist_estimator_velocity >= {valid_velocity_threshold:.3} to ignore noise effect when stationary.")
        print(f"    - Note: If this value exceeds {threshold:.3}, vehicle velocity is not accurate -> do velocity calibration (apply scale factor, etc.), or global pose is broken -> change data.")
        print(f"    - Optimal scale factor for vehicle_velocity: {scale_factor_vehicle:.5}")
        show_result_messages(avg_diff_vehicle > threshold or avg_diff_converted > threshold)
        print('------------------------------------------------------------------')

        plt.figure(self.figure_id)
        self.figure_id += 1
        plt.plot(global_pose_timestamps, smoothed_global_pose_v, label=f'Global Pose Derivative Velocity (moving average = {self.window_size})')
        plt.plot(vehicle_velocity_timestamps, vehicle_velocity, label='Vehicle Velocity')
        plt.plot(twist_estimator_velocity_timestamps, twist_estimator_velocity, '--', label='Converted Vehicle Velocity')
        plt.plot(twist_estimator_velocity_timestamps, np.array(twist_estimator_velocity) * scale_factor_converted, '-', label=f'Converted Vehicle Velocity (x{scale_factor_converted:.3} scaled)')
        plt.plot(vehicle_velocity_timestamps, np.array(vehicle_velocity) * scale_factor_vehicle, '--', label=f'Vehicle Velocity (x{scale_factor_vehicle:.3} scaled)')

        plt.xlabel('Time [s]')
        plt.ylabel('Velocity [m/s]')
        self.show_plot_and_wait_enter()

    def compare_yawrate(self):
        # Global pose yawrate
        global_pose_yawrate = moving_average(self.global_pose_yawrate["data"], self.window_size)
        global_pose_timestamps = self.global_pose_yawrate["timestamp"]

        # IMU yawrate (z-axis rotational velocity)
        imu_yawrate = [data.angular_velocity.z for data in self.imu_data["data"]]
        imu_timestamps = self.imu_data["timestamp"]

        # Vehicle velocity and steering data
        vehicle_velocity = [data.longitudinal_velocity for data in self.vehicle_velocity_data["data"]]
        vehicle_steering = [data.steering_tire_angle for data in self.vehicle_steering_data["data"]]
        vehicle_velocity_timestamps = self.vehicle_velocity_data["timestamp"]
        vehicle_steering_timestamps = self.vehicle_steering_data["timestamp"]

        # Resample using linear interpolation
        common_timestamps, [resampled_vehicle_velocity, resampled_vehicle_steering,
                            resampled_global_pose_yawrate, resampled_imu_yawrate] = \
            resample_data((vehicle_velocity_timestamps, vehicle_velocity), (vehicle_steering_timestamps, vehicle_steering),
                          (global_pose_timestamps, global_pose_yawrate), (imu_timestamps, imu_yawrate))

        # Yaw rate from vehicle kinematics = vehicle_velocity * tan(vehicle_steering) / wheelbase
        resampled_kinematic_yawrate = resampled_vehicle_velocity * np.tan(resampled_vehicle_steering) / self.wheelbase

        # Calculate the average difference
        avg_diff_yawrate = np.mean(np.abs(resampled_global_pose_yawrate - resampled_imu_yawrate))
        avg_diff_kinematic = np.mean(np.abs(resampled_global_pose_yawrate - resampled_kinematic_yawrate))
        avg_diff_imu_kinematic = np.mean(np.abs(resampled_imu_yawrate - resampled_kinematic_yawrate))

        # Calculate optimal scale factors
        scale_factor_imu = calculate_scale_factor(resampled_global_pose_yawrate, resampled_imu_yawrate)
        scale_factor_kinematic = calculate_scale_factor(resampled_global_pose_yawrate, resampled_kinematic_yawrate)
        scale_factor_imu_kinematic = calculate_scale_factor(resampled_imu_yawrate, resampled_kinematic_yawrate)

        print('------------------------------------------------------------------')
        threshold = self.error_threshold["yawrate"]
        print('Yaw Rate Analysis:')
        print(f" - Average difference between global_pose_yawrate and imu_yawrate: {avg_diff_yawrate:.5}")
        if avg_diff_yawrate > threshold:
            print(f"    - {self.over_threshold_warning_message}")
        print(f"    - Note: If this value exceeds {threshold:.3}, imu_yawrate is not accurate -> do IMU calibration (remove bias, etc.), or global pose is broken -> change data.")
        print(f"    - Optimal scale factor for imu_yawrate: {scale_factor_imu:.5}")
        print(f" - Average difference between global_pose_yawrate and kinematic_yawrate: {avg_diff_kinematic:.5}")
        if avg_diff_kinematic > threshold:
            print(f"    - {self.over_threshold_warning_message}")
        print(f"    - Note: If this value exceeds {threshold:.3}, vehicle steering or vehicle velocity is not accurate -> do calibration (remove steering bias, apply steering scale, etc.), or global pose is broken -> change data.")
        print(f"    - Optimal scale factor for imu_yawrate: {scale_factor_kinematic:.5}")
        print(f" - Average difference between imu_yawrate and kinematic_yawrate: {avg_diff_imu_kinematic:.5}")
        if avg_diff_imu_kinematic > threshold:
            print(f"    - {self.over_threshold_warning_message}")
        print(f"    - Note: If this value exceeds {threshold:.3}, vehicle steering or vehicle velocity or imu yawrate is not accurate -> do calibration with further investigation.")
        print(f"    - Optimal scale factor between imu_yawrate and kinematic_yawrate: {scale_factor_imu_kinematic:.5}")
        show_result_messages(avg_diff_yawrate > threshold or avg_diff_kinematic > threshold or avg_diff_imu_kinematic > threshold)

        print('------------------------------------------------------------------')

        plt.figure(self.figure_id)
        self.figure_id += 1
        plt.plot(global_pose_timestamps, global_pose_yawrate, label=f'Global Pose Yawrate (moving average = {self.window_size})')
        plt.plot(imu_timestamps, imu_yawrate, '-', label='IMU Yawrate')
        plt.plot(common_timestamps, resampled_kinematic_yawrate, '-', label=f'Kinematic Yawrate (wheelbase = {self.wheelbase})')
        plt.plot(common_timestamps, np.array(resampled_kinematic_yawrate) * scale_factor_imu_kinematic, '--', label=f'Kinematic Yawrate (wheelbase = {self.wheelbase}, x{scale_factor_imu_kinematic:.3} scaled)')

        plt.xlabel('Time [s]')
        plt.ylabel('Yaw Rate [rad/s]')
        self.show_plot_and_wait_enter()

    def compare_steering_angles(self):
        control_cmd_steering = [data.lateral.steering_tire_angle for data in self.control_cmd_data["data"]]
        vehicle_steering = [data.steering_tire_angle for data in self.vehicle_steering_data["data"]]

        control_cmd_timestamps = self.control_cmd_data["timestamp"]
        vehicle_steering_timestamps = self.vehicle_steering_data["timestamp"]

        # Resample using linear interpolation
        _, [resampled_control_cmd_steering, resampled_vehicle_steering] = \
            resample_data((control_cmd_timestamps, control_cmd_steering), (vehicle_steering_timestamps, vehicle_steering))

        avg_diff_steering = np.mean(np.abs(resampled_control_cmd_steering - resampled_vehicle_steering))

        print('------------------------------------------------------------------')
        threshold = self.error_threshold["steering_control"]
        print( "Steering Angle Control Check:")
        print(f" - Average difference between control_cmd_steering and vehicle_steering: {avg_diff_steering:.5}")
        if avg_diff_steering > threshold:
            print(f"    - {self.over_threshold_warning_message}")
        print(f"    - Note: If this value exceeds {threshold:.3}, if a bias or scaling is wrong -> perform calibration, or is the oscillation or tracking performance is poor -> do control parameter tuning (e.g. PID gains).")
        print( "    - Note: If the command and status does not match at all, make sure the data is record in Autonomous mode.")
        show_result_messages(avg_diff_steering > threshold)

        print('------------------------------------------------------------------')

        plt.figure(self.figure_id)
        self.figure_id += 1
        plt.plot(control_cmd_timestamps, control_cmd_steering, label='Control Cmd Steering Angle')
        plt.plot(vehicle_steering_timestamps, vehicle_steering, '--', label='Vehicle Steering Angle')

        plt.xlabel('Time [s]')
        plt.ylabel('Steering Angle [rad]')
        self.show_plot_and_wait_enter()

def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser(description='Analyze rosbag data and compare velocities.')
    parser.add_argument('-r', '--rosbag', type=str, required=True, help='Path to the rosbag file')
    parser.add_argument('-ni', '--non_interactive_mode', action='store_true', default=False, help='Show all plots at once without waiting for input (default: False)')
    parsed_args = parser.parse_args()

    analyzer = ROSBagAnalyzer(parsed_args.rosbag, parsed_args.non_interactive_mode)
    analyzer.read_rosbag()
    analyzer.calculate_global_pose_derivatives()
    analyzer.compare_velocity()
    analyzer.compare_yawrate()
    analyzer.compare_steering_angles()

    input("Press Enter to close.")

    rclpy.shutdown()

if __name__ == '__main__':
    main()
