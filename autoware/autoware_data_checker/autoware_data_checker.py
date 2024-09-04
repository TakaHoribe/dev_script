import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, TwistWithCovarianceStamped
from autoware_auto_vehicle_msgs.msg import VelocityReport, SteeringReport  # Old format
# from autoware_vehicle_msgs.msg import VelocityReport, SteeringReport  # New format
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

class ROSBagAnalyzer(Node):
    def __init__(self, rosbag_path):
        super().__init__('rosbag_analyzer')
        self.rosbag_path = rosbag_path
        self.global_pose_data = []
        self.vehicle_velocity_data = []
        self.vehicle_steering_data = []
        self.vehicle_velocity_converted_data = []
        self.imu_data = []
        self.time_stamps = []
        self.window_size = 10  # 移動平均のウィンドウサイズ
        # self.wheelbase = 1.087  # go-kart
        self.wheelbase = 2.75  # jpntaxi

        self.global_pose_topic = "/sensing/gnss/pose_with_covariance"
        # self.global_pose_topic = "/localization/pose_estimator/pose_with_covariance" # "/sensing/gnss/pose_with_covariance"
        self.vehicle_velocity_topic = "/vehicle/status/velocity_status"
        self.vehicle_steering_topic = "/vehicle/status/steering_status"
        # self.twist_estimator_topic = "/sensing/gnss/ublox/fix_velocity" # "/sensing/vehicle_velocity_converter/twist_with_covariance"
        self.twist_estimator_topic = "/sensing/vehicle_velocity_converter/twist_with_covariance"
        self.imu_topic = "/sensing/imu/imu_data"

        # 必要なトピックのみをフィルタとして定義
        self.required_topics = {
            self.global_pose_topic,
            self.vehicle_velocity_topic,
            self.vehicle_steering_topic,
            self.twist_estimator_topic,
            self.imu_topic
        }

        print("---------------------------------")
        print("Setting:")
        print(f" - wheelbase = {self.wheelbase} [m]")
        print(f" - global_pose_topic (global estimation logic e.g. NDT, GNSS) = {self.global_pose_topic}")
        print(f" - vehicle_velocity_topic (vehicle original status) = {self.vehicle_velocity_topic}")
        print(f" - vehicle_steering_topic (vehicle original steering) = {self.vehicle_steering_topic}")
        print(f" - twist_estimator_topic (modified velocity status) = {self.twist_estimator_topic}")
        print(f" - imu_topic = {self.imu_topic}")
        print("---------------------------------")

    def read_rosbag(self):
        # rosbagからデータを読み込む
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
                continue  # このトピックの処理をスキップ

            # メッセージのデシリアライズ
            msg = rclpy.serialization.deserialize_message(data, message_type)

            # Do not use elif so that duplicated topic name is acceptable.
            if topic == self.global_pose_topic:
                self.global_pose_data.append((msg, timestamp))
            if topic == self.vehicle_velocity_topic:
                self.vehicle_velocity_data.append((msg, timestamp))
            if topic == self.vehicle_steering_topic:
                self.vehicle_steering_data.append((msg, timestamp))
            if topic == self.twist_estimator_topic:
                self.vehicle_velocity_converted_data.append((msg, timestamp))
            if topic == self.imu_topic:
                self.imu_data.append((msg, timestamp))

        print("---------------------------------")
        print("topic data num:")
        print(f" - global_pose = {len(self.global_pose_data)}")
        print(f" - vehicle_velocity = {len(self.vehicle_velocity_data)}")
        print(f" - vehicle_steering = {len(self.vehicle_steering_data)}")
        print(f" - twist_estimator = {len(self.vehicle_velocity_converted_data)}")
        print(f" - imu = {len(self.imu_data)}")
        print("---------------------------------")
        

    def moving_average(self, data, window_size):
        """
        Apply moving average filter
        Note: with mode='same', the filter is applied for both forward and backward, which does not cause a time delay.
        """
        return np.convolve(data, np.ones(window_size)/window_size, mode='same')

    def calculate_global_pose_derivatives(self):
        self.global_pose_vx = []
        self.global_pose_vy = []
        self.global_pose_yawrate = []

        for i in range(1, len(self.global_pose_data)):
            prev_pose, prev_time = self.global_pose_data[i - 1]
            curr_pose, curr_time = self.global_pose_data[i]

            dt = (curr_time - prev_time) * 1e-9  # タイムスタンプの差を秒に変換

            # x, y, yawの速度を計算
            dx = curr_pose.pose.pose.position.x - prev_pose.pose.pose.position.x
            dy = curr_pose.pose.pose.position.y - prev_pose.pose.pose.position.y
            yaw1 = 2 * atan2(prev_pose.pose.pose.orientation.z, prev_pose.pose.pose.orientation.w)
            yaw2 = 2 * atan2(curr_pose.pose.pose.orientation.z, curr_pose.pose.pose.orientation.w)
            
            # yawの連続性を担保（-πからπにマッピング）
            dyaw = yaw2 - yaw1
            if dyaw > np.pi:
                dyaw -= 2 * np.pi
            elif dyaw < -np.pi:
                dyaw += 2 * np.pi

            vx = dx / dt
            vy = dy / dt
            yawrate = dyaw / dt

            self.global_pose_vx.append(vx)
            self.global_pose_vy.append(vy)
            self.global_pose_yawrate.append(yawrate)
            self.time_stamps.append(curr_time)


    def compare_velocity(self):
        # グローバルポーズの速度
        global_pose_v = [hypot(vx, vy) for vx, vy in zip(self.global_pose_vx, self.global_pose_vy)]
        global_pose_timestamps = [ts * 1e-9 for ts in self.time_stamps]  # タイムスタンプを秒に変換

        # smoothed_global_pose_v (移動平均)
        smoothed_global_pose_v = self.moving_average(global_pose_v, self.window_size)

        # 車両速度のデータ
        vehicle_velocity = [data.longitudinal_velocity for data, _ in self.vehicle_velocity_data]
        vehicle_velocity_timestamps = [(ts * 1e-9) for _, ts in self.vehicle_velocity_data]

        # 変換後の車両速度
        vehicle_velocity_converted = [data.twist.twist.linear.x for data, _ in self.vehicle_velocity_converted_data]
        vehicle_velocity_converted_timestamps = [(ts * 1e-9) for _, ts in self.vehicle_velocity_converted_data]

        # --- 線形補間によるリサンプリング ---
        # グローバルポーズと変換後の車両速度を同じ時間軸でリサンプリングする
        f_smoothed_global_pose_v = interpolate.interp1d(global_pose_timestamps, smoothed_global_pose_v, fill_value="extrapolate")
        f_vehicle_velocity_converted = interpolate.interp1d(vehicle_velocity_converted_timestamps, vehicle_velocity_converted, fill_value="extrapolate")
        f_vehicle_velocity = interpolate.interp1d(vehicle_velocity_timestamps, vehicle_velocity, fill_value="extrapolate")

        # 共通の時間軸を決定
        common_timestamps_converted = np.linspace(max(min(global_pose_timestamps), min(vehicle_velocity_converted_timestamps)),
                                                  min(max(global_pose_timestamps), max(vehicle_velocity_converted_timestamps)),
                                                  num=min(len(smoothed_global_pose_v), len(vehicle_velocity_converted)))

        common_timestamps_vehicle = np.linspace(max(min(global_pose_timestamps), min(vehicle_velocity_timestamps)),
                                                min(max(global_pose_timestamps), max(vehicle_velocity_timestamps)),
                                                num=min(len(smoothed_global_pose_v), len(vehicle_velocity)))

        # リサンプリングしたデータを取得
        resampled_global_pose_v_converted = f_smoothed_global_pose_v(common_timestamps_converted)
        resampled_vehicle_velocity_converted = f_vehicle_velocity_converted(common_timestamps_converted)

        resampled_global_pose_v_vehicle = f_smoothed_global_pose_v(common_timestamps_vehicle)
        resampled_vehicle_velocity = f_vehicle_velocity(common_timestamps_vehicle)

        # --- 変換後の車両速度が0.01以上のデータに対して差の総和を計算 ---
        valid_velocity_threshold = 0.01
        valid_indices_converted = resampled_vehicle_velocity_converted >= valid_velocity_threshold
        valid_indices_vehicle = resampled_vehicle_velocity >= valid_velocity_threshold

        avg_diff_converted = np.mean(np.abs(resampled_global_pose_v_converted[valid_indices_converted] - resampled_vehicle_velocity_converted[valid_indices_converted]))
        avg_diff_vehicle = np.mean(np.abs(resampled_global_pose_v_vehicle[valid_indices_vehicle] - resampled_vehicle_velocity[valid_indices_vehicle]))

        # --- smoothed_global_pose_v に最も近づくための倍率を計算（最小二乗法） ---
        scale_factor_converted = np.sum(resampled_global_pose_v_converted[valid_indices_converted] * resampled_vehicle_velocity_converted[valid_indices_converted]) / \
                                 np.sum(resampled_vehicle_velocity_converted[valid_indices_converted] ** 2)

        scale_factor_vehicle = np.sum(resampled_global_pose_v_vehicle[valid_indices_vehicle] * resampled_vehicle_velocity[valid_indices_vehicle]) / \
                               np.sum(resampled_vehicle_velocity[valid_indices_vehicle] ** 2)


        print("---------------------------------")
        velocity_error_threshold = 0.3
        print("Velocity Analysis:")
        print(f"- Average difference between global_pose_velocity and twist_estimator_velocity: {avg_diff_converted:.5}.")
        if avg_diff_converted > velocity_error_threshold:
            print("!!! CAUTION !!! difference is toot large, need calibration.")
        print(f"    - Note: This is performed for data with twist_estimator_velocity >= {valid_velocity_threshold:.3} to ignore noise effect when stationary.")
        print(f"    - Note: If this value exceeds {velocity_error_threshold:.3}, vehicle velocity is not accurate -> do velocity calibration (apply scale factor, etc.), or global pose is broken -> change data.")
        print(f"    - Optimal scale factor for vehicle_velocity_converted: {scale_factor_converted:.5}")
        print(f"- Average difference between global_pose_velocity and vehicle_velocity: {avg_diff_vehicle:.5}.")
        if avg_diff_vehicle > velocity_error_threshold:
            print("!!! CAUTION !!! difference is toot large, need calibration.")
        print(f"    - Note: This is performed for data with twist_estimator_velocity >= {valid_velocity_threshold:.3} to ignore noise effect when stationary.")
        print(f"    - Note: If this value exceeds {velocity_error_threshold:.3}, vehicle velocity is not accurate -> do velocity calibration (apply scale factor, etc.), or global pose is broken -> change data.")
        print(f"    - Optimal scale factor for vehicle_velocity: {scale_factor_vehicle:.5}")
        print("---------------------------------")


        # グローバルポーズの速度プロット
        plt.plot(global_pose_timestamps, smoothed_global_pose_v, label=f'Global Pose Derivative Velocity (moving average = {self.window_size})')
        plt.plot(vehicle_velocity_timestamps, vehicle_velocity, label='Vehicle Velocity')
        plt.plot(vehicle_velocity_converted_timestamps, vehicle_velocity_converted, '--', label='Converted Vehicle Velocity')
        plt.plot(vehicle_velocity_converted_timestamps, np.array(vehicle_velocity_converted) * scale_factor_converted, ':', label=f'Converted Vehicle Velocity (x{scale_factor_converted:.3} scaled))')
        plt.plot(vehicle_velocity_timestamps, np.array(vehicle_velocity) * scale_factor_vehicle, ':', label=f'Vehicle Velocity (x{scale_factor_vehicle:.3} scaled))')

        # グラフ設定
        plt.xlabel('Time [s]')
        plt.ylabel('Velocity [m/s]')
        plt.grid()
        plt.legend()
        plt.show()


    def compare_yawrate(self):
        # グローバルポーズのyawrate
        global_pose_yawrate = self.moving_average(self.global_pose_yawrate, self.window_size)
        global_pose_timestamps = [ts * 1e-9 for ts in self.time_stamps]  # タイムスタンプを秒に変換

        # IMUデータのyawrate (z軸回転速度)
        imu_yawrate = [data.angular_velocity.z for data, _ in self.imu_data]
        imu_timestamps = [(ts * 1e-9) for _, ts in self.imu_data]

        # 車両速度とステアリングのデータ
        vehicle_velocity = [data.longitudinal_velocity for data, _ in self.vehicle_velocity_data]
        vehicle_steering = [data.steering_tire_angle for data, _ in self.vehicle_steering_data]
        vehicle_velocity_timestamps = [(ts * 1e-9) for _, ts in self.vehicle_velocity_data]
        vehicle_steering_timestamps = [(ts * 1e-9) for _, ts in self.vehicle_steering_data]

        # --- 線形補間によるリサンプリング ---
        # 車両速度とステアリング角度を同じ時間軸でリサンプリング
        f_vehicle_velocity = interpolate.interp1d(vehicle_velocity_timestamps, vehicle_velocity, fill_value="extrapolate")
        f_vehicle_steering = interpolate.interp1d(vehicle_steering_timestamps, vehicle_steering, fill_value="extrapolate")

        # 共通の時間軸を決定（車両速度とステアリング）
        common_vehicle_timestamps = np.linspace(max(min(vehicle_velocity_timestamps), min(vehicle_steering_timestamps)),
                                                min(max(vehicle_velocity_timestamps), max(vehicle_steering_timestamps)),
                                                num=min(len(vehicle_velocity), len(vehicle_steering)))

        # リサンプリングした車両速度とステアリング角度を取得
        resampled_vehicle_velocity = f_vehicle_velocity(common_vehicle_timestamps)
        resampled_vehicle_steering = f_vehicle_steering(common_vehicle_timestamps)

        # 車両キネマティクスによるyaw rate = vehicle_velocity * tan(vehicle_steering) / wheelbase
        kinematic_yawrate = resampled_vehicle_velocity * np.tan(resampled_vehicle_steering) / self.wheelbase

        # --- 線形補間によるグローバルポーズとIMUのyawrateのリサンプリング ---
        f_global_pose_yawrate = interpolate.interp1d(global_pose_timestamps, global_pose_yawrate, fill_value="extrapolate")
        f_imu_yawrate = interpolate.interp1d(imu_timestamps, imu_yawrate, fill_value="extrapolate")

        # 共通の時間軸を決定（グローバルポーズ、IMU、車両キネマティクス）
        common_timestamps = np.linspace(max(min(global_pose_timestamps), min(imu_timestamps), min(common_vehicle_timestamps)),
                                        min(max(global_pose_timestamps), max(imu_timestamps), max(common_vehicle_timestamps)),
                                        num=min(len(global_pose_yawrate), len(imu_yawrate), len(kinematic_yawrate)))

        # リサンプリングしたデータを取得
        resampled_global_pose_yawrate = f_global_pose_yawrate(common_timestamps)
        resampled_imu_yawrate = f_imu_yawrate(common_timestamps)
        resampled_kinematic_yawrate = interpolate.interp1d(common_vehicle_timestamps, kinematic_yawrate, fill_value="extrapolate")(common_timestamps)

        # --- 差分の平均を計算 ---
        avg_diff_yawrate = np.mean(np.abs(resampled_global_pose_yawrate - resampled_imu_yawrate))
        avg_diff_kinematic = np.mean(np.abs(resampled_global_pose_yawrate - resampled_kinematic_yawrate))
        avg_diff_imu_kinematic = np.mean(np.abs(resampled_imu_yawrate - resampled_kinematic_yawrate))

        # --- Optimal scale factor を計算 ---
        scale_factor_imu = np.sum(resampled_global_pose_yawrate * resampled_imu_yawrate) / np.sum(resampled_imu_yawrate ** 2)
        scale_factor_kinematic = np.sum(resampled_global_pose_yawrate * resampled_kinematic_yawrate) / np.sum(resampled_kinematic_yawrate ** 2)
        scale_factor_imu_kinematic = np.sum(resampled_imu_yawrate * resampled_kinematic_yawrate) / np.sum(resampled_kinematic_yawrate ** 2)


        print("---------------------------------")
        yaw_rate_error_threshold = 0.03
        print("Yaw Rate Analysis:")
        print(f" - Average difference between global_pose_yawrate and imu_yawrate: {avg_diff_yawrate:.5}")
        if avg_diff_yawrate > yaw_rate_error_threshold:
            print("!!! CAUTION !!! difference is toot large, need calibration.")
        print(f"    - Note: If this value exceeds {yaw_rate_error_threshold:.3}, imu_yawrate is not accurate -> do IMU calibration (remove bias, etc.), or global pose is broken -> change data.")
        print(f"    - Optimal scale factor for imu_yawrate: {scale_factor_imu:.5}")
        print(f" - Average difference between global_pose_yawrate and kinematic_yawrate: {avg_diff_kinematic:.5}")
        if avg_diff_kinematic > yaw_rate_error_threshold:
            print("!!! CAUTION !!! difference is toot large, need calibration.")
        print(f"    - Note: If this value exceeds {yaw_rate_error_threshold:.3}, vehicle steering or vehicle velocity is not accurate -> do calibration (remove steering bias, apply steering scale, etc.), or global pose is broken -> change data.")
        print(f"    - Optimal scale factor for imu_yawrate: {scale_factor_kinematic:.5}")
        print(f" - Average difference between imu_yawrate and kinematic_yawrate: {avg_diff_imu_kinematic:.5}")
        if avg_diff_imu_kinematic > yaw_rate_error_threshold:
            print("!!! CAUTION !!! difference is toot large, need calibration.")
        print(f"    - Note: If this value exceeds {yaw_rate_error_threshold:.3}, vehicle steering or vehicle velocity or imu yawrate is not accurate -> do calibration with further investigation.")
        print(f"    - Optimal scale factor between imu_yawrate and kinematic_yawrate: {scale_factor_imu_kinematic:.5}")
        print("---------------------------------")

        # --- プロット ---
        plt.plot(common_timestamps, resampled_global_pose_yawrate, label=f'Global Pose Yawrate (moving average = {self.window_size})')
        plt.plot(common_timestamps, resampled_imu_yawrate, '--', label='IMU Yawrate')
        plt.plot(common_timestamps, resampled_kinematic_yawrate, ':', label=f'Kinematic Yawrate (wheelbase = {self.wheelbase})')
        plt.plot(common_timestamps, np.array(resampled_kinematic_yawrate) * scale_factor_imu_kinematic, ':', label=f'Kinematic Yawrate (wheelbase = {self.wheelbase}, x{scale_factor_imu_kinematic:.3} scaled)')

        # グラフ設定
        plt.xlabel('Time [s]')
        plt.ylabel('Yaw Rate [rad/s]')
        plt.grid()
        plt.legend()
        plt.show()


def main(args=None):
    rclpy.init(args=args)

    # argparseを使用した引数の処理
    parser = argparse.ArgumentParser(description='Analyze rosbag data and compare velocities.')
    parser.add_argument('-r', '--rosbag', type=str, required=True, help='Path to the rosbag file')
    parsed_args = parser.parse_args()

    analyzer = ROSBagAnalyzer(parsed_args.rosbag)
    analyzer.read_rosbag()
    analyzer.calculate_global_pose_derivatives()
    analyzer.compare_velocity()
    analyzer.compare_yawrate()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
