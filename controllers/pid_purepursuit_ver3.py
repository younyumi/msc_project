import rospy
import numpy as np
import pandas as pd
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Vector3Stamped

class Controller(object):
    def __init__(self, ref_path):
        self.ref = pd.read_csv(ref_path, header=None).values
        self.max_steer_angle = np.deg2rad(30.0) 
        self.wheelbase = 1.023
        self.target_speed = 20.0 / 3.6
        self.error_sum = 0
        self.error_prev = 0
        self.dt = 0.1 
        
        self.paras_by_section = {
            "section_1": {"look_ahead_dist": 5, "kp": 2.0, "kd": 0.25, "ki": 0.01},  # 최대 곡률 구간
            "section_2": {"look_ahead_dist": 7, "kp": 1.5, "kd": 0.2, "ki": 0.02},  # 곡선 구간
        }

    def get_paras_for_section(self, idx):
        if 579 <= idx <= 696:  # Section 1: 최대 곡률 구간
            return self.paras_by_section["section_1"]
        elif 318 <= idx < 562:  # Section 2: 곡선 구간
            return self.paras_by_section["section_2"]
        else:  # 나머지 구간
            return {"look_ahead_dist": 10, "kp": 1.5, "kd": 0.3, "ki": 0.03}

    def update(self, x, y, yaw, speed, steering_angle):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.speed = speed
        self.steering_angle = steering_angle

    def control(self):
        steer = self.control_steer()
        throttle = self.control_speed()
        brake = 0

        if throttle < 0:
            brake = -throttle
            throttle = 0

        steer = np.clip(steer, -self.max_steer_angle, self.max_steer_angle)
        throttle = np.clip(throttle, 0, 1)
        brake = np.clip(brake, 0, 1)

        return throttle, steer, brake

    def control_speed(self):
        error = self.speed - self.target_speed
        d_error = (error - self.error_prev) / self.dt
        throttle = np.clip(-self.kp * error - self.kd * d_error - self.ki * self.error_sum, 0, 1)
        self.error_sum += error * self.dt
        self.error_sum = np.clip(self.error_sum, -40, 40)
        self.error_prev = error
        return throttle

    def control_steer(self):
        nearest_idx = np.argmin(np.hypot(self.x - self.ref[:, 0], self.y - self.ref[:, 1]))
        section_params = self.get_paras_for_section(nearest_idx)
        self.look_ahead_dist = section_params["look_ahead_dist"]
        self.kp = section_params["kp"]
        self.kd = section_params["kd"]
        self.ki = section_params["ki"]

        look_ahead_idx = (nearest_idx + self.look_ahead_dist) % len(self.ref)
        look_ahead_point = self.ref[look_ahead_idx, :2] - np.array([self.x, self.y])

        rotation_matrix = np.array([[np.cos(self.yaw), np.sin(self.yaw)], [-np.sin(self.yaw), np.cos(self.yaw)]])
        x, y = np.matmul(rotation_matrix, look_ahead_point)

        steer = -(1 / self.max_steer_angle) * np.arctan2(2 * self.wheelbase * y, x * x + y * y)
        rospy.loginfo("Look ahead index: %s", look_ahead_idx)
        rospy.loginfo("Look ahead point: %s", look_ahead_point)

        return steer

def vehicle_data_callback(data):
    global controller
    x, y, yaw, speed, steering_angle = data.data[0], data.data[1], data.data[2], data.data[3], data.data[4]
    controller.update(x, y, yaw, speed, steering_angle)
    throttle, steer, brake = controller.control()

    control_msg = Vector3Stamped()
    control_msg.header.frame_id = "team6"
    control_msg.vector.x = throttle
    control_msg.vector.y = steer
    control_msg.vector.z = brake

    control_publisher.publish(control_msg)

def main():
    global controller, control_publisher

    rospy.init_node('carla_vehicle_control')
    ref_path = '/home/yumi/catkin_ws/src/my_msc_package/src/densified_shortest_path.csv'
    controller = Controller(ref_path)

    rospy.Subscriber('/mobile_system_control/ego_vehicle', Float32MultiArray, vehicle_data_callback)
    control_publisher = rospy.Publisher('/mobile_system_control/control_msg', Vector3Stamped, queue_size=10)

    rospy.spin()

if __name__ == '__main__':
    main()
