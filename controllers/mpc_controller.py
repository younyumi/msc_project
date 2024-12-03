#!/usr/bin/env python3
import rospy
import math
import pandas as pd
import numpy as np
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Vector3Stamped
from scipy.optimize import minimize

class MPCNode:
    def __init__(self):
        self.cur_x = 0
        self.cur_y = 0
        self.cur_theta = 0
        self.cur_vel = 0
        self.cur_steer = 0
        self.Path = []

        rospy.Subscriber('/mobile_system_control/ego_vehicle', Float32MultiArray, self.callback, queue_size=10)

        self.pub_msg = rospy.Publisher('/mobile_system_control/control_msg', Vector3Stamped, queue_size=10)

        self.x_ref = []
        self.y_ref = []
        self.theta_ref = 0
        csv_file_Path = 'reference_path.csv'  # 경로 파일 경로
        df = pd.read_csv(csv_file_Path)
        matrix = df.to_numpy()

        for i in range(len(matrix)):
            x_Point = matrix[i][0]
            y_Point = matrix[i][1]
            self.x_ref.append(x_Point)
            self.y_ref.append(y_Point)

    def callback(self, msg):
        self.cur_x = msg.data[0]
        self.cur_y = msg.data[1]
        self.cur_theta = msg.data[2]
        self.cur_vel = msg.data[3]
        self.cur_steer = msg.data[4]
        self.cur_pos = [self.cur_x, self.cur_y, self.cur_theta, self.cur_vel, self.cur_steer]

        self.find_nearest_point()

        throttle, steer, brake = self.mpc_control()

        control_msg = Vector3Stamped()
        control_msg.header.frame_id = "team6"
        control_msg.vector.x = throttle
        control_msg.vector.y = steer
        control_msg.vector.z = brake

        self.pub_msg.publish(control_msg)
        rospy.loginfo("Publishing control: throttle=%s, steer=%s, brake=%s", throttle, steer, brake)

    def find_nearest_point(self):
        min_distance = float('inf')
        for i in range(len(self.x_ref)):
            distance = ((self.cur_x - self.x_ref[i]) ** 2 + (self.cur_y - self.y_ref[i]) ** 2) ** 0.5
            if distance < min_distance:
                min_distance = distance
                if i == len(self.x_ref) - 1:
                    i = 0
                self.Index = i
                self.nearest_Path = [self.x_ref[self.Index + 1], self.y_ref[self.Index + 1]]
                self.theta_ref = math.atan2(self.y_ref[self.Index + 1] - self.y_ref[self.Index],
                                            self.x_ref[self.Index + 1] - self.x_ref[self.Index])
                break

    def mpc_control(self):

        def cost_function(u):
            throttle, omega = u  
            x_e = (self.nearest_Path[0] - self.cur_x) * math.cos(self.cur_theta) + (self.nearest_Path[1] - self.cur_y) * math.sin(self.cur_theta)
            y_e = -(self.nearest_Path[0] - self.cur_x) * math.sin(self.cur_theta) + (self.nearest_Path[1] - self.cur_y) * math.cos(self.cur_theta)
            theta_e = self.theta_ref - self.cur_theta

            vel_cost = (throttle - self.cur_vel)**2
            steer_cost = omega**2
            path_tracking_cost = (x_e**2 + y_e**2)

            return vel_cost + steer_cost + path_tracking_cost

        u0 = [1.0, 0.05]

        result = minimize(cost_function, u0, bounds=[(0, 1), (-1, 1)])

        throttle, omega = result.x

        steer_fake = (math.atan2(1.023 * omega, self.cur_vel) * 180 / math.pi)  # 1.023은 휠베이스
        steer = -min(20, max(steer_fake, -20)) / 20  # 조향각 제한

        brake = 0 
        return throttle, steer, brake

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('mpc_control_node', anonymous=False)
    node = MPCNode()
    node.main()
