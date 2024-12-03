#!/usr/bin/env python3
import rospy
import math
import pandas as pd
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Vector3Stamped

class PathTrackingNode:
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
        self.min_distance = float('inf')  
        self.nearest_Path = None
        self.Index = 0

        csv_file_Path = 'reference_path.csv'  
        df = pd.read_csv(csv_file_Path)
        matrix = df.to_numpy()

        # 경로 정보 저장
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

        # 목표 경로에서 가장 가까운 지점 찾기
        self.find_nearest_point()

        throttle, steer, brake = self.kanayama_control()

        control_msg = Vector3Stamped()
        control_msg.header.frame_id = "team6"
        control_msg.vector.x = throttle
        control_msg.vector.y = steer
        control_msg.vector.z = brake

        self.pub_msg.publish(control_msg)
        rospy.loginfo("Publishing control: throttle=%s, steer=%s, brake=%s", throttle, steer, brake)

    def find_nearest_point(self):
        # 가장 가까운 경로 지점 찾기
        self.min_distance = float('inf')
        for i in range(len(self.x_ref)):
            distance = ((self.cur_x - self.x_ref[i]) ** 2 + (self.cur_y - self.y_ref[i]) ** 2) ** 0.5
            if distance < self.min_distance:
                self.min_distance = distance
                if i == len(self.x_ref) - 1:
                    i = 0
                self.Index = i
                self.nearest_Path = [self.x_ref[self.Index + 1], self.y_ref[self.Index + 1]]
                self.theta_ref = math.atan2(self.y_ref[self.Index + 1] - self.y_ref[self.Index],
                                            self.x_ref[self.Index + 1] - self.x_ref[self.Index])
                break

    def kanayama_control(self):
        # Kanayama 제어기 로직
        x_e = (self.nearest_Path[0] - self.cur_x) * math.cos(self.cur_theta) + (self.nearest_Path[1] - self.cur_y) * math.sin(self.cur_theta)
        y_e = -(self.nearest_Path[0] - self.cur_x) * math.sin(self.cur_theta) + (self.nearest_Path[1] - self.cur_y) * math.cos(self.cur_theta)
        theta_e = self.theta_ref - self.cur_theta

        v_ref = 20 * 1000 / 3600  
        w_ref = 0.05 
        kx = 4  
        ky = 0.3
        kw = 0.6

        kana_vel = v_ref * math.cos(theta_e) + kx * x_e
        omega = w_ref + v_ref * (ky * y_e + kw * math.sin(theta_e))

        print("Calculated velocity: " + str(kana_vel))
        print("Calculated omega: " + str(omega))

    
        throttle = 1  
        brake = 0  

        steer_fake = (math.atan2(1.023 * omega, self.cur_vel) * 180 / math.pi)  
        steer = -min(20, max(steer_fake, -20)) / 20  

        return throttle, steer, brake

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('kanayama_control_node', anonymous=False)
    node = PathTrackingNode()
    node.main()
