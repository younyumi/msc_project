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

        csv_file_Path = '/home/yumi/catkin_ws/src/my_msc_package/src/reference_path2.csv'  
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
        self.min_distance = float('inf')
        self.Index = 0
        
        for i, (x_ref, y_ref) in enumerate(zip(self.x_ref, self.y_ref)):
            distance = ((self.cur_x - x_ref) ** 2 + (self.cur_y - y_ref) ** 2) ** 0.5
            if distance < self.min_distance:
                self.min_distance = distance
                self.Index = i
        next_index = (self.Index + 1) % len(self.x_ref)
        self.nearest_Path = [self.x_ref[next_index], self.y_ref[next_index]]
        self.theta_ref = math.atan2(
            self.y_ref[next_index] - self.y_ref[self.Index],
            self.x_ref[next_index] - self.x_ref[self.Index]
        )
        
    def adjust_target_speed(self, steer):
        max_speed = 20 /3.6
        min_speed = 10 /3.6
        
        if abs(steer) > 0.8:
            return min_speed
        elif abs(steer) > 0.5:
            return (max_speed + min_speed) / 2
        else:
            return max_speed

        
    def dynamic_speed_control(self, target_speed):
        speed_error = target_speed - self.cur_vel
        if speed_error > 0:
            throttle = min(speed_error / target_speed, 1.0)
            brake = 0
        else:
            throttle = 0
            brake = min(-speed_error / target_speed, 1.0)
        
        return throttle, brake
        

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
        
        if self.cur_vel ==0:
            steer_fake=0
        else:
            steer_fake = (math.atan2(1.023 * omega, self.cur_vel) * 180 / math.pi)  
        steer = -min(20, max(steer_fake, -20)) / 20  

        return throttle, steer, brake

    def main(self):
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('kanayama_control_node', anonymous=False)
    node = PathTrackingNode()
    node.main()