#!/usr/bin/env python3
import rospy
import numpy as np
import pandas as pd
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Vector3Stamped

class Controller(object):
    def __init__(self, ref_path):
        self.ref = pd.read_csv(ref_path, header=None).values
        self.max_steer_angle = np.deg2rad(20.0)
        self.wheelbase = 1.023
        self.target_speed = 20.0 / 3.6
        self.look_ahead_dist = 4
        self.kp = 1.5
        self.kd = 0.3
        self.ki = 0.03
        self.error_sum = 0
        self.error_prev = 0
        self.dt = 0.1
    
    def wrap_angle(self, angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi
    
    def update(self, x, y, yaw, speed, steering_angle):
        self.x = x
        self.y = y
        self.yaw = self.wrap_angle(yaw)
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
        throttle = -self.kp * error - self.kd * d_error - self.ki * self.error_sum
        self.error_sum += error * self.dt
        self.error_sum = np.clip(self.error_sum, -40, 40)
        self.error_prev = error
        return throttle
    
    def control_steer(self):
        """Pure pursuit control for steering."""
        nearest_idx = np.argmin(np.hypot(self.x - self.ref[:, 0], self.y - self.ref[:, 1]))
        # 곡률에 따라 룩어헤드 동적 조정
        curvature = 1 / (np.linalg.norm(self.ref[nearest_idx, :2] - self.ref[(nearest_idx + 1) % len(self.ref), :2]) + 1e-6)
        self.look_ahead_dist = max(4, min(10, 1 / (curvature + 1e-3)))

        look_ahead_idx = (nearest_idx + int(self.look_ahead_dist)) % len(self.ref)
        look_ahead_point = self.ref[look_ahead_idx, :2] - np.array([self.x, self.y])

        # Transform look-ahead point to the vehicle's coordinate frame
        rotation_matrix = np.array([[np.cos(self.yaw), np.sin(self.yaw)],
                                     [-np.sin(self.yaw), np.cos(self.yaw)]])
        x, y = np.matmul(rotation_matrix, look_ahead_point)

        steer = np.arctan2(2 * self.wheelbase * y, x**2 + y**2)
        steer = np.clip(steer, -self.max_steer_angle, self.max_steer_angle)
        return steer


def vehicle_data_callback(data):
    global controller
    x,y,yaw,speed,steering_angle = data.data[0], data.data[1], data.data[2],data.data[3], data.data[4]
    controller.update(x,y,yaw,speed, steering_angle)
    throttle,steer,brake = controller.control()
            
    control_msg = Vector3Stamped()
    control_msg.header.frame_id = "team6"
    control_msg.vector.x = throttle
    control_msg.vector.y = steer
    control_msg.vector.z = brake
    
    control_publisher.publish(control_msg)
            
    rospy.loginfo("Pubishing Vector3Stamped data:throttle=%s, steer=%s, brake=%s", 
                  control_msg.vector.x, control_msg.vector.y, control_msg.vector.z)
        
def main():
    global controller, control_publisher
        
    rospy.init_node('carla_vehicle_control')
    print("hello")
    
    ref_path = '/home/yumi/catkin_ws/src/my_msc_package/src/reference_path2.csv'
    controller = Controller(ref_path)
        
    rospy.Subscriber('/mobile_system_control/ego_vehicle',Float32MultiArray, vehicle_data_callback)
        
    control_publisher = rospy.Publisher('/mobile_system_control/control_msg', Vector3Stamped, queue_size=10)
            
    rospy.spin()
        
if __name__=='__main__':
    main()