#!/usr/bin/env python3
import rospy
import numpy as np
import pandas as pd
from scipy.optimize import minimize
#from scipy.interpolate import splprep, splev
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Vector3Stamped

class MPCController(object):
    def __init__(self, ref_path, N=10, dt=0.1):
        self.ref = pd.read_csv(ref_path, header=None).values
        
        #self.preprocess_path()
        
        self.N = 10  
        self.dt = 0.1  
        self.wheelbase = 1.023
        self.max_steer_angle = np.deg2rad(20.0)
        self.target_speed = 20.0 / 3.6
        # self.prev_u = np.zeros(2*N)
        
    #경로 보간 필요시
    # def preprocess_path(self):
    #     tck, u = splprep([self.ref[:, 0], self.ref[:, 1]], s=0)
    #     u_new = np.linspace(0, 1, 1000)  
    #     x_new, y_new = splev(u_new, tck)
    #     self.ref = np.vstack((x_new, y_new)).T

    def update(self, x, y, yaw, speed, steering_angle):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.speed = speed
        self.steering_angle = steering_angle
        
    def calculate_curvature(self,idx):
        if idx >= len(self.ref) -2:
            return 0
        p1, p2, p3 = self.ref[idx], self.ref[idx+1], self.ref[idx+2]
        dx1, dy1, = p2[0] - p1[0], p2[1] - p1[1]
        dx2, dy2 = p3[0] - p2[0], p3[1] - p2[1]
        curvature = abs(dx1 * dy2 - dy1 * dx2) / ((dx1**2 + dy1**2)**1.5 + 1e-6)
        return curvature
    
    def control_speed(self,nearest_idx):
        curvature = self.calculate_curvature(nearest_idx)
        self.target_speed = max(10.0 /3.6, 20.0/3.6 - curvature * 10.0)

    def control(self):
        u0 = np.zeros(2 * self.N) 
        bounds = [(-self.max_steer_angle, self.max_steer_angle)] * self.N + [(0, 1)] * self.N

        res = minimize(
            self.cost_function, 
            u0, 
            bounds=bounds, 
            args=(self.x, self.y, self.yaw, self.speed), 
            method='SLSQP'   #sequential Least Swuares Programming 최적화
        )
        
        # #최적화 실패시 이전 steer & throttle
        # if res.success:
        #     self.prev_u = res.x  # 최적화 결과 저장
        #     steer = res.x[0]    # 첫 번째 조향각
        #     throttle = res.x[self.N]  # 첫 번째 가속도
        # else:
        #     rospy.logwarn("Optimization failed. Using previous control inputs.")
        #     steer = self.prev_u[0]  # 이전 조향각 사용
        #     throttle = self.prev_u[self.N]  # 이전 가속도 사용
        
        if res.success:
            steer = res.x[0]
            throttle = res.x[self.N]
        else:
            steer = 0.0
            throttle = 0.0

        brake = 0
        if throttle < 0:
            brake = -throttle
            throttle = 0

        return throttle, steer, brake

    def cost_function(self, u, x, y, yaw, speed):
        steer_angles = u[:self.N]
        throttles = u[self.N:]
        cost = 0
        x_pred, y_pred, yaw_pred, speed_pred = x, y, yaw, speed

        for i in range(self.N):
            #예측모델
            x_pred += speed_pred * np.cos(yaw_pred) * self.dt
            y_pred += speed_pred * np.sin(yaw_pred) * self.dt
            yaw_pred += speed_pred / self.wheelbase * steer_angles[i] * self.dt
            speed_pred += throttles[i] * self.dt

            # Find the nearest reference point
            nearest_idx = np.argmin(np.sqrt((self.ref[:, 0] - x_pred)**2 + (self.ref[:, 1] - y_pred)**2))
            ref_x, ref_y = self.ref[nearest_idx, 0], self.ref[nearest_idx, 1]

            cost += (x_pred - ref_x)**2 + (y_pred - ref_y)**2

            if i > 0:
                cost += 0.05 * (steer_angles[i] - steer_angles[i-1])**2

        return cost


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

    rospy.loginfo("Publishing Vector3Stamped data: throttle=%s, steer=%s, brake=%s",
                  control_msg.vector.x, control_msg.vector.y, control_msg.vector.z)


def main():
    global controller, control_publisher

    rospy.init_node('carla_vehicle_control')
    rospy.loginfo("Starting MPC Controller Node")

    ref_path = '/home/yumi/catkin_ws/src/my_msc_package/src/reference_path.csv'
    controller = MPCController(ref_path)

    rospy.Subscriber('/mobile_system_control/ego_vehicle', Float32MultiArray, vehicle_data_callback)
    control_publisher = rospy.Publisher('/mobile_system_control/control_msg', Vector3Stamped, queue_size=10)

    rospy.spin()


if __name__ == '__main__':
    main()

#Kinematic Bicycle Model 비선형 방정식 구현
