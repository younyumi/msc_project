#!/usr/bin/env python3
import rospy
import numpy as np
import pandas as pd
from scipy.optimize import minimize
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Vector3Stamped

class MPCPurePursuitController:
    def __init__(self, ref_path):
        # Load reference path
        self.ref = pd.read_csv(ref_path, header=None).values

        # Vehicle parameters
        self.wheelbase = 1.023
        self.dt = 0.1  # Time step
        self.Np = 10  # Prediction horizon
        self.max_steer_angle = np.deg2rad(20)
        self.min_steer_angle = -np.deg2rad(20)
        self.max_vel = 20.0 / 3.6
        self.min_vel = 0.0

        # MPC cost matrices
        self.Q = np.diag([1, 1, 10])  # State cost
        self.R = np.diag([10, 0.5])   # Input cost

        # Current state
        self.x = 0
        self.y = 0
        self.yaw = 0
        self.speed = 0
        self.steering_angle = 0
        self.target_speed = 20.0 / 3.6  # Target speed in m/s

    def update(self, x, y, yaw, speed, steering_angle):
        """Update the current state of the vehicle."""
        self.x = x
        self.y = y
        self.yaw = yaw
        self.speed = speed
        self.steering_angle = steering_angle

    def compute_pure_pursuit(self, look_ahead_distance=2.0):
        """Pure Pursuit algorithm for initial steering angle calculation."""
        nearest_idx = np.argmin(np.hypot(self.x - self.ref[:, 0], self.y - self.ref[:, 1]))
        look_ahead_idx = (nearest_idx + int(look_ahead_distance / self.dt)) % len(self.ref)

        target_x = self.ref[look_ahead_idx, 0]
        target_y = self.ref[look_ahead_idx, 1]

        alpha = np.arctan2(target_y - self.y, target_x - self.x) - self.yaw
        ld = np.hypot(target_x - self.x, target_y - self.y)
        steering_angle = np.arctan2(2 * self.wheelbase * np.sin(alpha), ld)

        return np.clip(steering_angle, self.min_steer_angle, self.max_steer_angle)

    def linearize_dynamics(self, X, U):
        """Linearize the vehicle dynamics at a given state and input."""
        v, delta = U
        theta = X[2]

        A = np.array([
            [1, 0, -v * np.sin(theta) * self.dt],
            [0, 1,  v * np.cos(theta) * self.dt],
            [0, 0,  1]
        ])

        B = np.array([
            [np.cos(theta) * self.dt, 0],
            [np.sin(theta) * self.dt, 0],
            [np.tan(delta) * self.dt / self.wheelbase, v * self.dt / (self.wheelbase * np.cos(delta)**2)]
        ])

        w = np.array([
            v * np.cos(theta) * self.dt,
            v * np.sin(theta) * self.dt,
            v * np.tan(delta) * self.dt / self.wheelbase
        ])

        return A, B, w

    def compute_trajectory(self, x0, X_ref, U_ref):
        """Compute optimal control inputs using MPC."""
        def cost(U_flat):
            U = U_flat.reshape((2, self.Np))
            X = np.zeros((3, self.Np))
            X[:, 0] = x0

            J = 0
            for k in range(self.Np - 1):
                A, B, w = self.linearize_dynamics(X[:, k], U[:, k])
                X[:, k + 1] = A @ X[:, k] + B @ U[:, k] + w

                dx = X[:, k + 1] - X_ref[:, k + 1]
                du = U[:, k] - U_ref[:, k]
                J += dx.T @ self.Q @ dx + du.T @ self.R @ du

            return J

        U0 = np.zeros((2, self.Np)).flatten()  # Initial guess
        bounds = [(self.min_vel, self.max_vel), (self.min_steer_angle, self.max_steer_angle)] * self.Np

        result = minimize(cost, U0, bounds=bounds)
        U_opt = result.x.reshape((2, self.Np))

        return U_opt[:, 0]  # Return the first control input

    def control(self):
        """Combine Pure Pursuit and MPC for control."""
        # Pure Pursuit for initial steering angle
        pure_pursuit_steer = self.compute_pure_pursuit()

        # MPC for optimal control
        nearest_idx = np.argmin(np.hypot(self.x - self.ref[:, 0], self.y - self.ref[:, 1]))
        
        X_ref = np.zeros((3, self.Np))
        U_ref = np.zeros((2, self.Np))

        for i in range(self.Np):
            idx = (nearest_idx + i) % len(self.ref)
            X_ref[:2, i] = self.ref[idx]
            
            if idx + 1 < len(self.ref):
                dx = self.ref[idx + 1, 0] - self.ref[idx, 0]
                dy = self.ref[idx + 1, 1] - self.ref[idx, 1]
                X_ref[2, i] = np.arctan2(dy, dx)
            else:
                X_ref[2, i] = X_ref[2, i-1]
            U_ref[0, i] = self.target_speed

        x0 = np.array([self.x, self.y, self.yaw])
        vel, mpc_steer = self.compute_trajectory(x0, X_ref, U_ref)

        # Combine Pure Pursuit and MPC results
        final_steer = 0.7 * mpc_steer + 0.3 * pure_pursuit_steer

        vel = np.clip(vel, self.min_vel, self.max_vel)
        final_steer = np.clip(final_steer, self.min_steer_angle, self.max_steer_angle)

        return vel, final_steer, 0  # No brake logic for now


def vehicle_data_callback(data):
    """ROS callback function for vehicle state."""
    global controller, control_publisher

    x, y, yaw, speed, steering_angle = data.data[0], data.data[1], data.data[2], data.data[3], data.data[4]
    controller.update(x, y, yaw, speed, steering_angle)

    throttle, steer, brake = controller.control()

    control_msg = Vector3Stamped()
    control_msg.header.frame_id = "team6"
    control_msg.vector.x = throttle
    control_msg.vector.y = steer
    control_msg.vector.z = brake

    control_publisher.publish(control_msg)

    rospy.loginfo("Publishing Vector3Stamped: throttle=%s, steer=%s, brake=%s",
                  control_msg.vector.x, control_msg.vector.y, control_msg.vector.z)


def main():
    """Main function to initialize the node and run the controller."""
    global controller, control_publisher

    rospy.init_node('mpcpurepursuit_vehicle_control')

    ref_path = '/home/yumi/catkin_ws/src/my_msc_package/src/ref_path2.csv'
    controller = MPCPurePursuitController(ref_path)

    rospy.Subscriber('/mobile_system_control/ego_vehicle', Float32MultiArray, vehicle_data_callback)
    control_publisher = rospy.Publisher('/mobile_system_control/control_msg', Vector3Stamped, queue_size=10)

    rospy.spin()


if __name__ == '__main__':
    main()

