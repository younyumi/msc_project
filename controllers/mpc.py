#!/usr/bin/env python3
import rospy
import numpy as np
import pandas as pd
from scipy.optimize import minimize
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Vector3Stamped


class MPCController:
    def __init__(self, ref_path, N=10, dt=0.1):
        self.ref = pd.read_csv(ref_path, header=None).iloc[:, :2].values.astype(float)

        # Vehicle parameters
        self.N = N
        self.dt = dt
        self.wheelbase = 1.023
        self.max_steer_angle = np.deg2rad(20.0)
        self.min_steer_angle = -np.deg2rad(20.0)
        self.max_speed = 20.0 / 3.6
        self.min_speed = 0.0

        # Cost weights
        self.Q = np.diag([1, 1, 20])  # State cost x, y, yaw
        self.R = np.diag([0.1, 0.2])  # Input cost throttle, steering

        # Vehicle state
        self.x = 0
        self.y = 0
        self.yaw = 0 
        self.speed = 0
        self.steering_angle = 0
        self.nearest_idx = 0  # Initialize nearest_idx

    def update(self, x, y, yaw, speed, steering_angle):
        """Update the current state of the vehicle."""
        self.x = x
        self.y = y
        self.yaw = yaw
        self.speed = speed
        self.steering_angle = steering_angle

        # Update nearest_idx dynamically
        distances = np.hypot(self.ref[:, 0] - self.x, self.ref[:, 1] - self.y)
        new_nearest_idx = np.argmin(distances)

        # Ensure progress along the path
        if new_nearest_idx > self.nearest_idx or \
           distances[new_nearest_idx] > distances[self.nearest_idx]:
            self.nearest_idx = new_nearest_idx
            
        rospy.loginfo(new_nearest_idx)

    def linearize_dynamics(self, X, U):
        """Linearize the vehicle dynamics at a given state and input."""
        v, delta = U
        theta = X[2]

        # State transition matrix A
        A = np.array([
            [1, 0, -v * np.sin(theta) * self.dt],
            [0, 1,  v * np.cos(theta) * self.dt],
            [0, 0,  1]
        ])

        # Input matrix B
        B = np.array([
            [np.cos(theta) * self.dt, 0],
            [np.sin(theta) * self.dt, 0],
            [np.tan(delta) * self.dt / self.wheelbase, v * self.dt / (self.wheelbase * np.cos(delta)**2)]
        ])

        # Nonlinear component
        w = np.array([
            v * np.cos(theta) * self.dt,
            v * np.sin(theta) * self.dt,
            v * np.tan(delta) * self.dt / self.wheelbase
        ])

        return A, B, w

    def control(self):
        """Compute the control inputs using linearized MPC."""
        X_ref = np.zeros((3, self.N))
        U_ref = np.zeros((2, self.N))
        

        # Generate reference trajectory for the prediction horizon
        for i in range(self.N):
            idx = (self.nearest_idx + i) % len(self.ref)
            X_ref[:2, i] = self.ref[idx]

            if idx + 1 < len(self.ref):
                dx = self.ref[idx + 1, 0] - self.ref[idx, 0]
                dy = self.ref[idx + 1, 1] - self.ref[idx, 1]
                X_ref[2, i] = np.arctan2(dy, dx)
            else:
                X_ref[2, i] = X_ref[2, i - 1]

            U_ref[0, i] = self.max_speed  # Constant target speed

        # Initial state
        x0 = np.array([self.x, self.y, self.yaw])

        def cost(U_flat):
            """Cost function for the linear MPC."""
            U = U_flat.reshape((2, self.N))
            X = np.zeros((3, self.N))
            X[:, 0] = x0

            J = 0
            for k in range(self.N - 1):
                A, B, w = self.linearize_dynamics(X[:, k], U[:, k])
                X[:, k + 1] = A @ X[:, k] + B @ U[:, k] + w

                # State and input cost
                dx = X[:, k + 1] - X_ref[:, k + 1]
                du = U[:, k] - U_ref[:, k]
                J += dx.T @ self.Q @ dx + du.T @ self.R @ du

            return J

        # Solve optimization problem
        U0 = np.zeros((2, self.N)).flatten()
        bounds = [(self.min_speed, self.max_speed), (self.min_steer_angle, self.max_steer_angle)] * self.N
        result = minimize(cost, U0, bounds=bounds)

        if result.success:
            U_opt = result.x.reshape((2, self.N))
            throttle, steer = U_opt[:, 0]
        else:
            rospy.logwarn("Optimization failed. Using default control inputs.")
            throttle, steer = 0.0, 0.0

        return throttle, steer, 0  # No braking logic


def vehicle_data_callback(data):
    """ROS callback function for vehicle state."""
    global controller, control_publisher

    x, y, yaw, speed, steering_angle = data.data[0], data.data[1], data.data[2], data.data[3], data.data[4]
    controller.update(x, y, yaw, speed, steering_angle)

    throttle, steer, brake = controller.control()
    
    throttle = np.clip(throttle, 0.0, 1.0)
    steer = np.clip(steer, -1.0, 1.0)
    brake = np.clip(brake, 0.0, 1.0)

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

    rospy.init_node('mpc_vehicle_control')

    ref_path = '/home/yumi/catkin_ws/src/my_msc_package/src/reference_path2.csv'
    controller = MPCController(ref_path)

    rospy.Subscriber('/mobile_system_control/ego_vehicle', Float32MultiArray, vehicle_data_callback)
    control_publisher = rospy.Publisher('/mobile_system_control/control_msg', Vector3Stamped, queue_size=10)

    rospy.spin()


if __name__ == '__main__':
    main()