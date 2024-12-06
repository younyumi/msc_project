#!/usr/bin/python3

import math
import numpy as np
import pandas as pd
import cvxpy
import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Vector3Stamped

# MPC 파라미터
NX = 4  # 상태 변수 (x, y, 속도, yaw)
NU = 2  # 제어 입력 (가속도, 조향각)
T = 5  # 예측 시간 지평 (horizon length)

# 비용 행렬
Q = np.diag([1.0, 1.0, 0.5, 0.5])  # 상태 오차 비용 행렬
R = np.diag([0.01, 0.01])  # 제어 입력 비용 행렬
R_d = np.diag([0.01, 1.0])  # 제어 입력 변화량 비용 행렬

# 차량 파라미터
MAX_SPEED = 20.0 / 3.6  # [m/s] 최대 속도 (20 km/h)
MAX_STEER = np.deg2rad(20.0)  # [rad] 최대 조향각
WB = 1.023  # [m] 축간 거리
DT = 0.2  # [s] 시간 간격

class State:
    """차량 상태"""
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v


class MPCNode:
    def __init__(self):
        rospy.init_node("mpc_node", anonymous=True)

        # ROS 퍼블리셔와 서브스크라이버 설정
        self.control_pub = rospy.Publisher('/mobile_system_control/control_msg', Vector3Stamped, queue_size=1)
        rospy.Subscriber('/mobile_system_control/ego_vehicle', Float32MultiArray, self.vehicle_state_callback)

        # 차량 상태 초기화
        self.state = State()

        # 경로 로드
        self.ref_path = pd.read_csv('/home/yumi/catkin_ws/src/my_msc_package/src/reference_path2.csv', names=['x', 'y'], header=None)
        self.cx = self.ref_path['x'].values
        self.cy = self.ref_path['y'].values
        self.cyaw = self.calculate_yaw(self.cx, self.cy)

    @staticmethod
    def calculate_yaw(cx, cy):
        """경로의 yaw 계산"""
        yaw = np.zeros(len(cx))
        for i in range(len(cx) - 1):
            yaw[i] = math.atan2(cy[i + 1] - cy[i], cx[i + 1] - cx[i])
        yaw[-1] = yaw[-2]
        return yaw

    def vehicle_state_callback(self, data):
        """차량 상태 업데이트"""
        self.state.x = data.data[0]
        self.state.y = data.data[1]
        self.state.yaw = data.data[2]
        self.state.v = data.data[3]

    def publish_control(self, throttle, steer, brake):
        """제어 명령 퍼블리시"""
        control_msg = Vector3Stamped()
        control_msg.header.frame_id = "team6"
        control_msg.vector.x = throttle
        control_msg.vector.y = steer
        control_msg.vector.z = brake
        self.control_pub.publish(control_msg)

    def linearize_dynamics(self, v, yaw, delta):
        """동역학 선형화"""
        A = np.eye(NX)
        A[0, 2] = DT * math.cos(yaw)
        A[0, 3] = -DT * v * math.sin(yaw)
        A[1, 2] = DT * math.sin(yaw)
        A[1, 3] = DT * v * math.cos(yaw)
        A[3, 2] = DT * math.tan(delta) / WB

        B = np.zeros((NX, NU))
        B[2, 0] = DT
        B[3, 1] = DT * v / (WB * math.cos(delta) ** 2)

        C = np.zeros(NX)
        return A, B, C

    def calculate_control(self):
        """MPC 제어 계산"""
        x0 = np.array([self.state.x, self.state.y, self.state.v, self.state.yaw])  # 현재 상태
        target_idx = self.find_nearest_index()
        x_ref = self.generate_reference(target_idx)

        # CVXPY 최적화 변수
        x = cvxpy.Variable((NX, T + 1))
        u = cvxpy.Variable((NU, T))

        # 비용 함수 정의
        cost = 0.0
        constraints = []

        for t in range(T):
            cost += cvxpy.quad_form(x[:, t] - x_ref[:, t], Q)  # 상태 오차 비용
            cost += cvxpy.quad_form(u[:, t], R)  # 제어 입력 비용
            if t > 0:
                cost += cvxpy.quad_form(u[:, t] - u[:, t - 1], R_d)  # 제어 입력 변화량 비용

            # 동역학 제약 조건
            A, B, C = self.linearize_dynamics(x_ref[2, t], x_ref[3, t], u[1, t])
            constraints += [x[:, t + 1] == A @ x[:, t] + B @ u[:, t] + C]

        cost += cvxpy.quad_form(x[:, T] - x_ref[:, T], Q)  # 최종 상태 비용
        constraints += [x[:, 0] == x0]  # 초기 상태
        constraints += [cvxpy.abs(u[1, :]) <= MAX_STEER]  # 조향 제한
        constraints += [u[0, :] <= MAX_SPEED]  # 속도 제한

        # 최적화 문제 해결
        prob = cvxpy.Problem(cvxpy.Minimize(cost), constraints)
        prob.solve()

        if prob.status == cvxpy.OPTIMAL or prob.status == cvxpy.OPTIMAL_INACCURATE:
            throttle, steer = u.value[:, 0]
            brake = 0.0 if throttle >= 0 else -throttle
            throttle = max(0.0, throttle)
            return throttle, steer, brake
        else:
            rospy.logwarn("MPC optimization failed.")
            return 0.0, 0.0, 0.0

    def generate_reference(self, target_idx):
        """예측 지평선에 대한 참조 경로 생성"""
        x_ref = np.zeros((NX, T + 1))
        for t in range(T + 1):
            idx = min(target_idx + t, len(self.cx) - 1)
            x_ref[0, t] = self.cx[idx]
            x_ref[1, t] = self.cy[idx]
            x_ref[2, t] = min(MAX_SPEED, self.state.v)  # 일정 속도 유지
            x_ref[3, t] = self.cyaw[idx]
        return x_ref

    def find_nearest_index(self):
        """현재 위치에서 가장 가까운 경로 인덱스 찾기"""
        distances = np.sqrt((self.cx - self.state.x) ** 2 + (self.cy - self.state.y) ** 2)
        return np.argmin(distances)

    def run(self):
        """메인 루프"""
        rate = rospy.Rate(1 / DT)  # 제어 주기 설정
        while not rospy.is_shutdown():
            throttle, steer, brake = self.calculate_control()
            self.publish_control(throttle, steer, brake)
            rospy.loginfo("Throttle: %.2f, Steer: %.2f, Brake: %.2f", throttle, steer, brake)
            rate.sleep()


if __name__ == "__main__":
    mpc_node = MPCNode()
    mpc_node.run()
