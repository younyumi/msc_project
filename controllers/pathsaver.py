#!/usr/bin/env python3
import rospy
import csv
from std_msgs.msg import Float32MultiArray
import os

class VehicleDataSaver:
    def __init__(self, save_path, save_interval=0.2):
        """
        Args:
            save_path (str): CSV 파일을 저장할 경로
            save_interval (float): 데이터를 저장할 간격 (초)
        """
        self.save_path = save_path
        self.save_interval = save_interval
        self.last_save_time = rospy.Time.now()

        # CSV 파일 준비
        self.prepare_csv()

        # Subscriber 설정
        rospy.Subscriber('/mobile_system_control/ego_vehicle', Float32MultiArray, self.vehicle_data_callback)

    def prepare_csv(self):
        """CSV 파일을 생성하거나 초기화합니다."""
        # 디렉토리가 없으면 생성
        os.makedirs(os.path.dirname(self.save_path), exist_ok=True)

        # CSV 파일이 없으면 초기화
        if not os.path.isfile(self.save_path):
            with open(self.save_path, mode='w', newline='') as file:
                pass  # 빈 파일 생성

    def vehicle_data_callback(self, data):
        """/mobile_system_control/ego_vehicle 토픽의 콜백 함수."""
        x, y, z = data.data[0], data.data[1], data.data[2]  # 차량의 x, y, z 좌표
        current_time = rospy.Time.now()

        # 저장 간격 확인
        if (current_time - self.last_save_time).to_sec() >= self.save_interval:
            self.save_to_csv(x, y, z)
            self.last_save_time = current_time

    def save_to_csv(self, x, y, z):
        """좌표를 CSV 파일에 저장합니다."""
        with open(self.save_path, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([x, y, z])
        rospy.loginfo(f"Saved data: x={x}, y={y}, z={z}")


def main():
    rospy.init_node('vehicle_data_saver')

    # 저장할 CSV 파일 경로 설정
    save_path = '/home/yumi/catkin_ws/src/my_msc_package/src/path3.csv'

    # VehicleDataSaver 초기화 (0.1초 간격으로 저장)
    saver = VehicleDataSaver(save_path, save_interval=0.2)

    rospy.spin()


if __name__ == '__main__':
    main()