import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# CSV 파일 경로
road_file_path = '/home/yumi/catkin_ws/src/my_msc_package/src/reference_path2.csv'  # 도로 시각화를 위한 파일
waypoint_file_path = '/home/yumi/catkin_ws/src/my_msc_package/src/output.csv'  # 웨이포인트 시각화를 위한 파일

# 도로 데이터 읽기
road_data = pd.read_csv(road_file_path, header=None)
road_data.columns = ['x', 'y']
road_x = road_data['x'].values
road_y = road_data['y'].values

# 웨이포인트 데이터 읽기
waypoint_data = pd.read_csv(waypoint_file_path, header=None)
waypoint_data.columns = ['x', 'y']
waypoint_x = waypoint_data['x'].values
waypoint_y = waypoint_data['y'].values

# 도로 폭 (양쪽으로 4m씩)
road_half_width = 4.0

# 도로의 법선 벡터 계산
dx = np.gradient(road_x)  
dy = np.gradient(road_y)  
norm = np.sqrt(dx**2 + dy**2)
normal_x = -dy / norm  
normal_y = dx / norm   

# 도로 경계 계산
left_x = road_x + normal_x * road_half_width
left_y = road_y + normal_y * road_half_width
right_x = road_x - normal_x * road_half_width
right_y = road_y - normal_y * road_half_width

# 2D 플롯 생성
plt.figure(figsize=(12, 8))

# 도로 중심선 플롯
#plt.plot(road_x, road_y, label='Road Center Line', color='blue', linewidth=2)

# 도로 경계 플롯
plt.plot(left_x, left_y, label='Left Road Boundary', color='red', linestyle='--', linewidth=1)
plt.plot(right_x, right_y, label='Right Road Boundary', color='green', linestyle='--', linewidth=1)

# 웨이포인트 플롯
plt.scatter(waypoint_x, waypoint_y, label='Waypoints', color='blue', marker='o', s=10)

# 라벨 및 제목 추가
plt.title('Densified Waypoints Visualization (2D)', fontsize=14)
plt.xlabel('X Coordinate', fontsize=12)
plt.ylabel('Y Coordinate', fontsize=12)
plt.legend()
plt.axis('equal')

# 플롯 표시
plt.show()
