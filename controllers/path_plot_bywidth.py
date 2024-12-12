import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# CSV 파일 경로
road_file_path = '/home/yumi/catkin_ws/src/my_msc_package/src/ref_short.csv'  # 도로 시각화를 위한 파일
waypoint_file_path = '/home/yumi/catkin_ws/src/my_msc_package/src/opt_short.csv'  # 웨이포인트 시각화를 위한 파일

# 도로 데이터 읽기
road_data = pd.read_csv(road_file_path, header=None)
road_data.columns = ['x', 'y', 'z']
road_x = road_data['x'].values
road_y = road_data['y'].values
road_z = road_data['z'].values

# 웨이포인트 데이터 읽기
waypoint_data = pd.read_csv(waypoint_file_path, header=None)
waypoint_data.columns = ['x', 'y', 'z']
waypoint_x = waypoint_data['x'].values
waypoint_y = waypoint_data['y'].values
waypoint_z = waypoint_data['z'].values

# 도로 폭 설정
road_half_width = np.full_like(road_x, 8.0)  # 기본 도로폭: 8m
road_half_width[318:563] = 3.0  # 인덱스 318부터 562까지 도로폭: 3m

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

# 3D 플롯 생성
fig = plt.figure(figsize=(12, 10))
ax = fig.add_subplot(111, projection='3d')

# 도로 경계 플롯
ax.plot(left_x, left_y, road_z, label='Left Road Boundary', color='red', linestyle='--', linewidth=1)
ax.plot(right_x, right_y, road_z, label='Right Road Boundary', color='green', linestyle='--', linewidth=1)

# 웨이포인트 플롯
ax.scatter(waypoint_x, waypoint_y, waypoint_z, label='Waypoints', color='blue', marker='o', s=3)

# 라벨 및 제목 추가
ax.set_title('optimized short path', fontsize=14)
ax.set_xlabel('X Coordinate', fontsize=12)
ax.set_ylabel('Y Coordinate', fontsize=12)
ax.set_zlabel('Z Coordinate', fontsize=12)
ax.legend()

# 축 비율 동기화 (도로 시각화에 적합)
x_range = max(road_x) - min(road_x)
y_range = max(road_y) - min(road_y)
z_range = max(road_z) - min(road_z)
max_range = max(x_range, y_range, z_range) / 2.0

mid_x = (max(road_x) + min(road_x)) / 2.0
mid_y = (max(road_y) + min(road_y)) / 2.0
mid_z = (max(road_z) + min(road_z)) / 2.0

ax.set_xlim(mid_x - max_range, mid_x + max_range)
ax.set_ylim(mid_y - max_range, mid_y + max_range)
ax.set_zlim(mid_z - max_range, mid_z + max_range)

# 플롯 표시
plt.show()
