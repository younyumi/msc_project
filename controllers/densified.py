import pandas as pd
import numpy as np
from scipy.interpolate import CubicSpline

# CSV 파일 경로 설정
input_file_path = '/home/yumi/catkin_ws/src/my_msc_package/src/shortest_path3.csv' # 입력 파일 경로
output_file_path = 'densified_shortest_path.csv'  # 출력 파일 경로

# CSV 파일 읽기
waypoints = pd.read_csv(input_file_path, header=None)
waypoints.columns = ['x', 'y', 'z']

# 기존 좌표 추출
x = waypoints['x'].values
y = waypoints['y'].values
z = waypoints['z'].values

# 기존 웨이포인트를 따라 동일한 간격으로 분포된 거리 생성
original_distances = np.linspace(0, 1, len(x))

# 새로운 웨이포인트 수 설정 (10배 증가)
num_densified_points = len(x) * 7
densified_distances = np.linspace(0, 1, num_densified_points)

# 스플라인 보간법 적용
cs_x = CubicSpline(original_distances, x)
cs_y = CubicSpline(original_distances, y)
cs_z = CubicSpline(original_distances, z)

# 보간된 웨이포인트 계산
densified_x = cs_x(densified_distances)
densified_y = cs_y(densified_distances)
densified_z = cs_z(densified_distances)

# 결과를 DataFrame으로 저장
densified_waypoints = pd.DataFrame({
    'x': densified_x,
    'y': densified_y,
    'z': densified_z
})

# 새로운 CSV 파일로 저장
densified_waypoints.to_csv(output_file_path, index=False, header=False)

print(f"Densified waypoints saved to {output_file_path}")
