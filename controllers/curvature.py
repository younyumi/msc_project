# import pandas as pd
# import numpy as np
# import matplotlib.pyplot as plt

# def calculate_curvature(file_path):
#     # Load waypoints from CSV file
#     waypoints = pd.read_csv(file_path, header=None).values
#     x = waypoints[:, 0]
#     y = waypoints[:, 1]

#     # Calculate first and second derivatives
#     dx = np.gradient(x)  # x'(t)
#     dy = np.gradient(y)  # y'(t)
#     ddx = np.gradient(dx)  # x''(t)
#     ddy = np.gradient(dy)  # y''(t)

#     # Calculate curvature
#     curvature = np.abs(dx * ddy - dy * ddx) / (dx**2 + dy**2)**1.5
#     curvature[np.isnan(curvature)] = 0  # Handle division by zero

#     return curvature

# def filter_outliers(curvature):
#     # Calculate IQR
#     q1 = np.percentile(curvature, 25)
#     q3 = np.percentile(curvature, 75)
#     iqr = q3 - q1

#     # Define outlier bounds
#     lower_bound = q1 - 1.5 * iqr
#     upper_bound = q3 + 1.5 * iqr

#     # Filter outliers
#     filtered_curvature = curvature[(curvature >= lower_bound) & (curvature <= upper_bound)]
#     return filtered_curvature

# # Load curvature data
# file_path = '/home/yumi/catkin_ws/src/my_msc_package/src/final_path.csv'
# curvature = calculate_curvature(file_path)

# # Remove outliers
# filtered_curvature = filter_outliers(curvature)

# # Analyze curvature
# min_curvature = np.min(filtered_curvature)
# max_curvature = np.max(filtered_curvature)
# mean_curvature = np.mean(filtered_curvature)
# std_curvature = np.std(filtered_curvature)

# print(f"Curvature Analysis (Without Outliers):")
# print(f"Min: {min_curvature:.6f}")
# print(f"Max: {max_curvature:.6f}")
# print(f"Mean: {mean_curvature:.6f}")
# print(f"Standard Deviation: {std_curvature:.6f}")

# # Visualize curvature distribution
# plt.hist(filtered_curvature, bins=50, edgecolor='k', alpha=0.7)
# plt.title('Curvature Distribution (Without Outliers)')
# plt.xlabel('Curvature')
# plt.ylabel('Frequency')
# plt.show()


import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

def calculate_curvature(file_path):
    # Load waypoints from CSV file
    waypoints = pd.read_csv(file_path, header=None).values
    x = waypoints[:, 0]
    y = waypoints[:, 1]

    # Calculate first and second derivatives
    dx = np.gradient(x)  # x'(t)
    dy = np.gradient(y)  # y'(t)
    ddx = np.gradient(dx)  # x''(t)
    ddy = np.gradient(dy)  # y''(t)

    # Calculate curvature
    curvature = np.abs(dx * ddy - dy * ddx) / (dx**2 + dy**2)**1.5
    curvature[np.isnan(curvature)] = 0  # Handle division by zero

    return x, y, curvature

def filter_outliers(curvature):
    # Calculate IQR
    q1 = np.percentile(curvature, 25)
    q3 = np.percentile(curvature, 75)
    iqr = q3 - q1

    # Define outlier bounds
    lower_bound = q1 - 1.5 * iqr
    upper_bound = q3 + 1.5 * iqr

    # Filter outliers
    filtered_curvature = np.clip(curvature, lower_bound, upper_bound)
    return filtered_curvature

# Load curvature data
file_path = '/home/yumi/catkin_ws/src/my_msc_package/src/reference_path.csv'
x, y, curvature = calculate_curvature(file_path)

# Filter outliers
filtered_curvature = filter_outliers(curvature)

# Analyze curvature
min_curvature = np.min(filtered_curvature)
max_curvature = np.max(filtered_curvature)
mean_curvature = np.mean(filtered_curvature)
std_curvature = np.std(filtered_curvature)

print(f"Curvature Analysis (With Outliers Filtered):")
print(f"Min: {min_curvature:.6f}")
print(f"Max: {max_curvature:.6f}")
print(f"Mean: {mean_curvature:.6f}")
print(f"Standard Deviation: {std_curvature:.6f}")

# Display curvature values along the path
print("Curvature values at each point:")
for i in range(len(x)):
    print(f"Point {i}: X={x[i]:.2f}, Y={y[i]:.2f}, Curvature={filtered_curvature[i]:.6f}")

# Plot the path
plt.figure(figsize=(10, 6))
plt.plot(x, y, label='Path', color='blue')
sc = plt.scatter(x, y, c=filtered_curvature, cmap='jet', label='Curvature')
plt.colorbar(sc, label='Curvature')
plt.title('Path with Curvature')
plt.xlabel('X')
plt.ylabel('Y')
plt.legend()
plt.grid(True)
plt.show()
