import csv
import math

def calculate_distance(x1, y1, x2, y2):
    """Calculate Euclidean distance between two points."""
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

def calculate_total_path_length(csv_file_path):
    """Calculate the total path length from waypoints in a CSV file."""
    total_length = 0.0

    # Read CSV file
    with open(csv_file_path, mode='r') as file:
        reader = csv.reader(file)
        waypoints = []

        # Skip header if present
        next(reader, None)

        for row in reader:
            x, y = map(float, row[:2])  # Assumes x, y are in the first two columns
            waypoints.append((x, y))

    # Calculate total length
    for i in range(len(waypoints) - 1):
        total_length += calculate_distance(*waypoints[i], *waypoints[i + 1])

    return total_length

# Example usage
csv_file_path = '/home/yumi/catkin_ws/src/my_msc_package/src/ref_short.csv'  # Replace with your CSV file path
total_length = calculate_total_path_length(csv_file_path)
print(f"Long Path Length: {total_length:.2f} m")
