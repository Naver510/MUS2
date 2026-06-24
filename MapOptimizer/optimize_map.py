import math
import sys
import os

def calculate_distance(p1, p2):
    return math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)

def calculate_angle_diff(p_prev, p_curr, p_next):
    v1_x = p_curr[0] - p_prev[0]
    v1_y = p_curr[1] - p_prev[1]
    
    v2_x = p_next[0] - p_curr[0]
    v2_y = p_next[1] - p_curr[1]
    
    dot = v1_x * v2_x + v1_y * v2_y
    cross = v1_x * v2_y - v1_y * v2_x
    
    return math.atan2(cross, dot)

def optimize_map(input_file="GRUZIK.txt", output_file="map.txt", max_speed=200, min_speed=100, lookahead=10):
    if not os.path.exists(input_file):
        print(f"Error: {input_file} not found!")
        return

    points = []
    
    # 1. Load data
    with open(input_file, 'r') as f:
        for line in f:
            parts = line.strip().split('\t')
            if len(parts) >= 4:
                try:
                    # Using x_pc and y_pc (sensor array coordinates) as targets
                    x_pc = float(parts[2])
                    y_pc = float(parts[3])
                    points.append([x_pc, y_pc])
                except ValueError:
                    continue

    if len(points) < 3:
        print("Not enough points to optimize.")
        return

    print(f"Loaded {len(points)} raw points.")

    # 2. Downsampling / Filtering (remove consecutive points that are too close)
    filtered_points = [points[0]]
    for i in range(1, len(points)):
        if calculate_distance(filtered_points[-1], points[i]) > 0.01: # 1 cm min distance
            filtered_points.append(points[i])
            
    points = filtered_points
    print(f"Downsampled to {len(points)} points.")

    # 3. Calculate curvature and assign base speed
    speeds = [max_speed] * len(points)
    curvatures = [0.0] * len(points)
    
    for i in range(1, len(points) - 1):
        angle = abs(calculate_angle_diff(points[i-1], points[i], points[i+1]))
        dist = calculate_distance(points[i-1], points[i]) + calculate_distance(points[i], points[i+1])
        
        if dist > 0.0001:
            curvature = angle / dist
            curvatures[i] = curvature
            
            # Map curvature to speed
            # If curvature is 0 (straight), speed = max_speed
            # If curvature is high (e.g. > 15 rad/m), speed = min_speed
            speed_ratio = max(0.0, 1.0 - (curvature / 15.0))
            speeds[i] = min_speed + (max_speed - min_speed) * speed_ratio
            speeds[i] = max(min_speed, min(max_speed, speeds[i]))

    # 4. Braking lookahead (Deceleration profile)
    # Iterate backwards so the robot slows down BEFORE entering the corner
    for i in range(len(points) - 2, -1, -1):
        dist = calculate_distance(points[i], points[i+1])
        # Max deceleration factor (adjust to brake earlier or later)
        max_decel = dist * 200.0  # Allow speed to drop by X units per meter
        if speeds[i] > speeds[i+1] + max_decel:
            speeds[i] = speeds[i+1] + max_decel

    # 5. Export to map.txt
    with open(output_file, 'w') as f:
        for i in range(len(points)):
            f.write(f"{points[i][0]:.3f}\t{points[i][1]:.3f}\t{speeds[i]:.1f}\n")

    print(f"Optimization complete! Saved optimized route to {output_file}.")

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description='Optimize GRUZIK map')
    parser.add_argument('--input', type=str, default='GRUZIK.txt', help='Input raw map file')
    parser.add_argument('--output', type=str, default='map.txt', help='Output optimized map file')
    parser.add_argument('--max_speed', type=float, default=220.0, help='Maximum PWM speed on straights')
    parser.add_argument('--min_speed', type=float, default=110.0, help='Minimum PWM speed in corners')
    
    args = parser.parse_args()
    optimize_map(args.input, args.output, args.max_speed, args.min_speed)
