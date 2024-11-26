import numpy as np

# Function to generate a rectangular path based on the coordinates of two opposite corners
def generate_rectangle_path(x1, y1, x2, y2, m, n):
    
    length = abs(x2 - x1)
    width = abs(y2 - y1)
    
    # Divide the length into n segments and the width into m segments
    x_points = np.linspace(x1, x2, n + 1)
    y_points = np.linspace(y1, y2, m + 1)
    
    path = []
    
    # Generate the points for the path
    for i in range(len(y_points)):
        if i % 2 == 0:
            # For even rows, add points from left (x1) to right (x2)
            for x in x_points:
                path.append((x, y_points[i]))
        else:
            # For odd rows, add points from right (x2) to left (x1)
            for x in reversed(x_points):
                path.append((x, y_points[i]))
    
    return path
