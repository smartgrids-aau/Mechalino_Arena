import numpy as np  # Import the NumPy library, which provides support for arrays and numerical operations

# Function to generate a rectangular path based on the coordinates of two opposite corners
def generate_rectangle_path(x1, y1, x2, y2, m, n):
    # Calculate the length (x-axis) and width (y-axis) of the rectangle
    length = abs(x2 - x1)  # Absolute difference between x1 and x2 to get the length
    width = abs(y2 - y1)   # Absolute difference between y1 and y2 to get the width
    
    # Divide the length into n segments and the width into m segments
    # This creates evenly spaced points along the x and y axes
    x_points = np.linspace(x1, x2, n + 1)  # Create n+1 evenly spaced points between x1 and x2
    y_points = np.linspace(y1, y2, m + 1)  # Create m+1 evenly spaced points between y1 and y2
    
    path = []  # Initialize an empty list to store the path points
    
    # Generate the points for the path
    for i in range(len(y_points)):  # Loop through each y-coordinate
        if i % 2 == 0:
            # For even rows, add points from left (x1) to right (x2)
            for x in x_points:
                path.append((x, y_points[i]))  # Append (x, y) coordinates to the path
        else:
            # For odd rows, add points from right (x2) to left (x1)
            for x in reversed(x_points):  # Reverse the x_points to go from right to left
                path.append((x, y_points[i]))  # Append (x, y) coordinates to the path
    
    return path  # Return the generated path as a list of (x, y) tuples
