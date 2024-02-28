import cv2
import numpy as np

def calculate_reprojection_error(found_corners, object_points, rvec, tvec, camera_matrix, dist_coeffs=None):
    # Project object points to image plane
    projected_points, _ = cv2.projectPoints(object_points, rvec, tvec, camera_matrix, dist_coeffs)
    
    # Calculate reprojection error
    reprojection_errors = np.linalg.norm(found_corners - projected_points.squeeze(), axis=1)
    
    # Convert reprojection error to centimeters
    reprojection_error_cm = np.mean(reprojection_errors)
    
    return reprojection_error_cm

def convert_to_image_cord(rvec, tvec, camera_matrix, distortion_coeffs):
    # Convert rotation vector to rotation matrix
    rotation_matrix, _ = cv2.Rodrigues(rvec)

    # Combine rotation and translation into extrinsic matrix
    extrinsic_matrix = np.column_stack((rotation_matrix, tvec))

    # Project marker's center onto image plane
    marker_points = np.array([[0, 0, 0]], dtype=np.float64)
    image_points, _ = cv2.projectPoints(marker_points, rotation_matrix, tvec, camera_matrix, distortion_coeffs)

    # Extract pixel coordinates
    pixel_x = int(image_points[0][0][0])
    pixel_y = int(image_points[0][0][1])

    return pixel_x, pixel_y