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
