"""
Python equivalent of C++ ChArUco camera calibration code.

This is the Python conversion of the C++ OpenCV ChArUco calibration code.
Key differences from C++:
- Uses cv2.aruco module instead of cv::aruco
- Python lists instead of C++ vectors
- numpy arrays for matrix operations
- Different API for some functions (e.g., board.getChessboardCorners() vs board.matchImagePoints())
"""

import cv2
import numpy as np
from cv2 import aruco

def calibrate_camera_charuco(input_video_path, squares_x, squares_y, square_length, marker_length, 
                           dictionary_id=aruco.DICT_6X6_250, calibration_flags=0, aspect_ratio=1.0):
    """
    Calibrate camera using ChArUco board detection
    
    Args:
        input_video_path: Path to input video file or camera index
        squares_x: Number of squares in X direction
        squares_y: Number of squares in Y direction  
        square_length: Length of square side (in meters or preferred unit)
        marker_length: Length of marker side (in same unit as square_length)
        dictionary_id: ArUco dictionary to use
        calibration_flags: Calibration flags for cv2.calibrateCamera
        aspect_ratio: Aspect ratio if CALIB_FIX_ASPECT_RATIO flag is used
    
    Returns:
        camera_matrix: Camera intrinsic matrix
        dist_coeffs: Distortion coefficients
        rep_error: Reprojection error
        all_images: List of captured images used for calibration
    """
    
    # Create ArUco dictionary and ChArUco board
    dictionary = aruco.getPredefinedDictionary(dictionary_id)
    board = aruco.CharucoBoard((squares_x, squares_y), square_length, marker_length, dictionary)
    
    # Create detector with default parameters
    detector_params = aruco.DetectorParameters()
    charuco_params = aruco.CharucoParameters()
    detector = aruco.CharucoDetector(board, charuco_params, detector_params)
    
    # Open video capture
    input_video = cv2.VideoCapture(input_video_path)
    
    # Storage for calibration data
    all_charuco_corners = []
    all_charuco_ids = []
    all_image_points = []
    all_object_points = []
    all_images = []
    image_size = None
    
    print("Press 'c' to capture frame, 'q' to quit and proceed with calibration")
    
    while True:
        ret, image = input_video.read()
        if not ret:
            break
            
        image_copy = image.copy()
        
        # Detect ChArUco board
        charuco_corners, charuco_ids, marker_corners, marker_ids = detector.detectBoard(image)
        
        # Draw detected corners and markers for visualization
        if marker_ids is not None:
            aruco.drawDetectedMarkers(image_copy, marker_corners, marker_ids)
        
        if charuco_corners is not None and len(charuco_corners) > 3:
            aruco.drawDetectedCornersCharuco(image_copy, charuco_corners, charuco_ids)
            
        # Display image
        cv2.imshow('ChArUco Detection', image_copy)
        key = cv2.waitKey(1) & 0xFF
        
        # Capture frame when 'c' is pressed
        if key == ord('c') and charuco_corners is not None and len(charuco_corners) > 3:
            # Get object and image points from ChArUco detection
            try:
                # Get board object points for detected corners
                chessboard_corners = board.getChessboardCorners()
                object_points = np.array([chessboard_corners[int(id)] for id in charuco_ids.flatten()], dtype=np.float32)
                image_points = charuco_corners.reshape(-1, 2).astype(np.float32)
                
                if len(image_points) == 0 or len(object_points) == 0:
                    print("Point matching failed, try again.")
                    continue
                    
                print(f"Frame captured with {len(image_points)} corner points")
                
                all_charuco_corners.append(charuco_corners)
                all_charuco_ids.append(charuco_ids)
                all_image_points.append(image_points)
                all_object_points.append(object_points)
                all_images.append(image.copy())
                
                if image_size is None:
                    image_size = (image.shape[1], image.shape[0])  # (width, height)
            except Exception as e:
                print(f"Error processing frame: {e}")
                continue
                
        elif key == ord('q'):
            break
    
    input_video.release()
    cv2.destroyAllWindows()
    
    if len(all_image_points) == 0 or image_size is None:
        print("No frames captured for calibration or image size not determined!")
        return None, None, None, None
    
    print(f"Calibrating camera with {len(all_image_points)} frames...")
    
    # Initialize camera matrix
    if calibration_flags & cv2.CALIB_FIX_ASPECT_RATIO:
        camera_matrix = np.eye(3, dtype=np.float64)
        camera_matrix[0, 0] = aspect_ratio
    else:
        camera_matrix = np.eye(3, dtype=np.float64)
    
    # Initialize distortion coefficients
    dist_coeffs = np.zeros((4, 1))
    
    # Calibrate camera using standard calibrateCamera with ChArUco points
    rep_error, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        all_object_points, all_image_points, image_size, camera_matrix, dist_coeffs, flags=calibration_flags
    )
    
    print(f"Calibration completed!")
    print(f"Reprojection error: {rep_error}")
    print(f"Camera matrix:\n{camera_matrix}")
    print(f"Distortion coefficients: {dist_coeffs.ravel()}")
    
    return camera_matrix, dist_coeffs, rep_error, all_images


def main():
    # Example usage
    # Configure ChArUco board parameters
    squares_x = 7  # Number of squares in X direction
    squares_y = 5  # Number of squares in Y direction  
    square_length = 0.04  # Length of square side in meters
    marker_length = 0.032  # Length of marker side in meters
    
    # Calibration parameters
    calibration_flags = 0  # Default flags
    # calibration_flags = cv2.CALIB_FIX_ASPECT_RATIO  # Example flag
    
    # Input video (can also use 0 for webcam)
    input_video_path = 0  # Use webcam, or specify path to video file
    
    # Perform calibration
    camera_matrix, dist_coeffs, rep_error, images = calibrate_camera_charuco(
        input_video_path, squares_x, squares_y, square_length, marker_length, 
        calibration_flags=calibration_flags
    )
    
    if camera_matrix is not None and dist_coeffs is not None:
        # Save calibration results
        np.savez('camera_calibration.npz', 
                camera_matrix=camera_matrix, 
                dist_coeffs=dist_coeffs,
                reprojection_error=rep_error)
        print("Calibration saved to camera_calibration.npz")
    else:
        print("Calibration failed!")


if __name__ == "__main__":
    main()
