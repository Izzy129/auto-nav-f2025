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
import argparse

def calibrate_camera_charuco(input_video_path, squares_x, squares_y, square_length, marker_length, 
                           dictionary_id=aruco.DICT_4X4_50, calibration_flags=0, aspect_ratio=1.0):
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
    
    if not input_video.isOpened():
        print(f"Error: Could not open video source {input_video_path}")
        return None, None, None, None
    
    print(f"Video source opened successfully")
    
    # Storage for calibration data
    all_charuco_corners = []
    all_charuco_ids = []
    all_image_points = []
    all_object_points = []
    all_images = []
    image_size = None
    
    print("Auto mode: capturing every frame with a detected board. Press 'q' to stop (webcam) or let the video finish.")
    
    frame_count = 0
    while True:
        ret, image = input_video.read()
        if not ret:
            print(f"Failed to read frame or end of video reached (frame {frame_count})")
            break
        
        frame_count += 1
        if frame_count % 30 == 0:  # Print every 30 frames
            print(f"Processing frame {frame_count}...")
            
        image_copy = image.copy()
        
        # Detect ChArUco board
        charuco_corners, charuco_ids, marker_corners, marker_ids = detector.detectBoard(image)
        
        # Draw detected corners and markers for visualization
        if marker_ids is not None:
            aruco.drawDetectedMarkers(image_copy, marker_corners, marker_ids)
        
        if charuco_corners is not None and len(charuco_corners) > 3:
            aruco.drawDetectedCornersCharuco(image_copy, charuco_corners, charuco_ids)
            # Add status text when board is detected
            cv2.putText(image_copy, f"Board detected (Auto capture)  Captured: {len(all_images)}", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        else:
            cv2.putText(image_copy, f"No board detected  Captured: {len(all_images)}", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
        # Display image
        cv2.imshow('ChArUco Detection', image_copy)
        key = cv2.waitKey(1) & 0xFF  # keep UI responsive

        # Auto-capture whenever a valid board is detected
        if charuco_corners is not None and len(charuco_corners) > 3 and charuco_ids is not None:
            try:
                chessboard_corners = board.getChessboardCorners()
                object_points = np.array([chessboard_corners[int(id)] for id in charuco_ids.flatten()], dtype=np.float32)
                image_points = charuco_corners.reshape(-1, 2).astype(np.float32)

                if len(image_points) == 0 or len(object_points) == 0:
                    # Skip if matching failed
                    continue

                all_charuco_corners.append(charuco_corners)
                all_charuco_ids.append(charuco_ids)
                all_image_points.append(image_points)
                all_object_points.append(object_points)
                all_images.append(image.copy())

                if image_size is None:
                    image_size = (image.shape[1], image.shape[0])  # (width, height)

            except Exception as e:
                # Skip frame on error
                continue

        # Allow user to stop early (useful for webcam)
        if key == ord('q'):
            break
    
    input_video.release()
    cv2.destroyAllWindows()
    
    if len(all_image_points) == 0 or image_size is None:
        print("No frames captured for calibration or image size not determined!")
        return None, None, None, None
    
    print(f"Calibrating camera with {len(all_image_points)} frames...")
    

    # Always initialize camera matrix as 3x3 identity
    camera_matrix = np.eye(3, dtype=np.float64)
    if calibration_flags & cv2.CALIB_FIX_ASPECT_RATIO:
        camera_matrix[0, 0] = aspect_ratio

    # Initialize distortion coefficients (1D, length 5 for most cameras)
    dist_coeffs = np.zeros(5, dtype=np.float64)

    # Ensure object/image points are lists of arrays with correct shape
    obj_points = [np.array(pts, dtype=np.float32).reshape(-1, 3) for pts in all_object_points]
    img_points = [np.array(pts, dtype=np.float32).reshape(-1, 2) for pts in all_image_points]

    mat:cv2.typing.MatLike = None # type: ignore
    # Calibrate camera using standard calibrateCamera with ChArUco points
    rep_error, camera_matrix, dist_coeffs, _ , _ = cv2.calibrateCamera(
        objectPoints=obj_points, imagePoints=img_points, imageSize=image_size, cameraMatrix=camera_matrix, distCoeffs=dist_coeffs  # type: ignore
    ) # type: ignore
    
    print(f"Calibration completed!")
    print(f"Reprojection error: {rep_error}")
    print(f"Camera matrix:\n{camera_matrix}")
    print(f"Distortion coefficients: {dist_coeffs.ravel()}")
    
    return camera_matrix, dist_coeffs, rep_error, all_images


def main():
    # CLI arguments
    parser = argparse.ArgumentParser(description="ChArUco camera calibration")
    parser.add_argument("--video", type=str, default='recordings/out.mp4', help="Path to input video file. If omitted, uses webcam.")
    parser.add_argument("--camera", type=int, default=0, help="Webcam index to use when --video is not provided.")
    parser.add_argument("--squares-x", type=int, default=11, help="Number of ChArUco squares in X direction.")
    parser.add_argument("--squares-y", type=int, default=8, help="Number of ChArUco squares in Y direction.")
    parser.add_argument("--square-length", type=float, default=0.017, help="Square side length in meters.")
    parser.add_argument("--marker-length", type=float, default=0.012, help="Marker side length in meters.")
    args = parser.parse_args()

    # Configure ChArUco board parameters
    squares_x = args.squares_x
    squares_y = args.squares_y
    square_length = args.square_length
    marker_length = args.marker_length

    # Calibration parameters
    calibration_flags = 0  # Default flags
    # calibration_flags = cv2.CALIB_FIX_ASPECT_RATIO  # Example flag

    # Select source
    input_source = args.video if args.video else args.camera
    print(f"Using input source: {input_source}")

    # Perform calibration
    camera_matrix, dist_coeffs, rep_error, images = calibrate_camera_charuco(
        input_source, squares_x, squares_y, square_length, marker_length, 
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
