import cv2
import cv2.aruco as aruco

def main():
    # Open the webcam
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Cannot open camera")
        return

    # Load a predefined dictionary (e.g., 4x4_50)
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    parameters = aruco.DetectorParameters()
    detector = aruco.ArucoDetector(aruco_dict, parameters)

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            break

        # Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect markers
        corners, ids, rejectedImgPoints = detector.detectMarkers(gray)

        # Draw boxes and IDs
        if ids is not None:
            aruco.drawDetectedMarkers(frame, corners, ids)

        # Show the result
        cv2.imshow('ArUco Marker Detection', frame)

        # Break with 'q'
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
