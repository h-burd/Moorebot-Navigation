import cv2 as cv
import numpy as np
import cv2.aruco as aruco

# Use refrence tutorial to detect markers: https://www.geeksforgeeks.org/detecting-aruco-markers-with-opencv-and-python-1/
# https://youtu.be/GEWoGDdjlSc?feature=shared
# detect_markers finds aruco markers in the image and uses the camera calibration parameters and builtin functions to find the camera position
# returns the marker ID and the camera position in 3D space
def detect_markers(image):
    # get the dictionary of markers
    dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_6X6_250)
    parameters =  cv.aruco.DetectorParameters()
    detector = cv.aruco.ArucoDetector(dictionary, parameters)

    markerCorners, markerIds, rejectedCandidates = detector.detectMarkers(image)

    # camera calibration parameters from previous labs 
    camera_matrix = np.array([
        [675.9452775628, 0, 529.8337503423],
        [0, 682.3705999404, 307.1502458470],
        [0, 0, 1]
    ])
    dist_coeffs = np.array([-0.4149188226, 0.1203670324, 0.0, 0.0, 0.0])

    marker_rotation = None
    marker_position = None

    if markerIds is not None:
        for i in range(len(markerIds)):
            cv.aruco.drawDetectedMarkers(image, markerCorners, markerIds)
            rvec, tvec, _ = cv.aruco.estimatePoseSingleMarkers(markerCorners[i], 0.05, camera_matrix, dist_coeffs)
            rvec = rvec[0]
            tvec = tvec[0] # this is a matrix x,y,z of the distance of the marker from the camera, z is positive facing outward

            # Convert rotation vector to rotation matrix
            R, _ = cv.Rodrigues(rvec) 


            marker_position = tvec.reshape(3,)

            marker_rotation = R


    else:
        print("No markers detected.")
        return None, None, None
    
    return markerIds, marker_position, marker_rotation