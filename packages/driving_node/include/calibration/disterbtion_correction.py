import numpy as np
import cv2
import yaml

# This file is needed for different distortion correction methods
# It is planned to be used as a library

#correction of brown-conrady (plumb bob) distribution

class Corrector():
    def __init__(self):
        self.yaml_file = "./calibration_val.yaml"
        const = {}
        with open(self.yaml_file, 'r') as stream:
            try:
                const = yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                print(exc)
        stream.close()
        self.mtx, self.dist = const["mtx"], const["dist"]
        if "homography_matrix" in const.keys():
            self.homography_matrix = const["homography_matrix"]
            self.homography_matrix = np.array(self.homography_matrix, dtype=np.float32)
        self.mtx, self.dist = np.array(self.mtx, dtype=np.float32), np.array(self.dist, dtype=np.float32)

        self.chessboard_size = (7, 5)  # 6 rows, 8 columns
        self.square_size = 3.1  # in cm
        self.bot_origin = (16.0, 12.4)  # Bottom-right corner in bot coordinates in cm



    def calibrate_camera_to_plane(self, image_points):
        """
        Calibrates the camera to the road plane using a chessboard pattern.

        :param chessboard_size: Tuple (rows, cols) of the chessboard (e.g., (6, 8)).
        :param square_size: Size of each square in mm.
        :param image_points: Detected corners in the image (from cv2.findChessboardCorners).
        :param bot_origin: Coordinates of the bottom-right corner in bot coordinates (e.g., (160, 124)).
        :return: Homography matrix for mapping camera coordinates to road coordinates.
        """
        rows, cols = self.chessboard_size

        # Generate 3D points in the road coordinate system
        obj_points = []
        for i in range(1, cols + 1):
            for j in range(1, rows + 1):
                obj_points.append([i * self.square_size, -j * self.square_size])  # y is negative to match the bot's coordinate system
        obj_points = np.array(obj_points, dtype=np.float32)
        # Adjust the origin to the bot's coordinate system

        # Adjust the origin to the bot's coordinate system
        obj_points += np.array(self.bot_origin, dtype=np.float32)

        # Compute the homography matrix
        self.homography_matrix, _ = cv2.findHomography(image_points, obj_points)
        with open(self.yaml_file, 'w') as stream:
            yaml.dump({"mtx": self.mtx.tolist(),
                             "dist":self.dist.tolist(),
                             "homography_matrix":self.homography_matrix.tolist()}, stream)
        stream.close()

        return self.homography_matrix

    def map_camera_to_plane(self, points):
        """
        Maps points from camera coordinates to road coordinates using the homography matrix.

        :param points: List of points in camera coordinates (pixel coordinates).
        :param homography_matrix: Homography matrix from calibrate_camera_to_plane.
        :return: Transformed points in road coordinates.
        """
        points = np.array(points).reshape(-1, 1, 2)
        points = (np.array([480, 320]) - np.multiply(points, np.array([1, -1])))[:, :, ::-1]
        points = self.correction(points)
        points = np.array(points, dtype=np.float32).reshape(-1, 1, 2)
        transformed_points = cv2.perspectiveTransform(points, self.homography_matrix)
        return transformed_points.reshape(-1, 2)


    # TODO: Implement th functionality to get photos from ros
    def plane_calibr(self, image):

        # Load an image of the chessboard
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Detect chessboard corners
        ret, corners = cv2.findChessboardCorners(gray, self.chessboard_size, None)
        if not ret:
            raise ValueError("Chessboard corners not found in the image.")

        corners = self.correction(corners)


        # Calibrate the camera to the road plane
        homography_matrix = self.calibrate_camera_to_plane(corners)


        return homography_matrix

    #-----------------------------------------------------------------------------

    def _brown_conrady(self, points, mtx, dist):
        try:
            points = np.array(points, dtype=np.float32).reshape(1, -1, 2)
        except Exception as e:
            raise ValueError("Invalid points format. It should be able to be converted to flat " +
                            f"and should content the even number of values. Error {e}")
        undistorted_points = cv2.undistortPoints(points, mtx, dist, P=mtx)
        return undistorted_points

    def correction(self, point, distortion="plumb_bob"):
        if distortion == "plumb_bob":
            return self._brown_conrady(point, self.mtx, self.dist)
        else:
            raise ValueError("Unknown distortion type")
