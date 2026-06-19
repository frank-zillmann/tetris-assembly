import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseStamped

from std_msgs.msg import Int32, Int32MultiArray # for custom msg

import cv2
from cv_bridge import CvBridge

import tf2_ros
from tf2_ros import TransformBroadcaster

import math
import numpy as np
from scipy.spatial.transform import Rotation

# currated list of our markers with additional information and parameters
MARKER_CONFIG = {
    0: {
        "name": "tetris_grid",
        "size_m": 0.1,  # 10cm marker
        # [not used here] Offset in marker frame where robot should navigate to.
        # Marker Z points out (towards camera), X right, Y down.
        "target_offset": {"x": 0.0, "y": 0.0, "z": 0.0},
        # [not used here] Rotation to apply to the robot.
        "target_yaw_offset": math.pi 
    },
    1: {
        "name": "Block",
        "size_m": 0.03,  # 3cm
        "target_offset": {"x": 0.0, "y": 0.0, "z": 0.0},
        "target_yaw_offset": math.pi
    },
    2: {
        "name": "Block",
        "size_m": 0.03,  # 3cm
        "target_offset": {"x": 0.0, "y": 0.0, "z": 0.0},
        "target_yaw_offset": math.pi
    },
    3: {
        "name": "Block",
        "size_m": 0.03,  # 3cm
        "target_offset": {"x": 0.0, "y": 0.0, "z": 0.0},
        "target_yaw_offset": math.pi
    },
    4: {
        "name": "Block",
        "size_m": 0.03,  # 3cm
        "target_offset": {"x": 0.0, "y": 0.0, "z": 0.0},
        "target_yaw_offset": math.pi
    },
    5: {
        "name": "Block",
        "size_m": 0.03,  # 3cm
        "target_offset": {"x": 0.0, "y": 0.0, "z": 0.0},
        "target_yaw_offset": math.pi
    },
    6: {
        "name": "Block",
        "size_m": 0.03,  # 3cm
        "target_offset": {"x": 0.0, "y": 0.0, "z": 0.0},
        "target_yaw_offset": math.pi
    },
    7: {
        "name": "Block",
        "size_m": 0.03,  # 3cm
        "target_offset": {"x": 0.0, "y": 0.0, "z": 0.0},
        "target_yaw_offset": math.pi
    },
    8: {
        "name": "Block",
        "size_m": 0.03,  # 3cm
        "target_offset": {"x": 0.0, "y": 0.0, "z": 0.0},
        "target_yaw_offset": math.pi
    },
    9: {
        "name": "storage_area",
        "size_m": 0.1,  # 10cm
        "target_offset": {"x": 0.0, "y": 0.0, "z": 0.0},
        "target_yaw_offset": math.pi
    }
}


class DetectionNode(Node):
    def __init__(self) -> None:
        super().__init__('nth_image_detection')

        # -- declare parameters --
        self.declare_parameter('process_every_n', 6)

        self.declare_parameter('image_topic', '/gripper_camera/image_raw/compressed')
        self.declare_parameter('use_compressed_img', True)
        self.declare_parameter('camera_info_topic', '/gripper_camera/camera_info')
        self.declare_parameter('use_hardcoded_camera_info', True)
        self.declare_parameter('fallback_camera_k', [827.01, 0.0, 306.50, 0.0, 817.84, 221.24, 0.0, 0.0, 1.0]) # camera matrix
        self.declare_parameter('fallback_camera_d', [-1.147382, 21.242253, 0.013422, 0.000006, -147.716365]) # camera distortion (Radial Distortion (3) & Tangential Distortion (2))

        self.declare_parameter('aruco_dict', 'DICT_4X4_1000')

        # -- assign parameter --
        self._process_every_n = max(1, int(self.get_parameter('process_every_n').value))

        self._image_topic = self.get_parameter('image_topic').value
        self._use_compressed_img = self.get_parameter('use_compressed_img').value
        self._camera_info_topic = self.get_parameter('camera_info_topic').value
        self._use_hardcoded_camera_info = self.get_parameter('use_hardcoded_camera_info').value
        k = self.get_parameter('fallback_camera_k').value
        d = self.get_parameter('fallback_camera_d').value

        dict_str = self.get_parameter('aruco_dict').value

        # others
        self._frame_count = 0
        self._look_for_marker_ids: set[int] = set()  # empty -> not looking for any marker
        self._cv_bridge = CvBridge()

        # -- TF2 Setup --
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # -- create subscriptions --
        self._target_sub = self.create_subscription(
            Int32MultiArray,
            '/detection/target_marker_ids',
            self._target_callback,
            10,
        )
        
        # Keep the queue short so we always work on fresh frames.
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        if self._use_compressed_img:
            self._image_sub = self.create_subscription(
            CompressedImage,
            self._image_topic,
            self._image_callback,
            qos,
            )
        else:
            self._image_sub = self.create_subscription(
                Image,
                self._image_topic,
                self._image_callback,
                qos,
            )
        
        # Build the camera matrix: 
        if self._use_hardcoded_camera_info: # directly from hardcoded/computed values
            self._camera_matrix = np.array(k, dtype=np.float64).reshape(3, 3) # 3x3
            self._dist_coeffs   = np.array(d, dtype=np.float64) # 5 values
        else: # or from camera info subscription
            self._camera_matrix = None
            self._dist_coeffs   = None
            self._cam_info_sub = self.create_subscription(
                CameraInfo,
                self._camera_info_topic,
                self._cam_info_callback,
                1,
            )
        self.get_logger().info("Subscriptions & Node initialized...")

        # -- Publisher --
        self._marker_pose_pub = self.create_publisher(PoseStamped, '/detection/marker_pose', 10)
        self._found_marker_pub = self.create_publisher(Int32, '/detection/found_marker_id', 10)

        # -- Aruco setup --
        self._aruco_dict = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dict_str))
        self._aruco_params = cv2.aruco.DetectorParameters_create()

        # depending on OpenCV version use cv2.aruco.ArucoDetector or cv2.aruco.detectMarkers()
        if hasattr(cv2.aruco, 'ArucoDetector'):
            self._detector = cv2.aruco.ArucoDetector(self._aruco_dict, self._aruco_params)
            self._use_new_aruco_api = True
        else:
            self._detector = None
            self._use_new_aruco_api = False


    # Called when topic /detection/target_marker_ids is triggered
    def _target_callback(self, msg: Int32MultiArray):
        self._look_for_marker_ids = set(msg.data)
        self.get_logger().info(f"Now tracking marker IDs: {self._look_for_marker_ids}")


    # Called only when _use_hardcoded_camera_info == False
    def _camera_info_callback(self, msg: CameraInfo):
        if self._camera_matrix is None:
            self._camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self._dist_coeffs   = np.array(msg.d, dtype=np.float64)
            self.get_logger().info("Received camera intrinsics.")


    # Called when Image is received
    def _image_callback(self, msg: Image | CompressedImage) -> None:
        if self._camera_matrix is None:
            self.get_logger().info("No Camera matrix found.")
            return

        # Execute only every n-th frame (implementing a time-base approach might be better)
        self._frame_count = (self._frame_count + 1) % self._process_every_n
        if self._frame_count != 0:
            return

        # Convert image from ros-img to cv-img
        try:
            if self._use_compressed_img:
                cv_img = self._cv_bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
            else:
                cv_img = self._cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().debug(f"Failed to convert image: {e}")
            return

        # convert to grayscale
        gray_cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)

        if self._use_new_aruco_api:
            corners, ids, rejected = self._detector.detectMarkers(gray_cv_img)
        else:
            corners, ids, rejected = cv2.aruco.detectMarkers(
                gray_cv_img, self._aruco_dict, parameters=self._aruco_params
            )
        self.get_logger().debug(f"Detected markers: {ids}")

        # Filter out markers based on the defined _look_for_marker_ids set
        if ids is not None:
            for i, marker_id in enumerate(ids.flatten()):
                # only defined marker should be processed && the once we are looking for at the moment (via target marker sub)
                if marker_id in self._look_for_marker_ids and marker_id in MARKER_CONFIG:
                    self.get_logger().info(f"Found Target Marker ID {marker_id}.")
                    
                    # Marker found, now process marker
                    self._process_single_marker(corners[i], marker_id, msg.header.frame_id, msg.header.stamp)


    def _process_single_marker(self, corners, marker_id, camera_frame, stamp): # corners[i] = [[[x1,y1], [x2,y2], [x3,y3], [x4,y4]]]
        cur_marker_config = MARKER_CONFIG[marker_id]
        marker_size = cur_marker_config["size_m"]

        # Define the marker in its local coordinate system
        marker_3d_corners = np.array([
            [-marker_size/2,  marker_size/2, 0],   # top-left
            [ marker_size/2,  marker_size/2, 0],   # top-right
            [ marker_size/2, -marker_size/2, 0],   # bottom-right
            [-marker_size/2, -marker_size/2, 0]    # bottom-left
        ], dtype=np.float32)
        
        # We have 2D pixel coordinates (corners - from the camera image) and
        # want to calculate 3D position of marker relative to camera.
        # solvePnP solves this by matching known 3D points to their 2D projections.
        # rotation_vec, translation_vec tells us where the camera is relative to the marker in 3D space
        # see: https://www.geeksforgeeks.org/computer-vision/camera-position-in-world-coordinate-from-cv-solvepnp/
        success, rotation_vec, translation_vec = cv2.solvePnP(
            marker_3d_corners,      # what the marker looks like in 3D (defined above)
            corners.reshape(4, 2),  # where those corners appeared in the 2D image
            self._camera_matrix,    # camera intrinsics (focal length, principal point)
            self._dist_coeffs#,     # lens distortion
            #flags=cv2.SOLVEPNP_ITERATIVE
        )
        if not success:
            self.get_logger().warn(f"solvePnP failed.")
            return

        # A message that describes one relationship between two frames
        t = TransformStamped() 
        t.header.stamp = stamp  # use the image's original timestamp
        t.header.frame_id = camera_frame  # called default_cam
        marker_frame_name = f"aruco_marker_{marker_id}"
        t.child_frame_id = marker_frame_name

        # Note: the TF chain will look something like: 
        # map → odom → base_link → ... → wrist → default_cam → aruco_marker_X
        # where wrist → default_cam is provided by the static transform in the launch file

        # add transform
        tvec = translation_vec.flatten()  # shape (3,) now
        t.transform.translation.x = float(tvec[0])
        t.transform.translation.y = float(tvec[1])
        t.transform.translation.z = float(tvec[2])

        # add rotation
        rot = Rotation.from_rotvec(rotation_vec.flatten())
        quat = rot.as_quat() # [x, y, z, w]
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        # Here be careful. We broadcast the transform message, but its only done
        # every x frame right now. So movement while running this node might be prone to errors.
        self._tf_broadcaster.sendTransform(t)
        map_tf = self._get_marker_in_map(marker_frame_name)
        if map_tf is None:
            self.get_logger().info(f"No transform for this marker found.")
            return

        # Pubishing the marker pose so other nodes can act on it
        pos = map_tf.transform.translation
        rotation = map_tf.transform.rotation

        # Create Pose Message of Marker relative to map 
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.position.x = pos.x
        pose_msg.pose.position.y = pos.y
        pose_msg.pose.position.z = marker_id  # z value not required really, so we use it as marker_id field (hacky method)
        pose_msg.pose.orientation = rotation
        self._marker_pose_pub.publish(pose_msg)

        # Success signal — the nav node listens to this (might not be in sync with pose)
        found_msg = Int32()
        found_msg.data = int(marker_id)
        self._found_marker_pub.publish(found_msg)

        self.get_logger().info(
            f"[DEBUG] Marker {marker_id} in map: "
            f"x={pos.x:.3f}  y={pos.y:.3f}  z={pos.z:.3f}"
        )

    # To observe how the TF tree actually looks like if we cant find the map tf
    def _get_marker_in_map(self, marker_frame_name: str):
        try:
            return self._tf_buffer.lookup_transform(
                'map',
                marker_frame_name,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05),
            )
        except tf2_ros.LookupException as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            self.get_logger().warn(f"Known TF frames:\n{self._tf_buffer.all_frames_as_string()}")
        except tf2_ros.ConnectivityException as e:
            self.get_logger().warn(f"TF tree not connected: {e}")
            self.get_logger().warn(f"Known TF frames:\n{self._tf_buffer.all_frames_as_string()}")
        except tf2_ros.ExtrapolationException as e:
            self.get_logger().warn(f"TF extrapolation error: {e}")
        return None


def main(args=None):
    rclpy.init(args=args)
    node = DetectionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()