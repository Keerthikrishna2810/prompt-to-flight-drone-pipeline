"""
vision_camera_gz.py

Challenge 3 -- bridges a Gazebo camera stream into the CameraSource
interface vision_executor.py expects (get_frame() -> numpy array).

-------------------------------------------------------------------------
HONESTY NOTE, read before relying on this file
-------------------------------------------------------------------------
This is the ONE part of Challenge 3 that could not be verified live in
the environment this was built in -- no GPU/display/camera stream was
available. Everything else (schema, detector interface, follow-steering
math, search/follow executor logic, safety validation, LLM interpreter)
is unit-tested and passes; this file is the honest exception.

Given today's real experience getting Challenge 1's Gazebo integration
working -- multiple genuine issues (a wrong CLI flag, a paused world
clock, a missing ROS2 environment source) that each needed live
iteration to find -- this file should be treated as a stretch goal, not
something to block a submission on. If it doesn't work on the first try,
that is expected, not a sign anything upstream is broken.

-------------------------------------------------------------------------
Chosen approach, and why
-------------------------------------------------------------------------
Reuses the ROS 2 (Humble) install already added for Challenge 1's RViz
visualization, via the `ros_gz_bridge` / `ros_gz_image` packages, rather
than writing a new binding directly against gz-transport's raw protobuf
image messages. This is deliberately the lower-risk option: it reuses
infrastructure (ROS2, rclpy) that Challenge 1 already got working live,
instead of introducing an entirely new, unverified integration path.

Requires (NOT yet added to the Dockerfile -- add before using):
    apt-get install -y ros-humble-ros-gz-bridge ros-humble-cv-bridge

And a running bridge process alongside the simulation, e.g.:
    ros2 run ros_gz_bridge parameter_bridge \\
        /world/default/model/x500_mono_cam_0/link/camera_link/sensor/imager/image@sensor_msgs/msg/Image[gz.msgs.Image

The exact topic name depends on the vehicle model variant used
(PX4_SIM_MODEL=gz_x500_mono_cam ships a camera; plain gz_x500 does not --
see fly_vision.sh) and may need adjusting per PX4/Gazebo version; run
`gz topic -l` while the sim is running to find the real topic name, the
same diagnostic step fly_squad.sh's own comments point to for the world
name.
"""

import numpy as np


class GzCameraSource:
    """Implements vision_executor.py's CameraSource protocol
    (get_frame()) by subscribing to a bridged ROS2 Image topic."""

    def __init__(self, ros2_image_topic: str = "/camera", node_name: str = "vision_camera_bridge"):
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import Image
        from cv_bridge import CvBridge

        if not rclpy.ok():
            rclpy.init(args=None)
        self._node = Node(node_name)
        self._bridge = CvBridge()
        self._latest_frame = None
        self._node.create_subscription(Image, ros2_image_topic, self._on_image, 10)

    def _on_image(self, msg) -> None:
        self._latest_frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")

    def get_frame(self) -> "np.ndarray | None":
        """Spins the ROS2 node briefly to process any pending image
        callbacks, then returns whatever the most recent frame is (or
        None if no frame has arrived yet -- vision_executor.py's
        detector calls handle a None/empty frame the same way a
        no-detection frame is handled, no special-casing needed)."""
        import rclpy
        rclpy.spin_once(self._node, timeout_sec=0.1)
        return self._latest_frame
