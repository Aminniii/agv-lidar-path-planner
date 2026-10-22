"""Optional ROS 2 dependency boundary; no ROS imports in standalone mode."""

def load_ros_dependencies():
    try:
        import rclpy
        from action_msgs.msg import GoalStatus
        from geometry_msgs.msg import PoseStamped
        from nav2_msgs.action import FollowPath
        from nav_msgs.msg import Path as RosPath
        from rclpy.action import ActionClient
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from tf2_ros import Buffer, TransformException, TransformListener
        return True, None, (rclpy, Node, ActionClient, FollowPath, GoalStatus, Buffer,
                            TransformListener, TransformException, RosPath, PoseStamped,
                            QoSProfile, DurabilityPolicy, ReliabilityPolicy)
    except ImportError as exc:
        return False, str(exc), None
