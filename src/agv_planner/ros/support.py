"""Optional ROS 2 dependency boundary; no ROS imports in standalone mode."""

def load_ros_dependencies():
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.action import ActionClient
        from nav2_msgs.action import FollowPath
        from action_msgs.msg import GoalStatus
        from tf2_ros import Buffer, TransformListener, TransformException
        from nav_msgs.msg import Path as RosPath
        from geometry_msgs.msg import PoseStamped
        from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
        return True, None, (rclpy, Node, ActionClient, FollowPath, GoalStatus, Buffer,
                            TransformListener, TransformException, RosPath, PoseStamped,
                            QoSProfile, DurabilityPolicy, ReliabilityPolicy)
    except ImportError as exc:
        return False, str(exc), None
