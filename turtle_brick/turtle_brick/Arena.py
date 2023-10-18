"""Draw robotic grippers using RVIZ markers and make them interactive. """


import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from interactive_markers.interactive_marker_server import InteractiveMarkerServer, InteractiveMarker
from visualization_msgs.msg import Marker, InteractiveMarkerControl, MarkerArray
from rclpy.qos import QoSProfile, QoSDurabilityPolicy


class Arena(Node):
    """
    PUBLISHES:
    visualization_marker (visualization_messages/msg/Marker) - The markers that we are drawing

    SUBSCRIBES:
    Subscribes: to an interactive marker server
    """
    def __init__(self):
        super().__init__("arena")


        # We use TRANSIENT_LOCAL durability for the publisher. By setting both publisher and subscriber
        # Durability to TRANSIENT_LOCAL we emulate the effect of "latched publishers" from ROS 1
        # (See https://github.com/ros2/ros2/issues/464)
        # Essentially this means that when subscribers first connect to the topic they receive the
        # last message published on the topic. Useful for example because rviz might open after
        # the initial markers are published
        markerQoS = QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.pub1 = self.create_publisher(Marker, "visualization_marker", markerQoS)
        # The second link is oriented at 90 degrees
        self.m = Marker()
        self.m.header.frame_id = "world"
        self.m.header.stamp = self.get_clock().now().to_msg()
        self.m.id = 1
        self.m.type = Marker.CUBE
        self.m.action = Marker.ADD
        self.m.scale.x = 1.0
        self.m.scale.y = 4.0
        self.m.scale.z = 11.0
        self.m.pose.position.x = 11.5
        self.m.pose.position.y = 5.5
        self.m.pose.position.z = 2.0
        self.m.pose.orientation.x = .707
        self.m.pose.orientation.y = 0.0
        self.m.pose.orientation.z = 0.0
        self.m.pose.orientation.w = .707
        self.m.color.r = 0.0
        self.m.color.g = 0.0
        self.m.color.b = 1.0
        self.m.color.a = 0.3
        self.pub1.publish(self.m)

        self.m1 = Marker()
        self.m1.header.frame_id = "world"
        self.m1.header.stamp = self.get_clock().now().to_msg()
        self.m1.id = 2
        self.m1.type = Marker.CUBE
        self.m1.action = Marker.ADD
        self.m1.scale.x = 1.0
        self.m1.scale.y = 4.0
        self.m1.scale.z = 11.0
        self.m1.pose.position.x = -0.5
        self.m1.pose.position.y = 5.5
        self.m1.pose.position.z = 2.0
        self.m1.pose.orientation.x = .707
        self.m1.pose.orientation.y = 0.0
        self.m1.pose.orientation.z = 0.0
        self.m1.pose.orientation.w = .707
        self.m1.color.r = 0.0
        self.m1.color.g = 0.0
        self.m1.color.b = 1.0
        self.m1.color.a = 0.3
        self.pub1.publish(self.m1)

        self.m2 = Marker()
        self.m2.header.frame_id = "world"
        self.m2.header.stamp = self.get_clock().now().to_msg()
        self.m2.id = 3
        self.m2.type = Marker.CUBE
        self.m2.action = Marker.ADD
        self.m2.scale.x = 11.0
        self.m2.scale.y = 4.0
        self.m2.scale.z = 1.0
        self.m2.pose.position.x = 5.5
        self.m2.pose.position.y = -0.5
        self.m2.pose.position.z = 2.0
        self.m2.pose.orientation.x = .707
        self.m2.pose.orientation.y = 0.0
        self.m2.pose.orientation.z = 0.0
        self.m2.pose.orientation.w = 0.707
        self.m2.color.r = 0.0
        self.m2.color.g = 0.0
        self.m2.color.b = 1.0
        self.m2.color.a = 0.3
        self.pub1.publish(self.m2)

        self.m3 = Marker()
        self.m3.header.frame_id = "world"
        self.m3.header.stamp = self.get_clock().now().to_msg()
        self.m3.id = 4
        self.m3.type = Marker.CUBE
        self.m3.action = Marker.ADD
        self.m3.scale.x = 11.0
        self.m3.scale.y = 4.0
        self.m3.scale.z = 1.0
        self.m3.pose.position.x = 5.5
        self.m3.pose.position.y = 11.5
        self.m3.pose.position.z = 2.0
        self.m3.pose.orientation.x = .707
        self.m3.pose.orientation.y = 0.0
        self.m3.pose.orientation.z = 0.0
        self.m3.pose.orientation.w = 0.707
        self.m3.color.r = 0.0
        self.m3.color.g = 0.0
        self.m3.color.b = 1.0
        self.m3.color.a = 0.3
        self.pub1.publish(self.m3)

        self.brick = Marker()
        self.brick.header.frame_id = "world"
        self.brick.header.stamp = self.get_clock().now().to_msg()
        self.brick.id = 5
        self.brick.type = Marker.CUBE
        self.brick.action = Marker.ADD
        self.brick.scale.x = 1.0
        self.brick.scale.y = 0.2
        self.brick.scale.z = 0.5
        self.brick.pose.position.x = 5.5
        self.brick.pose.position.y = 11.5
        self.brick.pose.position.z = 2.0
        self.brick.pose.orientation.x = .707
        self.brick.pose.orientation.y = 0.0
        self.brick.pose.orientation.z = 0.0
        self.brick.pose.orientation.w = 0.707
        self.brick.color.r = 1.0
        self.brick.color.g = 1.0
        self.brick.color.b = 1.0
        self.brick.color.a = 0.3
        self.pub1.publish(self.brick)


def main(args=None):
    rclpy.init(args=args)
    node = Arena()
    rclpy.spin(node)
    rclpy.shutdown()