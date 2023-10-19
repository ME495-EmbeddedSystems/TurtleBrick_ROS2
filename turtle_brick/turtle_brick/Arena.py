"""Draw robotic grippers using RVIZ markers and make them interactive. """


import rclpy
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from interactive_markers.interactive_marker_server import InteractiveMarkerServer, InteractiveMarker
from visualization_msgs.msg import Marker, InteractiveMarkerControl, MarkerArray
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from enum import Enum, auto
#from turtle_brick_interfaces.srv import 
from geometry_msgs.msg import Twist, Vector3, PoseStamped, TransformStamped
from sensor_msgs.msg import JointState
from math import pi
from random import uniform
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from std_srvs.srv import Empty
from math import sqrt, atan2
from rclpy.callback_groups import ReentrantCallbackGroup
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import tf_transformations
from tf2_ros import StaticTransformBroadcaster
from turtle_brick_interfaces.srv import Place
from turtlesim.msg import Pose


class State(Enum):
    COLLIDE=auto()
    FALL=auto()
    STOP=auto()

    
class Arena(Node):
    """
    PUBLISHES:
    visualization_marker (visualization_messages/msg/Marker) - The markers that we are drawing
    
    SUBSCRIBES:
    Subscribes: to an interactive marker server
    """

    def __init__(self):
        super().__init__("arena")
        
        #Parameters and functions
        

        self.b_curr_x = 6.0
        self.b_curr_y = 6.0
        self.b_curr_z = 6.0
        self.state = State.STOP
        self.declare_parameter("frequency", 250.0, 
                               ParameterDescriptor(description="The frequency"))
        self.frequency = self.get_parameter("frequency").get_parameter_value().double_value
        self.timer = self.create_timer(1.0/self.frequency, self.timer_callback)
        self.brick_world_broadcaster = TransformBroadcaster(self)
        self.place = self.create_service(Place, "place",  self.place_callback)
        self.drop = self.create_service(Empty, "drop", self.drop_callback)
        markerQoS = QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.pub1 = self.create_publisher(Marker, "visualization_marker", markerQoS)
        self.base_link_pos = self.create_subscription(Pose, "turtle1/pose", self.listener_callback, 10)
        #bool parameter for show brick
        self.if_show = False
        #Drop process parameters
        self.plat_z = 5.1
        self.plat_r = 2.0 #radius of platform
        self.gravity = -1.0 
        self.period = 1.0/self.frequency
        self.v0 = 0.0
        self.base_x = None
        self.base_y = None
        
        


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


        


   


        

    

    def brick_show(self):
        brick = Marker()
        brick.header.frame_id = "brick"
        brick.header.stamp = self.get_clock().now().to_msg()
        brick.id = 5
        brick.type = Marker.CUBE
        brick.action = Marker.ADD
        brick.scale.x = 1.0
        brick.scale.y = 0.2
        brick.scale.z = 0.5
        brick.pose.position.x = 0.0
        brick.pose.position.y = 0.0
        brick.pose.position.z = 0.0
        brick.pose.orientation.x = .707
        brick.pose.orientation.y = 0.0
        brick.pose.orientation.z = 0.0
        brick.pose.orientation.w = 0.707
        brick.color.r = 1.0
        brick.color.g = 1.0
        brick.color.b = 1.0
        brick.color.a = 0.3
        brick.frame_locked = True
        self.pub1.publish(brick)
            

    def timer_callback(self):
        #transfer the brick frame to world frame
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'
        t.child_frame_id = 'brick'

        
        t.transform.translation.x = self.b_curr_x
        t.transform.translation.y = self.b_curr_y
        t.transform.translation.z = self.b_curr_z
        
        q = tf_transformations.quaternion_about_axis(0, (0.0, 0.0, 1.0))
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.brick_world_broadcaster.sendTransform(t)


        #activate fall process if state is changing to fall
        if self.state == State.FALL:
            self.b_curr_z = self.b_curr_z + self.v0*self.period + 0.5*self.gravity*self.period**2
            self.v0 = self.v0 + self.gravity*self.period
            
            x_lower_bound = self.base_x - self.plat_r
            x_upper_bound = self.base_x + self.plat_r
            y_upper_bound = self.base_y + self.plat_r
            y_lower_bound = self.base_y - self.plat_r
            if self.b_curr_x < x_upper_bound and self.b_curr_x > x_lower_bound and self.b_curr_y < y_upper_bound and self.b_curr_y > y_lower_bound and (self.b_curr_z-0.1-0.001)<self.plat_z:
                #hit the platform
                self.state = State.COLLIDE
            elif self.b_curr_z < 0 or self.b_curr_z == 0:
                #hit the groud
                self.state = State.STOP


            


    def place_callback(self, request, response):
        self.b_curr_x = request.x
        self.b_curr_y = request.y
        self.b_curr_z = request.z
        if self.if_show == False:
            self.brick_show()
            self.if_show = True

        return response

    def drop_callback(self, request, response):
        self.state = State.FALL
        return response

    def listener_callback(self, msg:Pose):
        self.base_x = msg.x
        self.base_y = msg.y
        



def main(args=None):
    rclpy.init(args=args)
    node = Arena()
    rclpy.spin(node)
    rclpy.shutdown()
