from enum import Enum, auto
#from turtle_brick_interfaces.srv import 
from geometry_msgs.msg import Twist, Vector3, PoseStamped, TransformStamped
from sensor_msgs.msg import JointState
from math import pi
from random import uniform
import rclpy
from rclpy.node import Node
from turtlesim.srv import TeleportAbsolute, SetPen
from rcl_interfaces.msg import ParameterDescriptor
from std_srvs.srv import Empty
#from turtle_brick_interfaces.srv import 
from math import sqrt, atan2
from rclpy.callback_groups import ReentrantCallbackGroup
from turtlesim.msg import Pose
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import tf_transformations
from tf2_ros import StaticTransformBroadcaster

#from turtle_brick_interfaces.msg import Tilt

""" ros2 topic list -t
/clicked_point [geometry_msgs/msg/PointStamped]
/goal_pose [geometry_msgs/msg/PoseStamped]
/initialpose [geometry_msgs/msg/PoseWithCovarianceStamped]
/joint_states [sensor_msgs/msg/JointState]
/parameter_events [rcl_interfaces/msg/ParameterEvent]
/robot_description [std_msgs/msg/String]
/rosout [rcl_interfaces/msg/Log]
/tf [tf2_msgs/msg/TFMessage]
/tf_static [tf2_msgs/msg/TFMessage]

"""

def turtle_twist(xdot):
    """ Create a twist suitable for a turtle

        Args:
           xdot (float) : the forward velocity
           omega (float) : the angular velocity

        Returns:
           Twist - a 2D twist object corresponding to linear/angular velocity
    """
    return Twist(linear = Vector3(x = xdot, y = 0.0, z = 0.0),
                  angular = Vector3(x = 0.0, y = 0.0, z = 0.0))


class turtle_robot(Node):
        
    def __init__(self):
        super().__init__("turtle_robot")
        #global variables:
        # self.turtle_init_x = None
        # self.turtle_init_y = None
        self.turtle_init_x = 0.0
        self.turtle_init_y = 0.0
        self.turtle_init_theta = 0.0
        self.curr_x = None
        self.curr_y = None
        self.curr_theta = None
        self.tf_broadcaster = StaticTransformBroadcaster(self)

        #adding a fixed frame odom
        t = TransformStamped()
        t.header.frame_id = 'world'
        t.child_frame_id = 'odom'
        
        t.transform.translation.x = self.turtle_init_x
        t.transform.translation.y = self.turtle_init_y
        t.transform.translation.z = 3.0 #height from ground to center of cube 
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 1.0
        t.transform.rotation.w = 0.0

        self.tf_broadcaster.sendTransform(t)
        
        


        


        self.cbgroup = ReentrantCallbackGroup()

        self.declare_parameter("frequency", 100.0, 
                               ParameterDescriptor(description="The frequency of the DEBUG message Issuing Command"))
        self.declare_parameter("tolerance", 0.05, 
                               ParameterDescriptor(description="The tolerance of the DEBUG message Issuing Command"))
        self.frequency = self.get_parameter("frequency").get_parameter_value().double_value
        self.tolerance = self.get_parameter("tolerance").get_parameter_value().double_value
        
        self.joint_state_publisher = self.create_publisher(JointState, "joint_states", 10)
        self.odom_publisher = self.create_publisher(Odometry, "odom", 10)
        self.cmd_publisher=self.create_publisher(Twist, "cmd_vel", 10)
        # self.goal_pose_subscriber = self.create_subscription(PoseStamped, "goal_pose", self.listerner_callback_goal_pose, 10)
        #self.tilt_subscriber = self.create_subscription(Tilt, "tilt", self.listener_callback_tilt, 10)
        self.subscription = self.create_subscription(Pose, "turtle1/pose", self.listener_callback, 10)
        self.timer = self.create_timer(1.0/self.frequency, self.timer_callback)
        
        #initialize the broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
    
    def timer_callback(self):
        joint_pub_msg = JointState()
        joint_pub_msg.header.stamp = self.get_clock().now().to_msg()
        joint_pub_msg.name = ["cyl_platform_joint","base_stem_joint","stem_wheel_joint"]
        joint_pub_msg.position = [1.0, 1.0, 1.0]
        joint_pub_msg.velocity = []
        joint_pub_msg.effort = []
        self.joint_state_publisher.publish(joint_pub_msg)
        
    def listener_callback(self, msg):
        if self.turtle_init_x == None and self.turtle_init_y == None and self.turtle_init_theta == None :
            self.turtle_init_x = msg.x
            self.turtle_init_y = msg.y
            self.turtle_init_theta = msg.theta

        else:
            self.curr_x = msg.x
            self.curr_y = msg.y
            self.curr_theta = msg.theta

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        dx = self.curr_x - self.turtle_init_x
        dy = self.curr_y - self.turtle_init_y
        dtheta = self.curr_theta - self.turtle_init_theta
        #CHANGE HERE
        t.transform.translation.x = dx
        t.transform.translation.y = dy
        t.transform.translation.z = 0.0
        #CHANGE HERE!
        q = tf_transformations.quaternion_about_axis(dtheta, (0.0, 0.0, 1.0))
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    mymove = turtle_robot()
    rclpy.spin(mymove)
    rclpy.shutdown()

if __name__ == '__main__':
    main()