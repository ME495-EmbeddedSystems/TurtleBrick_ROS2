#from turtle_brick_interfaces.srv import 
from geometry_msgs.msg import Twist, Vector3, PoseStamped, TransformStamped
from sensor_msgs.msg import JointState
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from math import sqrt, atan2
from rclpy.callback_groups import ReentrantCallbackGroup
from turtlesim.msg import Pose
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import tf_transformations
from tf2_ros import StaticTransformBroadcaster
from turtle_brick_interfaces.msg import Tilt


def turtle_twist(xdot, ydot):
    """ Create a twist suitable for a turtle

        Args:
           xdot (float) : the forward velocity
           omega (float) : the angular velocity

        Returns:
           Twist - a 2D twist object corresponding to linear/angular velocity
    """
    return Twist(linear = Vector3(x = xdot, y = ydot, z = 0.0),
                  angular = Vector3(x = 0.0, y = 0.0, z = 0.0))


class turtle_robot(Node):
    """A turtle robot node that could control the robot and react to the click on the map

    Args:
        Node (ros node):
        PARAMETER:
        frequency:The frequency of timer callback function
        tolerance:The tolerance distance of reaching target
        max_velocity: The max velocity the robot should go
        PUBLISH:
        joint_states:three joints's angle value  [JointState]
        odom: odometry message [Odometry]
        cmd_vel:command [Twist]
    
        SUBSCRIBE:
        goal_pose [PoseStamped]
        tilt [Tilt]
        turtle1/pose [Pose]

        PARAMETER:
        frequency:frequency of timer_callback
        tolerance:tolerance for turtle to find destination
    """
        
    def __init__(self):
        super().__init__("turtle_robot")
        #global variables:
        self.turtle_init_x = None
        self.turtle_init_y = None
        self.turtle_init_theta = None
        self.curr_x = None
        self.curr_y = None
        self.curr_theta = None
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        self.isPublish = False
        self.tar_x = None
        self.tar_y = None 
        self.wheel_dir = 0
        self.tilt_angle_platform = 0

        self.cbgroup = ReentrantCallbackGroup()

        self.declare_parameter("frequency", 100.0, 
                               ParameterDescriptor(description="The frequency of timer callback function"))
        self.declare_parameter("tolerance", 0.05, 
                               ParameterDescriptor(description="The tolerance distance of reaching target"))
        self.declare_parameter("max_velocity", 1.0, 
                               ParameterDescriptor(description="The max velocity the robot should go"))
        self.frequency = self.get_parameter("frequency").get_parameter_value().double_value
        self.tolerance = self.get_parameter("tolerance").get_parameter_value().double_value
        self.max_velocity = self.get_parameter("max_velocity").get_parameter_value().double_value
        
        self.joint_state_publisher = self.create_publisher(JointState, "joint_states", 10)
        self.odom_publisher = self.create_publisher(Odometry, "odom", 10)
        self.cmd_publisher=self.create_publisher(Twist, "cmd_vel", 10)
        self.goal_pose_subscriber = self.create_subscription(PoseStamped, "goal_pose", self.listerner_callback_goal_pose, 10)
        self.tilt_subscriber = self.create_subscription(Tilt, "tilt", self.listener_callback_tilt, 10)
        self.subscription = self.create_subscription(Pose, "turtle1/pose", self.listener_callback, 10)
        self.timer = self.create_timer(1.0/self.frequency, self.timer_callback)
        
        #initialize the broadcaster for base_link frame relative to the odom frame
        self.odom_base_broadcaster = TransformBroadcaster(self)
        
        
    def timer_callback(self):
        """
        timer_callback:
        1. adding a fixed frame odom, if initial value is given, then odom frame is posed
        2. move the turtle to the goal pose
        """
        #Joint_publisher set_up
        joint_pub_msg = JointState()
        joint_pub_msg.header.stamp = self.get_clock().now().to_msg()
        joint_pub_msg.name = ["cyl_platform_joint","base_stem_joint","stem_wheel_joint"]
        joint_pub_msg.position = [self.tilt_angle_platform, self.wheel_dir, 0.0]
        joint_pub_msg.velocity = []
        joint_pub_msg.effort = []
        self.joint_state_publisher.publish(joint_pub_msg)
        #world-odom setup/ fixed frame broadcaster
        #adding a fixed frame odom, if initial value is given, then odom frame is posed
        if self.turtle_init_x != None and self.turtle_init_y != None and self.turtle_init_theta != None and not self.isPublish:
            
            t = TransformStamped()
            t.header.frame_id = 'world'
            t.child_frame_id = 'odom'
            
            q = tf_transformations.quaternion_about_axis(0, (0,0,1))
            t.transform.translation.x = self.turtle_init_x
            t.transform.translation.y = self.turtle_init_y
            t.transform.translation.z = 3.0 #height from ground to center of cube 
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
            self.isPublish = True
            self.tf_broadcaster.sendTransform(t)
        #send the turtle_sim node to the desired places
        if self.curr_y != None and self.curr_x != None and self.tar_y != None and self.tar_x != None:
            dist = sqrt((self.tar_y-self.curr_y)**2+(self.tar_x-self.curr_x)**2)
            self.wheel_dir = atan2((self.tar_y-self.curr_y),(self.tar_x-self.curr_x))
            if (dist > self.tolerance):
                deltax = self.tar_x- self.curr_x
                deltay = self.tar_y - self.curr_y
                x_velocity = self.max_velocity*(deltax)/(sqrt(deltax**2+deltay**2))
                y_velocity = self.max_velocity*(deltay)/(sqrt(deltax**2+deltay**2))

                self.cmd_publisher.publish(turtle_twist(x_velocity, y_velocity))


            else:
                self.cmd_publisher.publish(turtle_twist(0.0, 0.0))

                

    def listener_callback(self, msg):
        """callback function that continually update the current turtle position

        Args:
            msg (Pose): the position of turtle current position
        """
        if self.turtle_init_x == None and self.turtle_init_y == None and self.turtle_init_theta == None :
            self.turtle_init_x = msg.x
            self.turtle_init_y = msg.y
            self.turtle_init_theta = msg.theta

        else:
            self.curr_x = msg.x
            self.curr_y = msg.y
            self.curr_theta = msg.theta

            #broadcast base_link relative to odom frame
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

            self.odom_base_broadcaster.sendTransform(t)
    
    def listerner_callback_goal_pose(self, msg:PoseStamped):
        """a callback function receives the position of the target position clicked on map

        Args:
            msg (PoseStamped): position of the target position clicked on map
        """
        self.tar_x = msg.pose.position.x
        self.tar_y = msg.pose.position.y
    
    def listener_callback_tilt(self, msg:Tilt):
        self.tilt_angle_platform = msg.tilt_angle

def main(args=None):
    rclpy.init(args=args)
    mymove = turtle_robot()
    rclpy.spin(mymove)
    rclpy.shutdown()

if __name__ == '__main__':
    main()