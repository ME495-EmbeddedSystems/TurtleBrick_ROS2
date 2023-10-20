import rclpy
from rclpy.node import Node
#from turtle_brick_interfaces.srv import 
from geometry_msgs.msg import Twist, Vector3, PoseStamped
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from math import sqrt
from turtlesim.msg import Pose
from tf2_msgs.msg import TFMessage

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


class Catcher(Node):
    """
    PARAMETERS:
    frequency -frequency of the timer_callback() function should be executed
    max_velocity -max velocity the robot should go

    PUBLISHES:
    visualization_marker (visualization_messages/msg/Marker) - The markers that we are drawing
    cmd_vel (geometry_msgs/msg/Twist) -constantly publish command 
    SUBSCRIBES:
    turtle1/pose (turtlesim/msg/Pose) -the turtle's real time locations
    goal_pose (geometry_msgs/msg/PoseStamped) -the goal position the robot should reach
    """

    def __init__(self):
        super().__init__("catcher")
        
        self.declare_parameter("con_frequency", 250.0, 
                               ParameterDescriptor(description="The frequency"))
        self.frequency = self.get_parameter("con_frequency").get_parameter_value().double_value
        self.timer = self.create_timer(1.0/self.frequency, self.timer_callback)

        self.brick_location_subscriber = self.create_subscription(TFMessage, "tf", self.br_callback, 10)
        self.base_pose = self.create_subscription(Pose, "turtle1/pose", self.listener_callback, 10)
        self.brick_x = None
        self.brick_y = None
        self.brick_z = None
        self.brick_z_last = None
        self.base_x = None
        self.base_y = None
        self.robot_height = 5.1
        self.gravity = -0.3
        self.falling = False

        self.declare_parameter("max_velocity", 1.5, 
                               ParameterDescriptor(description="The max velocity of robot"))
        self.max_velocity = self.get_parameter("max_velocity").get_parameter_value().double_value
        self.cmd_publisher = self.create_publisher(Twist, "cmd_vel", 10)
        self.goal_publisher =self.create_publisher(PoseStamped, "goal_pose", 10)
    #Parameters and functions
    def br_callback(self, msg:TFMessage):
        """callback funciton constantly received brick's location 

        Args:
            msg (TFMessage): msg contains frame's location
        """
        for item in msg.transforms:
            if item.header.frame_id == "world" and item.child_frame_id == "brick":
                self.brick_z_last = self.brick_z
                self.brick_x_last = self.brick_x
                self.brick_y_last = self.brick_y
                self.brick_x = item.transform.translation.x
                self.brick_y = item.transform.translation.y
                self.brick_z = item.transform.translation.z
                

                


    def listener_callback(self, msg:Pose):
        """continually update the current base link position

        Args:
            msg (Pose): the position of base link
        """
        self.base_x = msg.x
        self.base_y = msg.y

    def timer_callback(self):
        """Timer callback function execuated at the fixed frequency and will constantly publish the goal
        location and test if the brick is reachable
        """
        if self.base_x != None and self.brick_x != None:
            dx = self.base_x - self.brick_x
            dy = self.base_y - self.brick_y

            brick_base_planar_dist = sqrt(dx**2 + dy**2)
            time_robot_reach = brick_base_planar_dist/self.max_velocity
            
            if(self.brick_z - self.robot_height > 0):
                time_brick_falling = sqrt(-2*(self.brick_z - self.robot_height)/self.gravity)-0.5 #0.5 for time tolerance
                
                if self.brick_z != self.brick_z_last and self.brick_x == self.brick_x_last and self.brick_y == self.brick_y_last:
                    self.falling = True
                    
                else:
                    self.falling = False
                    
                if time_brick_falling >= time_robot_reach and self.falling == True:
                    
                    goal = PoseStamped()
                    goal.header.stamp = self.get_clock().now().to_msg()
                    goal.header.frame_id = "world"
                    goal.pose.position.x = self.brick_x
                    goal.pose.position.y = self.brick_y
                    self.goal_publisher.publish(goal)
                
                    
            else:
                self.get_logger().info("Unreachable")
        

def main(args=None):
    rclpy.init(args=args)
    control = Catcher()
    rclpy.spin(control)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
