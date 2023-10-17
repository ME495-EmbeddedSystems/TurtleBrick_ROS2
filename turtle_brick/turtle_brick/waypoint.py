from enum import Enum, auto
from crazy_turtle_interfaces.srv import Switch
from geometry_msgs.msg import Twist, Vector3
from math import pi
from random import uniform
import rclpy
from rclpy.node import Node
from turtlesim.srv import TeleportAbsolute, SetPen
from rcl_interfaces.msg import ParameterDescriptor
from std_srvs.srv import Empty
from turtle_interfaces.srv import Waypoints
from math import sqrt, atan2
from rclpy.callback_groups import ReentrantCallbackGroup
from turtlesim.msg import Pose
from turtle_interfaces.msg import ErrorMetric

def turtle_twist(xdot, omega):
    """ Create a twist suitable for a turtle

        Args:
           xdot (float) : the forward velocity
           omega (float) : the angular velocity

        Returns:
           Twist - a 2D twist object corresponding to linear/angular velocity
    """
    return Twist(linear = Vector3(x = xdot, y = 0.0, z = 0.0),
                  angular = Vector3(x = 0.0, y = 0.0, z = omega))

class State(Enum):
    """ Current state of the system.
        Determines what the timer and toggle function should be doing on each iteration
    """
    MOVING = auto(),
    STOPPED = auto()

class Waypoint(Node):
    """     Set up the waypints as input in the launch files
            Publish ErrorMetric msg/TWIST commands at a fixed rate
            Service on 1. load waypoints set by launchfiles 2. toggle to change the turtle's mode
    """
    def __init__(self):
        super().__init__("waypoint")
        self.cbgroup = ReentrantCallbackGroup()
        self.state = State.STOPPED
        self.curr_x = 0
        self.curr_y = 0
        self.curr_theta = 0.0
        self.x_list = []
        self.y_list = []
        self.index = 1
        self.cycleNumber = 0
        self.total_dist_real = 0
        self.cycle_error = 0
        self.total_dist_ideal = 0

        self.declare_parameter("frequency", 100.0, 
                               ParameterDescriptor(description="The frequency of the DEBUG message Issuing Command"))
        self.declare_parameter("tolerance", 0.05, 
                               ParameterDescriptor(description="The tolerance of the DEBUG message Issuing Command"))
        self.srv = self.create_service(Empty, "toggle", self.toggle_callback)
        self.frequency = self.get_parameter("frequency").get_parameter_value().double_value
        self.tolerance = self.get_parameter("tolerance").get_parameter_value().double_value
        self.timer = self.create_timer(1/self.frequency, self.timer_callback)
        self.loadwaypoints = self.create_service(Waypoints, "load", self.load_callback, callback_group=self.cbgroup)
        self.teleport =  self.create_client(TeleportAbsolute, "turtle1/teleport_absolute", callback_group=self.cbgroup)
        self.setpen = self.create_client(SetPen, "turtle1/set_pen", callback_group=self.cbgroup)
        self.reset = self.create_client(Empty, "reset", callback_group = self.cbgroup)
        self.subscription = self.create_subscription(Pose, "turtle1/pose", self.listener_callback, 10)
        self.publisher_command=self.create_publisher(Twist, "cmd_vel", 10)
        self.publisher_errorM=self.create_publisher(ErrorMetric, "loop_metrics", 10)


        

        if not self.teleport.wait_for_service(timeout_sec=1.0):
            raise RuntimeError('Timeout waiting for "teleport" service to become available')

        if not self.setpen.wait_for_service(timeout_sec=1.0):
            raise RuntimeError('Timeout waiting for "setpen" service to become available')
        


    def listener_callback(self, msg):
        """Callback function for Position Feedback subscription. 
        Subscribe to a turtlesim topic that provides the position and orientation of the turtle.
        Also calculate the actual distance the turtle travelled 

        Args:
            msg (Pose): the current positions of the turtle, 
            contains float x, float y and float theta
        """
        self.get_logger().debug(f"I heard Position feedback!{msg}")
        #calculate the little step 
        prev_x = self.curr_x
        prev_y = self.curr_y
        dx = msg.x - prev_x
        dy = msg.y - prev_y
        step_size = sqrt(dx**2 + dy**2)
        self.total_dist_real += step_size
        

        #update the current x y with message
        self.curr_x = msg.x
        self.curr_y = msg.y
        self.curr_theta = msg.theta
        

    def timer_callback(self):
        """Timer callback function for 
        1. lead the turtle to the target waypoints
        2. Publish the cycle numbers, actual distance turtle travelled, errors in distance for turtle ideal and actual path
        at the end of the cycle
    

        """
        dist = 1000000
        if self.state == State.MOVING:
            self.get_logger().debug("Issuing Command!")
            length = len(self.x_list)
            if length == 0 :
                self.get_logger().error(f'No waypoints loaded. Load them with the "load" service.')
            else:
                self.index %= len(self.x_list)
                wpTar_x = self.x_list[self.index]
                wpTar_y = self.y_list[self.index]
                dist = sqrt((wpTar_y-self.curr_y)**2+(wpTar_x-self.curr_x)**2)
                if (dist > self.tolerance):
                    wp_turtle_angle = atan2((wpTar_y-self.curr_y),(wpTar_x-self.curr_x))
                    angle_diff = wp_turtle_angle - self.curr_theta

                    
                    
                    if angle_diff > 0:
                        self.publisher_command.publish(turtle_twist(1.0, angle_diff*2))
                    else:
                        self.publisher_command.publish(turtle_twist(1.0, angle_diff*2))
                    
                else:
                    if self.index == 0:
                        self.cycleNumber += 1
                        self.cycle_error = self.total_dist_real - self.total_dist_ideal
                        self.publisher_errorM.publish(ErrorMetric(complete_loops = self.cycleNumber, actual_distance = self.total_dist_real, error = self.cycle_error))
                        self.total_dist_real = 0 #reset real distance after completing one cycle
                    self.index += 1



                    

            

    def toggle_callback(self, request, response):
        """A callback function for switching Turtle's state

        Args:
            request (Empty): empty
            response (Empty): empty

        Returns:
            Empty
        """
        if self.state == State.STOPPED:
            self.state = State.MOVING
        else:
            self.state = State.STOPPED
            self.get_logger().info("Stopping")
        return response
    
    async def load_callback(self, request, response):
        """A callback function to make turtle load the waypoints and draw a cross on 
        each point

        Args:
            request (Waypoint_request): list of x position, list of y position
            response (Waypoint_response): a float to represent the ideal path's distance

        Returns:
            _type_: _description_
        """
        self.x_list = request.x
        self.y_list = request.y
        length = len(self.x_list)
        total_dist = 0
        for i in range(1, length):
            dist = sqrt((self.x_list[i]-self.x_list[i-1])**2+(self.y_list[i]-self.y_list[i-1])**2)
            total_dist = total_dist + dist
        
        return_dist = sqrt((self.y_list[length-1]-self.y_list[0])**2 + (self.x_list[length-1]-self.x_list[0])**2)
        final_dist = return_dist + total_dist
        self.total_dist_ideal = final_dist
        response.distance = final_dist



        for i in range(0, length):
            #draw to upper left
            draw_x1 = self.x_list[i]-0.2
            draw_y1 = self.y_list[i]+0.2
            
            await self.setpen.call_async(SetPen.Request(r = 255, g = 255, b = 255, width = 2, off = 1))
            await self.teleport.call_async(TeleportAbsolute.Request(x = draw_x1, y = draw_y1, theta = 0.0))
            await self.setpen.call_async(SetPen.Request(r = 255, g = 255, b = 255, width = 2, off = 0))
            #draw to lower right
            draw_x2 = self.x_list[i]+0.2
            draw_y2 = self.y_list[i]-0.2
            await self.teleport.call_async(TeleportAbsolute.Request(x = draw_x2, y = draw_y2, theta = 0.0))
            #drop pen, pen off  
            await self.setpen.call_async(SetPen.Request(r = 255, g = 255, b = 255, width = 2, off = 1))
            
            #draw to lower left
            draw_x3 = self.x_list[i]-0.2
            draw_y3 = self.y_list[i]-0.2
            await self.teleport.call_async(TeleportAbsolute.Request(x = draw_x3, y = draw_y3, theta = 0.0))
            #pick up pen, pen on
            await self.setpen.call_async(SetPen.Request(r = 255, g = 255, b = 255, width = 2, off = 0))
            
            #draw to upper right
            draw_x4 = self.x_list[i]+0.2
            draw_y4 = self.y_list[i]+0.2
            await self.teleport.call_async(TeleportAbsolute.Request(x = draw_x4, y = draw_y4, theta = 0.0))
            #drop off pen, pen off
            await self.setpen.call_async(SetPen.Request(r = 255, g = 255, b = 255, width = 2, off = 1))
            

            
            
        
        await self.teleport.call_async(TeleportAbsolute.Request(x = self.x_list[0], y = self.y_list[0], theta = 0.0))
        await self.setpen.call_async(SetPen.Request(r = 255, g = 255, b = 255, width = 2, off = 0))
        self.state = State.STOPPED
        self.total_dist_real = 0
        return response
    

    



    






def main(args=None):
    """the main() function"""
    rclpy.init(args=args)
    mymove = Waypoint()
    rclpy.spin(mymove)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
 
