from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import EqualsSubstitution, LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument 
from launch_ros.substitutions import FindPackageShare, ExecutableInPackage



#use Xacro files to make life easier
def generate_launch_description():
    
   return LaunchDescription([   
    DeclareLaunchArgument(name = "use_jsp",
                          default_value = "gui",
                          description="use_jsp: gui->joint_state_publisher_gui jsp->joint_state_publisher"),
    DeclareLaunchArgument(name = "rviz_config",
                          default_value = "turtle_test.rviz",
                          description = "rviz_config <file_name>"),

    Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    name='rsp',
    parameters=[
       {"robot_description" :
        Command([ExecutableInPackage("xacro", "xacro"), " ",
                 PathJoinSubstitution(
                [FindPackageShare("turtle_brick"), "robot.urdf.xacro"])])}
    ]
    ),
    Node(
    package='joint_state_publisher',
    executable='joint_state_publisher',
    name='jsp',
    condition=IfCondition(EqualsSubstitution(LaunchConfiguration("use_jsp"), "jsp"))
    ),
    Node(
    package='joint_state_publisher_gui',
    executable='joint_state_publisher_gui',
    name='jsp_gui',
    condition=IfCondition(EqualsSubstitution(LaunchConfiguration("use_jsp"), "gui"))
    ),
    Node(
    package='rviz2',
    executable='rviz2',
    name='rviz',
    arguments=[
       '-d',
        PathJoinSubstitution([FindPackageShare("turtle_brick"),LaunchConfiguration("rviz_config")]),
    ]
    ),
    
    
    ])
