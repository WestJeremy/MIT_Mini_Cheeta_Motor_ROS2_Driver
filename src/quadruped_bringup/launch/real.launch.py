from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # Define the package and launch file paths
    urdf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('robot_urdf'),
                'launch',
                'real_urdf.launch.py'
            ])
        ])
    )

    
    # # Define the package and launch file paths
    # hardware_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         PathJoinSubstitution([
    #             FindPackageShare('mit_mini_cheeta_motor_driver'),
    #             'launch',
    #             'controller_manager.launch.py'
    #         ])
    #     ])
    # )

    
    # Return launch description with timed execution
    return LaunchDescription([
        urdf_launch
        
        # hardware_launch
        
        # joint_state_broadcaster_spawner
        # joint_state_publisher_node
        
    ])