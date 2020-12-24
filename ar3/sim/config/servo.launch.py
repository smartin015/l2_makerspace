import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

YAML_PATH = 'servo.yaml'
URDF_PATH = 'ar2.urdf'
SRDF_PATH = 'ar2.srdf'

def generate_launch_description():
    # TODO build configs into package
    # package_path = get_package_share_directory('l2_ar3')
    package_path = '/volume/config/'

    # TODO moveit yaml file
    # TODO kinematics yaml file
    # TODO Controllers yaml file

    # Get parameters for the Servo node
    with open(os.path.join(package_path, YAML_PATH), 'r') as f:
        servo_yaml = yaml.safe_load(f)
    servo_params = {'moveit_servo' : servo_yaml}

    # Get URDF and SRDF
    with open(os.path.join(package_path, URDF_PATH), 'r') as f:
        robot_description_config = f.read()
    robot_description = {'robot_description' : robot_description_config}

    with open(os.path.join(package_path, SRDF_PATH), 'r') as f:
        robot_description_semantic_config = f.read()
    robot_description_semantic = {'robot_description_semantic' : robot_description_semantic_config}

    # Launch as much as possible in components
    container = ComposableNodeContainer(
            name='ar2_servo',
            namespace='/',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='robot_state_publisher',
                    plugin='robot_state_publisher::RobotStatePublisher',
                    name='robot_state_publisher',
                    parameters=[robot_description, {'publish_frequency': 20.0}],
                    extra_arguments=[{'--ros-args': '--log-level DEBUG'}]),
                #ComposableNode(
                #    package='tf2_ros',
                #    plugin='tf2_ros::StaticTransformBroadcasterNode',
                #    name='static_tf2_broadcaster',
                #    parameters=[ {'/child_frame_id' : 'base_link', '/frame_id' : 'world'} ]),
                ComposableNode(
                    package='moveit_servo',
                    plugin='moveit_servo::ServoServer',
                    name='servo_server',
                    parameters=[servo_params, robot_description, robot_description_semantic],
                    extra_arguments=[{'use_intra_process_comms' : True}])
            ],
            output='screen',
    )
    
    return LaunchDescription([container])

