import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, TextSubstitution, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackagePrefix


def generate_launch_description():

    # Common arguments
    sensor = DeclareLaunchArgument(name='sensor',
                                   default_value='S21',
                                   description='Type of multisense: S21, SL, S7, S7S, S27, S30, KS21, KS21i')

    mtu = DeclareLaunchArgument(name='mtu',
                                default_value='1500',
                                description='Sensor MTU')

    fps = DeclareLaunchArgument(name='fps',
                                default_value="15.0",
                                description='Sensor frame rate per seconds')

    use_sensor_qos = DeclareLaunchArgument(name='use_sensor_qos',
                                           default_value='False',
                                           description='Use the sensor data QoS for publishing')

    launch_robot_state_publisher = DeclareLaunchArgument(name='launch_robot_state_publisher',
                                                         default_value='True',
                                                         description='Launch the robot_state_publisher')

    # Entry sensor specific arguments
    launch_entry_sensor = DeclareLaunchArgument(name='launch_entry_sensor',
                                                default_value='True',
                                                description='Launch the entry sensor')

    entry_namespace = DeclareLaunchArgument(name='entry_namespace',
                                            default_value='multisense_entry',
                                            description='Namespace for the entry MultiSense instance')

    entry_ip_address = DeclareLaunchArgument(name='entry_ip_address',
                                             default_value='10.66.171.21',
                                             description='Entry sensor IP address')

    # Exit sensor specific arguments
    launch_exit_sensor = DeclareLaunchArgument(name='launch_exit_sensor',
                                               default_value='True',
                                               description='Launch the exit sensor')

    exit_namespace = DeclareLaunchArgument(name='exit_namespace',
                                           default_value='multisense_exit',
                                           description='Namespace for the exit MultiSense instance')

    exit_ip_address = DeclareLaunchArgument(name='exit_ip_address',
                                            default_value='10.66.170.21',
                                            description='Exit sensor IP address')

    # Entry sensor node
    multisense_ros_entry = Node(package='multisense_ros',
                                namespace=[LaunchConfiguration('entry_namespace')],
                                executable='ros_driver',
                                condition=IfCondition(LaunchConfiguration('launch_entry_sensor')),
                                parameters=[{'sensor_ip': LaunchConfiguration('entry_ip_address'),
                                             'sensor_mtu': LaunchConfiguration('mtu'),
                                             'tf_prefix': LaunchConfiguration('entry_namespace'),
                                             'fps': LaunchConfiguration('fps'),
                                             'use_sensor_qos': LaunchConfiguration('use_sensor_qos')}])

    # Exit sensor node
    multisense_ros_exit = Node(package='multisense_ros',
                               namespace=[LaunchConfiguration('exit_namespace')],
                               executable='ros_driver',
                               condition=IfCondition(LaunchConfiguration('launch_exit_sensor')),
                               parameters=[{'sensor_ip': LaunchConfiguration('exit_ip_address'),
                                            'sensor_mtu': LaunchConfiguration('mtu'),
                                            'tf_prefix': LaunchConfiguration('exit_namespace'),
                                            'fps': LaunchConfiguration('fps'),
                                            'use_sensor_qos': LaunchConfiguration('use_sensor_qos')}])

    # Entry robot state publisher
    robot_state_publisher_entry = Node(package='robot_state_publisher',
                                       executable='robot_state_publisher',
                                       namespace=[LaunchConfiguration('entry_namespace')],
                                       condition=IfCondition(LaunchConfiguration('launch_entry_sensor')),
                                       parameters=[{'robot_description': Command([
                                                       PathJoinSubstitution([FindPackagePrefix('xacro'), 'bin', 'xacro ']),
                                                       PathJoinSubstitution([get_package_share_directory('multisense_ros'),
                                                                             'urdf',
                                                                             LaunchConfiguration('sensor'),
                                                                             'standalone.urdf.xacro']),
                                                       " name:=", LaunchConfiguration('entry_namespace')])}])

    # Exit robot state publisher
    robot_state_publisher_exit = Node(package='robot_state_publisher',
                                      executable='robot_state_publisher',
                                      namespace=[LaunchConfiguration('exit_namespace')],
                                      condition=IfCondition(LaunchConfiguration('launch_exit_sensor')),
                                      parameters=[{'robot_description': Command([
                                                      PathJoinSubstitution([FindPackagePrefix('xacro'), 'bin', 'xacro ']),
                                                      PathJoinSubstitution([get_package_share_directory('multisense_ros'),
                                                                            'urdf',
                                                                            LaunchConfiguration('sensor'),
                                                                            'standalone.urdf.xacro']),
                                                      " name:=", LaunchConfiguration('exit_namespace')])}])

    # Static TF for entry sensor
    static_tf_entry = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_multisense_entry',
        namespace=[LaunchConfiguration('entry_namespace')],
        condition=IfCondition(LaunchConfiguration('launch_entry_sensor')),
        arguments=['-0.035', '0', '1.0', '0.0', '0.9', '0.0', 'base_link', 'multisense_entry/head']
    )

    # Static TF for exit sensor
    static_tf_exit = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_multisense_exit',
        namespace=[LaunchConfiguration('exit_namespace')],
        condition=IfCondition(LaunchConfiguration('launch_exit_sensor')),
        arguments=['2.01', '0.02', '1.0', '3.1415', '0.89', '0.0', 'base_link', 'multisense_exit/head']
    )

    return LaunchDescription([sensor,
                              mtu,
                              fps,
                              use_sensor_qos,
                              launch_robot_state_publisher,
                              launch_entry_sensor,
                              entry_namespace,
                              entry_ip_address,
                              launch_exit_sensor,
                              exit_namespace,
                              exit_ip_address,
                              multisense_ros_entry,
                              multisense_ros_exit,
                              robot_state_publisher_entry,
                               robot_state_publisher_exit,
                               static_tf_entry,
                               static_tf_exit])