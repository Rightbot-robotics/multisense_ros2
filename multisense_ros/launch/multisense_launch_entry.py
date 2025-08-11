import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, TextSubstitution, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackagePrefix


def generate_launch_description():

    sensor = DeclareLaunchArgument(name='sensor',
                                   default_value='S21',
                                   description='Type of multisense: S21, SL, S7, S7S, S27, S30, KS21, KS21i')

    namespace = DeclareLaunchArgument(name='namespace',
                                      default_value='multisense_entry',
                                      description='Namespace for this MultiSense instance')

    mtu = DeclareLaunchArgument(name='mtu',
                                default_value='1500',
                                description='Sensor MTU')

    ip_address = DeclareLaunchArgument(name='ip_address',
                                       default_value='10.66.171.21',
                                       description='Sensor IP address')

    fps = DeclareLaunchArgument(name='fps',
                                default_value="10.0",
                                description='Sensor frame rate per seconds')

    use_sensor_qos = DeclareLaunchArgument(name='use_sensor_qos',
                                           default_value='False',
                                           description='Use the sensor data QoS for publishing')

    launch_robot_state_publisher = DeclareLaunchArgument(name='launch_robot_state_publisher',
                                                         default_value='True',
                                                         description='Launch the robot_state_publisher')

    multisense_ros = Node(package='multisense_ros',
                         namespace=[LaunchConfiguration('namespace')],
                         executable='ros_driver',
                         parameters=[{'sensor_ip': LaunchConfiguration('ip_address'),
                                      'sensor_mtu': LaunchConfiguration('mtu'),
                                      'tf_prefix': LaunchConfiguration('namespace'),
                                      'fps': LaunchConfiguration('fps'),
                                      'use_sensor_qos': LaunchConfiguration('use_sensor_qos')}])

    robot_state_publisher = Node(package='robot_state_publisher',
                                 executable='robot_state_publisher',
                                 namespace=[LaunchConfiguration('namespace')],
                                 condition=IfCondition(LaunchConfiguration('launch_robot_state_publisher')),
                                 parameters=[{'robot_description': Command([
                                             PathJoinSubstitution([FindPackagePrefix('xacro'), 'bin', 'xacro ']),
                                             PathJoinSubstitution([get_package_share_directory('multisense_ros'),
                                                                   'urdf',
                                                                   LaunchConfiguration('sensor'),
                                                                   'standalone.urdf.xacro']),
                                             " name:=", LaunchConfiguration('namespace')])}])
    # Static TF for entry sensor
    static_tf_entry = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_multisense_entry',
        namespace=[LaunchConfiguration('entry_namespace')],
        condition=IfCondition(LaunchConfiguration('launch_entry_sensor')),
        arguments=['-0.035', '0', '1.0', '0.0', '0.9', '0.0', 'base_link', 'multisense_entry/head']
    )

    return LaunchDescription([sensor,
                              namespace,
                              mtu,
                              ip_address,
                              fps,
                              use_sensor_qos,
                              launch_robot_state_publisher,
                              multisense_ros,
                              robot_state_publisher,
                              static_tf_entry])
