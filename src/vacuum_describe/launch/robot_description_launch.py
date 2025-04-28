import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    pkg_name = 'vacuum_describe'
    share_dir = get_package_share_directory(pkg_name)

    # Use simulation time (standard practice for ROS launch files)
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # URDF file path
    urdf_file_name = 'vacuum.urdf'
    urdf_path = os.path.join(share_dir, 'urdf', urdf_file_name)

    # Load the URDF file content
    # Using xacro just in case you decide to use it later, otherwise reads URDF directly
    try:
        robot_desc_raw = xacro.process_file(urdf_path).toxml()
    except Exception as e:
        print(f"Error processing URDF/Xacro file: {e}")
        # Fallback to reading directly if xacro fails or isn't needed
        try:
            with open(urdf_path, 'r') as infp:
                robot_desc_raw = infp.read()
        except FileNotFoundError:
             print(f"Error: URDF file not found at {urdf_path}")
             return None # Or raise an error
        except Exception as e:
             print(f"Error reading URDF file: {e}")
             return None # Or raise an error


    # Robot State Publisher Node
    # Publishes TF transforms based on URDF and /joint_states
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc_raw,
            'use_sim_time': use_sim_time
        }]
    )

    # Joint State Publisher Node (Optional but good practice)
    # Publishes default values for non-fixed joints if any exist.
    # If you add non-fixed joints later (like wheels), you might need joint_state_publisher_gui
    # or a dedicated node to publish actual wheel joint states.
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
        # condition=UnlessCondition(LaunchConfiguration('gui')) # Example if adding GUI option
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        # DeclareLaunchArgument( # Example if adding GUI option
        #     'gui',
        #     default_value='false',
        #     description='Flag to enable joint_state_publisher_gui'),

        joint_state_publisher_node, # Publish default joint states (for non-fixed joints)
        robot_state_publisher_node, # Publish TF transforms
    ])
