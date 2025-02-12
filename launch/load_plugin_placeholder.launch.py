from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import os

from launch.substitutions import FindExecutable
from launch.actions import ExecuteProcess


def generate_launch_description():
    ld = LaunchDescription()

    # Get the placeholder sdf file
    sdf_file = os.path.join(get_package_share_directory('hector_gazebo_plugins'), 'plugin_placeholder_sdf', 'plugin_placeholder.sdf')

    call_entitity_creation_srv= ExecuteProcess(
        cmd=[
            FindExecutable(name='gz'),
            "service",
            "-s", "/world/default/create",
            "--reqtype", "gz.msgs.EntityFactory",
            "--reptype", "gz.msgs.Boolean",
            "-r", '\'sdf_filename:"' + sdf_file + '"\''
        ],
        shell=True
    )

    ld.add_action(call_entitity_creation_srv)
    return ld

