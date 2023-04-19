import launch
import launch_ros.actions
import os
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    dialogflow_dir = get_package_share_directory('dialogflow_ros2')


    soundplay_node = launch_ros.actions.Node(
        package='sound_play',
        executable='soundplay_node.py',
        output='screen'
    )

    dialogflow_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                                            dialogflow_dir, 'launch/',
                                            'dialogflow.launch.py')),
            launch_arguments={
                'config_file': config_file,
            }.items())

    return launch.LaunchDescription([soundplay_node, dialogflow_launch])