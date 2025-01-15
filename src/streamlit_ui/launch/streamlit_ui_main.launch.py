import os
import launch
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    publisher_node = Node(
        package = 'streamlit_ui',
        executable = 'streamlit_publisher',
        name = 'streamlit_publisher'
    )

    subscriber_node = Node(
        package = 'streamlit_ui',
        executable = 'streamlit_subscriber',
        name = 'streamlit_subscriber'
    )

    agent_node = Node(
        package = 'openai_ros2',
        executable = 'agent',
        name = 'agent_node'
    )

    spawn_streamlit = ExecuteProcess(
        cmd = [[
            os.path.join(os.path.expanduser('~'), '.local/bin/streamlit'),
            ' run ',
            os.path.join(os.path.expanduser('~'), 'ros2_ws/src',
                'streamlit_ui/streamlit_ui/streamlit_ui.py')
        ]],
        shell=True
    )

    return launch.LaunchDescription([
        spawn_streamlit,
        publisher_node,
        subscriber_node,
        agent_node
    ])
