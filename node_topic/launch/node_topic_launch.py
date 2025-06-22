from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    return LaunchDescription([
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='node_topic',
                    executable='subscriber',
                    name='subscriber',
                    output='screen',
                    namespace='robot'
                )
            ]
        ),
        Node(
            package='node_topic',
            executable='publisher',
            name='publisher',
            output='screen',
            remappings=[
                ('/topic', '/robot1/topic')
            ]
        )
    ])