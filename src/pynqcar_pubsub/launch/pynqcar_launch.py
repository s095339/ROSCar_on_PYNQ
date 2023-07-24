from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pynqcar_pubsub',
            executable='Driver',
            name='Driver'
        ),
        Node(
            package='pynqcar_pubsub',
            executable='listen',
            name='listen'
        ),
        Node(
            package='pynqcar_pubsub',
            executable='Car',
            name='Car',
            #remappings=[
            #    ('/', '/turtlesim1/turtle1/pose'),
            #]
        )
    ])
