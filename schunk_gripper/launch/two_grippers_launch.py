from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='schunk_gripper', 
            executable='alice_gripper',
            name='Alice_Subscriber'
        ),

        Node(
            package='schunk_gripper', 
            executable='bob_gripper',
            name='Bob_Subscriber'
        ),
    ])