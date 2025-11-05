from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """Launch GPS/RTK node for M300 RTK."""

    gps_rtk_node = Node(
        package='dji_m300_gps_rtk',
        executable='gps_rtk_node',
        name='gps_rtk_node',
        output='screen',
        parameters=[
            {'use_sim_time': False},
        ],
        remappings=[
            ('/m300/gps/fix', '/m300/gps/fix'),
            ('/m300/rtk/fix', '/m300/rtk/fix'),
        ]
    )

    return LaunchDescription([
        gps_rtk_node,
    ])
