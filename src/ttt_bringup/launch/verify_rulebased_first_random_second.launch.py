from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory("ttt_bringup")

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bringup_share, "launch", "moveit_ik.launch.py")
                )
            ),
            Node(
                package="ttt_referee",
                executable="ttt_referee_node",
                name="ttt_referee",
                output="screen",
            ),
            Node(
                package="ttt_engine",
                executable="ttt_engine_node",
                name="ttt_engine",
                output="screen",
            ),
            Node(
                package="ttt_visualizer",
                executable="ttt_visualizer_node",
                name="ttt_visualizer",
                output="screen",
            ),
            Node(
                package="ttt_dummy",
                executable="rule_based_player_node",
                name="player_0_rule_based",
                output="screen",
                parameters=[
                    {
                        "player_name": "RuleBased",
                        "plan_turn_service": "/player_0_rule_based/plan_turn",
                    }
                ],
            ),
            TimerAction(
                period=1.0,
                actions=[
                    Node(
                        package="ttt_dummy",
                        executable="random_player_node",
                        name="player_1_random",
                        output="screen",
                        parameters=[
                            {
                                "player_name": "Random",
                                "plan_turn_service": "/player_1_random/plan_turn",
                                "seed": 0,
                            }
                        ],
                    )
                ],
            ),
        ]
    )
