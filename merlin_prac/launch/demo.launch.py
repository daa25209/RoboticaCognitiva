
import os

from launch import LaunchDescription 
from launch.substitutions import LaunchConfiguration
from launch.actions import TimerAction, DeclareLaunchArgument, SetEnvironmentVariable
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import ament_index_python
from kant_dao.dao_factory import DaoFamilies
from merlin2_planner import Merlin2Planners
from launch_ros.actions import Node

def generate_launch_description():

    planning_layer_share_dir = get_package_share_directory(
        "merlin2_planning_layer")

    waypoint_navigation_share_dir = get_package_share_directory(
    "waypoint_navigation")

    prac_share_dir = get_package_share_directory(
        "merlin_prac")

    speech_to_text_share_dir = get_package_share_directory(
        "speech_to_text")
    text_to_speech_share_dir = get_package_share_directory(
        "text_to_speech")

    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED", "1")


    rb1_sbox_dir = get_package_share_directory(
        "rb1_sandbox")
   

    dao_family = LaunchConfiguration("dao_family")
    dao_family_cmd = DeclareLaunchArgument(
        "dao_family",
        default_value=str(int(DaoFamilies.ROS2)),
        description="DAO family")

    

    planner = LaunchConfiguration("planner")
    planner_cmd = DeclareLaunchArgument(
        "planner",
        default_value=str(int(Merlin2Planners.POPF)),
        description="PDDL planner")

  

    topological_nav_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(waypoint_navigation_share_dir, "waypoint_navigation.launch.py")),
        launch_arguments={"wps": ament_index_python.get_package_share_directory(
            "merlin_prac") + "/rsc/wps.yaml"}.items()
    )

    speech_to_text_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(speech_to_text_share_dir, "speech_to_text.launch.py"))
    )

    text_to_speech_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(text_to_speech_share_dir, "text_to_speech.launch.py"))
    )

    merlin2_planning_layer_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(planning_layer_share_dir, "merlin2_planning_layer.launch.py")),
        launch_arguments={"dao_family": dao_family,
                          "planner": planner}.items()
    )


    hospital_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rb1_sbox_dir, "launch/hospital.launch.py")))

    mision_node = Node(
            package='merlin_prac',
            namespace='merlin_prac_nam',
            executable='mision_node.py',
            name='mision_node'
        )

    action_node = Node(
            package='merlin_prac',
            namespace='merlin_prac_nam',
            executable='nav_action_node.py',
            name='action_node'
        )

    ld = LaunchDescription()

 

    ld.add_action(hospital_launch_cmd)
    ld.add_action(TimerAction(period=10.0, actions=[stdout_linebuf_envvar]))

    ld.add_action(topological_nav_cmd)
    ld.add_action(dao_family_cmd)
    ld.add_action(planner_cmd)
    
    ld.add_action(merlin2_planning_layer_cmd)


    return ld
