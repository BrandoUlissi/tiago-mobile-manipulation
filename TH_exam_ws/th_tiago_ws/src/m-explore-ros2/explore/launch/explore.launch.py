import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument, TimerAction
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    ld = LaunchDescription()

    # Percorso del file di configurazione
    config = os.path.join(
        get_package_share_directory("explore_lite"), "config", "params.yaml"
    )

    # Argomenti di lancio
    use_sim_time = LaunchConfiguration("use_sim_time")
    namespace = LaunchConfiguration("namespace")

    declare_use_sim_time_argument = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use simulation/Gazebo clock"
    )
    declare_namespace_argument = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Namespace for the explore node",
    )

    # Remapping dei topic
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    # Nodo explore_lite
    explore_node = Node(
        package="explore_lite",
        name="explore_node",
        namespace=namespace,
        executable="explore",
        parameters=[config, {"use_sim_time": use_sim_time}],
        output="screen",
        remappings=remappings,
    )

    # Comando per pubblicare su /explore/resume con data = false (solo una volta)
    pub_resume_false = ExecuteProcess(
        cmd=["ros2", "topic", "pub", "--once", "/explore/resume", "std_msgs/msg/Bool", "{data: false}"],
        output="screen",
    )

    # Timer per ritardare la pubblicazione del messaggio (opzionale)
    delayed_pub_resume_false = TimerAction(
        period=0.5,  # Ritardo di 0.5 secondi
        actions=[pub_resume_false],
    )

    # Aggiungi le azioni al LaunchDescription
    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_namespace_argument)
    ld.add_action(explore_node)
    ld.add_action(delayed_pub_resume_false)  # Aggiungi il timer per la pubblicazione

    return ld