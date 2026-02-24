import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory("smol_bringup")
    camera_dir = get_package_share_directory("camera")

    use_sim_time = LaunchConfiguration("use_sim_time")
    base_params = LaunchConfiguration("base_params")
    hardware_params = LaunchConfiguration("hardware_params")
    model_params = LaunchConfiguration("model_params")
    mode_params = LaunchConfiguration("mode_params")
    site_params = LaunchConfiguration("site_params")
    validation_params = LaunchConfiguration("validation_params")
    watchdog_params = LaunchConfiguration("watchdog_params")
    test_mode_params = LaunchConfiguration("test_mode_params")
    camera_namespace = LaunchConfiguration("camera_namespace")
    validation_mode = LaunchConfiguration("validation_mode")
    enable_chassis_bringup = LaunchConfiguration("enable_chassis_bringup")
    enable_chassis_driver = LaunchConfiguration("enable_chassis_driver")
    enable_camera = LaunchConfiguration("enable_camera")
    enable_speech = LaunchConfiguration("enable_speech")
    enable_vla = LaunchConfiguration("enable_vla")

    params = [
        base_params,
        hardware_params,
        model_params,
        mode_params,
        site_params,
        validation_params,
        watchdog_params,
        test_mode_params,
        {"use_sim_time": use_sim_time},
    ]

    chassis_bringup = Node(
        package="chassis",
        executable="ugv_bringup",
        name="chassis_bringup",
        output="screen",
        parameters=params,
        condition=IfCondition(enable_chassis_bringup),
    )

    chassis_driver = Node(
        package="chassis",
        executable="ugv_driver",
        name="chassis_driver",
        output="screen",
        parameters=params,
        condition=IfCondition(enable_chassis_driver),
    )

    speech_node = Node(
        package="speech",
        executable="speech_node",
        name="speech_node",
        output="screen",
        parameters=params,
    )

    vla_bridge_node = Node(
        package="vla",
        executable="vla_bridge_node",
        name="vla_bridge_node",
        output="screen",
        parameters=params,
    )

    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(camera_dir, "launch", "camera.launch.py")),
        launch_arguments={"namespace": camera_namespace}.items(),
    )

    camera_group = GroupAction(
        condition=IfCondition(enable_camera),
        actions=[TimerAction(period=2.0, actions=[camera_launch])],
    )

    speech_group = GroupAction(
        condition=IfCondition(enable_speech),
        actions=[TimerAction(period=2.0, actions=[speech_node])],
    )

    vla_group = GroupAction(
        condition=IfCondition(enable_vla),
        actions=[TimerAction(period=4.0, actions=[vla_bridge_node])],
    )

    validation_group = GroupAction(
        condition=IfCondition(validation_mode),
        actions=[
            LogInfo(msg="Validation mode enabled"),
            TimerAction(
                period=6.0,
                actions=[ExecuteProcess(cmd=["ros2", "node", "list"], output="screen")],
            ),
            TimerAction(
                period=7.0,
                actions=[ExecuteProcess(cmd=["ros2", "topic", "list"], output="screen")],
            ),
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("validation_mode", default_value="false"),
            DeclareLaunchArgument("enable_chassis_bringup", default_value="true"),
            DeclareLaunchArgument("enable_chassis_driver", default_value="true"),
            DeclareLaunchArgument("enable_camera", default_value="true"),
            DeclareLaunchArgument("enable_speech", default_value="true"),
            DeclareLaunchArgument("enable_vla", default_value="true"),
            DeclareLaunchArgument(
                "base_params", default_value=os.path.join(bringup_dir, "config", "base.yaml")
            ),
            DeclareLaunchArgument(
                "hardware_params", default_value=os.path.join(bringup_dir, "config", "hardware.yaml")
            ),
            DeclareLaunchArgument(
                "model_params", default_value=os.path.join(bringup_dir, "config", "model.yaml")
            ),
            DeclareLaunchArgument(
                "mode_params", default_value=os.path.join(bringup_dir, "config", "mode.yaml")
            ),
            DeclareLaunchArgument(
                "site_params", default_value=os.path.join(bringup_dir, "config", "site.yaml")
            ),
            DeclareLaunchArgument(
                "validation_params", default_value=os.path.join(bringup_dir, "config", "validation.yaml")
            ),
            DeclareLaunchArgument(
                "watchdog_params", default_value=os.path.join(bringup_dir, "config", "watchdog.yaml")
            ),
            DeclareLaunchArgument(
                "test_mode_params", default_value=os.path.join(bringup_dir, "config", "test_mode.yaml")
            ),
            DeclareLaunchArgument("camera_namespace", default_value="camera"),
            chassis_bringup,
            chassis_driver,
            camera_group,
            speech_group,
            vla_group,
            validation_group,
        ]
    )
