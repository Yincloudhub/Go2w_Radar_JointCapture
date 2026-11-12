import os
from launch import LaunchDescription
from launch.actions import EmitEvent, ExecuteProcess, RegisterEventHandler
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # ============================================================
    # 1.启动融合里程计
    # ============================================================
    bringup_share = get_package_share_directory("go2_experiment_bringup")
    script_path = os.path.join(bringup_share,"scripts","fused_odom_service.sh")

    fused_odom = ExecuteProcess(
        cmd=[
            script_path,
            "start"
        ],
        output="screen"
    )


    # ============================================================
    # 2.Odometry串口发送节点
    # ============================================================

    odom_serial_sender = Node(
        package="odom_serial_sender",
        executable="odom_serial_sender_node",
        name="odom_serial_sender_node",
        output="screen"
    )


    # ============================================================
    # 3.摄像头录像节点
    # ============================================================

    camera_recorder = Node(
        package="camera_remote_recorder",
        executable="camera_remote_recorder_node",
        name="camera_remote_recorder_node",
        output="screen"
    )


    # ============================================================
    # 4.遥控器管理节点
    # ============================================================

    remote_manager = Node(
        package="remote_control_manager",
        executable="remote_control_manager_node",
        name="remote_control_manager_node",
        output="screen"
    )
    # ============================================================
    # 5.注册关闭事件处理器，停止融合里程计服务
    # ============================================================

    stop_fused_odom = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[
                ExecuteProcess(
                    cmd=[script_path, "stop"],
                    output="screen",
                )
            ]
        )
    )

    restart_on_driver_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=fused_odom,
            on_exit=[EmitEvent(event=Shutdown(reason="SLAM or XT16 driver exited"))],
        )
    )

    return LaunchDescription([
        fused_odom,
        odom_serial_sender,
        camera_recorder,
        remote_manager,
        stop_fused_odom,
        restart_on_driver_exit,
    ])
