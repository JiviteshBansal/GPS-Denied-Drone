#!/usr/bin/env python3
"""
Launch file for Gazebo Harmonic with the indoor Curved Gridroom facility.
Spawns 3 Iris quadcopters at:
- drone_1 @ (-4.0, -6.0)
- drone_2 @ ( 0.0, -6.0)
- drone_3 @ ( 4.0, -6.0)

Track: Multi-Drone Simulation, Physics Environment & Cloud Deployment
"""

import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    simulation_dir = Path(__file__).parent.parent.absolute()
    worlds_dir = simulation_dir / "worlds"
    models_dir = simulation_dir / "models"

    world_file = worlds_dir / "curved_gridroom.sdf"
    gazebo_model_path = str(models_dir)

    world_arg = DeclareLaunchArgument(
        "world",
        default_value=str(world_file),
        description="Path to the curved gridroom Gazebo world file",
    )

    verbose_arg = DeclareLaunchArgument(
        "verbose",
        default_value="false",
        description="Enable verbose output for Gazebo Harmonic",
    )

    gazebo_launch = ExecuteProcess(
        cmd=[
            "gz",
            "sim",
            "-r",
            LaunchConfiguration("world"),
        ],
        output="screen",
        additional_env={
            "GZ_SIM_RESOURCE_PATH": gazebo_model_path,
        },
    )

    return LaunchDescription([
        world_arg,
        verbose_arg,
        gazebo_launch,
    ])
