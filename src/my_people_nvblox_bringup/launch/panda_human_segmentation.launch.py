# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

from isaac_ros_launch_utils.all_types import *
import isaac_ros_launch_utils as lu

from nvblox_ros_python_utils.nvblox_launch_utils import NvbloxMode, NvbloxCamera
from nvblox_ros_python_utils.nvblox_constants import NVBLOX_CONTAINER_NAME


def generate_launch_description() -> LaunchDescription:
    args = lu.ArgumentContainer()
    args.add_arg('log_level', 'info', choices=['debug', 'info', 'warn'], cli=True)
    args.add_arg(
        'mode',
        NvbloxMode.static,
        choices=NvbloxMode.names(),
        description='The nvblox mode.',
        cli=True)
    args.add_arg(
        'num_cameras',
        3,
        choices=['0', '1', '3'],
        description='Number of cameras that should be used for 3d reconstruction',
        cli=True)
    args.add_arg(
        'run_rviz',
        True,
        description='Whether to run RViz.',
        cli=True)
    args.add_arg(
        'run_panda_voxel_classifier',
        True,
        description='Publish Panda collision-mesh voxels from link TFs.',
        cli=True)
    args.add_arg(
        'run_panda_debug_rviz',
        True,
        description='Run a second RViz for Panda sphere placement.',
        cli=True)
    args.add_arg(
        'run_closest_panda_human',
        True,
        description='Compute and visualize the closest Panda/human voxel pair.',
        cli=True)

    actions = args.get_launch_actions()

    # Globally set use_sim_time
    actions.append(SetParameter('use_sim_time', True))

    # Isaac Sim publishes the camera tree below odom while Panda is below map.
    # Keep the camera and Panda perception trees connected.
    actions.append(lu.static_transform('map', 'odom'))

    # Container
    actions.append(
        lu.component_container(
            NVBLOX_CONTAINER_NAME, container_type='isolated', log_level=args.log_level))

    # Nvblox
    actions.append(
        lu.include(
            'my_people_nvblox_bringup',
            'launch/perception/nvblox.launch.py',
            launch_arguments={
                'container_name': NVBLOX_CONTAINER_NAME,
                'mode': args.mode,
                'camera': NvbloxCamera.isaac_sim,
                'num_cameras': args.num_cameras,
                'lidar': False,
            }))

    # Panda collision-mesh voxels transformed directly by the latest link TFs.
    actions.append(
        Node(
            package='my_people_nvblox_bringup',
            executable='panda_voxel_classifier',
            name='panda_voxel_classifier',
            parameters=[
                lu.get_path(
                    'my_people_nvblox_bringup', 'config/panda_spheres.yaml'),
                {'voxel_template_path': str(lu.get_path(
                    'my_people_nvblox_bringup',
                    'config/panda_collision_voxels.npz'))},
            ],
            output='screen',
            condition=IfCondition(args.run_panda_voxel_classifier)))

    actions.append(
        Node(
            package='rviz2',
            executable='rviz2',
            name='panda_sphere_debug_rviz',
            arguments=['-d', lu.get_path(
                'my_people_nvblox_bringup',
                'config/visualization/panda_sphere_debug.rviz')],
            output='screen',
            condition=IfCondition(args.run_panda_debug_rviz)))

    actions.append(
        Node(
            package='my_people_nvblox_bringup',
            executable='closest_panda_human_voxels',
            name='closest_panda_human_voxels',
            parameters=[lu.get_path(
                'my_people_nvblox_bringup', 'config/panda_spheres.yaml')],
            output='screen',
            condition=IfCondition(args.run_closest_panda_human)))

    # Visualization
    actions.append(
        lu.include(
            'my_people_nvblox_bringup',
            'launch/visualization/visualization.launch.py',
            launch_arguments={
                'mode': args.mode,
                'camera': NvbloxCamera.isaac_sim,
                'run_rviz': args.run_rviz,
            }))

    return LaunchDescription(actions)
