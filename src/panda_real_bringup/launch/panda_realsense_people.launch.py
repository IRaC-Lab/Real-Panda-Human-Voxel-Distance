import os

from isaac_ros_launch_utils.all_types import *
import isaac_ros_launch_utils as lu

from nvblox_ros_python_utils.nvblox_constants import NVBLOX_CONTAINER_NAME


def generate_launch_description() -> LaunchDescription:
    args = lu.ArgumentContainer()
    args.add_arg('run_realsense', True, cli=True)
    args.add_arg('run_alignment', True, cli=True)
    args.add_arg('run_rviz', True, cli=True)
    # Set False when no Panda is connected (desktop + camera only). Also drives
    # the default global_frame below, since panda_link0 only exists when the
    # franka driver is running.
    args.add_arg('run_panda', True, cli=True)
    args.add_arg('global_frame', 'panda_link0', cli=True)
    args.add_arg(
        'alignment_mode', 'static', choices=['static', 'dynamic'], cli=True)
    # Real-robot deployments only run the vanilla PeopleSemSegNet model;
    # ShuffleSeg is Isaac-Sim-only (see the main README's Isaac Sim section).
    args.add_arg(
        'vanilla_engine_file_path',
        os.path.join(os.path.expanduser('~'), 'panda_real_ws', 'models',
                     'peoplesemsegnet', 'vanilla', '1', 'model_vanilla_v2_0_2.plan'),
        cli=True)
    args.add_arg(
        'segmentation_output_binding_names', '["argmax_1"]', cli=True)

    actions = args.get_launch_actions()
    actions.append(lu.component_container(
        NVBLOX_CONTAINER_NAME, condition=IfCondition(args.run_realsense)))

    actions.append(
        lu.include(
            'nvblox_examples_bringup',
            'launch/sensors/realsense.launch.py',
            launch_arguments={
                'container_name': NVBLOX_CONTAINER_NAME,
                'num_cameras': '1',
            },
            condition=IfCondition(args.run_realsense)))

    actions.append(
        lu.include(
            'panda_camera_alignment',
            'launch/aruco_align.launch.py',
            launch_arguments={
                'mode': args.alignment_mode,
                'camera_mount_frame': 'camera0_link',
                'show_image': 'False',
            },
            condition=IfCondition(args.run_alignment)))

    actions.append(
        lu.include(
            'nvblox_examples_bringup',
            'launch/perception/segmentation.launch.py',
            launch_arguments={
                'people_segmentation': 'peoplesemsegnet_vanilla',
                'num_cameras': '1',
                'namespace_list': '["camera0"]',
                'input_topic_list': '["/camera0/camera/color/image_raw"]',
                'input_camera_info_topic_list':
                    '["/camera0/camera/color/camera_info"]',
                'output_resized_image_topic_list':
                    '["/camera0/segmentation/image_resized"]',
                'output_resized_camera_info_topic_list':
                    '["/camera0/segmentation/camera_info_resized"]',
                'vanilla_engine_file_path': args.vanilla_engine_file_path,
                'output_binding_names':
                    args.segmentation_output_binding_names,
                'one_container_per_camera': 'True',
            }))

    base_config = lu.get_path(
        'my_people_nvblox_bringup', 'config/nvblox/nvblox_base.yaml')
    segmentation_config = lu.get_path(
        'my_people_nvblox_bringup',
        'config/nvblox/specializations/nvblox_segmentation.yaml')
    realsense_config = lu.get_path(
        'my_people_nvblox_bringup',
        'config/nvblox/specializations/nvblox_realsense.yaml')

    nvblox_node = ComposableNode(
        name='nvblox_node',
        package='nvblox_ros',
        plugin='nvblox::NvbloxNode',
        parameters=[
            base_config,
            segmentation_config,
            realsense_config,
            {
                'global_frame': args.global_frame,
                'num_cameras': 1,
                'use_lidar': False,
                'workspace_height_bounds_visualization_attachment_frame_id':
                    args.global_frame,
            },
        ],
        remappings=[
            ('camera_0/depth/image',
             '/camera0/realsense_splitter_node/output/depth'),
            ('camera_0/depth/camera_info',
             '/camera0/camera/depth/camera_info'),
            ('camera_0/color/image',
             '/camera0/segmentation/image_resized'),
            ('camera_0/color/camera_info',
             '/camera0/segmentation/camera_info_resized'),
            ('camera_0/mask/image', '/camera0/segmentation/people_mask'),
            ('camera_0/mask/camera_info',
             '/camera0/segmentation/camera_info_resized'),
        ])
    actions.append(lu.load_composable_nodes(NVBLOX_CONTAINER_NAME, [nvblox_node]))

    panda_config = lu.get_path(
        'my_people_nvblox_bringup', 'config/panda_spheres.yaml')
    actions.append(
        Node(
            package='my_people_nvblox_bringup',
            executable='panda_voxel_classifier',
            name='panda_voxel_classifier',
            parameters=[
                panda_config,
                {
                    'voxel_frame': args.global_frame,
                    'voxel_template_path': str(lu.get_path(
                        'my_people_nvblox_bringup',
                        'config/panda_collision_voxels.npz')),
                },
            ],
            condition=IfCondition(args.run_panda),
            output='screen'))
    actions.append(
        Node(
            package='my_people_nvblox_bringup',
            executable='closest_panda_human_voxels',
            name='closest_panda_human_voxels',
            parameters=[
                panda_config,
                {'camera_info_topic': '/camera0/camera/color/camera_info'},
            ],
            condition=IfCondition(args.run_panda),
            output='screen'))

    actions.append(
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', str(lu.get_path(
                'panda_real_bringup',
                'config/panda_realsense_people.rviz'))],
            condition=IfCondition(args.run_rviz),
            output='screen'))

    return LaunchDescription(actions)
