import os

from isaac_ros_launch_utils.all_types import *
import isaac_ros_launch_utils as lu

from nvblox_ros_python_utils.nvblox_constants import NVBLOX_CONTAINER_NAME


def generate_launch_description() -> LaunchDescription:
    args = lu.ArgumentContainer()
    args.add_arg('run_realsense', True, cli=True)
    args.add_arg('run_alignment', True, cli=True)
    args.add_arg('run_rviz', True, cli=True)
    # Set False when no Panda is connected (desktop + camera only).
    args.add_arg('run_panda', True, cli=True)
    args.add_arg('optimize_perception', True, cli=True)
    args.add_arg('max_segmentation_in_flight', 3, cli=True)
    args.add_arg('limit_segmentation_input', True, cli=True)
    args.add_arg('mask_component_subsampling', 1, cli=True)
    args.add_arg('segmentation_resolution', '960x544',
                 choices=['424x240', '960x544'], cli=True)
    # Empty means "pick automatically from run_panda" (see _launch_setup): panda_link0
    # only exists once the franka driver is up, so a Panda-less run needs a frame the
    # camera publishes on its own instead -- camera0_link (see the realsense_splitter
    # patch's base_frame_id fix). Pass this explicitly only to override that choice.
    args.add_arg('global_frame', '', cli=True)
    args.add_arg(
        'alignment_mode', 'static', choices=['static', 'dynamic'], cli=True)
    # Real-robot deployments only run the vanilla PeopleSemSegNet model;
    # ShuffleSeg is Isaac-Sim-only (see the main README's Isaac Sim section).
    args.add_arg(
        'vanilla_engine_file_path',
        '',
        cli=True)
    args.add_arg(
        'segmentation_output_binding_names', '["argmax_1"]', cli=True)

    return LaunchDescription(
        args.get_launch_actions() + [OpaqueFunction(function=_launch_setup, args=[args])])


def _launch_setup(context, args: lu.ArgumentContainer) -> list:
    run_panda = args.run_panda.perform(context).lower() in ('true', '1')
    global_frame = args.global_frame.perform(context)
    if not global_frame:
        global_frame = 'panda_link0' if run_panda else 'camera0_link'

    width, height = map(int, args.segmentation_resolution.perform(context).split('x'))
    engine_path = args.vanilla_engine_file_path.perform(context)
    if not engine_path:
        engine_name = ('model_vanilla_v2_0_2_424x240.plan' if (width, height) == (424, 240)
                       else 'model_vanilla_v2_0_2.plan')
        engine_path = os.path.join(os.path.expanduser('~'), 'panda_real_ws', 'models',
                                   'peoplesemsegnet', 'vanilla', '1', engine_name)
    # Keep the minimum component area proportional to model image area.
    component_threshold = round(2000 * width * height / (960 * 544))
    mask_subsampling = int(args.mask_component_subsampling.perform(context))
    if mask_subsampling not in (1, 2):
        raise ValueError('mask_component_subsampling must be 1 or 2')
    optimized = args.optimize_perception.perform(context).lower() in ('true', '1')
    bounded_input = optimized and args.limit_segmentation_input.perform(context).lower() in ('true', '1')
    segmentation_image = ('/camera0/segmentation/input/image' if bounded_input
                          else '/camera0/camera/color/image_raw')
    segmentation_info = ('/camera0/segmentation/input/camera_info' if bounded_input
                         else '/camera0/camera/color/camera_info')

    actions = []
    if bounded_input:
        actions.append(
            Node(package='panda_pick_place', executable='segmentation_input_node',
                 parameters=[{'max_in_flight': int(args.max_segmentation_in_flight.perform(context))}],
                 output='screen'))
    if optimized:
        actions.append(
            Node(package='panda_pick_place', executable='native_mask_node',
                 parameters=[{'component_threshold': component_threshold,
                              'component_subsampling': mask_subsampling}], output='screen'))
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
                'input_qos': 'DEFAULT' if bounded_input else 'SENSOR_DATA',
                'num_cameras': '1',
                'namespace_list': '["camera0"]',
                'input_topic_list': f'["{segmentation_image}"]',
                'input_camera_info_topic_list':
                    f'["{segmentation_info}"]',
                'output_resized_image_topic_list':
                    '["/camera0/segmentation/image_resized"]',
                'output_resized_camera_info_topic_list':
                    '["/camera0/segmentation/camera_info_resized"]',
                'vanilla_engine_file_path': engine_path,
                'network_image_width': str(width),
                'network_image_height': str(height),
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

    nvblox_overrides = {
        'global_frame': global_frame,
        'num_cameras': 1,
        'use_lidar': False,
        'multi_mapper.connected_mask_component_size_threshold': component_threshold,
        'workspace_height_bounds_visualization_attachment_frame_id': global_frame,
    }
    if optimized:
        # native_mask_node applies the same component filter once before resizing.
        nvblox_overrides['multi_mapper.remove_small_connected_components'] = False

    nvblox_node = ComposableNode(
        name='nvblox_node',
        package='nvblox_ros',
        plugin='nvblox::NvbloxNode',
        parameters=[
            base_config,
            segmentation_config,
            realsense_config,
            nvblox_overrides,
        ],
        remappings=[
            # Read depth directly without emitter-frame splitting.
            ('camera_0/depth/image',
             '/camera0/camera/depth/image_rect_raw'),
            ('camera_0/depth/camera_info',
             '/camera0/camera/depth/camera_info'),
            ('camera_0/color/image',
             ('/camera0/camera/color/image_raw' if optimized
              else '/camera0/segmentation/image_resized')),
            ('camera_0/color/camera_info',
             ('/camera0/camera/color/camera_info' if optimized
              else '/camera0/segmentation/camera_info_resized')),
            ('camera_0/mask/image',
             ('/camera0/segmentation/people_mask_native' if optimized
              else '/camera0/segmentation/people_mask')),
            ('camera_0/mask/camera_info',
             ('/camera0/segmentation/camera_info_native' if optimized
              else '/camera0/segmentation/camera_info_resized')),
        ])
    actions.append(lu.load_composable_nodes(NVBLOX_CONTAINER_NAME, [nvblox_node]))

    if run_panda:
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
                        'voxel_frame': global_frame,
                        'voxel_template_path': str(lu.get_path(
                            'my_people_nvblox_bringup',
                            'config/panda_collision_voxels.npz')),
                    },
                ],
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

    return actions
