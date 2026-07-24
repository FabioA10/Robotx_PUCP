import launch
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    container = ComposableNodeContainer(
        name='perception_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='usv_perception',
                plugin='usv_perception::CameraNode',
                name='camera_node',
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            ComposableNode(
                package='usv_perception',
                plugin='usv_perception::ProcessingNode',
                name='processing_node',
                extra_arguments=[{'use_intra_process_comms': True}]
            )
        ],
        output='screen'
    )

    return LaunchDescription([container])
