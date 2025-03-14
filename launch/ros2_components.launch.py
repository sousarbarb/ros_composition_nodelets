import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
  """One publisher and multiple subscribers."""
  container = ComposableNodeContainer(
      name='components_container',
      namespace='',
      package='rclcpp_components',
      executable='component_container',
      composable_node_descriptions=[
          ComposableNode(
              package='ros_composition_nodelets',
              plugin='ros_composition_nodelets::PubROS2',
              name='pub0'),
          ComposableNode(
              package='ros_composition_nodelets',
              plugin='ros_composition_nodelets::SubROS2',
              name='sub1'),
          ComposableNode(
              package='ros_composition_nodelets',
              plugin='ros_composition_nodelets::SubROS2',
              name='sub2')
      ],
      output='screen',
  )

  return launch.LaunchDescription([container])
