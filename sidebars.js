// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'ROS 2 Basics',
      items: [
        'modules/ros2-basics/introduction',
        'modules/ros2-basics/nodes',
        'modules/ros2-basics/topics',
        'modules/ros2-basics/services',
        'modules/ros2-basics/examples',
        'modules/ros2-basics/exercises',
        'modules/ros2-basics/assessment'
      ],
      link: {
        type: 'generated-index',
        title: 'ROS 2 Basics',
        description: 'Learn the fundamental concepts of ROS 2 communication architecture',
        slug: '/modules/ros2-basics'
      }
    },
    {
      type: 'category',
      label: 'AI-to-Robot Control',
      items: [
        'modules/ai-robot-control/perception',
        'modules/ai-robot-control/decision-making',
        'modules/ai-robot-control/action',
        'modules/ai-robot-control/rclpy-integration',
        'modules/ai-robot-control/examples',
        'modules/ai-robot-control/exercises',
        'modules/ai-robot-control/assessment'
      ],
      link: {
        type: 'generated-index',
        title: 'AI-to-Robot Control',
        description: 'Connect Python AI agents to ROS 2 nodes',
        slug: '/modules/ai-robot-control'
      }
    },
    {
      type: 'category',
      label: 'URDF Modeling',
      items: [
        'modules/urdf-modeling/introduction',
        'modules/urdf-modeling/links',
        'modules/urdf-modeling/joints',
        'modules/urdf-modeling/kinematic-chains',
        'modules/urdf-modeling/ros2-integration',
        'modules/urdf-modeling/examples',
        'modules/urdf-modeling/exercises',
        'modules/urdf-modeling/assessment'
      ],
      link: {
        type: 'generated-index',
        title: 'URDF Modeling',
        description: 'Model humanoid robots using URDF and integrate with ROS 2',
        slug: '/modules/urdf-modeling'
      }
    }
  ],
};

module.exports = sidebars;