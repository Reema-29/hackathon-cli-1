// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  bookSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Chapters',
      items: [
        'ros2_nervous_system/concepts',
        'ros2_nervous_system/architecture',
        'ros2_nervous_system/jetson_integration',
        'ros2_nervous_system/lab_exercises',
        'ros2_nervous_system/quiz'
      ],
    },
    {
      type: 'category',
      label: 'Research',
      items: ['research/concurrent', 'research/validation'],
    },
    {
      type: 'category',
      label: 'AI Integration',
      items: ['ai/generation', 'ai/validation'],
    },
    {
      type: 'category',
      label: 'Citation Management',
      items: ['citations/apa', 'citations/validation'],
    },
    {
      type: 'category',
      label: 'Digital Twin (Gazebo & Unity)',
      items: [
        'digital-twin/physics-simulation',
        'digital-twin/unity-digital-twins',
        'digital-twin/sensor-simulation'
      ],
    },
  ],
};

module.exports = sidebars;