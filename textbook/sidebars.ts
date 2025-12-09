import type { SidebarsConfig } from '@docusaurus/plugin-content-docs';

const sidebars: SidebarsConfig = {
  tutorialSidebar: [
    'intro',

    {
      type: 'category',
      label: 'Course Overview',
      link: { type: 'doc', id: 'course-overview/index' },
      items: [
        'course-overview/physical-ai-intro',
        'course-overview/embodied-intelligence',
        'course-overview/humanoid-robotics-landscape',
        'course-overview/sensor-systems',
      ],
    },

        {
      type: 'category',
      label: 'Weekly Breakdown',
      link: { type: 'doc', id: 'weekly-breakdown/index' },
      items: [
        'weekly-breakdown/week-1',
        'weekly-breakdown/week-2',
        'weekly-breakdown/week-3',
        'weekly-breakdown/week-4',
        'weekly-breakdown/week-5',
        'weekly-breakdown/week-6',
      ],
    },

        {
      type: 'category',
      label: 'Hardware Requirements',
      link: { type: 'doc', id: 'hardware-requirements/index' },
      items: [
        'hardware-requirements/workstation',
        'hardware-requirements/edge-ai-kit',
      ],
    },

    {
      type: 'category',
      label: 'Module 1 — ROS2',
      link: { type: 'doc', id: 'module-1-ros-2/index' },
      items: [
        'module-1-ros-2/ros2-architecture',
        // 'module-1-ros-2/nodes-topics-services-actions',
        'module-1-ros-2/ros2-packages-python',
        'module-1-ros-2/launch-files-params',
        'module-1-ros-2/urdf-for-humanoids',
        'module-1-ros-2/exercises',
      ],
    },

    {
      type: 'category',
      label: 'Module 2 — Digital Twin',
      link: { type: 'doc', id: 'module-2-digital-twin-simulation-foundations/index' },
      items: [
        'module-2-digital-twin-simulation-foundations/gazebo-basics',
        'module-2-digital-twin-simulation-foundations/urdf-sdf',
        'module-2-digital-twin-simulation-foundations/physics-simulation',
        'module-2-digital-twin-simulation-foundations/exercises',
      ],
    },

    {
      type: 'category',
      label: 'Module 3 — NVIDIA Isaac',
      link: { type: 'doc', id: 'module-3-nvidia-isaac/index' },
      items: [
        'module-3-nvidia-isaac/isaac-sdk-overview',
        'module-3-nvidia-isaac/isaac-sim-setup',
        'module-3-nvidia-isaac/perception-and-manipulation',
        'module-3-nvidia-isaac/reinforcement-learning',
        'module-3-nvidia-isaac/exercises',
      ],
    },

    {
      type: 'category',
      label: 'Module 4 — VLA',
      link: { type: 'doc', id: 'module-4-vla/index' },
      items: [
        'module-4-vla/voice-to-action',
        'module-4-vla/cognitive-planning',
        'module-4-vla/exercises',
      ],
    },
    {
      type: 'category',
      label: 'Assessments',
      link: { type: 'doc', id: 'assessments/index' },
      items: [
        'assessments/ros2-project',
        'assessments/capstone-project',
      ],
    },
    {
      type: 'category',
      label: 'Capstone',
      link: { type: 'doc', id: 'capstone/index' },
      items: [
        'capstone/architecture',
        'capstone/perception',
        'capstone/manipulation',
        'capstone/complete-build-guide',
      ],
    },
    {
      type: 'category',
      label: 'Glossary',
      link: { type: 'doc', id: 'glossary/index' },
      items: [
        'glossary/robotics-terms',
        'glossary/ai-terms',
      ],
    },
  ],
};

export default sidebars;
