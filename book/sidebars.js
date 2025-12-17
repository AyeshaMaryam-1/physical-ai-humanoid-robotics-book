// @ts-check

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.

 @type {import('@docusaurus/plugin-content-docs').SidebarsConfig}
 */
const sidebars = {
  // Manual sidebar structure for the Physical AI & Humanoid Robotics book
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Front Matter',
      collapsible: true,
      collapsed: false,
      items: [
        'front_matter/preface',
        'front_matter/how_to_use',
        'front_matter/requirements',
        'front_matter/lab_setup',
        'front_matter/safety_guidelines'
      ]
    },
    {
      type: 'category',
      label: 'Module 1 — The Robotic Nervous System (ROS 2)',
      collapsible: true,
      collapsed: false,
      items: [
        'module_1/1_1_foundations',
        'module_1/1_2_packages',
        'module_1/1_3_urdf',
        'module_1/1_4_ai_controllers'
      ]
    },
    {
      type: 'category',
      label: 'Module 2 — The Digital Twin (Gazebo & Unity)',
      collapsible: true,
      collapsed: false,
      items: [
        'module_2/2_1_gazebo_fundamentals',
        'module_2/2_2_sim_sensors',
        'module_2/2_3_unity_viz',
        'module_2/2_4_digital_twin'
      ]
    },
    {
      type: 'category',
      label: 'Module 3 — The AI-Robot Brain (NVIDIA Isaac)',
      collapsible: true,
      collapsed: false,
      items: [
        'module_3/3_1_isaac_sim_essentials',
        'module_3/3_2_isaac_ros_perception',
        'module_3/3_3_nav2',
        'module_3/3_4_sim_to_real'
      ]
    },
    {
      type: 'category',
      label: 'Module 4 — Vision-Language-Action (VLA)',
      collapsible: true,
      collapsed: false,
      items: [
        'module_4/4_1_whisper_voice',
        'module_4/4_2_llm_planning',
        'module_4/4_3_multi_modal',
        'module_4/4_4_capstone'
      ]
    }
  ],
};

export default sidebars;
