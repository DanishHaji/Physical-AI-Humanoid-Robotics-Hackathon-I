/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */

// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  // By default, Docusaurus generates a sidebar from the docs folder structure
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module 1: ROS 2 Nervous System',
      items: [
        'module-01-ros2/week-01-physical-ai-intro',
        'module-01-ros2/week-02-ros2-fundamentals',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Digital Twin (Gazebo + Unity)',
      items: [
        'module-02-digital-twin/week-05-digital-twin',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: NVIDIA Isaac AI Brain',
      items: [
        'module-03-isaac-sim/week-08-isaac-sim',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      items: [
        'module-04-vla/week-11-vla-systems',
      ],
    },
    {
      type: 'category',
      label: 'Capstone Project',
      items: [
        'capstone/week-13-capstone-project',
      ],
    },
    {
      type: 'category',
      label: 'Appendices',
      items: [
        'appendices/glossary',
        'appendices/hardware-setup',
        'appendices/troubleshooting',
      ],
    },
  ],
};

export default sidebars;
