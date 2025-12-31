import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  // By default, Docusaurus generates a sidebar from the docs folder structure
  tutorialSidebar: [
    'intro', // Main introduction page
    {
      type: 'category',
      label: 'Tutorial Basics',
      items: [
        'tutorial-basics/create-a-page',
        'tutorial-basics/create-a-document',
        'tutorial-basics/create-a-blog-post',
        'tutorial-basics/markdown-features',
        'tutorial-basics/deploy-your-site',
        'tutorial-basics/congratulations',
      ],
    },
    {
      type: 'category',
      label: 'Tutorial Extras',
      items: [
        'tutorial-extras/manage-docs-versions',
        'tutorial-extras/translate-your-site',
      ],
    },
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      items: [
        'module-1-ros2/intro',
        'module-1-ros2/chapter-1-fundamentals',
        'module-1-ros2/chapter-2-python-agents',
        'module-1-ros2/chapter-3-urdf-modeling',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Digital Twin & Simulation',
      items: [
        'module-2-digital-twin/intro',
        'module-2-digital-twin/chapter-1-gazebo-physics',
        'module-2-digital-twin/chapter-2-sensor-simulation',
        'module-2-digital-twin/chapter-3-unity-interaction',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: AI Brain & Navigation',
      items: [
        'module-3-ai-brain/intro',
        'module-3-ai-brain/chapter-1-isaac-sim',
        'module-3-ai-brain/chapter-2-visual-slam',
        'module-3-ai-brain/chapter-3-navigation-nav2',
        'module-3-ai-brain/isaac-package-documentation',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action Models (VLA)',
      items: [
        'module-4-vla/chapter-1-voice-to-action',
        'module-4-vla/chapter-2-language-planning',
        'module-4-vla/chapter-3-capstone',

      ],
    },
    {
      type: 'category',
      label: 'Reference',
      items: [
        'reference/client-libraries',
      ],
    },
  ],
};

export default sidebars;
