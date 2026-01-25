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
  // Manual sidebar structure to match root sidebars.js
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Books',
      items: ['books/create', 'books/manage'],
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
      label: 'Module 3: The AI-Robot Brain',
      items: [
        'module-3/intro',
        'module-3/chapter-1-isaac-sim',
        'module-3/chapter-2-isaac-ros-vslam',
        'module-3/chapter-3-nav2-navigation',
      ],
    },
  ],
};

export default sidebars;
