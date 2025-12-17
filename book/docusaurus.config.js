// @ts-check
// `@type` JSDoc annotations allow editor autocompletion and type checking
// (when paired with `@ts-check`).
// There are various equivalent ways to declare your Docusaurus config.
// See: https://docusaurus.io/docs/api/docusaurus-config

import {themes as prismThemes} from 'prism-react-renderer';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'Building Intelligent Robots from Simulation to Reality',
  favicon: 'img/favicon.ico',

  // Future flags, see https://docusaurus.io/docs/api/docusaurus-config#future
  future: {
    v4: true, // Improve compatibility with the upcoming Docusaurus v4
  },

  // Set the production url of your site here
  url: 'https://physical-ai-humanoid-robotics-book-five.vercel.app/', // Replace with your GitHub Pages URL
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/', // GitHub Pages subdirectory

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'AyeshaMaryam-1', // Usually your GitHub org/user name.
  projectName: 'physical-ai-humanoid-robotics-book', // Usually your repo name.

  onBrokenLinks: 'warn',

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: './sidebars.js',
          routeBasePath: '/',
          editUrl:
            'https://github.com/AyeshaMaryam-1/physical-ai-humanoid-robotics-book/tree/main/book/',
        },
        blog: false, // Disable blog for textbook companion site
        theme: {
          customCss: './src/css/custom.css',
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      // Replace with your project's social card
      image: 'img/docusaurus-social-card.jpg',
      colorMode: {
        respectPrefersColorScheme: true,
      },
      navbar: {
        title: 'Physical AI & Humanoid Robotics',
        logo: {
          alt: 'Physical AI & Humanoid Robotics Logo',
          src: 'img/logo.png',
        },
        items: [
          {
            type: 'dropdown',
            label: 'Modules',
            position: 'left',
            items: [
              {
                type: 'doc',
                docId: 'module_1/1_1_foundations',
                label: 'Module 1: The Robotic Nervous System (ROS 2)',
              },
              {
                type: 'doc',
                docId: 'module_2/2_1_gazebo_fundamentals',
                label: 'Module 2: The Digital Twin (Gazebo & Unity)',
              },
              {
                type: 'doc',
                docId: 'module_3/3_1_isaac_sim_essentials',
                label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac)',
              },
              {
                type: 'doc',
                docId: 'module_4/4_1_whisper_voice',
                label: 'Module 4: Vision-Language-Action (VLA)',
              },
            ],
          },
          {
            type: 'docSidebar',
            sidebarId: 'tutorialSidebar',
            href: '/module_4/4_4_capstone',
            position: 'left',
            label: 'Capstone',
          },
          {
            href: 'https://github.com/AyeshaMaryam-1/physical-ai-humanoid-robotics-book',
            label: 'GitHub',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Textbook',
            items: [
              {
                label: 'Preface',
                to: '/front_matter/preface',
              },
              {
                label: "Reader's Guide",
                to: '/front_matter/how_to_use',
              },
              {
                label: 'Setup Guides',
                to: '/front_matter/lab_setup',
              },
            ],
          },
          {
            title: 'Resources',
            items: [
              {
                label: 'GitHub Repository',
                href: 'https://github.com/AyeshaMaryam-1/physical-ai-humanoid-robotics-book',
              },
            ],
          },
        ],
        copyright: `© 2025 Physical AI & Humanoid Robotics Textbook. Built with Docusaurus.`,
      },
      prism: {
        theme: prismThemes.github,
        darkTheme: prismThemes.dracula,
      },
    }),
};

export default config;
