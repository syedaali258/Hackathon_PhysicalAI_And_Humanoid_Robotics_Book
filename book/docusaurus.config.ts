import { themes as prismThemes } from 'prism-react-renderer';
import type { Config } from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

const config: Config = {
  title: 'AI/Robotics Book',
  tagline: 'Physical AI & Humanoid Robotics with ROS 2',
  favicon: 'img/favicon.ico',

  future: { v4: true },

  url: 'https://syeda-ai-book.netlify.app', // ✅ Replace with your Netlify URL
  baseUrl: '/',

  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',

  i18n: { defaultLocale: 'en', locales: ['en'] },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          routeBasePath: '/',   // ⭐ MUST for homepage
          editUrl: 'https://github.com/syedaali258/Hackathon_PhysicalAI_And_Humanoid_Robotics_Book/edit/main/',
        },
        blog: { showReadingTime: true },
        theme: { customCss: './src/css/custom.css' }
      } satisfies Preset.Options
    ]
  ],

  themeConfig: {
    colorMode: { respectPrefersColorScheme: true },
    navbar: {
      title: 'AI/Robotics Book',
      logo: { alt: 'Logo', src: 'img/logo.svg' },
      items: [
        { type: 'docSidebar', sidebarId: 'tutorialSidebar', position: 'left', label: 'Book' },
        { to: '/blog', label: 'Blog', position: 'left' },
        { href: 'https://github.com/syedaali258/Hackathon_PhysicalAI_And_Humanoid_Robotics_Book', label: 'GitHub', position: 'right' }
      ]
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Book Content',
          items: [{ label: 'Module 1: The Robotic Nervous System (ROS 2)', to: '/docs/module-1-ros2/intro' }]
        },
        {
          title: 'Community',
          items: [
            { label: 'GitHub Discussions', href: 'https://github.com/syedaali258/Hackathon_PhysicalAI_And_Humanoid_Robotics_Book/discussions' },
            { label: 'ROS Answers', href: 'https://answers.ros.org/' }
          ]
        }
      ],
      copyright: `Copyright © ${new Date().getFullYear()} AI/Robotics Book.`
    },
    prism: { theme: prismThemes.github, darkTheme: prismThemes.dracula }
  } satisfies Preset.ThemeConfig
};

export default config;
