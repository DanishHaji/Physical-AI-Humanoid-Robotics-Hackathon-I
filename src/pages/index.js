import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <div className={styles.heroContent}>
          <div className={styles.heroIcon}>🤖</div>
          <h1 className={styles.heroTitle}>{siteConfig.title}</h1>
          <p className={styles.heroSubtitle}>{siteConfig.tagline}</p>
          <p className={styles.heroDescription}>
            Master the complete journey from ROS 2 fundamentals to Vision-Language-Action systems.
            Build intelligent humanoid robots that perceive, understand, and interact with the physical world.
          </p>
          <div className={styles.buttons}>
            <Link
              className={clsx('button button--primary button--lg', styles.heroButton)}
              to="/docs/intro">
              Start Learning 🚀
            </Link>
            <Link
              className={clsx('button button--secondary button--lg', styles.heroButton)}
              to="/docs/module-01-ros2/week-01-physical-ai-intro">
              Jump to Week 1
            </Link>
          </div>
        </div>
      </div>
    </header>
  );
}

function HomepageFeatures() {
  const features = [
    {
      title: '🧠 Module 1: ROS 2 Nervous System',
      description: 'Master the Robot Operating System 2 - the foundation for modern robotics. Learn nodes, topics, services, and URDF.',
      link: '/docs/module-01-ros2/week-01-physical-ai-intro',
    },
    {
      title: '🌐 Module 2: Digital Twin Simulation',
      description: 'Create photorealistic robot simulations with Gazebo and Unity. Master sim-to-real transfer techniques.',
      link: '/docs/module-02-digital-twin/week-05-digital-twin',
    },
    {
      title: '⚡ Module 3: NVIDIA Isaac AI Brain',
      description: 'Harness GPU-accelerated robotics with Isaac Sim. Train humanoid robots using reinforcement learning.',
      link: '/docs/module-03-isaac-sim/week-08-isaac-sim',
    },
    {
      title: '👁️ Module 4: Vision-Language-Action',
      description: 'Integrate computer vision, LLMs, and robot control. Build robots that understand language and execute tasks.',
      link: '/docs/module-04-vla/week-11-vla-systems',
    },
    {
      title: '🎯 Capstone Project',
      description: 'Build a complete autonomous humanoid system integrating all modules: voice → planning → execution.',
      link: '/docs/capstone/week-13-capstone-project',
    },
    {
      title: '📚 Complete Appendices',
      description: 'Access glossary, hardware setup guides, and comprehensive troubleshooting resources.',
      link: '/docs/appendices/glossary',
    },
  ];

  return (
    <section className={styles.features}>
      <div className="container">
        <h2 className={styles.featuresTitle}>Build Something Intelligent</h2>
        <div className={styles.featureGrid}>
          {features.map((feature, idx) => (
            <Link key={idx} to={feature.link} className={styles.featureCard}>
              <h3>{feature.title}</h3>
              <p>{feature.description}</p>
              <div className={styles.featureArrow}>→</div>
            </Link>
          ))}
        </div>
      </div>
    </section>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Home`}
      description="The Complete AI-Native Textbook for Physical AI & Humanoid Robotics - From ROS 2 to Vision-Language-Action Systems">
      <HomepageHeader />
      <main>
        <HomepageFeatures />
      </main>
    </Layout>
  );
}
