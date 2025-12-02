import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import styles from './index.module.css';

function HomepageHeader() {
  const { siteConfig } = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <h1 className="hero__title animate-fade-in">
          {siteConfig.title}
        </h1>
        <p className={`hero__subtitle animate-fade-in ${styles.subtitle}`}>
          {siteConfig.tagline}
        </p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/intro">
            Start Learning 🚀
          </Link>
        </div>
        <div className={styles.badges}>
          <span className={styles.badge}>🤖 AI-Powered</span>
          <span className={styles.badge}>🌍 Urdu Translation</span>
          <span className={styles.badge}>✨ Personalized</span>
          <span className={styles.badge}>🎮 Interactive Simulation</span>
        </div>
      </div>
    </header>
  );
}

function FeatureCard({ icon, title, description }) {
  return (
    <div className={clsx('col col--4', styles.feature)}>
      <div className={styles.featureCard}>
        <div className={styles.iconWrapper}>
          <span className={styles.icon}>{icon}</span>
        </div>
        <h3>{title}</h3>
        <p>{description}</p>
      </div>
    </div>
  );
}

export default function Home(): React.JSX.Element {
  const { siteConfig } = useDocusaurusContext();
  return (
    <Layout
      title={`Welcome to ${siteConfig.title}`}
      description="Interactive AI-Native Textbook for Humanoid Robotics and Physical AI">
      <HomepageHeader />
      <main>
        <section className={styles.features}>
          <div className="container">
            <div className="row">
              <FeatureCard
                icon="📚"
                title="Comprehensive Content"
                description="5 in-depth chapters covering Physical AI, ROS 2, Gazebo, Isaac Sim, and Vision-Language-Action models."
              />
              <FeatureCard
                icon="🤖"
                title="AI Assistant"
                description="Ask questions and get instant answers powered by RAG technology and OpenAI Agents."
              />
              <FeatureCard
                icon="🎮"
                title="Interactive Simulation"
                description="Control simulated robots directly from your browser with voice command support."
              />
            </div>
            <div className="row" style={{ marginTop: '2rem' }}>
              <FeatureCard
                icon="✨"
                title="Personalized Learning"
                description="Content adapts to your background - from beginner to researcher level."
              />
              <FeatureCard
                icon="🌍"
                title="Multilingual"
                description="Toggle between English and Urdu (اردو) for accessible learning."
              />
              <FeatureCard
                icon="⚡"
                title="Modern Tech Stack"
                description="Built with Docusaurus, TypeScript, React, and cutting-edge AI technologies."
              />
            </div>
          </div>
        </section>

        <section className={styles.ctaSection}>
          <div className="container">
            <h2 className={styles.ctaTitle}>Ready to Build the Future of Robotics?</h2>
            <p className={styles.ctaDescription}>
              Join thousands of students, researchers, and engineers learning humanoid robotics and Physical AI.
            </p>
            <div className={styles.buttons}>
              <Link
                className="button button--primary button--lg"
                to="/docs/intro">
                Get Started Now →
              </Link>
            </div>
          </div>
        </section>
      </main>
    </Layout>
  );
}
