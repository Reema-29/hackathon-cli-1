import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <h1 className="hero__title">{siteConfig.title}</h1>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/intro">
            Start Reading the Book
          </Link>
        </div>
      </div>
    </header>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Welcome to ${siteConfig.title}`}
      description="Physical AI & Humanoid Robotics Book - Advancing the frontier of embodied artificial intelligence">
      <HomepageHeader />
      <main>
        <section className={styles.features}>
          <div className="container">
            <div className="row">
              <div className="col col--4">
                <div className="text--center padding-horiz--md">
                  <h3>Advanced AI Integration</h3>
                  <p>Learn how to integrate AI systems with robotic platforms for enhanced capabilities.</p>
                </div>
              </div>
              <div className="col col--4">
                <div className="text--center padding-horiz--md">
                  <h3>Humanoid Robotics</h3>
                  <p>Explore the design and implementation of humanoid robotic systems.</p>
                </div>
              </div>
              <div className="col col--4">
                <div className="text--center padding-horiz--md">
                  <h3>Embodied Intelligence</h3>
                  <p>Discover how physical embodiment enhances artificial intelligence systems.</p>
                </div>
              </div>
            </div>
          </div>
        </section>
      </main>
    </Layout>
  );
}