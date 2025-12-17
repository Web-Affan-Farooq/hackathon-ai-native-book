import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <Heading as="h1" className="hero__title">
          {siteConfig.title}
        </Heading>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/intro">
            Get Started with Robotics
          </Link>
        </div>
      </div>
    </header>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Welcome to ${siteConfig.title}`}
      description="Learn Modern Physical AI And Humanoid Robotics">
      <HomepageHeader />
      <main>
        <section className={styles.features}>
          <div className="container">
            <div className="row">
              <div className="col col--4">
                <h2>Learn Robotics</h2>
                <p>Master ROS 2, simulation, NVIDIA Isaac, and Vision-Language-Action models.</p>
              </div>
              <div className="col col--4">
                <h2>Hands-on Experience</h2>
                <p>Gain practical experience with humanoid robotics platforms and AI systems.</p>
              </div>
              <div className="col col--4">
                <h2>AI Integration</h2>
                <p>Understand how modern AI techniques apply to robotics and autonomous systems.</p>
              </div>
            </div>
          </div>
        </section>
      </main>
    </Layout>
  );
}
