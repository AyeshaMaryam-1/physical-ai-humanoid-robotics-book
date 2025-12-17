import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';

import Heading from '@theme/Heading';
import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <div className={styles.heroContent}>
          
          <div className={styles.heroImage}>
            <img
              src="img/logo.png"
              alt="Humanoid Robot" 
              className={styles.robotImage}
            />
          </div>
          <div className={styles.heroText}>
            <Heading as="h1" className="hero__title">
              {siteConfig.title}
            </Heading>
            <p className="hero__subtitle">{siteConfig.tagline}</p>
            <div className={styles.buttons}>
              <Link
                className="button button--secondary button--lg"
                to="/docs/front_matter/preface">
                Read the Book
              </Link>
            </div>
          </div>
        </div>
      </div>
    </header>
  );
}

function LearningOutcomes() {
  return (
    <section className={styles.learningOutcomes}>
      <div className="container padding-horiz--md">
        <Heading as="h2" className="text--center margin-bottom--lg">
          What You'll Learn
        </Heading>
        <div className="row">
          <div className="col col--3">
            <div className={styles.outcomeCard}>
              <h3>Build Humanoid Robot Simulations</h3>
              <p>Design and implement complete humanoid robot simulations with ROS 2, Gazebo, and Unity digital twins.</p>
            </div>
          </div>
          <div className="col col--3">
            <div className={styles.outcomeCard}>
              <h3>Integrate Perception, Navigation, and Control</h3>
              <p>Combine sensor data, navigation algorithms, and control systems for autonomous robot operation.</p>
            </div>
          </div>
          <div className="col col--3">
            <div className={styles.outcomeCard}>
              <h3>Use voice + LLMs for robot decision making</h3>
              <p>Implement voice recognition, LLM-based planning, and multi-modal decision making systems.</p>
            </div>
          </div>
          <div className="col col--3">
            <div className={styles.outcomeCard}>
              <h3>Deploy AI Pipelines to Jetson Hardware</h3>
              <p>Transfer from simulation to real hardware with optimized perception and control pipelines.</p>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}

export default function Home() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Physical AI & Humanoid Robotics`}
      description="Academic textbook companion: Building Intelligent Robots from Simulation to Reality">
      <HomepageHeader />
      <main>
        <HomepageFeatures />
        <LearningOutcomes />
      </main>
    </Layout>
  );
}
