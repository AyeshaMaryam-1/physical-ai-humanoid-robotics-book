import clsx from 'clsx';
import Link from '@docusaurus/Link';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

const FeatureList = [
  {
    title: 'Module 1: The Robotic Nervous System (ROS 2)',
    description: (
      <>
        Master the Robot Operating System 2 fundamentals, packages, URDF modeling, and AI controllers
        that form the backbone of modern robotics applications.
      </>
    ),
    link: '/docs/module_1/1_1_foundations'
  },
  {
    title: 'Module 2: The Digital Twin (Gazebo & Unity)',
    description: (
      <>
        Create high-fidelity simulations using Gazebo and Unity, implementing digital twins
        for safe testing and development of robot behaviors.
      </>
    ),
    link: '/docs/module_2/2_1_gazebo_fundamentals'
  },
  {
    title: 'Module 3: The AI-Robot Brain (NVIDIA Isaac)',
    description: (
      <>
        Develop AI-powered robot brains using NVIDIA Isaac, including perception systems,
        navigation, and sim-to-real transfer techniques.
      </>
    ),
    link: '/docs/module_3/3_1_isaac_sim_essentials'
  },
  {
    title: 'Module 4: Vision-Language-Action (VLA)',
    description: (
      <>
        Integrate voice recognition, LLM planning, and multi-modal perception to create
        autonomous humanoid robots capable of complex decision making.
      </>
    ),
    link: '/docs/module_4/4_1_whisper_voice'
  },
];

function Feature({title, description, link}) {
  return (
    <div className={clsx('col col--3')}>
      <div className={styles.featureCard}>
        <Heading as="h3">
          <Link to={link} className={styles.featureTitle}>
            {title}
          </Link>
        </Heading>
        <p className={styles.featureDescription}>{description}</p>
      </div>
    </div>
  );
}

export default function HomepageFeatures() {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className="row">
          {FeatureList.map((props, idx) => (
            <Feature key={idx} {...props} />
          ))}
        </div>
      </div>
    </section>
  );
}
