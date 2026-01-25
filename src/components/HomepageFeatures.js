import React from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import {
  usePluginData
} from '@docusaurus/useGlobalData';
import Heading from '@theme/Heading';
import styles from './HomepageFeatures.module.css';

const FeatureList = [
  {
    title: 'AI-Assisted Content Generation',
    description: (
      <>
        Generate content with AI assistance while maintaining quality control
      </>
    ),
  },
  {
    title: 'Concurrent Research',
    description: (
      <>
        Research while writing, not all upfront
      </>
    ),
  },
  {
    title: 'APA Citation Compliance',
    description: (
      <>
        Automatic validation of APA-style citations
      </>
    ),
  },
];

function Feature({title, description}) {
  return (
    <div className={clsx('col col--4')}>
      <div className="text--center padding-horiz--md">
        <Heading as="h3">{title}</Heading>
        <p>{description}</p>
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