import React from 'react';
import Layout from '@theme/Layout';
import { useParams } from '@docusaurus/router';
import ChapterManager from '../../../../components/ChapterManager';

export default function ChapterListPage() {
  const { bookId } = useParams();

  return (
    <Layout title={`Chapters: ${bookId}`} description="Manage chapters in your book">
      <ChapterManager bookId={bookId} />
    </Layout>
  );
}