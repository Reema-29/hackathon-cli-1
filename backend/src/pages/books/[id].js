import React from 'react';
import Layout from '@theme/Layout';
import { useParams } from '@docusaurus/router';
import BookDetail from '../../components/BookDetail';

export default function BookPage() {
  const { id } = useParams();

  return (
    <Layout title={`Book: ${id}`} description="Manage your book project">
      <BookDetail bookId={id} />
    </Layout>
  );
}