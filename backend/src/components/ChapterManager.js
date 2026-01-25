import React, { useState, useEffect } from 'react';
import Link from '@docusaurus/Link';

// Mock component for chapter management
const ChapterManager = ({ bookId }) => {
  const [chapters, setChapters] = useState([]);
  const [loading, setLoading] = useState(true);

  // Simulate loading chapters
  useEffect(() => {
    // In a real implementation, this would fetch from the API
    setTimeout(() => {
      setChapters([
        {
          id: 'chapter-1',
          title: 'Introduction',
          number: 1,
          status: 'draft',
          content: 'This is the introduction chapter content.',
          order: 0
        },
        {
          id: 'chapter-2',
          title: 'Getting Started',
          number: 2,
          status: 'in-progress',
          content: 'This chapter covers the basics of getting started.',
          order: 1
        }
      ]);
      setLoading(false);
    }, 500);
  }, [bookId]);

  if (loading) {
    return <div>Loading chapters...</div>;
  }

  return (
    <div className="container margin-vert--lg">
      <div className="row">
        <div className="col col--8 col--offset-2">
          <div className="margin-vert--md">
            <Link to={`/books/${bookId}`} className="button button--sm">
              ← Back to Book
            </Link>
          </div>

          <h1>Manage Chapters</h1>
          <p>Manage the chapters in your book: {bookId}</p>

          {chapters.length === 0 ? (
            <div>
              <p>No chapters created yet.</p>
              <Link to={`/books/${bookId}/chapters/new`} className="button button--primary">
                Create First Chapter
              </Link>
            </div>
          ) : (
            <div>
              <table className="table">
                <thead>
                  <tr>
                    <th>#</th>
                    <th>Title</th>
                    <th>Status</th>
                    <th>Actions</th>
                  </tr>
                </thead>
                <tbody>
                  {chapters.map((chapter) => (
                    <tr key={chapter.id}>
                      <td>{chapter.number}</td>
                      <td>{chapter.title}</td>
                      <td>
                        <span className={`badge badge--${getStatusColor(chapter.status)}`}>
                          {chapter.status}
                        </span>
                      </td>
                      <td>
                        <Link
                          to={`/books/${bookId}/chapters/${chapter.id}`}
                          className="button button--sm button--outline button--primary"
                        >
                          Edit
                        </Link>
                      </td>
                    </tr>
                  ))}
                </tbody>
              </table>

              <div className="margin-vert--lg">
                <Link to={`/books/${bookId}/chapters/new`} className="button button--primary">
                  Add New Chapter
                </Link>
              </div>
            </div>
          )}
        </div>
      </div>
    </div>
  );
};

// Helper function to get status badge color
const getStatusColor = (status) => {
  switch (status) {
    case 'published':
      return 'success';
    case 'review':
      return 'warning';
    case 'in-progress':
      return 'info';
    case 'complete':
      return 'success';
    default:
      return 'secondary';
  }
};

export default ChapterManager;