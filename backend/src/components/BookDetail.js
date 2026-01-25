import React, { useState, useEffect } from 'react';
import Link from '@docusaurus/Link';

// Mock component for book detail page
const BookDetail = ({ bookId }) => {
  const [book, setBook] = useState(null);
  const [chapters, setChapters] = useState([]);
  const [loading, setLoading] = useState(true);

  // Simulate loading book data
  useEffect(() => {
    // In a real implementation, this would fetch from the API
    setTimeout(() => {
      setBook({
        id: bookId,
        title: 'Sample Book',
        author: 'Sample Author',
        description: 'This is a sample book for demonstration purposes.',
        status: 'draft',
        creation_date: new Date().toISOString(),
        last_modified: new Date().toISOString(),
        target_audience: 'Developers'
      });

      setChapters([
        {
          id: 'chapter-1',
          title: 'Introduction',
          number: 1,
          status: 'draft',
          order: 0
        },
        {
          id: 'chapter-2',
          title: 'Getting Started',
          number: 2,
          status: 'in-progress',
          order: 1
        }
      ]);

      setLoading(false);
    }, 500);
  }, [bookId]);

  if (loading) {
    return <div>Loading book...</div>;
  }

  if (!book) {
    return <div>Book not found</div>;
  }

  return (
    <div className="container margin-vert--lg">
      <div className="row">
        <div className="col col--8 col--offset-2">
          <div className="margin-vert--md">
            <Link to="/dashboard" className="button button--sm">
              ← Back to Dashboard
            </Link>
          </div>

          <div className="margin-vert--md">
            <h1>{book.title}</h1>
            <p><strong>Author:</strong> {book.author}</p>
            <p><strong>Status:</strong> <span className={`badge badge--${getStatusColor(book.status)}`}>{book.status}</span></p>
            <p><strong>Target Audience:</strong> {book.target_audience}</p>
            {book.description && <p><strong>Description:</strong> {book.description}</p>}
          </div>

          <div className="margin-vert--lg">
            <div className="button-group button-group--block">
              <Link to={`/books/${book.id}/edit`} className="button button--primary">
                Edit Book Details
              </Link>
              <Link to={`/books/${book.id}/chapters`} className="button button--secondary">
                Manage Chapters
              </Link>
              <Link to={`/books/${book.id}/validate`} className="button button--info">
                Validate Content
              </Link>
            </div>
          </div>

          <h2>Chapters</h2>
          {chapters.length === 0 ? (
            <p>No chapters created yet. Add your first chapter to get started!</p>
          ) : (
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
                      <Link to={`/books/${book.id}/chapters/${chapter.id}`} className="button button--sm button--outline">
                        Edit
                      </Link>
                    </td>
                  </tr>
                ))}
              </tbody>
            </table>
          )}

          <div className="margin-vert--lg">
            <Link to={`/books/${book.id}/chapters/new`} className="button button--primary">
              Add New Chapter
            </Link>
          </div>
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

export default BookDetail;