import React, { useState, useEffect } from 'react';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

// Mock component for book dashboard
const BookDashboard = () => {
  const { siteConfig } = useDocusaurusContext();
  const [books, setBooks] = useState([]);
  const [loading, setLoading] = useState(true);

  // Simulate loading books
  useEffect(() => {
    // In a real implementation, this would fetch from the API
    setTimeout(() => {
      setBooks([
        {
          id: 'book-1',
          title: 'Sample Book',
          author: 'Sample Author',
          status: 'draft',
          last_modified: new Date().toISOString()
        }
      ]);
      setLoading(false);
    }, 500);
  }, []);

  if (loading) {
    return <div>Loading books...</div>;
  }

  return (
    <div className="container margin-vert--lg">
      <div className="row">
        <div className="col col--8 col--offset-2">
          <h1>Book Dashboard</h1>
          <p>Welcome to the AI/Spec-Driven Book Creation System.</p>

          <div className="margin-vert--lg">
            <Link
              className="button button--primary button--lg"
              to="/create-book">
              Create New Book
            </Link>
          </div>

          <h2>Your Books</h2>
          {books.length === 0 ? (
            <p>No books created yet. Create your first book to get started!</p>
          ) : (
            <table className="table">
              <thead>
                <tr>
                  <th>Title</th>
                  <th>Author</th>
                  <th>Status</th>
                  <th>Last Modified</th>
                  <th>Actions</th>
                </tr>
              </thead>
              <tbody>
                {books.map((book) => (
                  <tr key={book.id}>
                    <td>{book.title}</td>
                    <td>{book.author}</td>
                    <td>
                      <span className={`badge badge--${getStatusColor(book.status)}`}>
                        {book.status}
                      </span>
                    </td>
                    <td>{new Date(book.last_modified).toLocaleDateString()}</td>
                    <td>
                      <Link to={`/books/${book.id}`} className="button button--sm">
                        Edit
                      </Link>
                    </td>
                  </tr>
                ))}
              </tbody>
            </table>
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
    default:
      return 'secondary';
  }
};

export default BookDashboard;