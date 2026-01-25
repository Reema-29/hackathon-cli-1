import React, { useState } from 'react';
import { navigate } from '@docusaurus/retry';

// Mock component for book creation form
const BookCreationForm = () => {
  const [formData, setFormData] = useState({
    title: '',
    author: '',
    description: '',
    target_audience: ''
  });
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState('');

  const handleChange = (e) => {
    const { name, value } = e.target;
    setFormData(prev => ({
      ...prev,
      [name]: value
    }));
  };

  const handleSubmit = async (e) => {
    e.preventDefault();
    setLoading(true);
    setError('');

    // Simulate API call to create book
    try {
      // In a real implementation, this would call the API
      await new Promise(resolve => setTimeout(resolve, 1000));

      // Simulate successful creation
      const newBookId = `book-${Date.now()}`;

      // Redirect to the new book page
      navigate(`/books/${newBookId}`);
    } catch (err) {
      setError('Failed to create book. Please try again.');
      setLoading(false);
    }
  };

  return (
    <div className="container margin-vert--lg">
      <div className="row">
        <div className="col col--6 col--offset-3">
          <h1>Create New Book</h1>

          {error && (
            <div className="alert alert--danger" role="alert">
              {error}
            </div>
          )}

          <form onSubmit={handleSubmit}>
            <div className="margin-vert--md">
              <label htmlFor="title">Book Title *</label>
              <input
                type="text"
                id="title"
                name="title"
                className="form-control"
                value={formData.title}
                onChange={handleChange}
                required
                placeholder="Enter book title"
              />
            </div>

            <div className="margin-vert--md">
              <label htmlFor="author">Author *</label>
              <input
                type="text"
                id="author"
                name="author"
                className="form-control"
                value={formData.author}
                onChange={handleChange}
                required
                placeholder="Enter author name"
              />
            </div>

            <div className="margin-vert--md">
              <label htmlFor="description">Description</label>
              <textarea
                id="description"
                name="description"
                className="form-control"
                value={formData.description}
                onChange={handleChange}
                placeholder="Enter book description"
                rows="4"
              />
            </div>

            <div className="margin-vert--md">
              <label htmlFor="target_audience">Target Audience</label>
              <input
                type="text"
                id="target_audience"
                name="target_audience"
                className="form-control"
                value={formData.target_audience}
                onChange={handleChange}
                placeholder="e.g., Developers, Students, etc."
              />
            </div>

            <div className="margin-vert--lg">
              <button
                type="submit"
                className="button button--primary button--lg"
                disabled={loading}
              >
                {loading ? 'Creating...' : 'Create Book'}
              </button>
            </div>
          </form>
        </div>
      </div>
    </div>
  );
};

export default BookCreationForm;