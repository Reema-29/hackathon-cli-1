import React, { useState } from 'react';
import sourceValidation from '../utils/sourceValidation';

// Component for validating sources
const SourceValidator = ({ onValidationComplete }) => {
  const [sourceInfo, setSourceInfo] = useState({
    url: '',
    title: '',
    authors: '',
    publisher: '',
    publication_date: '',
    source_type: 'webpage'
  });
  const [isProcessing, setIsProcessing] = useState(false);
  const [validationResult, setValidationResult] = useState(null);

  // Handle input changes
  const handleInputChange = (e) => {
    const { name, value } = e.target;
    setSourceInfo(prev => ({
      ...prev,
      [name]: name === 'authors' ? value.split(',').map(a => a.trim()).filter(a => a) : value
    }));
  };

  // Validate the source
  const handleValidate = async () => {
    setIsProcessing(true);
    setValidationResult(null);

    try {
      const result = await sourceValidation.validateSource(sourceInfo);
      setValidationResult(result);

      // Call parent callback if provided
      if (onValidationComplete) {
        onValidationComplete(result, sourceInfo);
      }
    } catch (error) {
      console.error('Source validation error:', error);
      alert('Error validating source: ' + error.message);
    } finally {
      setIsProcessing(false);
    }
  };

  return (
    <div className="source-validator">
      <div className="margin-vert--md">
        <h3>Source Validation</h3>
        <p>Validate research sources to ensure credibility and accuracy.</p>
      </div>

      <div className="margin-vert--md">
        <div className="row">
          <div className="col col--6">
            <label htmlFor="url">URL:</label>
            <input
              type="text"
              id="url"
              name="url"
              className="form-control"
              value={sourceInfo.url}
              onChange={handleInputChange}
              placeholder="https://example.com"
            />
          </div>
          <div className="col col--6">
            <label htmlFor="source_type">Source Type:</label>
            <select
              id="source_type"
              name="source_type"
              className="form-control"
              value={sourceInfo.source_type}
              onChange={handleInputChange}
            >
              <option value="webpage">Web Page</option>
              <option value="book">Book</option>
              <option value="journal">Journal Article</option>
              <option value="conference_paper">Conference Paper</option>
              <option value="blog">Blog Post</option>
              <option value="news">News Article</option>
            </select>
          </div>
        </div>

        <div className="margin-vert--sm">
          <label htmlFor="title">Title:</label>
          <input
            type="text"
            id="title"
            name="title"
            className="form-control"
            value={sourceInfo.title}
            onChange={handleInputChange}
            placeholder="Source title"
          />
        </div>

        <div className="margin-vert--sm">
          <label htmlFor="authors">Authors (comma separated):</label>
          <input
            type="text"
            id="authors"
            name="authors"
            className="form-control"
            value={sourceInfo.authors.join(', ')}
            onChange={handleInputChange}
            placeholder="Author 1, Author 2, etc."
          />
        </div>

        <div className="margin-vert--sm">
          <label htmlFor="publisher">Publisher:</label>
          <input
            type="text"
            id="publisher"
            name="publisher"
            className="form-control"
            value={sourceInfo.publisher}
            onChange={handleInputChange}
            placeholder="Publisher or organization"
          />
        </div>

        <div className="margin-vert--sm">
          <label htmlFor="publication_date">Publication Date:</label>
          <input
            type="date"
            id="publication_date"
            name="publication_date"
            className="form-control"
            value={sourceInfo.publication_date}
            onChange={handleInputChange}
          />
        </div>
      </div>

      <div className="margin-vert--md">
        <button
          className="button button--primary"
          onClick={handleValidate}
          disabled={isProcessing}
        >
          {isProcessing ? 'Validating...' : 'Validate Source'}
        </button>
      </div>

      {isProcessing && (
        <div className="alert alert--info">
          Validating source, please wait...
        </div>
      )}

      {validationResult && (
        <div className="margin-vert--md">
          <h4>Validation Results:</h4>

          <div className="row">
            <div className="col col--6">
              <div className={`alert ${validationResult.valid ? 'alert--success' : 'alert--danger'}`}>
                <strong>Overall Status:</strong> {validationResult.valid ? 'Valid' : 'Issues Found'}
              </div>
            </div>
            <div className="col col--6">
              <div className="alert alert--info">
                <strong>Credibility Score:</strong> {(validationResult.credibility * 100).toFixed(1)}%
                <span className={`badge margin-left--sm ${validationResult.isTrusted ? 'badge--success' : 'badge--warning'}`}>
                  {validationResult.isTrusted ? 'Trusted' : 'Caution'}
                </span>
              </div>
            </div>
          </div>

          {validationResult.issues.length > 0 && (
            <div className="margin-vert--sm">
              <h5>Issues Found:</h5>
              <ul>
                {validationResult.issues.map((issue, index) => (
                  <li key={index} className="alert alert--warning">
                    {issue}
                  </li>
                ))}
              </ul>
            </div>
          )}

          <div className="margin-vert--sm">
            <div className="alert alert--secondary">
              <strong>Confidence Level:</strong> {(validationResult.confidence * 100).toFixed(1)}%
            </div>
          </div>
        </div>
      )}
    </div>
  );
};

export default SourceValidator;