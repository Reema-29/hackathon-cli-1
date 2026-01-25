import React, { useState } from 'react';
import researchService from '../utils/researchService';

// Component for managing research tasks
const ResearchManager = ({ bookId, onResearchComplete }) => {
  const [researchQuery, setResearchQuery] = useState('');
  const [isResearching, setIsResearching] = useState(false);
  const [researchResults, setResearchResults] = useState([]);
  const [researchHistory, setResearchHistory] = useState([]);

  // Submit a research query
  const handleResearchSubmit = async (e) => {
    e.preventDefault();

    if (!researchQuery.trim()) {
      alert('Please enter a research query');
      return;
    }

    setIsResearching(true);
    setResearchResults([]);

    try {
      const researchItem = await researchService.submitQuery(researchQuery, { bookId });
      setResearchResults(researchItem.results);

      // Update research history
      const updatedHistory = [...researchHistory, researchItem];
      setResearchHistory(updatedHistory);

      // Call parent callback if provided
      if (onResearchComplete) {
        onResearchComplete(researchItem);
      }
    } catch (error) {
      console.error('Research error:', error);
      alert('Error with research: ' + error.message);
    } finally {
      setIsResearching(false);
      setResearchQuery('');
    }
  };

  return (
    <div className="research-manager">
      <div className="margin-vert--md">
        <h3>Concurrent Research</h3>
        <p>Perform research while writing to validate facts and find supporting information.</p>
      </div>

      <form onSubmit={handleResearchSubmit}>
        <div className="margin-vert--md">
          <label htmlFor="research-query">Research Query:</label>
          <input
            type="text"
            id="research-query"
            className="form-control"
            value={researchQuery}
            onChange={(e) => setResearchQuery(e.target.value)}
            placeholder="Enter your research query..."
            disabled={isResearching}
          />
        </div>

        <div className="margin-vert--md">
          <button
            type="submit"
            className="button button--primary"
            disabled={isResearching || !researchQuery.trim()}
          >
            {isResearching ? 'Researching...' : 'Submit Research Query'}
          </button>
        </div>
      </form>

      {isResearching && (
        <div className="alert alert--info">
          Conducting research, please wait...
        </div>
      )}

      {researchResults.length > 0 && (
        <div className="margin-vert--md">
          <h4>Research Results:</h4>
          <div className="research-results">
            {researchResults.map((result, index) => (
              <div key={result.id || index} className="margin-vert--sm padding--sm card">
                <h5>{result.title}</h5>
                <p>{result.summary}</p>
                <div className="margin-vert--sm">
                  <span className="badge badge--info">Credibility: {(result.credibility_score * 100).toFixed(1)}%</span>
                  <span className="badge badge--success margin-left--sm">Relevance: {(result.relevance_score * 100).toFixed(1)}%</span>
                  {result.publication_date && (
                    <span className="badge badge--secondary margin-left--sm">Date: {result.publication_date}</span>
                  )}
                </div>
                {result.url && (
                  <div className="margin-vert--sm">
                    <a href={result.url} target="_blank" rel="noopener noreferrer" className="button button--sm button--outline button--primary">
                      View Source
                    </a>
                  </div>
                )}
              </div>
            ))}
          </div>
        </div>
      )}

      {researchHistory.length > 0 && (
        <div className="margin-vert--md">
          <h4>Research History:</h4>
          <table className="table">
            <thead>
              <tr>
                <th>Query</th>
                <th>Credibility</th>
                <th>Status</th>
                <th>Date</th>
              </tr>
            </thead>
            <tbody>
              {researchHistory.map((item, index) => (
                <tr key={item.id || index}>
                  <td>{item.query}</td>
                  <td>
                    <span className={`badge badge--${item.credibility_score > 0.8 ? 'success' : item.credibility_score > 0.6 ? 'warning' : 'danger'}`}>
                      {(item.credibility_score * 100).toFixed(1)}%
                    </span>
                  </td>
                  <td>
                    <span className={`badge badge--${item.status === 'verified' ? 'success' : item.status === 'rejected' ? 'danger' : 'info'}`}>
                      {item.status}
                    </span>
                  </td>
                  <td>{new Date(item.research_date).toLocaleDateString()}</td>
                </tr>
              ))}
            </tbody>
          </table>
        </div>
      )}
    </div>
  );
};

export default ResearchManager;