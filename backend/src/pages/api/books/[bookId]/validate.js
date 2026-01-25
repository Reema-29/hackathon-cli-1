// Mock API for comprehensive validation
// Based on contracts/content-management.yaml

import { mockData, findEntity } from '../../../../utils/dataStorage';

export default function handler(req, res) {
  const { method } = req;
  const { bookId } = req.query;

  // Verify the book exists
  const book = findEntity(mockData.books, bookId);
  if (!book) {
    return res.status(404).json({
      success: false,
      error: {
        code: 'BOOK_NOT_FOUND',
        message: `Book with ID ${bookId} not found`
      },
      timestamp: new Date().toISOString()
    });
  }

  switch (method) {
    case 'POST':
      // POST /api/books/{bookId}/validate - Run comprehensive validation on book content
      const { validation_types } = req.body;

      const typesToValidate = validation_types || ['content', 'citation', 'style', 'accessibility', 'seo'];
      const validationResults = performValidation(bookId, typesToValidate);

      const overallStatus = calculateOverallStatus(validationResults);
      const overallScore = calculateOverallScore(validationResults);

      res.status(200).json({
        success: true,
        data: {
          validation_results: validationResults,
          overall_status: overallStatus,
          overall_score: overallScore
        },
        message: 'Validation completed successfully',
        timestamp: new Date().toISOString()
      });
      break;

    default:
      res.status(405).json({
        success: false,
        error: {
          code: 'METHOD_NOT_ALLOWED',
          message: `Method ${method} not allowed`
        },
        timestamp: new Date().toISOString()
      });
      break;
  }
}

// Perform validation on different aspects
function performValidation(bookId, validationTypes) {
  const results = {};

  for (const type of validationTypes) {
    switch (type) {
      case 'content':
        results.content_accuracy = validateContentAccuracy(bookId);
        break;
      case 'citation':
        results.citation_compliance = validateCitationCompliance(bookId);
        break;
      case 'style':
        results.style_compliance = validateStyleCompliance(bookId);
        break;
      case 'accessibility':
        results.accessibility = validateAccessibility(bookId);
        break;
      case 'seo':
        results.seo = validateSEO(bookId);
        break;
      default:
        // Skip unknown validation types
        break;
    }
  }

  return results;
}

// Calculate overall status based on individual validation results
function calculateOverallStatus(validationResults) {
  const statuses = Object.values(validationResults).map(result => result.status);

  if (statuses.includes('fail')) return 'fail';
  if (statuses.includes('warning')) return 'warning';
  return 'pass';
}

// Calculate overall score based on individual validation scores
function calculateOverallScore(validationResults) {
  const scores = Object.values(validationResults).map(result => result.score);
  if (scores.length === 0) return 0;

  const total = scores.reduce((sum, score) => sum + score, 0);
  return Math.round(total / scores.length);
}

// Mock validation functions
function validateContentAccuracy(bookId) {
  // Simulate content accuracy validation
  return {
    status: 'pass',
    issues: [],
    score: 95
  };
}

function validateCitationCompliance(bookId) {
  // Simulate citation compliance validation
  const issues = [];

  // Check for citations that might have issues
  const citations = mockData.citations.filter(c => c.book_id === bookId);
  if (citations.length === 0) {
    issues.push({
      type: 'missing_citation',
      location: 'book-wide',
      message: 'No citations found in book'
    });
  }

  return {
    status: issues.length > 0 ? 'warning' : 'pass',
    issues,
    score: issues.length > 0 ? 85 : 100
  };
}

function validateStyleCompliance(bookId) {
  // Simulate style compliance validation
  return {
    status: 'pass',
    issues: [],
    score: 90
  };
}

function validateAccessibility(bookId) {
  // Simulate accessibility validation
  return {
    status: 'pass',
    issues: [],
    score: 100
  };
}

function validateSEO(bookId) {
  // Simulate SEO validation
  return {
    status: 'pass',
    issues: [],
    score: 88
  };
}