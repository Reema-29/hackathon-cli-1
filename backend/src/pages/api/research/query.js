// Mock API for research queries
// Based on plan.md research API contracts

import { mockData, addEntity } from '../../../utils/dataStorage';
import researchService from '../../../utils/researchService';

export default function handler(req, res) {
  const { method } = req;

  switch (method) {
    case 'POST':
      // POST /api/research/query - Submit research query
      const { query, bookId } = req.body;

      if (!query) {
        return res.status(400).json({
          success: false,
          error: {
            code: 'VALIDATION_ERROR',
            message: 'Query is required',
            details: { query: 'Query is required' }
          },
          timestamp: new Date().toISOString()
        });
      }

      // Submit research query using the research service
      researchService.submitQuery(query, { bookId })
        .then(researchItem => {
          // Add the research item to our mock data
          const createdResearch = addEntity(mockData.researchItems, {
            ...researchItem,
            book_id: bookId
          });

          res.status(200).json({
            success: true,
            data: {
              research_item: createdResearch,
              results: researchItem.results,
              status: 'completed'
            },
            message: 'Research query processed successfully',
            timestamp: new Date().toISOString()
          });
        })
        .catch(error => {
          console.error('Research Service Error:', error);
          res.status(500).json({
            success: false,
            error: {
              code: 'RESEARCH_SERVICE_ERROR',
              message: 'Error processing research query',
              details: error.message
            },
            timestamp: new Date().toISOString()
          });
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