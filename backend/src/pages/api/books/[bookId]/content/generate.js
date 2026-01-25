// Mock API for AI content generation
// Based on contracts/content-management.yaml

import { mockData, findEntity, addEntity } from '../../../../../utils/dataStorage';
import aiService from '../../../../../utils/aiService';

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
      // POST /api/books/{bookId}/content/generate - Generate content using AI assistance
      const { chapter_id, section_id, prompt, content_type, length } = req.body;

      if (!prompt) {
        return res.status(400).json({
          success: false,
          error: {
            code: 'VALIDATION_ERROR',
            message: 'Prompt is required',
            details: { prompt: 'Prompt is required' }
          },
          timestamp: new Date().toISOString()
        });
      }

      // Verify chapter exists if provided
      if (chapter_id) {
        const chapter = findEntity(mockData.chapters, chapter_id);
        if (!chapter || chapter.book_id !== bookId) {
          return res.status(404).json({
            success: false,
            error: {
              code: 'CHAPTER_NOT_FOUND',
              message: `Chapter with ID ${chapter_id} not found in book ${bookId}`
            },
            timestamp: new Date().toISOString()
          });
        }
      }

      // Verify section exists if provided
      if (section_id) {
        const section = findEntity(mockData.sections, section_id);
        if (!section) {
          return res.status(404).json({
            success: false,
            error: {
              code: 'SECTION_NOT_FOUND',
              message: `Section with ID ${section_id} not found`
            },
            timestamp: new Date().toISOString()
          });
        }
      }

      try {
        // Generate content using the AI service
        aiService.generateContent(prompt, { type: content_type, length })
          .then(aiResponse => {
            // If chapter_id and section_id are provided, create a content block
            if (chapter_id && section_id) {
              const contentBlock = {
                section_id,
                type: content_type || 'paragraph',
                content: aiResponse.content,
                validation_status: aiResponse.validation_status
              };

              addEntity(mockData.contentBlocks, contentBlock);
            }

            res.status(200).json({
              success: true,
              data: {
                content: aiResponse.content,
                validation_status: aiResponse.validation_status,
                suggestions: [
                  'Consider adding a code example',
                  'Add more technical details',
                  'Include relevant citations'
                ],
                model: aiResponse.model,
                usage: aiResponse.usage
              },
              message: 'Content generated successfully',
              timestamp: new Date().toISOString()
            });
          })
          .catch(error => {
            console.error('AI Service Error:', error);
            res.status(500).json({
              success: false,
              error: {
                code: 'AI_SERVICE_ERROR',
                message: 'Error generating content with AI service',
                details: error.message
              },
              timestamp: new Date().toISOString()
            });
          });
      } catch (error) {
        console.error('Request Error:', error);
        res.status(500).json({
          success: false,
          error: {
            code: 'INTERNAL_ERROR',
            message: 'An error occurred processing the request',
            details: error.message
          },
          timestamp: new Date().toISOString()
        });
      }
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