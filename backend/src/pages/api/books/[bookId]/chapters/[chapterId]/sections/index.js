// Mock API for section management
// Based on contracts/content-management.yaml

import { mockData, addEntity, findEntity } from '../../../../../../utils/dataStorage';

export default function handler(req, res) {
  const { method } = req;
  const { bookId, chapterId } = req.query;

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

  // Verify the chapter exists
  const chapter = findEntity(mockData.chapters, chapterId);
  if (!chapter || chapter.book_id !== bookId) {
    return res.status(404).json({
      success: false,
      error: {
        code: 'CHAPTER_NOT_FOUND',
        message: `Chapter with ID ${chapterId} not found in book ${bookId}`
      },
      timestamp: new Date().toISOString()
    });
  }

  switch (method) {
    case 'POST':
      // POST /api/books/{bookId}/chapters/{chapterId}/sections - Create a new section in a chapter
      const { title, level, content, parent_id, order } = req.body;

      // Validation
      if (!title) {
        return res.status(400).json({
          success: false,
          error: {
            code: 'VALIDATION_ERROR',
            message: 'Title is required',
            details: { title: 'Title is required' }
          },
          timestamp: new Date().toISOString()
        });
      }

      if (level !== undefined && (typeof level !== 'number' || level < 1 || level > 6)) {
        return res.status(400).json({
          success: false,
          error: {
            code: 'VALIDATION_ERROR',
            message: 'Level must be between 1 and 6',
            details: { level: 'Level must be between 1 and 6' }
          },
          timestamp: new Date().toISOString()
        });
      }

      if (order !== undefined && (typeof order !== 'number' || order < 0)) {
        return res.status(400).json({
          success: false,
          error: {
            code: 'VALIDATION_ERROR',
            message: 'Order must be a non-negative integer',
            details: { order: 'Order must be a non-negative integer' }
          },
          timestamp: new Date().toISOString()
        });
      }

      // If parent_id is provided, verify it exists
      if (parent_id) {
        const parentSection = findEntity(mockData.sections, parent_id);
        if (!parentSection) {
          return res.status(404).json({
            success: false,
            error: {
              code: 'SECTION_NOT_FOUND',
              message: `Parent section with ID ${parent_id} not found`
            },
            timestamp: new Date().toISOString()
          });
        }
      }

      const newSection = {
        chapter_id: chapterId,
        parent_id: parent_id || null,
        title,
        level: level || 2, // Default to level 2 (H2)
        content: content || '',
        order: order || mockData.sections.filter(s => s.chapter_id === chapterId).length,
        creation_date: new Date().toISOString(),
        last_modified: new Date().toISOString()
      };

      const createdSection = addEntity(mockData.sections, newSection);

      res.status(201).json({
        success: true,
        data: createdSection,
        message: 'Section created successfully',
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