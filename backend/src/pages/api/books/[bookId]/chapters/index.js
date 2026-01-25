// Mock API for chapter management
// Based on contracts/content-management.yaml

import { mockData, addEntity, findEntity } from '../../../../../utils/dataStorage';

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
    case 'GET':
      // GET /api/books/{bookId}/chapters - List all chapters in a book
      const { status } = req.query;
      let chapters = mockData.chapters.filter(chapter => chapter.book_id === bookId);

      if (status) {
        chapters = chapters.filter(chapter => chapter.status === status);
      }

      res.status(200).json({
        success: true,
        data: {
          chapters,
          total: chapters.length,
          limit: 20,
          offset: 0
        },
        message: 'Chapters retrieved successfully',
        timestamp: new Date().toISOString()
      });
      break;

    case 'POST':
      // POST /api/books/{bookId}/chapters - Create a new chapter in a book
      const { title, number, content, order } = req.body;

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

      if (number !== undefined && (typeof number !== 'number' || number <= 0)) {
        return res.status(400).json({
          success: false,
          error: {
            code: 'VALIDATION_ERROR',
            message: 'Number must be a positive integer',
            details: { number: 'Number must be a positive integer' }
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

      const newChapter = {
        book_id: bookId,
        title,
        number: number || mockData.chapters.filter(ch => ch.book_id === bookId).length + 1,
        status: 'draft',
        content: content || '',
        order: order || 0,
        creation_date: new Date().toISOString(),
        last_modified: new Date().toISOString()
      };

      const createdChapter = addEntity(mockData.chapters, newChapter);

      res.status(201).json({
        success: true,
        data: createdChapter,
        message: 'Chapter created successfully',
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