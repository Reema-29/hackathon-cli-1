// Mock API for book management
// Based on contracts/book-management.yaml

import { mockData, addEntity, findEntity, updateEntity, deleteEntity } from '../../../utils/dataStorage';

export default function handler(req, res) {
  const { method } = req;

  switch (method) {
    case 'GET':
      // GET /api/books - List all books for the authenticated user
      const { status } = req.query;
      let books = [...mockData.books];

      if (status) {
        books = books.filter(book => book.status === status);
      }

      res.status(200).json({
        success: true,
        data: {
          books,
          total: books.length,
          limit: 20,
          offset: 0
        },
        message: 'Books retrieved successfully',
        timestamp: new Date().toISOString()
      });
      break;

    case 'POST':
      // POST /api/books - Create a new book project
      const { title, author, description, target_audience } = req.body;

      // Validation
      if (!title || !author) {
        return res.status(400).json({
          success: false,
          error: {
            code: 'VALIDATION_ERROR',
            message: 'Title and author are required',
            details: { title: !title ? 'Title is required' : null, author: !author ? 'Author is required' : null }
          },
          timestamp: new Date().toISOString()
        });
      }

      if (title.length < 1 || title.length > 200) {
        return res.status(400).json({
          success: false,
          error: {
            code: 'VALIDATION_ERROR',
            message: 'Title must be between 1 and 200 characters',
            details: { title: 'Title length must be 1-200 characters' }
          },
          timestamp: new Date().toISOString()
        });
      }

      if (author.length < 1 || author.length > 100) {
        return res.status(400).json({
          success: false,
          error: {
            code: 'VALIDATION_ERROR',
            message: 'Author must be between 1 and 100 characters',
            details: { author: 'Author length must be 1-100 characters' }
          },
          timestamp: new Date().toISOString()
        });
      }

      const newBook = {
        title,
        author,
        description: description || '',
        status: 'draft',
        creation_date: new Date().toISOString(),
        last_modified: new Date().toISOString(),
        target_audience: target_audience || '',
        docusaurus_config: {}
      };

      const createdBook = addEntity(mockData.books, newBook);

      res.status(201).json({
        success: true,
        data: createdBook,
        message: 'Book created successfully',
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