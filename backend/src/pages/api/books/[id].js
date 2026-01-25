// Mock API for specific book operations
// Based on contracts/book-management.yaml

import { mockData, findEntity, updateEntity, deleteEntity } from '../../../utils/dataStorage';

export default function handler(req, res) {
  const { method } = req;
  const { id } = req.query;

  // Find the book
  const book = findEntity(mockData.books, id);

  if (!book) {
    return res.status(404).json({
      success: false,
      error: {
        code: 'BOOK_NOT_FOUND',
        message: `Book with ID ${id} not found`
      },
      timestamp: new Date().toISOString()
    });
  }

  switch (method) {
    case 'GET':
      // GET /api/books/{id} - Get details of a specific book
      res.status(200).json({
        success: true,
        data: book,
        message: 'Book retrieved successfully',
        timestamp: new Date().toISOString()
      });
      break;

    case 'PUT':
      // PUT /api/books/{id} - Update book details
      const { title, author, description, status } = req.body;

      const updates = {};
      if (title !== undefined) updates.title = title;
      if (author !== undefined) updates.author = author;
      if (description !== undefined) updates.description = description;
      if (status !== undefined) updates.status = status;

      const updatedBook = updateEntity(mockData.books, id, updates);

      if (!updatedBook) {
        return res.status(404).json({
          success: false,
          error: {
            code: 'BOOK_NOT_FOUND',
            message: `Book with ID ${id} not found`
          },
          timestamp: new Date().toISOString()
        });
      }

      res.status(200).json({
        success: true,
        data: updatedBook,
        message: 'Book updated successfully',
        timestamp: new Date().toISOString()
      });
      break;

    case 'DELETE':
      // DELETE /api/books/{id} - Delete a book project
      const deletedBook = deleteEntity(mockData.books, id);

      if (!deletedBook) {
        return res.status(404).json({
          success: false,
          error: {
            code: 'BOOK_NOT_FOUND',
            message: `Book with ID ${id} not found`
          },
          timestamp: new Date().toISOString()
        });
      }

      res.status(200).json({
        success: true,
        data: {},
        message: 'Book deleted successfully',
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