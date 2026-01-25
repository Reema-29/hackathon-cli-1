// Mock API for specific chapter operations
// Based on contracts/content-management.yaml

import { mockData, findEntity, updateEntity } from '../../../../../utils/dataStorage';

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

  // Find the chapter
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
    case 'PUT':
      // PUT /api/books/{bookId}/chapters/{chapterId} - Update chapter details
      const { title, number, content, status } = req.body;

      const updates = {};
      if (title !== undefined) updates.title = title;
      if (number !== undefined) updates.number = number;
      if (content !== undefined) updates.content = content;
      if (status !== undefined) updates.status = status;

      const updatedChapter = updateEntity(mockData.chapters, chapterId, updates);

      if (!updatedChapter) {
        return res.status(404).json({
          success: false,
          error: {
            code: 'CHAPTER_NOT_FOUND',
            message: `Chapter with ID ${chapterId} not found`
          },
          timestamp: new Date().toISOString()
        });
      }

      res.status(200).json({
        success: true,
        data: updatedChapter,
        message: 'Chapter updated successfully',
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