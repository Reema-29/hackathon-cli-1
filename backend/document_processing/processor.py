import asyncio
import os
from typing import List, Dict, Any
from pathlib import Path
import PyPDF2
from langchain_text_splitters import RecursiveCharacterTextSplitter
from langchain_openai import OpenAIEmbeddings
from qdrant_client.http import models
import uuid
from vector_store.qdrant_config import qdrant_manager
import logging

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class DocumentProcessor:
    def __init__(self):
        self.text_splitter = RecursiveCharacterTextSplitter(
            chunk_size=1000,
            chunk_overlap=200,
            length_function=len,
        )
        self._embeddings = None  # Lazy load embeddings when needed

    @property
    def embeddings(self):
        if self._embeddings is None:
            from langchain_openai import OpenAIEmbeddings
            self._embeddings = OpenAIEmbeddings()
        return self._embeddings
    
    async def extract_text_from_pdf(self, pdf_path: str) -> str:
        """Extract text from a PDF file"""
        text = ""
        try:
            with open(pdf_path, 'rb') as file:
                reader = PyPDF2.PdfReader(file)
                for page in reader.pages:
                    text += page.extract_text() + "\n"
        except Exception as e:
            logger.error(f"Error extracting text from PDF {pdf_path}: {str(e)}")
            raise
        return text
    
    async def extract_text_from_md(self, md_path: str) -> str:
        """Extract text from a markdown file"""
        try:
            async with aiofiles.open(md_path, 'r', encoding='utf-8') as file:
                content = await file.read()
                return content
        except Exception as e:
            logger.error(f"Error reading markdown file {md_path}: {str(e)}")
            raise
    
    async def process_document(self, file_path: str, metadata: Dict[str, Any] = None) -> bool:
        """Process a document and store embeddings in Qdrant"""
        try:
            file_path_obj = Path(file_path)
            file_extension = file_path_obj.suffix.lower()
            
            # Extract text based on file type
            if file_extension == '.pdf':
                content = await self.extract_text_from_pdf(file_path)
            elif file_extension in ['.md', '.txt']:
                content = await self.extract_text_from_md(file_path)
            else:
                raise ValueError(f"Unsupported file type: {file_extension}")
            
            # Split content into chunks
            chunks = self.text_splitter.split_text(content)
            
            # Prepare embeddings for Qdrant
            points = []
            
            for i, chunk in enumerate(chunks):
                # Generate embedding for the chunk
                embedding = await self.embeddings.aembed_query(chunk)
                
                # Create a point for Qdrant
                point = models.PointStruct(
                    id=str(uuid.uuid4()),
                    vector=embedding,
                    payload={
                        "content": chunk,
                        "source": str(file_path_obj.name),
                        "chunk_index": i,
                        "metadata": metadata or {}
                    }
                )
                points.append(point)
            
            # Check if Qdrant manager is available
            if qdrant_manager is None:
                logger.error("Qdrant manager is not available. Cannot process document.")
                return False
            
            # Upload points to Qdrant
            qdrant_manager.client.upsert(
                collection_name=qdrant_manager.collection_name,
                points=points
            )
            
            logger.info(f"Successfully processed {len(points)} chunks from document: {file_path_obj.name}")
            return True
            
        except Exception as e:
            logger.error(f"Error processing document {file_path}: {str(e)}")
            return False
    
    async def process_directory(self, directory_path: str, metadata: Dict[str, Any] = None) -> bool:
        """Process all supported documents in a directory"""
        try:
            directory = Path(directory_path)
            processed_count = 0
            
            for file_path in directory.rglob('*'):
                if file_path.is_file():
                    extension = file_path.suffix.lower()
                    if extension in ['.pdf', '.md', '.txt']:
                        success = await self.process_document(str(file_path), metadata)
                        if success:
                            processed_count += 1
            
            logger.info(f"Processed {processed_count} documents from {directory_path}")
            return True
        except Exception as e:
            logger.error(f"Error processing directory {directory_path}: {str(e)}")
            return False

# Global instance
document_processor = DocumentProcessor()

def get_document_processor():
    """Function to get the global document processor instance"""
    return document_processor