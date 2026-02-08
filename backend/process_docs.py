import os
import sys
from pathlib import Path
import asyncio
from dotenv import load_dotenv
import aiofiles
from typing import List

# Add the backend directory to the path so we can import main
sys.path.append(str(Path(__file__).parent))

from main import embed_document, DocumentUploadRequest

# Load environment variables
load_dotenv()

async def read_docusaurus_docs(docs_dir: str) -> str:
    """
    Recursively read all markdown files from the Docusaurus docs directory
    """
    content = ""
    
    for root, dirs, files in os.walk(docs_dir):
        for file in files:
            if file.endswith('.md') or file.endswith('.mdx'):
                file_path = os.path.join(root, file)
                async with aiofiles.open(file_path, 'r', encoding='utf-8') as f:
                    file_content = await f.read()
                    content += f"\n\n--- DOCUMENT: {file_path} ---\n\n{file_content}\n\n"
                    
    return content

async def process_book_documents():
    """
    Process all book documents and embed them into the vector store
    """
    print("Starting document processing...")
    
    # Read documents from the main docs directory
    docs_path = Path(__file__).parent.parent / "docs"
    if not docs_path.exists():
        print(f"Docs directory not found at {docs_path}")
        return
    
    print(f"Reading documents from {docs_path}")
    all_content = await read_docusaurus_docs(str(docs_path))
    
    if not all_content.strip():
        print("No content found in docs directory")
        return
    
    print(f"Found content with {len(all_content)} characters")
    
    # Create document upload request
    doc_request = DocumentUploadRequest(
        content=all_content,
        filename="physical_ai_humanoid_robotics_book.md",
        metadata={
            "source": "docusaurus_docs",
            "type": "book_content",
            "processed_at": str(asyncio.get_event_loop().time())
        }
    )
    
    try:
        # Embed the document
        print("Embedding document into vector store...")
        response = await embed_document(doc_request)
        print(f"Embedding result: {response.message}")
    except Exception as e:
        print(f"Error embedding document: {str(e)}")

if __name__ == "__main__":
    asyncio.run(process_book_documents())