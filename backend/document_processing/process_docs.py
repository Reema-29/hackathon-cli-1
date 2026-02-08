import asyncio
import os
from pathlib import Path
import sys
from dotenv import load_dotenv

# Add the backend directory to the path so we can import modules
sys.path.append(str(Path(__file__).parent.parent))

from .processor import document_processor

# Load environment variables
load_dotenv()

async def process_docusaurus_docs():
    """Process all markdown files in the Docusaurus docs directory"""
    # Get the path to the docs directory
    docs_path = Path(__file__).parent.parent.parent / "docs"
    
    if not docs_path.exists():
        print(f"Docs directory not found at {docs_path}")
        return False
    
    print(f"Processing documents from {docs_path}")
    
    # Process all markdown files in the docs directory
    success = await document_processor.process_directory(
        str(docs_path), 
        metadata={"source": "docusaurus_docs", "type": "book_content"}
    )
    
    if success:
        print("Successfully processed Docusaurus documentation!")
        return True
    else:
        print("Failed to process Docusaurus documentation.")
        return False

if __name__ == "__main__":
    asyncio.run(process_docusaurus_docs())