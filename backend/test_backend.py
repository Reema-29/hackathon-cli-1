import asyncio
import os
from pathlib import Path
import sys

# Add the backend directory to the path so we can import modules
sys.path.append(str(Path(__file__).parent))

async def test_backend():
    """Test the backend functionality"""
    print("Testing backend functionality...")
    
    # Test that required modules can be imported
    print("\n1. Testing module imports...")
    try:
        from main import app
        print("[OK] Main app import successful!")
    except ImportError as e:
        print(f"[ERR] Main app import failed: {e}")
    
    try:
        from chatbot.rag_service import rag_chatbot
        print("[OK] RAG chatbot import successful!")
    except ImportError as e:
        print(f"[ERR] RAG chatbot import failed: {e}")
    
    try:
        from vector_store.qdrant_config import qdrant_manager
        if qdrant_manager is not None:
            print("[OK] Qdrant manager import successful!")
        else:
            print("[WARN] Qdrant manager import successful but not connected (expected if Qdrant is not running)")
    except ImportError as e:
        print(f"[ERR] Qdrant manager import failed: {e}")
    
    try:
        from document_processing.processor import document_processor
        print("[OK] Document processor import successful!")
    except ImportError as e:
        print(f"[ERR] Document processor import failed: {e}")
    
    print("\nBackend functionality test completed!")

if __name__ == "__main__":
    asyncio.run(test_backend())