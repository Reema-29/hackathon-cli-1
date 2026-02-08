import os
from qdrant_client import QdrantClient
from qdrant_client.http import models
from typing import List, Dict, Any
import logging

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class QdrantManager:
    def __init__(self):
        # Get Qdrant configuration from environment variables
        self.host = os.getenv("QDRANT_HOST", "localhost")
        self.port = int(os.getenv("QDRANT_PORT", 6333))
        self.api_key = os.getenv("QDRANT_API_KEY")
        
        # Initialize Qdrant client
        if self.api_key:
            self.client = QdrantClient(
                url=self.host,
                api_key=self.api_key,
                port=self.port
            )
        else:
            self.client = QdrantClient(host=self.host, port=self.port)
        
        # Collection name for storing document chunks
        self.collection_name = "book_content"
        
        # Initialize the collection
        self._initialize_collection()
    
    def _initialize_collection(self):
        """Initialize the Qdrant collection if it doesn't exist"""
        try:
            collections = self.client.get_collections()
            collection_names = [collection.name for collection in collections.collections]
            
            if self.collection_name not in collection_names:
                self.client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=models.VectorParams(size=1536, distance=models.Distance.COSINE),  # OpenAI embedding size
                )
                logger.info(f"Created Qdrant collection: {self.collection_name}")
            else:
                logger.info(f"Qdrant collection {self.collection_name} already exists")
        except Exception as e:
            logger.error(f"Error initializing Qdrant collection: {str(e)}")
            # Don't raise the exception during initialization to allow the app to start
            # The error will be handled when actual operations are performed
    
    def add_vectors(self, vectors: List[Dict[str, Any]], payloads: List[Dict[str, Any]] = None):
        """Add vectors to the collection"""
        try:
            # Generate IDs for the vectors
            ids = [vec.pop('id') for vec in vectors]
            
            # Add points to the collection
            self.client.upsert(
                collection_name=self.collection_name,
                points=models.Batch(
                    ids=ids,
                    vectors=vectors,
                    payloads=payloads if payloads else [{} for _ in vectors]
                )
            )
            logger.info(f"Added {len(vectors)} vectors to collection {self.collection_name}")
            return True
        except Exception as e:
            logger.error(f"Error adding vectors to Qdrant: {str(e)}")
            return False
    
    def search(self, query_vector: List[float], limit: int = 5):
        """Search for similar vectors in the collection"""
        try:
            search_results = self.client.search(
                collection_name=self.collection_name,
                query_vector=query_vector,
                limit=limit,
                with_payload=True
            )
            return search_results
        except Exception as e:
            logger.error(f"Error searching in Qdrant: {str(e)}")
            return []
    
    def delete_collection(self):
        """Delete the collection (useful for resets)"""
        try:
            self.client.delete_collection(collection_name=self.collection_name)
            logger.info(f"Deleted Qdrant collection: {self.collection_name}")
            return True
        except Exception as e:
            logger.error(f"Error deleting Qdrant collection: {str(e)}")
            return False

# Global instance - Initialize but don't fail if Qdrant is not available
try:
    qdrant_manager = QdrantManager()
except Exception as e:
    logger.warning(f"Could not initialize Qdrant: {str(e)}. Some features may not work until Qdrant is available.")
    qdrant_manager = None

def get_qdrant_manager():
    """Function to get the global Qdrant manager instance"""
    return qdrant_manager