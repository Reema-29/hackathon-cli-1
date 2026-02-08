import asyncio
import os
from typing import List, Dict, Any
from langchain_openai import OpenAIEmbeddings
from qdrant_client.http import models
import openai
from vector_store.qdrant_config import qdrant_manager
# Optional database imports - commented out for now since they're not essential for core functionality
# from .db.models import ChatSession, ChatMessage
# from .db.database import get_db_session
import logging

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class RAGChatbot:
    def __init__(self):
        openai.api_key = os.getenv("OPENAI_API_KEY")
        self._embeddings = None  # Lazy load embeddings when needed
    
    @property
    def embeddings(self):
        if self._embeddings is None:
            self._embeddings = OpenAIEmbeddings()
        return self._embeddings
    
    async def retrieve_context(self, query: str, limit: int = 5) -> tuple[List[str], List[Dict[str, Any]]]:
        """Retrieve relevant context from the vector store based on the query"""
        try:
            # Check if Qdrant manager is available
            if qdrant_manager is None:
                logger.error("Qdrant manager is not available. Cannot retrieve context.")
                return [], []
            
            # Generate embedding for the query
            query_embedding = await self.embeddings.aembed_query(query)
            
            # Search for similar documents in Qdrant
            search_results = qdrant_manager.search(query_vector=query_embedding, limit=limit)
            
            # Extract content and sources
            context_texts = [result.payload["content"] for result in search_results]
            sources = [
                {
                    "text": result.payload["content"][:200] + "...",
                    "source": result.payload.get("source", "unknown"),
                    "score": result.score,
                    "chunk_index": result.payload.get("chunk_index", 0)
                }
                for result in search_results
            ]
            
            return context_texts, sources
        except Exception as e:
            logger.error(f"Error retrieving context: {str(e)}")
            return [], []
    
    async def generate_response(self, user_query: str, context_texts: List[str], 
                               conversation_history: List[Dict[str, str]] = None, 
                               temperature: float = 0.7) -> str:
        """Generate a response using OpenAI API with the provided context"""
        try:
            # Construct the context for the LLM
            context_str = "\n\n".join(context_texts)
            
            # Create the system message with context
            system_message = f"""
            You are an AI assistant for the book on Physical AI & Humanoid Robotics. 
            Answer the user's question based on the following context from the book:
            
            {context_str}
            
            If the context doesn't contain enough information to answer the question, say so.
            Be concise and accurate in your responses.
            """
            
            # Prepare messages for OpenAI API
            messages = [
                {"role": "system", "content": system_message},
            ]
            
            # Add conversation history if provided
            if conversation_history:
                for msg in conversation_history:
                    messages.append({"role": msg["role"], "content": msg["content"]})
            
            # Add the current user query
            messages.append({"role": "user", "content": user_query})
            
            # Call OpenAI API
            response = await openai.ChatCompletion.acreate(
                model="gpt-3.5-turbo",
                messages=messages,
                temperature=temperature,
                max_tokens=1000
            )
            
            assistant_reply = response.choices[0].message.content
            return assistant_reply
        except Exception as e:
            logger.error(f"Error generating response: {str(e)}")
            return "Sorry, I encountered an error while processing your request."
    
    async def chat_with_rag(self, user_query: str, selected_text: str = None, 
                           conversation_history: List[Dict[str, str]] = None, 
                           temperature: float = 0.7) -> Dict[str, Any]:
        """Main method to handle RAG-based chat"""
        try:
            context_texts = []
            sources = []
            
            if selected_text and len(selected_text.strip()) > 0:
                # Use only the selected text for context
                context_texts = [selected_text]
                sources = [{"text": selected_text[:200] + "...", "source": "selected_text", "score": 1.0}]
            else:
                # Retrieve context from vector store
                context_texts, sources = await self.retrieve_context(user_query)
            
            # Generate response
            response = await self.generate_response(
                user_query=user_query,
                context_texts=context_texts,
                conversation_history=conversation_history,
                temperature=temperature
            )
            
            return {
                "response": response,
                "sources": sources
            }
        except Exception as e:
            logger.error(f"Error in RAG chat: {str(e)}")
            return {
                "response": "Sorry, I encountered an error while processing your request.",
                "sources": []
            }

# Global instance
rag_chatbot = RAGChatbot()

def get_rag_chatbot():
    """Function to get the global RAG chatbot instance"""
    return rag_chatbot