from fastapi import FastAPI, HTTPException, Depends
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from typing import List, Optional
import os
import asyncio
from dotenv import load_dotenv
import openai
from qdrant_client import QdrantClient
from qdrant_client.http import models
import uuid
from langchain_text_splitters import RecursiveCharacterTextSplitter
from langchain_openai import OpenAIEmbeddings
import logging

# Load environment variables
load_dotenv()

# Initialize FastAPI app
app = FastAPI(
    title="RAG Chatbot API",
    description="API for Retrieval-Augmented Generation chatbot for the Physical AI & Humanoid Robotics book",
    version="1.0.0"
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with specific origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Initialize Qdrant client
QDRANT_HOST = os.getenv("QDRANT_HOST", "localhost")
QDRANT_PORT = int(os.getenv("QDRANT_PORT", 6333))
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")

if QDRANT_API_KEY:
    qdrant_client = QdrantClient(
        url=QDRANT_HOST,
        api_key=QDRANT_API_KEY,
        port=QDRANT_PORT
    )
else:
    qdrant_client = QdrantClient(host=QDRANT_HOST, port=QDRANT_PORT)

# Initialize OpenAI
openai.api_key = os.getenv("OPENAI_API_KEY")

# Define Pydantic models
class Message(BaseModel):
    role: str  # "user" or "assistant"
    content: str

class ChatRequest(BaseModel):
    messages: List[Message]
    user_query: str
    selected_text: Optional[str] = None  # Text selected by user for context
    temperature: float = 0.7

class DocumentChunk(BaseModel):
    content: str
    metadata: dict

class DocumentUploadRequest(BaseModel):
    content: str  # Document content as string
    filename: str
    metadata: dict = {}

class ChatResponse(BaseModel):
    response: str
    sources: List[dict]  # Sources used for the response

class EmbeddingResponse(BaseModel):
    success: bool
    message: str

# Collection name for storing document chunks
COLLECTION_NAME = "book_content"

# Initialize Qdrant collection if it doesn't exist
def initialize_qdrant_collection():
    try:
        collections = qdrant_client.get_collections()
        collection_names = [collection.name for collection in collections.collections]
        
        if COLLECTION_NAME not in collection_names:
            qdrant_client.create_collection(
                collection_name=COLLECTION_NAME,
                vectors_config=models.VectorParams(size=1536, distance=models.Distance.COSINE),  # OpenAI embedding size
            )
            logger.info(f"Created Qdrant collection: {COLLECTION_NAME}")
        else:
            logger.info(f"Qdrant collection {COLLECTION_NAME} already exists")
    except Exception as e:
        logger.error(f"Error initializing Qdrant collection: {str(e)}")
        raise

@app.on_event("startup")
async def startup_event():
    logger.info("Initializing Qdrant collection...")
    initialize_qdrant_collection()
    logger.info("Application startup complete")

@app.get("/")
async def root():
    return {"message": "RAG Chatbot API is running!"}

@app.post("/embed-document/", response_model=EmbeddingResponse)
async def embed_document(request: DocumentUploadRequest):
    """
    Embed a document into the vector store
    """
    try:
        # Initialize text splitter
        text_splitter = RecursiveCharacterTextSplitter(
            chunk_size=1000,
            chunk_overlap=200,
            length_function=len,
        )
        
        # Split the document content into chunks
        chunks = text_splitter.split_text(request.content)
        
        # Prepare embeddings for Qdrant
        embeddings = OpenAIEmbeddings()
        points = []
        
        for i, chunk in enumerate(chunks):
            # Generate embedding for the chunk
            embedding = await embeddings.aembed_query(chunk)
            
            # Create a point for Qdrant
            point = models.PointStruct(
                id=str(uuid.uuid4()),
                vector=embedding,
                payload={
                    "content": chunk,
                    "source": request.filename,
                    "chunk_index": i,
                    "metadata": request.metadata
                }
            )
            points.append(point)
        
        # Upload points to Qdrant
        qdrant_client.upsert(
            collection_name=COLLECTION_NAME,
            points=points
        )
        
        logger.info(f"Embedded {len(points)} chunks from document: {request.filename}")
        
        return EmbeddingResponse(
            success=True,
            message=f"Successfully embedded {len(points)} chunks from {request.filename}"
        )
    except Exception as e:
        logger.error(f"Error embedding document: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/chat/", response_model=ChatResponse)
async def chat(request: ChatRequest):
    """
    Main chat endpoint that handles RAG-based conversations
    """
    try:
        # If selected text is provided, use it as context
        context_texts = []
        sources = []
        
        if request.selected_text and len(request.selected_text.strip()) > 0:
            # Use only the selected text for context
            context_texts = [request.selected_text]
            sources = [{"text": request.selected_text[:200] + "...", "source": "selected_text"}]
        else:
            # Perform semantic search in the vector store
            embeddings = OpenAIEmbeddings()
            query_embedding = await embeddings.aembed_query(request.user_query)
            
            # Search for similar documents in Qdrant
            search_results = qdrant_client.search(
                collection_name=COLLECTION_NAME,
                query_vector=query_embedding,
                limit=5,  # Retrieve top 5 similar chunks
                with_payload=True
            )
            
            # Extract content and sources
            context_texts = [result.payload["content"] for result in search_results]
            sources = [
                {
                    "text": result.payload["content"][:200] + "...",
                    "source": result.payload.get("source", "unknown"),
                    "score": result.score
                }
                for result in search_results
            ]
        
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
        
        # Add previous conversation history (excluding the last user query which we're responding to)
        for msg in request.messages[:-1]:  # Exclude the last message since that's the current query
            messages.append({"role": msg.role, "content": msg.content})
        
        # Add the current user query
        messages.append({"role": "user", "content": request.user_query})
        
        # Call OpenAI API
        response = await openai.ChatCompletion.acreate(
            model="gpt-3.5-turbo",
            messages=messages,
            temperature=request.temperature,
            max_tokens=1000
        )
        
        assistant_reply = response.choices[0].message.content
        
        return ChatResponse(
            response=assistant_reply,
            sources=sources
        )
    except Exception as e:
        logger.error(f"Error in chat endpoint: {str(e)}")
        raise HTTPException(status_code=500, detail=str(e))

@app.get("/health")
async def health_check():
    """
    Health check endpoint
    """
    return {"status": "healthy"}

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)