from sqlalchemy import create_engine, Column, Integer, String, DateTime, Text, Boolean
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker
from datetime import datetime
import os
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Database URL from environment variable
DATABASE_URL = os.getenv("NEON_DB_URL", "postgresql://username:password@localhost/dbname")

# Create engine
engine = create_engine(DATABASE_URL)

# Create session
SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)

# Base class for models
Base = declarative_base()

# Define models
class Document(Base):
    __tablename__ = "documents"
    
    id = Column(Integer, primary_key=True, index=True)
    filename = Column(String, index=True)
    content_hash = Column(String, unique=True, index=True)  # To avoid duplicates
    content = Column(Text)  # Full content of the document
    metadata_json = Column(Text)  # Store metadata as JSON string
    processed = Column(Boolean, default=False)
    created_at = Column(DateTime, default=datetime.utcnow)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)

class DocumentChunk(Base):
    __tablename__ = "document_chunks"
    
    id = Column(Integer, primary_key=True, index=True)
    document_id = Column(Integer, index=True)  # Reference to Document.id
    chunk_text = Column(Text)
    chunk_index = Column(Integer)
    embedding_id = Column(String)  # Reference to Qdrant point ID
    created_at = Column(DateTime, default=datetime.utcnow)

class ChatSession(Base):
    __tablename__ = "chat_sessions"
    
    id = Column(Integer, primary_key=True, index=True)
    session_id = Column(String, unique=True, index=True)
    user_id = Column(String, index=True)  # Could be anonymous user ID
    created_at = Column(DateTime, default=datetime.utcnow)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)
    active = Column(Boolean, default=True)

class ChatMessage(Base):
    __tablename__ = "chat_messages"
    
    id = Column(Integer, primary_key=True, index=True)
    session_id = Column(String, index=True)  # Reference to ChatSession.session_id
    role = Column(String)  # 'user' or 'assistant'
    content = Column(Text)
    timestamp = Column(DateTime, default=datetime.utcnow)
    sources = Column(Text)  # Store sources as JSON string

# Create tables
Base.metadata.create_all(bind=engine)

# Dependency to get DB session
def get_db():
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()