from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker
from sqlalchemy.ext.declarative import declarative_base
from contextlib import contextmanager
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

@contextmanager
def get_db_session():
    """Context manager for database sessions"""
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()

def init_db():
    """Initialize the database and create tables"""
    Base.metadata.create_all(bind=engine)
    print("Database initialized successfully!")

if __name__ == "__main__":
    init_db()