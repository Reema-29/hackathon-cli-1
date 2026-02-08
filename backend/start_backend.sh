#!/bin/bash
# Script to start the RAG chatbot backend

# Navigate to the backend directory
cd "$(dirname "$0")/../backend"

# Check if virtual environment exists, if not create it
if [ ! -d "venv" ]; then
    echo "Creating virtual environment..."
    python -m venv venv
fi

# Activate virtual environment
source venv/bin/activate  # On Windows use: venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt

# Run the FastAPI application
uvicorn main:app --reload --host 0.0.0.0 --port 8000