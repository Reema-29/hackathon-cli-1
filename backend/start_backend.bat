@echo off
REM Batch script to start the RAG chatbot backend on Windows

REM Navigate to the backend directory
cd /d "%~dp0"

REM Check if virtual environment exists, if not create it
if not exist "venv" (
    echo Creating virtual environment...
    python -m venv venv
)

REM Activate virtual environment
call venv\Scripts\activate.bat

REM Install dependencies
pip install -r requirements.txt

REM Run the FastAPI application
uvicorn main:app --reload --host 0.0.0.0 --port 8000