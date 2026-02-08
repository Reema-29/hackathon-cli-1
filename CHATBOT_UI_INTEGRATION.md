# Enhanced Python Chatbot UI Integration

This project integrates a Python-based chatbot UI using Gradio that connects to the existing backend API for the Physical AI & Humanoid Robotics book assistant.

## Features

- Interactive chat interface built with Gradio
- Connects to the existing FastAPI backend
- Displays sources for AI-generated responses
- Conversation history management
- Ability to save conversations to text files
- Clean, user-friendly interface with avatars

## Setup Instructions

### Prerequisites

- Python 3.8+
- The backend server must be running on `http://localhost:8000`

### Installation

1. Navigate to the backend directory:
   ```bash
   cd backend
   ```

2. Install the required dependencies:
   ```bash
   pip install -r requirements.txt
   ```
   
   Or install Gradio specifically:
   ```bash
   pip install gradio
   ```

### Running the Application

1. Start the backend server in one terminal:
   ```bash
   cd backend
   python main.py
   ```

2. In a separate terminal, start the chatbot UI:
   ```bash
   cd backend
   python run_chatbot_ui.py
   ```
   
   Or use the startup script:
   - On Windows: `start_chatbot_ui.bat`
   - On Linux/Mac: `start_chatbot_ui.sh`

3. Open your browser and navigate to `http://localhost:7860`

## Files Added

- `chatbot_ui.py` - Main Gradio chatbot implementation
- `run_chatbot_ui.py` - Script to run the chatbot UI with automatic Gradio installation
- `start_chatbot_ui.bat` - Windows startup script
- `start_chatbot_ui.sh` - Linux/Mac startup script
- `test_chatbot.py` - Test script to verify functionality

## Configuration

The chatbot UI uses the following environment variable:
- `BACKEND_URL` - The URL of the backend API (defaults to `http://localhost:8000`)

## Usage

1. Type your question about Physical AI & Humanoid Robotics in the input box
2. Press Enter or click "Send" to get a response
3. The AI will use the book content to answer your questions
4. Sources used for the response will be displayed below the answer
5. Use "Clear History" to reset the conversation
6. Use "Save Conversation" to save the current chat to a text file

## Architecture

The enhanced chatbot UI follows these principles:
- Separation of concerns: UI layer (Gradio) communicates with backend API
- Reuses existing backend RAG functionality
- Maintains conversation history
- Formats sources appropriately for display