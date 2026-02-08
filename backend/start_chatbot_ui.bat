@echo off
echo Starting the Python Chatbot UI...

REM Activate the virtual environment if it exists
if exist venv\Scripts\activate.bat (
    call venv\Scripts\activate.bat
)

REM Install gradio if not already installed
python -c "import gradio" 2>nul || (
    echo Installing Gradio...
    pip install gradio
)

REM Run the chatbot UI
echo Starting Gradio chatbot UI...
python run_chatbot_ui.py

pause