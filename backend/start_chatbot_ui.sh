#!/bin/bash
echo "Starting the Python Chatbot UI..."

# Activate the virtual environment if it exists
if [ -f venv/bin/activate ]; then
    source venv/bin/activate
fi

# Install gradio if not already installed
python -c "import gradio" 2>/dev/null || {
    echo "Installing Gradio..."
    pip install gradio
}

# Run the chatbot UI
echo "Starting Gradio chatbot UI..."
python run_chatbot_ui.py