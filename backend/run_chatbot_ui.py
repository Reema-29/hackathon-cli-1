import subprocess
import sys
import os

def install_gradio_if_missing():
    """Install Gradio if it's not already installed"""
    try:
        import gradio
        print("Gradio is already installed.")
    except ImportError:
        print("Installing Gradio...")
        subprocess.check_call([sys.executable, "-m", "pip", "install", "gradio"])
        print("Gradio installed successfully.")

def run_chatbot_ui():
    """Run the Gradio chatbot UI"""
    # Install Gradio if needed
    install_gradio_if_missing()
    
    # Import here after potential installation
    import gradio as gr
    import requests
    import json
    from datetime import datetime
    
    # Backend API endpoint
    BACKEND_URL = os.getenv("BACKEND_URL", "http://localhost:8000")
    
    def format_sources(sources):
        """Format sources for display in the chatbot"""
        if not sources:
            return ""
        
        formatted_sources = "\n\n**Sources:**\n"
        for i, source in enumerate(sources[:3]):  # Limit to first 3 sources
            text_preview = source.get('text', '')[:100] + "..." if len(source.get('text', '')) > 100 else source.get('text', '')
            source_info = source.get('source', 'Unknown')
            formatted_sources += f"{i+1}. {text_preview} (Source: {source_info})\n"
        
        return formatted_sources
    
    def chat_with_bot(message, history):
        """
        Function to handle chat interactions with the backend API
        """
        try:
            # Prepare the conversation history for the API
            formatted_history = []
            for user_msg, bot_msg in history:
                formatted_history.append({"role": "user", "content": user_msg})
                if bot_msg:
                    formatted_history.append({"role": "assistant", "content": bot_msg})
            
            # Add the current user message
            formatted_history.append({"role": "user", "content": message})
            
            # Prepare the request payload
            payload = {
                "messages": formatted_history[-10:],  # Use last 10 messages to avoid exceeding limits
                "user_query": message,
                "selected_text": "",  # No selected text in this UI
                "temperature": 0.7
            }
            
            # Make request to backend API
            response = requests.post(
                f"{BACKEND_URL}/chat/",
                headers={"Content-Type": "application/json"},
                json=payload
            )
            
            if response.status_code == 200:
                data = response.json()
                bot_response = data.get("response", "Sorry, I couldn't process your request.")
                
                # Format sources if available
                sources_text = format_sources(data.get("sources", []))
                if sources_text:
                    bot_response += sources_text
                    
                return bot_response
            else:
                return f"Error: Received status code {response.status_code} from backend API."
        
        except requests.exceptions.ConnectionError:
            return "Error: Could not connect to the backend API. Please make sure the backend server is running."
        except Exception as e:
            return f"Error: {str(e)}"
    
    def clear_history():
        """Clear the chat history"""
        return []
    
    def save_conversation(history):
        """Save the current conversation to a file"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"conversation_{timestamp}.txt"
        
        with open(filename, "w", encoding="utf-8") as f:
            f.write("Chat Conversation History\n")
            f.write("="*30 + "\n\n")
            for i, (user_msg, bot_msg) in enumerate(history):
                f.write(f"Q{i+1}: {user_msg}\n")
                f.write(f"A{i+1}: {bot_msg}\n\n")
        
        return f"Conversation saved to {filename}"
    
    # Create Gradio interface
    with gr.Blocks(title="Physical AI & Humanoid Robotics Chatbot") as demo:
        gr.Markdown("# 🤖 Physical AI & Humanoid Robotics Chatbot")
        gr.Markdown("Ask questions about the Physical AI & Humanoid Robotics book. The AI will use the book content to answer your questions.")
        
        chatbot = gr.Chatbot(
            label="Conversation",
            bubble_full_width=False,
            avatar_images=(
                "https://cdn-icons-png.flaticon.com/512/4712/4712035.png",  # User avatar
                "https://cdn-icons-png.flaticon.com/512/4712/4712139.png"   # Bot avatar
            )
        )
        
        with gr.Row():
            msg = gr.Textbox(
                label="Your Question",
                placeholder="Ask something about Physical AI & Humanoid Robotics...",
                container=False,
                elem_classes="textbox"
            )
            submit_btn = gr.Button("Send", variant="primary")
        
        with gr.Row():
            clear_btn = gr.Button("Clear History", variant="secondary")
            save_btn = gr.Button("Save Conversation", variant="secondary")
            status = gr.Textbox(label="Status", interactive=False, visible=False)
        
        # Event handlers
        msg.submit(chat_with_bot, [msg, chatbot], [msg, chatbot], queue=False).then(
            lambda: gr.update(value=""), None, [msg], queue=False
        )
        submit_btn.click(chat_with_bot, [msg, chatbot], [msg, chatbot], queue=False).then(
            lambda: gr.update(value=""), None, [msg], queue=False
        )
        clear_btn.click(clear_history, None, chatbot, queue=False)
        save_btn.click(save_conversation, chatbot, status, queue=False)
    
    # Launch the app
    print("Starting Gradio chatbot UI...")
    print(f"Backend API URL: {BACKEND_URL}")
    print("Make sure the backend server is running on http://localhost:8000 before using the chatbot.")
    demo.launch(server_name="0.0.0.0", server_port=7860, share=False)

if __name__ == "__main__":
    run_chatbot_ui()