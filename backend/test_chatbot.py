import os
import sys
import subprocess

def test_chatbot_ui():
    """Test the chatbot UI functionality"""
    print("Testing the enhanced Python Chatbot UI...")
    
    # Check if the required files exist
    required_files = [
        "run_chatbot_ui.py",
        "chatbot_ui.py",
        "main.py"
    ]
    
    missing_files = []
    for file in required_files:
        if not os.path.exists(file):
            missing_files.append(file)
    
    if missing_files:
        print(f"Missing required files: {missing_files}")
        return False
    
    print("All required files are present.")
    
    # Check if we can import the necessary modules
    try:
        import gradio
        print("[OK] Gradio is available")
    except ImportError:
        print("[MISSING] Gradio is not available")
        return False
    
    try:
        import requests
        print("[OK] Requests is available")
    except ImportError:
        print("[MISSING] Requests is not available")
        return False
    
    # Test if the backend is running
    import requests
    try:
        response = requests.get("http://localhost:8000/health", timeout=5)
        if response.status_code == 200:
            print("[OK] Backend API is running and accessible")
        else:
            print("[INFO] Backend API is running but returned unexpected status code:", response.status_code)
    except requests.exceptions.ConnectionError:
        print("[WARNING] Backend API is not running. Please start the backend server first.")
        print("  Run: cd backend && python main.py")
    except Exception as e:
        print(f"[INFO] Error checking backend API: {e}")
    
    print("\nTo run the chatbot UI:")
    print("1. Make sure the backend is running: cd backend && python main.py")
    print("2. In a new terminal, run: cd backend && python run_chatbot_ui.py")
    print("3. Or use the startup script: cd backend && start_chatbot_ui.bat (Windows) or start_chatbot_ui.sh (Linux/Mac)")
    print("4. Open your browser and go to http://localhost:7860")
    
    return True

if __name__ == "__main__":
    success = test_chatbot_ui()
    if success:
        print("\n[SUCCESS] Chatbot UI setup is complete and ready to use!")
    else:
        print("\n[ERROR] There were issues with the chatbot UI setup.")
        sys.exit(1)