from flask import Flask, render_template_string
import webbrowser
import threading
import time

app = Flask(__name__)

# HTML template for your book content
HTML_TEMPLATE = '''
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Physical AI & Humanoid Robotics</title>
    <style>
        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            max-width: 1000px;
            margin: 0 auto;
            padding: 20px;
            line-height: 1.6;
            background-color: #f8f9fa;
            color: #333;
        }
        .header {
            text-align: center;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            color: white;
            padding: 30px;
            border-radius: 10px;
            margin-bottom: 30px;
            box-shadow: 0 4px 6px rgba(0,0,0,0.1);
        }
        h1 {
            margin: 0;
            font-size: 2.5em;
        }
        .subtitle {
            font-size: 1.2em;
            opacity: 0.9;
            margin-top: 10px;
        }
        .nav {
            background-color: #e9ecef;
            padding: 15px;
            border-radius: 8px;
            margin-bottom: 25px;
            text-align: center;
        }
        .nav a {
            margin: 0 10px;
            padding: 8px 15px;
            text-decoration: none;
            color: #495057;
            background-color: white;
            border-radius: 5px;
            border: 1px solid #ced4da;
        }
        .nav a:hover {
            background-color: #dee2e6;
            text-decoration: underline;
        }
        .content {
            background-color: white;
            padding: 30px;
            border-radius: 10px;
            box-shadow: 0 2px 4px rgba(0,0,0,0.1);
        }
        h2 {
            color: #495057;
            border-bottom: 2px solid #667eea;
            padding-bottom: 10px;
            margin-top: 30px;
        }
        h3 {
            color: #6c757d;
        }
        .chapter {
            margin-bottom: 30px;
            padding: 20px;
            background-color: #f8f9fa;
            border-radius: 8px;
            border-left: 4px solid #667eea;
        }
        ul, ol {
            padding-left: 20px;
        }
        li {
            margin-bottom: 8px;
        }
        .feature-box {
            background-color: #e8f4fd;
            padding: 15px;
            border-radius: 8px;
            margin: 15px 0;
        }
        footer {
            margin-top: 40px;
            text-align: center;
            color: #6c757d;
            font-size: 0.9em;
            padding: 20px;
            border-top: 1px solid #e9ecef;
        }
    </style>
</head>
<body>
    <div class="header">
        <h1>Physical AI & Humanoid Robotics</h1>
        <div class="subtitle">Advancing the frontier of embodied artificial intelligence</div>
    </div>
    
    <div class="nav">
        <a href="#intro">Introduction</a>
        <a href="#chapters">Chapters</a>
        <a href="#research">Research</a>
        <a href="#ai">AI Integration</a>
        <a href="#citations">Citations</a>
        <a href="#digital-twin">Digital Twin</a>
        <a href="#books">Books</a>
    </div>

    <div class="content">
        <div class="chapter" id="intro">
            <h2>Introduction</h2>
            <p>Welcome to the AI/Spec-Driven Book Creation System. This platform enables you to create high-quality books using AI assistance while maintaining spec-driven development principles and APA citation compliance.</p>
            
            <div class="feature-box">
                <h3>Key Features</h3>
                <ul>
                    <li><strong>AI-Assisted Content Generation</strong>: Generate content with AI assistance while maintaining quality control</li>
                    <li><strong>Concurrent Research</strong>: Research while writing, not all upfront</li>
                    <li><strong>APA Citation Compliance</strong>: Automatic validation of APA-style citations</li>
                    <li><strong>Spec-Driven Workflow</strong>: Follow specifications to ensure quality and consistency</li>
                    <li><strong>Docusaurus Integration</strong>: Built on Docusaurus for excellent documentation capabilities</li>
                </ul>
            </div>
            
            <h3>Getting Started</h3>
            <p>To get started with creating your first book:</p>
            <ol>
                <li>Create a new book project</li>
                <li>Define your specifications</li>
                <li>Begin writing with AI assistance</li>
                <li>Validate your content and citations</li>
                <li>Publish to your Docusaurus site</li>
            </ol>
            
            <h3>Architecture</h3>
            <p>The system consists of several key components:</p>
            <ul>
                <li>AI Integration Layer for content generation</li>
                <li>Research Manager for concurrent research</li>
                <li>Validation Engine for quality assurance</li>
                <li>Docusaurus Integration for publishing</li>
                <li>Data Storage for content management</li>
            </ul>
        </div>

        <div class="chapter" id="chapters">
            <h2>Chapters - ROS2 Nervous System</h2>
            <p>Your book contains several comprehensive chapters covering ROS2 Nervous System:</p>
            <ul>
                <li><strong>Concepts</strong>: Fundamental principles and theoretical foundations</li>
                <li><strong>Architecture</strong>: System design and component organization</li>
                <li><strong>Jetson Integration</strong>: Implementation on NVIDIA Jetson platforms</li>
                <li><strong>Lab Exercises</strong>: Practical hands-on activities</li>
                <li><strong>Quiz</strong>: Assessment and knowledge validation</li>
            </ul>
        </div>

        <div class="chapter" id="research">
            <h2>Research</h2>
            <p>This section covers concurrent research methodologies and validation techniques:</p>
            <ul>
                <li><strong>Concurrent Research</strong>: Research methodologies that happen alongside content creation</li>
                <li><strong>Validation</strong>: Techniques to verify research accuracy and relevance</li>
            </ul>
        </div>

        <div class="chapter" id="ai">
            <h2>AI Integration</h2>
            <p>Covers AI-assisted content generation and validation approaches:</p>
            <ul>
                <li><strong>Generation</strong>: AI-powered content creation techniques</li>
                <li><strong>Validation</strong>: Methods to ensure AI-generated content quality</li>
            </ul>
        </div>

        <div class="chapter" id="citations">
            <h2>Citation Management</h2>
            <p>Details about APA-style citations and validation processes:</p>
            <ul>
                <li><strong>APA Format</strong>: Standards for academic citations</li>
                <li><strong>Validation</strong>: Automated checking of citation formats</li>
            </ul>
        </div>

        <div class="chapter" id="digital-twin">
            <h2>Digital Twin (Gazebo & Unity)</h2>
            <p>Information about physics simulation, Unity digital twins, and sensor simulation:</p>
            <ul>
                <li><strong>Physics Simulation</strong>: Accurate modeling of physical behaviors</li>
                <li><strong>Unity Digital Twins</strong>: Realistic 3D representations</li>
                <li><strong>Sensor Simulation</strong>: Virtual sensors for testing and development</li>
            </ul>
        </div>

        <div class="chapter" id="books">
            <h2>Books</h2>
            <p>Management and creation of book projects:</p>
            <ul>
                <li><strong>Create</strong>: Setting up new book projects</li>
                <li><strong>Manage</strong>: Organizing and maintaining book content</li>
            </ul>
        </div>
    </div>

    <footer>
        <p>Physical AI & Humanoid Robotics Book - All content displayed</p>
        <p>© {{ year }} - Your Book Project</p>
    </footer>
</body>
</html>
'''

@app.route('/')
def home():
    from datetime import datetime
    return render_template_string(HTML_TEMPLATE, year=datetime.now().year)

def open_browser():
    # Wait a bit for the server to start, then open the browser
    time.sleep(2)
    webbrowser.open('http://localhost:3000')

if __name__ == '__main__':
    # Start the browser opening in a separate thread
    browser_thread = threading.Thread(target=open_browser)
    browser_thread.start()
    
    print("Starting server at http://localhost:3000")
    print("Your book content will be displayed in your browser shortly...")
    
    # Run the Flask app
    app.run(port=3000, debug=False, use_reloader=False)