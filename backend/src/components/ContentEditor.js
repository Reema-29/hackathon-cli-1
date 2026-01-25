import React, { useState, useEffect } from 'react';
import aiService from '../utils/aiService';
import PromptManager from './PromptManager';
import AIEditingTools from './AIEditingTools';

// Mock component for content editing with AI assistance
const ContentEditor = ({ initialContent = '', onSave, onCancel }) => {
  const [content, setContent] = useState(initialContent);
  const [isProcessing, setIsProcessing] = useState(false);
  const [aiSuggestions, setAiSuggestions] = useState([]);
  const [activeTab, setActiveTab] = useState('editor'); // 'editor', 'ai'
  const [showEditingTools, setShowEditingTools] = useState(false);

  // Handle content changes
  const handleContentChange = (e) => {
    setContent(e.target.value);
  };

  // Generate content with AI using a prompt
  const handleAIGenerate = async (prompt) => {
    setIsProcessing(true);
    try {
      const response = await aiService.generateContent(prompt, { type: 'paragraph', length: 'medium' });
      setContent(prev => prev + ' ' + response.content);
      setAiSuggestions(response.suggestions || []);
    } catch (error) {
      console.error('AI generation error:', error);
      alert('Error generating content: ' + error.message);
    } finally {
      setIsProcessing(false);
    }
  };

  // Handle AI processing from editing tools
  const handleAIProcess = (processedContent, suggestions) => {
    setContent(processedContent);
    if (suggestions && suggestions.length > 0) {
      setAiSuggestions(prev => [...prev, ...suggestions]);
    }
  };

  return (
    <div className="content-editor">
      <div className="margin-vert--md">
        <h3>Content Editor</h3>
      </div>

      <div className="margin-vert--md">
        <div className="button-group button-group--block">
          <button
            className={`button ${activeTab === 'editor' ? 'button--primary' : 'button--outline button--secondary'}`}
            onClick={() => setActiveTab('editor')}
          >
            Content Editor
          </button>
          <button
            className={`button ${activeTab === 'ai' ? 'button--primary' : 'button--outline button--secondary'}`}
            onClick={() => setActiveTab('ai')}
          >
            AI Assistant
          </button>
        </div>
      </div>

      {activeTab === 'editor' ? (
        <div>
          <div className="margin-vert--md">
            <textarea
              className="form-control"
              value={content}
              onChange={handleContentChange}
              rows="10"
              placeholder="Write your content here..."
              style={{ width: '100%', fontFamily: 'monospace' }}
            />
          </div>

          <div className="margin-vert--md">
            <div className="button-group button-group--block">
              <button
                className="button button--primary"
                onClick={() => onSave && onSave(content)}
              >
                Save Content
              </button>
              <button
                className="button button--secondary"
                onClick={() => onCancel && onCancel()}
              >
                Cancel
              </button>
              <button
                className="button button--info"
                onClick={() => setShowEditingTools(!showEditingTools)}
              >
                {showEditingTools ? 'Hide AI Tools' : 'Show AI Tools'}
              </button>
            </div>
          </div>

          {showEditingTools && (
            <div className="margin-vert--md">
              <AIEditingTools
                content={content}
                onContentUpdate={handleAIProcess}
              />
            </div>
          )}
        </div>
      ) : (
        <div>
          <PromptManager onGenerate={handleAIGenerate} />

          <div className="margin-vert--md">
            <div className="button-group button-group--block">
              <button
                className="button button--primary"
                onClick={() => setActiveTab('editor')}
              >
                Switch to Editor
              </button>
            </div>
          </div>
        </div>
      )}

      {aiSuggestions.length > 0 && (
        <div className="margin-vert--md">
          <h4>AI Suggestions:</h4>
          <ul>
            {aiSuggestions.map((suggestion, index) => (
              <li key={index} className="alert alert--info">
                {suggestion}
              </li>
            ))}
          </ul>
        </div>
      )}

      {(isProcessing) && (
        <div className="alert alert--info">
          AI is processing your request, please wait...
        </div>
      )}
    </div>
  );
};

export default ContentEditor;