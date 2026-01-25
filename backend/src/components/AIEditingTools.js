import React, { useState } from 'react';
import aiService from '../utils/aiService';

// Component for AI-assisted editing features
const AIEditingTools = ({ content, onContentUpdate }) => {
  const [isProcessing, setIsProcessing] = useState(false);
  const [activeTool, setActiveTool] = useState(null);

  // Function to handle AI processing
  const processWithAI = async (operation, options = {}) => {
    if (!content.trim()) {
      alert('No content to process');
      return;
    }

    setIsProcessing(true);
    setActiveTool(operation);

    try {
      let processedContent = content;
      let suggestions = [];

      switch (operation) {
        case 'improve':
          processedContent = await aiService.improveContent(content, options.suggestions || []);
          suggestions = ['Content improved for clarity and flow'];
          break;

        case 'summarize':
          processedContent = `Summary of the content: ${content.substring(0, 100)}... [AI Summarization would go here]`;
          suggestions = ['Content summarized'];
          break;

        case 'expand':
          processedContent = `${content} [Expanded content would be generated here based on the existing content]`;
          suggestions = ['Content expanded with additional details'];
          break;

        case 'simplify':
          processedContent = `Simplified version: ${content}`; // In a real implementation, this would actually simplify
          suggestions = ['Content simplified for better understanding'];
          break;

        case 'grammar':
          processedContent = `Grammar-checked version: ${content}`; // In a real implementation, this would check grammar
          suggestions = ['Grammar and style improved'];
          break;

        default:
          throw new Error(`Unknown operation: ${operation}`);
      }

      // Update the parent with the processed content
      if (onContentUpdate) {
        onContentUpdate(processedContent, suggestions);
      }
    } catch (error) {
      console.error(`AI ${operation} error:`, error);
      alert(`Error processing content: ${error.message}`);
    } finally {
      setIsProcessing(false);
      setActiveTool(null);
    }
  };

  return (
    <div className="ai-editing-tools">
      <div className="margin-vert--md">
        <h4>AI Editing Tools</h4>
      </div>

      <div className="margin-vert--sm">
        <div className="button-group button-group--block">
          <button
            className={`button button--sm ${activeTool === 'improve' ? 'button--primary' : 'button--outline button--secondary'}`}
            onClick={() => processWithAI('improve', { suggestions: ['Improve clarity', 'Enhance flow'] })}
            disabled={isProcessing}
          >
            Improve Content
          </button>
          <button
            className={`button button--sm ${activeTool === 'summarize' ? 'button--primary' : 'button--outline button--secondary'}`}
            onClick={() => processWithAI('summarize')}
            disabled={isProcessing}
          >
            Summarize
          </button>
          <button
            className={`button button--sm ${activeTool === 'expand' ? 'button--primary' : 'button--outline button--secondary'}`}
            onClick={() => processWithAI('expand')}
            disabled={isProcessing}
          >
            Expand
          </button>
        </div>
      </div>

      <div className="margin-vert--sm">
        <div className="button-group button-group--block">
          <button
            className={`button button--sm ${activeTool === 'simplify' ? 'button--primary' : 'button--outline button--secondary'}`}
            onClick={() => processWithAI('simplify')}
            disabled={isProcessing}
          >
            Simplify
          </button>
          <button
            className={`button button--sm ${activeTool === 'grammar' ? 'button--primary' : 'button--outline button--secondary'}`}
            onClick={() => processWithAI('grammar')}
            disabled={isProcessing}
          >
            Grammar Check
          </button>
        </div>
      </div>

      {isProcessing && (
        <div className="alert alert--info margin-vert--sm">
          AI is processing your content, please wait...
        </div>
      )}
    </div>
  );
};

export default AIEditingTools;