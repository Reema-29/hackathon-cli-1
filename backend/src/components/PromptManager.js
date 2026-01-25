import React, { useState } from 'react';
import { generatePromptFromTemplate, getAvailableTemplates } from '../utils/promptTemplates';

// Component for managing and using prompt templates
const PromptManager = ({ onGenerate }) => {
  const [selectedTemplate, setSelectedTemplate] = useState('');
  const [templateValues, setTemplateValues] = useState({});
  const [customPrompt, setCustomPrompt] = useState('');
  const [useTemplate, setUseTemplate] = useState(true);

  const templates = getAvailableTemplates();

  // Handle template selection
  const handleTemplateChange = (templateId) => {
    setSelectedTemplate(templateId);
    // Reset values when template changes
    const template = templates.find(t => t.id === templateId);
    if (template) {
      const initialValues = {};
      template.placeholders.forEach(placeholder => {
        initialValues[placeholder] = '';
      });
      setTemplateValues(initialValues);
    }
  };

  // Handle value changes for template placeholders
  const handleValueChange = (placeholder, value) => {
    setTemplateValues(prev => ({
      ...prev,
      [placeholder]: value
    }));
  };

  // Generate prompt based on template or custom input
  const generatePrompt = () => {
    let prompt = '';

    if (useTemplate && selectedTemplate) {
      try {
        prompt = generatePromptFromTemplate(selectedTemplate, templateValues);
      } catch (error) {
        alert('Error generating prompt: ' + error.message);
        return;
      }
    } else if (customPrompt.trim()) {
      prompt = customPrompt;
    } else {
      alert('Please enter a custom prompt or select a template with required values');
      return;
    }

    // Call the parent's generation function
    if (onGenerate) {
      onGenerate(prompt);
    }
  };

  return (
    <div className="prompt-manager">
      <div className="margin-vert--md">
        <h3>Prompt Management</h3>
      </div>

      <div className="margin-vert--md">
        <label>
          <input
            type="radio"
            checked={useTemplate}
            onChange={() => setUseTemplate(true)}
          /> Use Template
        </label>
        <label className="margin-left--md">
          <input
            type="radio"
            checked={!useTemplate}
            onChange={() => setUseTemplate(false)}
          /> Custom Prompt
        </label>
      </div>

      {useTemplate ? (
        <div>
          <div className="margin-vert--md">
            <label htmlFor="template-select">Select Template:</label>
            <select
              id="template-select"
              className="form-control"
              value={selectedTemplate}
              onChange={(e) => handleTemplateChange(e.target.value)}
            >
              <option value="">Choose a template...</option>
              {templates.map(template => (
                <option key={template.id} value={template.id}>
                  {template.name}
                </option>
              ))}
            </select>
          </div>

          {selectedTemplate && (() => {
            const template = templates.find(t => t.id === selectedTemplate);
            return (
              <div className="margin-vert--md">
                <div className="alert alert--info">
                  <strong>Description:</strong> {template.description}
                </div>

                {template.placeholders.map(placeholder => (
                  <div key={placeholder} className="margin-vert--sm">
                    <label htmlFor={`placeholder-${placeholder}`}>
                      {placeholder.charAt(0).toUpperCase() + placeholder.slice(1)}:
                    </label>
                    <input
                      type="text"
                      id={`placeholder-${placeholder}`}
                      className="form-control"
                      value={templateValues[placeholder] || ''}
                      onChange={(e) => handleValueChange(placeholder, e.target.value)}
                      placeholder={`Enter ${placeholder}...`}
                    />
                  </div>
                ))}
              </div>
            );
          })()}
        </div>
      ) : (
        <div className="margin-vert--md">
          <label htmlFor="custom-prompt">Custom Prompt:</label>
          <textarea
            id="custom-prompt"
            className="form-control"
            value={customPrompt}
            onChange={(e) => setCustomPrompt(e.target.value)}
            rows="4"
            placeholder="Enter your custom prompt here..."
          />
        </div>
      )}

      <div className="margin-vert--md">
        <button
          className="button button--primary"
          onClick={generatePrompt}
        >
          Generate Content
        </button>
      </div>
    </div>
  );
};

export default PromptManager;