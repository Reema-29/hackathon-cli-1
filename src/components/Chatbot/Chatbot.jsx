import React, { useState, useRef, useEffect } from 'react';
import { useColorMode } from '@docusaurus/theme-common';
import styles from './Chatbot.module.css';

const Chatbot = ({ title = "Book Assistant" }) => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([
    { id: 1, role: 'assistant', content: 'Hello! I\'m your AI assistant for the Physical AI & Humanoid Robotics book. How can I help you today?' }
  ]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [selectedText, setSelectedText] = useState('');
  const messagesEndRef = useRef(null);
  const inputRef = useRef(null);
  const { colorMode } = useColorMode();

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  // Function to get selected text from the page
  useEffect(() => {
    const handleSelection = () => {
      const selectedText = window.getSelection().toString().trim();
      if (selectedText) {
        setSelectedText(selectedText);
      }
    };

    document.addEventListener('mouseup', handleSelection);
    return () => {
      document.removeEventListener('mouseup', handleSelection);
    };
  }, []);

  const handleSubmit = async (e) => {
    e.preventDefault();
    if (!inputValue.trim() || isLoading) return;

    // Add user message
    const userMessage = {
      id: Date.now(),
      role: 'user',
      content: inputValue
    };
    
    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      // Call the backend API
      const response = await fetch('http://localhost:8000/chat/', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          messages: messages.map(msg => ({ role: msg.role, content: msg.content })),
          user_query: inputValue,
          selected_text: selectedText, // Pass selected text as context
          temperature: 0.7
        })
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data = await response.json();
      
      // Add assistant message
      const assistantMessage = {
        id: Date.now() + 1,
        role: 'assistant',
        content: data.response,
        sources: data.sources
      };
      
      setMessages(prev => [...prev, assistantMessage]);
    } catch (error) {
      console.error('Error:', error);
      const errorMessage = {
        id: Date.now() + 1,
        role: 'assistant',
        content: 'Sorry, I encountered an error. Please try again.'
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
      setSelectedText(''); // Clear selected text after sending
    }
  };

  const toggleChat = () => {
    setIsOpen(!isOpen);
    if (!isOpen && inputRef.current) {
      setTimeout(() => inputRef.current.focus(), 100);
    }
  };

  return (
    <div className={styles.chatContainer}>
      {/* Chatbot button */}
      <button 
        className={`${styles.chatButton} ${isOpen ? styles.hidden : ''}`}
        onClick={toggleChat}
        aria-label="Open chatbot"
      >
        💬
      </button>

      {/* Chat window */}
      {isOpen && (
        <div className={`${styles.chatWindow} ${colorMode === 'dark' ? styles.dark : ''}`}>
          <div className={styles.chatHeader}>
            <h3>{title}</h3>
            <button 
              className={styles.closeButton}
              onClick={toggleChat}
              aria-label="Close chat"
            >
              ×
            </button>
          </div>
          
          <div className={styles.chatMessages}>
            {messages.map((message) => (
              <div 
                key={message.id} 
                className={`${styles.message} ${styles[message.role]}`}
              >
                <div className={styles.messageContent}>
                  {message.content}
                </div>
                {message.sources && message.sources.length > 0 && (
                  <details className={styles.sources}>
                    <summary>Sources</summary>
                    <ul>
                      {message.sources.map((source, idx) => (
                        <li key={idx}>{source.text}</li>
                      ))}
                    </ul>
                  </details>
                )}
              </div>
            ))}
            {isLoading && (
              <div className={`${styles.message} ${styles.assistant}`}>
                <div className={styles.typingIndicator}>
                  <span></span>
                  <span></span>
                  <span></span>
                </div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>

          {selectedText && (
            <div className={styles.selectedTextNotice}>
              Using selected text as context: "{selectedText.substring(0, 100)}{selectedText.length > 100 ? '...' : ''}"
            </div>
          )}

          <form onSubmit={handleSubmit} className={styles.chatInputForm}>
            <input
              ref={inputRef}
              type="text"
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              placeholder="Ask about the book content..."
              className={styles.chatInput}
              disabled={isLoading}
            />
            <button 
              type="submit" 
              className={styles.sendButton}
              disabled={isLoading || !inputValue.trim()}
            >
              Send
            </button>
          </form>
        </div>
      )}
    </div>
  );
};

export default Chatbot;