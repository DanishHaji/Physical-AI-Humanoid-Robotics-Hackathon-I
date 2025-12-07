import React, { useState } from 'react';
import styles from './styles.module.css';

const API_URL = "http://localhost:8000";

interface Message {
  role: 'user' | 'assistant';
  content: string;
  citations?: Array<{
    chapter: string;
    section: string;
    url: string;
  }>;
}

export default function ChatbotWidget() {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([]);
  const [input, setInput] = useState('');
  const [mode, setMode] = useState<'auto' | 'explain' | 'code' | 'exam' | 'urdu'>('auto');
  const [loading, setLoading] = useState(false);

  const sendMessage = async () => {
    if (!input.trim() || loading) return;

    const userMessage: Message = { role: 'user', content: input };
    setMessages(prev => [...prev, userMessage]);
    setInput('');
    setLoading(true);

    try {
      const response = await fetch(`${API_URL}/query`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          question: input,
          mode: mode
        })
      });

      const data = await response.json();

      const assistantMessage: Message = {
        role: 'assistant',
        content: data.answer,
        citations: data.citations
      };

      setMessages(prev => [...prev, assistantMessage]);
    } catch (error) {
      const errorMessage: Message = {
        role: 'assistant',
        content: 'Error connecting to the chatbot. Please make sure the backend is running on http://localhost:8000'
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setLoading(false);
    }
  };

  const handleKeyPress = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      sendMessage();
    }
  };

  return (
    <>
      {/* Chatbot Button */}
      <button
        className={styles.chatbotButton}
        onClick={() => setIsOpen(!isOpen)}
        title="AI Chatbot"
      >
        🤖
      </button>

      {/* Chatbot Window */}
      {isOpen && (
        <div className={styles.chatbotWindow}>
          {/* Header */}
          <div className={styles.chatbotHeader}>
            <h3>AI Textbook Assistant</h3>
            <div className={styles.modeSelector}>
              <select
                value={mode}
                onChange={(e) => setMode(e.target.value as any)}
                className={styles.modeDropdown}
              >
                <option value="auto">Auto</option>
                <option value="explain">Explain 💡</option>
                <option value="code">Code 💻</option>
                <option value="exam">Quiz 📝</option>
                <option value="urdu">Urdu 🌐</option>
              </select>
            </div>
            <button
              className={styles.closeButton}
              onClick={() => setIsOpen(false)}
            >
              ✕
            </button>
          </div>

          {/* Messages */}
          <div className={styles.chatbotMessages}>
            {messages.length === 0 && (
              <div className={styles.welcomeMessage}>
                <p>👋 Hi! I'm your AI textbook assistant.</p>
                <p>Ask me anything about Physical AI, ROS 2, Isaac Sim, or VLA systems!</p>
              </div>
            )}

            {messages.map((msg, idx) => (
              <div key={idx} className={`${styles.message} ${styles[msg.role]}`}>
                <div className={styles.messageContent}>
                  {msg.content}
                </div>
                {msg.citations && msg.citations.length > 0 && (
                  <div className={styles.citations}>
                    <strong>Sources:</strong>
                    <ul>
                      {msg.citations.map((citation, cidx) => (
                        <li key={cidx}>
                          <a href={citation.url} target="_blank" rel="noopener noreferrer">
                            {citation.chapter} - {citation.section}
                          </a>
                        </li>
                      ))}
                    </ul>
                  </div>
                )}
              </div>
            ))}

            {loading && (
              <div className={`${styles.message} ${styles.assistant}`}>
                <div className={styles.messageContent}>
                  <div className={styles.loadingDots}>
                    <span>●</span><span>●</span><span>●</span>
                  </div>
                </div>
              </div>
            )}
          </div>

          {/* Input */}
          <div className={styles.chatbotInput}>
            <textarea
              value={input}
              onChange={(e) => setInput(e.target.value)}
              onKeyPress={handleKeyPress}
              placeholder="Ask a question..."
              rows={2}
              disabled={loading}
            />
            <button
              onClick={sendMessage}
              disabled={loading || !input.trim()}
            >
              Send
            </button>
          </div>
        </div>
      )}
    </>
  );
}
