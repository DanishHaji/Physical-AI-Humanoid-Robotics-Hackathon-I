import React, { useState, useEffect, useRef } from 'react';
import styles from './styles.module.css';

const API_URL = "https://physical-ai-textbook-api-i4ug.onrender.com";

// Suggested questions for users (detailed for better matching)
const SUGGESTED_QUESTIONS = [
  "Explain the ROS 2 architecture and how nodes communicate using topics and the DDS middleware layer",
  "What is NVIDIA Isaac Sim and how is it used for robot simulation and training?",
  "Explain Vision-Language-Action systems and how they combine perception with robot control",
  "What is a Digital Twin and how does it help in robot development and testing?",
  "Describe the publish-subscribe pattern in ROS 2 and how topics enable asynchronous communication"
];

// Follow-up questions based on topic (detailed for better matching)
const FOLLOW_UP_QUESTIONS = {
  "ros": [
    "Explain how to create a ROS 2 node using Python and rclpy with publishers and subscribers",
    "What are ROS 2 services and how do they differ from topics for synchronous communication?",
    "How do ROS 2 parameters work for runtime configuration of nodes?"
  ],
  "isaac": [
    "Describe the setup process for NVIDIA Isaac Sim and its system requirements",
    "What is Isaac Lab and how does it extend Isaac Sim for robot learning?",
    "Compare Isaac Sim and Gazebo for robot simulation capabilities"
  ],
  "vla": [
    "Explain the training process for Vision-Language-Action models using demonstration data",
    "Describe the VLA system architecture including vision encoder, language model, and action decoder",
    "What are the deployment strategies for VLA systems on real robots?"
  ],
  "digital": [
    "Compare URDF and SDF file formats for robot modeling",
    "Explain the process of creating a Digital Twin for robot simulation and testing",
    "How does Gazebo integrate with ROS 2 for robot simulation?"
  ]
};

export default function ChatbotWidget() {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [input, setInput] = useState('');
  const [loading, setLoading] = useState(false);
  const [showWelcomeTooltip, setShowWelcomeTooltip] = useState(false);
  const [copiedIndex, setCopiedIndex] = useState(null);
  const messagesEndRef = useRef(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  // Load conversation history from localStorage
  useEffect(() => {
    const savedMessages = localStorage.getItem('chatbot_history');
    if (savedMessages) {
      try {
        setMessages(JSON.parse(savedMessages));
      } catch (e) {
        console.error('Failed to load chat history:', e);
      }
    }
  }, []);

  // Save conversation to localStorage
  useEffect(() => {
    if (messages.length > 0) {
      localStorage.setItem('chatbot_history', JSON.stringify(messages));
    }
  }, [messages]);

  // Show welcome tooltip on page load
  useEffect(() => {
    // Show tooltip after 1 second on every page load
    const timer = setTimeout(() => {
      setShowWelcomeTooltip(true);
    }, 1000);

    // Auto-hide after 5 seconds
    const hideTimer = setTimeout(() => {
      setShowWelcomeTooltip(false);
    }, 6000);

    return () => {
      clearTimeout(timer);
      clearTimeout(hideTimer);
    };
  }, []);

  const sendMessage = async (questionText = null) => {
    const question = questionText || input;
    if (!question.trim() || loading) return;

    const userMessage = { role: 'user', content: question };
    setMessages(prev => [...prev, userMessage]);
    setInput('');
    setLoading(true);

    try {
      const response = await fetch(`${API_URL}/query`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          question: question,
          mode: 'auto'
        })
      });

      const data = await response.json();

      const assistantMessage = {
        role: 'assistant',
        content: data.answer,
        citations: data.citations,
        followUps: getFollowUpQuestions(question)
      };

      setMessages(prev => [...prev, assistantMessage]);
    } catch (error) {
      const errorMessage = {
        role: 'assistant',
        content: 'Connection error. Please check your connection to the backend service.'
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setLoading(false);
    }
  };

  const getFollowUpQuestions = (question) => {
    const lowerQ = question.toLowerCase();
    if (lowerQ.includes('ros')) return FOLLOW_UP_QUESTIONS.ros.slice(0, 2);
    if (lowerQ.includes('isaac')) return FOLLOW_UP_QUESTIONS.isaac.slice(0, 2);
    if (lowerQ.includes('vla')) return FOLLOW_UP_QUESTIONS.vla.slice(0, 2);
    if (lowerQ.includes('digital') || lowerQ.includes('twin')) return FOLLOW_UP_QUESTIONS.digital.slice(0, 2);
    return [];
  };

  const handleSuggestedQuestion = (question) => {
    setIsOpen(true);
    sendMessage(question);
  };

  const copyToClipboard = (text, index) => {
    navigator.clipboard.writeText(text).then(() => {
      setCopiedIndex(index);
      setTimeout(() => setCopiedIndex(null), 2000);
    });
  };

  const clearConversation = () => {
    if (window.confirm('Clear all messages?')) {
      setMessages([]);
      localStorage.removeItem('chatbot_history');
    }
  };

  const renderMarkdown = (text) => {
    // Simple markdown rendering
    return text
      .replace(/\*\*(.*?)\*\*/g, '<strong>$1</strong>')
      .replace(/\*(.*?)\*/g, '<em>$1</em>')
      .replace(/`(.*?)`/g, '<code>$1</code>')
      .replace(/\n/g, '<br/>');
  };

  const handleKeyPress = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      sendMessage();
    }
  };

  return (
    <>
      {/* Welcome Tooltip */}
      {showWelcomeTooltip && !isOpen && (
        <div className={styles.welcomeTooltip}>
          <button
            className={styles.tooltipClose}
            onClick={() => setShowWelcomeTooltip(false)}
          >
            ✕
          </button>
          <div className={styles.tooltipContent}>
            <span className={styles.tooltipIcon}>👋</span>
            <p>Hi! How can I assist you today?</p>
          </div>
        </div>
      )}

      {/* Chatbot Button */}
      <button
        className={`${styles.chatbotButton} ${isOpen ? styles.active : ''}`}
        onClick={() => setIsOpen(!isOpen)}
        title="Open Chat - Ask me anything about Physical AI!"
        aria-label="Toggle AI Assistant"
      >
        <span className={styles.buttonIcon}>🤖</span>
      </button>

      {/* Chatbot Window */}
      {isOpen && (
        <div className={styles.chatbotWindow}>
          {/* Header */}
          <div className={styles.chatbotHeader}>
            <div className={styles.headerContent}>
              <div className={styles.headerTitle}>
                <span className={styles.aiIcon}>✨</span>
                <h3>AI Textbook Assistant</h3>
              </div>
              <p className={styles.headerSubtitle}>Powered by Physical AI Knowledge</p>
            </div>
            <div className={styles.headerButtons}>
              {messages.length > 0 && (
                <button
                  className={styles.clearButton}
                  onClick={clearConversation}
                  title="Clear conversation"
                >
                  🗑️
                </button>
              )}
              <button
                className={styles.closeButton}
                onClick={() => setIsOpen(false)}
                aria-label="Close chat"
              >
                ✕
              </button>
            </div>
          </div>

          {/* Messages */}
          <div className={styles.chatbotMessages}>
            {messages.length === 0 && (
              <div className={styles.welcomeMessage}>
                <div className={styles.welcomeIcon}>👋</div>
                <h4>Welcome!</h4>
                <p>I'm your AI assistant for Physical AI, Robotics, ROS 2, Isaac Sim, and VLA systems.</p>
                <div className={styles.quickPrompts}>
                  <p className={styles.promptsLabel}>Try asking:</p>
                  {SUGGESTED_QUESTIONS.map((q, idx) => (
                    <button
                      key={idx}
                      className={styles.promptBadge}
                      onClick={() => handleSuggestedQuestion(q)}
                    >
                      {q}
                    </button>
                  ))}
                </div>
              </div>
            )}

            {messages.map((msg, idx) => (
              <div key={idx} className={`${styles.message} ${styles[msg.role]}`}>
                <div className={styles.messageContent}>
                  <div
                    className={styles.messageText}
                    dangerouslySetInnerHTML={{ __html: renderMarkdown(msg.content) }}
                  />

                  {/* Copy Button for Assistant Messages */}
                  {msg.role === 'assistant' && (
                    <button
                      className={styles.copyButton}
                      onClick={() => copyToClipboard(msg.content, idx)}
                      title="Copy answer"
                    >
                      {copiedIndex === idx ? '✓ Copied' : '📋 Copy'}
                    </button>
                  )}

                  {/* Citations removed as per user request */}

                  {/* Follow-up Questions */}
                  {msg.followUps && msg.followUps.length > 0 && (
                    <div className={styles.followUps}>
                      <p className={styles.followUpLabel}>Related questions:</p>
                      {msg.followUps.map((q, qidx) => (
                        <button
                          key={qidx}
                          className={styles.followUpButton}
                          onClick={() => sendMessage(q)}
                        >
                          {q}
                        </button>
                      ))}
                    </div>
                  )}
                </div>
              </div>
            ))}

            {loading && (
              <div className={`${styles.message} ${styles.assistant}`}>
                <div className={styles.messageContent}>
                  <div className={styles.loadingContainer}>
                    <div className={styles.loadingDots}>
                      <span></span>
                      <span></span>
                      <span></span>
                    </div>
                    <span className={styles.loadingText}>Thinking...</span>
                  </div>
                </div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>

          {/* Input */}
          <div className={styles.chatbotInput}>
            <textarea
              value={input}
              onChange={(e) => setInput(e.target.value)}
              onKeyPress={handleKeyPress}
              placeholder="Ask anything about the textbook..."
              rows={2}
              disabled={loading}
              className={styles.inputField}
            />
            <button
              onClick={() => sendMessage()}
              disabled={loading || !input.trim()}
              className={styles.sendButton}
              aria-label="Send message"
            >
              <span className={styles.sendIcon}>➤</span>
            </button>
          </div>
        </div>
      )}
    </>
  );
}
