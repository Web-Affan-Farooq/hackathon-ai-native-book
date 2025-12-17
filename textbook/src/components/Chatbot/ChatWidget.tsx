import React, { useState, useEffect, useRef } from 'react';
import './ChatWidget.css';
import { signup, login, sendMessage, getSessions, getChats, isAuthenticated, logout } from './api';

interface Message {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  timestamp: Date;
}

interface ChatSession {
  id: string;
  name: string;
  created_at: string;
}

const ChatWidget: React.FC = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [sessions, setSessions] = useState<ChatSession[]>([]);
  const [selectedSession, setSelectedSession] = useState<string | null>(null);
  const [userToken, setUserToken] = useState<string | null>(null);
  const [currentUser, setCurrentUser] = useState<any>(null);
  const messagesEndRef = useRef<null | HTMLDivElement>(null);

  // Check for existing token on component mount
  useEffect(() => {
    const token = isAuthenticated();
    if (token) {
      setUserToken('exists'); // Just to indicate auth status
    }
  }, []);

  // Scroll to bottom of messages when messages change
  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  const handleSignup = async (email: string, password: string, name: string, experienceLevel: number) => {
    const result = await signup({
      name,
      email,
      password,
      level_of_experience: experienceLevel
    });

    if (result.success) {
      setUserToken('exists');
      return { success: true };
    } else {
      return { success: false, error: result.error };
    }
  };

  const handleLogin = async (email: string, password: string) => {
    const result = await login({ email, password });

    if (result.success) {
      setUserToken('exists');
      return { success: true };
    } else {
      return { success: false, error: result.error };
    }
  };

  const handleSendMessage = async () => {
    if (!inputValue.trim() || isLoading || !userToken) return;

    const userMessage: Message = {
      id: Date.now().toString(),
      role: 'user',
      content: inputValue,
      timestamp: new Date(),
    };

    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      const result = await sendMessage(inputValue, selectedSession || undefined);

      if (result.success && result.response) {
        // Update session ID if it's new
        if (result.sessionId && !selectedSession) {
          setSelectedSession(result.sessionId);
        }

        const aiMessage: Message = {
          id: `ai-${Date.now()}`,
          role: 'assistant',
          content: result.response,
          timestamp: new Date(),
        };

        setMessages(prev => [...prev, aiMessage]);
      } else {
        const errorMessage: Message = {
          id: `error-${Date.now()}`,
          role: 'assistant',
          content: `Error: ${result.error || 'Failed to get response'}`,
          timestamp: new Date(),
        };
        setMessages(prev => [...prev, errorMessage]);
      }
    } catch (error) {
      const errorMessage: Message = {
        id: `error-${Date.now()}`,
        role: 'assistant',
        content: 'Error: Network connection failed',
        timestamp: new Date(),
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const loadSessions = async () => {
    if (!userToken) return;

    const result = await getSessions();
    if (result.success) {
      setSessions(result.sessions || []);
    } else {
      console.error('Error loading sessions:', result.error);
    }
  };

  const loadChats = async (sessionId: string) => {
    if (!userToken) return;

    const result = await getChats(sessionId);
    if (result.success) {
      const chatMessages: Message[] = result.chats?.map((chat: any) => ({
        id: chat.id,
        role: 'assistant',
        content: chat.answer,
        timestamp: new Date(chat.created_at),
      })) || [];

      // Add the user's questions to the chat history
      const fullMessages: Message[] = [];
      for (const chat of result.chats || []) {
        fullMessages.push({
          id: `user-${chat.id}`,
          role: 'user',
          content: chat.question,
          timestamp: new Date(chat.created_at),
        });
        fullMessages.push({
          id: chat.id,
          role: 'assistant',
          content: chat.answer,
          timestamp: new Date(chat.created_at),
        });
      }

      setMessages(fullMessages);
      setSelectedSession(sessionId);
    } else {
      console.error('Error loading chats:', result.error);
    }
  };

  const handleKeyDown = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSendMessage();
    }
  };

  if (!userToken) {
    return (
      <div className="chat-auth-container">
        <div className="chat-auth-widget">
          <div className="chat-auth-header">
            <h3>Physical AI Chatbot</h3>
            <button
              className="chat-close-btn"
              onClick={() => setIsOpen(false)}
            >
              ×
            </button>
          </div>
          <AuthForm
            onSignup={handleSignup}
            onLogin={handleLogin}
            onAuthSuccess={() => {
              setUserToken('exists');
            }}
          />
        </div>
      </div>
    );
  }

  return (
    <div className="chat-container">
      {isOpen ? (
        <div className="chat-widget">
          <div className="chat-header">
            <h3>Physical AI Chatbot</h3>
            <div className="chat-controls">
              <select
                value={selectedSession || ''}
                onChange={(e) => e.target.value ? loadChats(e.target.value) : setSelectedSession(null)}
                className="session-select"
              >
                <option value="">New Session</option>
                {sessions.map(session => (
                  <option key={session.id} value={session.id}>
                    {session.name}
                  </option>
                ))}
              </select>
              <button
                className="chat-minimize-btn"
                onClick={() => setIsOpen(false)}
              >
                −
              </button>
            </div>
          </div>

          <div className="chat-messages">
            {messages.map((message) => (
              <div
                key={message.id}
                className={`chat-message ${message.role}`}
              >
                <div className="message-content">
                  {message.content}
                </div>
                <div className="message-timestamp">
                  {message.timestamp.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}
                </div>
              </div>
            ))}
            {isLoading && (
              <div className="chat-message assistant">
                <div className="message-content">
                  <div className="typing-indicator">
                    <span></span>
                    <span></span>
                    <span></span>
                  </div>
                </div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>

          <div className="chat-input-area">
            <textarea
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              onKeyDown={handleKeyDown}
              placeholder="Ask about robotics concepts..."
              className="chat-input"
              rows={2}
            />
            <button
              onClick={handleSendMessage}
              disabled={isLoading || !inputValue.trim()}
              className="chat-send-btn"
            >
              Send
            </button>
          </div>
        </div>
      ) : (
        <button
          className="chat-open-btn"
          onClick={() => {
            setIsOpen(true);
            loadSessions();
          }}
        >
          💬 Chat
        </button>
      )}
    </div>
  );
};

// AuthForm component for login/signup
interface AuthFormProps {
  onSignup: (email: string, password: string, name: string, experienceLevel: number) => Promise<any>;
  onLogin: (email: string, password: string) => Promise<any>;
  onAuthSuccess: () => void;
}

const AuthForm: React.FC<AuthFormProps> = ({ onSignup, onLogin, onAuthSuccess }) => {
  const [isLoginView, setIsLoginView] = useState(true);
  const [name, setName] = useState('');
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [confirmPassword, setConfirmPassword] = useState('');
  const [experienceLevel, setExperienceLevel] = useState(1);
  const [error, setError] = useState('');
  const [loading, setLoading] = useState(false);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError('');
    setLoading(true);

    if (isLoginView) {
      const result = await onLogin(email, password);
      if (result.success) {
        onAuthSuccess();
      } else {
        setError(result.error);
      }
    } else {
      if (password !== confirmPassword) {
        setError('Passwords do not match');
        setLoading(false);
        return;
      }

      const result = await onSignup(email, password, name, experienceLevel);
      if (result.success) {
        onAuthSuccess();
      } else {
        setError(result.error);
      }
    }

    setLoading(false);
  };

  return (
    <form onSubmit={handleSubmit} className="auth-form">
      {!isLoginView && (
        <div className="form-group">
          <input
            type="text"
            value={name}
            onChange={(e) => setName(e.target.value)}
            placeholder="Full Name"
            required
          />
        </div>
      )}

      <div className="form-group">
        <input
          type="email"
          value={email}
          onChange={(e) => setEmail(e.target.value)}
          placeholder="Email"
          required
        />
      </div>

      <div className="form-group">
        <input
          type="password"
          value={password}
          onChange={(e) => setPassword(e.target.value)}
          placeholder="Password"
          required
        />
      </div>

      {!isLoginView && (
        <>
          <div className="form-group">
            <input
              type="password"
              value={confirmPassword}
              onChange={(e) => setConfirmPassword(e.target.value)}
              placeholder="Confirm Password"
              required
            />
          </div>

          <div className="form-group">
            <label>Experience Level:</label>
            <select
              value={experienceLevel}
              onChange={(e) => setExperienceLevel(Number(e.target.value))}
            >
              <option value={1}>Beginner (1)</option>
              <option value={2}>Intermediate (2)</option>
              <option value={3}>Advanced (3)</option>
            </select>
          </div>
        </>
      )}

      {error && <div className="error-message">{error}</div>}

      <button type="submit" disabled={loading} className="auth-submit-btn">
        {loading ? 'Processing...' : (isLoginView ? 'Login' : 'Sign Up')}
      </button>

      <button
        type="button"
        onClick={() => {
          setIsLoginView(!isLoginView);
          setError('');
        }}
        className="auth-toggle-btn"
      >
        {isLoginView ? 'Need an account? Sign Up' : 'Already have an account? Login'}
      </button>
    </form>
  );
};

export default ChatWidget;