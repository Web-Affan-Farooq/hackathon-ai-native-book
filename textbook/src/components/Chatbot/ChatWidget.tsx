import React, { useState, useEffect, useRef } from 'react';
import { useSessions } from '../../stores/session';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

interface Message {
  id: string;
  role: 'user' | 'assistant';
  content: string;
  timestamp: Date;
}

const ChatWidget: React.FC = () => {
  const { sessions, setSession, setSelectedSession, selectedSession, isUserAuthenticated, setIsUserAuthenticated } = useSessions();
  const [isOpen, setIsOpen] = useState(false);
  const [inputValue, setInputValue] = useState('');
  const [messages, setMessages] = useState<any[]>([]);
  const messagesEndRef = useRef<null | HTMLDivElement>(null);

  const {siteConfig: {
    customFields
  }} = useDocusaurusContext()

  const handleSendMessage = async () => {
    if (!inputValue.trim() || !isUserAuthenticated) return;

    try {
      // Get selected text from the page
      const selectedText = window.getSelection()?.toString() || '';

      // Prepare the payload
      const payload: any = {
        question: inputValue,
        user_selected_text: selectedText,
      };

      // Add session ID if available
      if (selectedSession?.id) {
        payload.session_id = selectedSession.id;
      }

      // Call the backend API
      const response = await fetch(`${customFields.api_url}/ask`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(payload),
        credentials: 'include',
      });

      if (response.status === 404) {
        setIsUserAuthenticated(false);
        window.location.href = `${customFields.client_url}/signup`;
        return;
      } else if (response.status === 401) {
        setIsUserAuthenticated(false);
        window.location.href = `${customFields.client_url}/login`;
        return;
      }

      const data = await response.json();
      if (response.ok) {
        // Update session if it's new
        if (data.chat?.session_id && !selectedSession) {
          // Fetch session details to get the name
          const sessionResponse = await fetch(`${customFields.api_url}/get-sessions`, {
            method: 'GET',
            credentials: 'include',
          });

          if (sessionResponse.ok) {
            const sessionData = await sessionResponse.json();
            const newSession = sessionData.sessions?.find((s: any) => s.id === data.chat.session_id);
            if (newSession) {
              const sessionObj = {
                id: newSession.id,
                name: newSession.name
              };
              setSelectedSession(sessionObj);
            }
          }
        }

        // Add both question and answer to messages
        setMessages(prev => [
          ...prev,
          { id: `user-${Date.now()}`, role: 'user', text: inputValue },
          { id: `assistant-${Date.now()}`, role: 'assistant', text: data.chat.answer }
        ]);
      } else {
        setMessages(prev => [
          ...prev,
          { id: `error-${Date.now()}`, role: 'assistant', text: `Error: ${data.message || 'Failed to get response'}` }
        ]);
      }
    } catch (error) {
      setMessages(prev => [
        ...prev,
        { id: `error-${Date.now()}`, role: 'assistant', text: 'Error: Network connection failed' }
      ]);
    }

    setInputValue('');
  };

  const loadSessions = async () => {
    if (!isUserAuthenticated) return;

    try {
      const response = await fetch(`${customFields.api_url}/get-sessions`, {
        method: 'GET',
        credentials: 'include',
      });

      if (response.status === 404) {
        setIsUserAuthenticated(false);
        window.location.href = `${customFields.client_url}/signup`;
        return;
      } else if (response.status === 401) {
        setIsUserAuthenticated(false);
        window.location.href = `${customFields.client_url}/login`;
        return;
      }

      const data = await response.json();
      if (response.ok) {
        // Map the server response to match our Session type
        const mappedSessions = data.sessions?.map((session: any) => ({
          id: session.id,
          name: session.name || `Session ${new Date(session.created_at).toLocaleDateString()}`
        })) || [];

        setSession(mappedSessions);
      }
    } catch (error) {
      console.error('Network error loading sessions:', error);
    }
  };

  const loadChats = async (sessionId: string) => {
    if (!isUserAuthenticated) return;

    try {
      const response = await fetch(`${customFields.api_url}/get-chats`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({ session_id: sessionId }),
        credentials: 'include',
      });

      if (response.status === 404) {
        setIsUserAuthenticated(false);
        window.location.href = `${customFields.client_url}/signup`;
        return;
      } else if (response.status === 401) {
        setIsUserAuthenticated(false);
        window.location.href = `${customFields.client_url}/login`;
        return;
      }

      const data = await response.json();
      if (response.ok) {
        // Convert chat data to messages format
        const chatMessages: any[] = [];
        for (const chat of data.chats || []) {
          chatMessages.push({
            id: `user-${chat.id}`,
            role: 'user',
            text: chat.question
          });
          chatMessages.push({
            id: chat.id,
            role: 'assistant',
            text: chat.answer
          });
        }

        setMessages(chatMessages);

        // Update the selected session in the zustand store
        const sessionToSelect = sessions.find(s => s.id === sessionId);
        if (sessionToSelect) {
          setSelectedSession(sessionToSelect);
        }
      }
    } catch (error) {
      console.error('Network error loading chats:', error);
    }
  };

  const handleKeyDown = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSendMessage();
    }
  };

  if (!isUserAuthenticated) {
    return (
      <div style={{
        position: 'fixed',
        bottom: '20px',
        right: '20px',
        zIndex: 1000
      }}>
        <div style={{
          backgroundColor: 'white',
          border: '1px solid #ccc',
          borderRadius: '8px',
          padding: '20px',
          width: '350px',
          boxShadow: '0 4px 12px rgba(0,0,0,0.15)'
        }}>
          <div style={{ display: 'flex', justifyContent: 'space-between', alignItems: 'center', marginBottom: '15px' }}>
            <h3 style={{ margin: 0, fontSize: '16px' }}>Physical AI Chatbot</h3>
            <button
              onClick={() => setIsOpen(false)}
              style={{
                background: 'none',
                border: 'none',
                fontSize: '20px',
                cursor: 'pointer',
                padding: '0',
                width: '24px',
                height: '24px'
              }}
            >
              ×
            </button>
          </div>
          <p>Please log in to use the chatbot</p>
          <div style={{ textAlign: 'center', marginTop: '10px' }}>
            <a href={`${customFields.client_url}/login`} style={{ color: '#007cba', textDecoration: 'none', marginRight: '10px' }}>
              Login
            </a>
            <span>or</span>
            <a href={`${customFields.client_url}/signup`} style={{ color: '#007cba', textDecoration: 'none', marginLeft: '10px' }}>
              Sign Up
            </a>
          </div>
        </div>
      </div>
    );
  }

  return (
    <div style={{
      position: 'fixed',
      bottom: '20px',
      right: '20px',
      zIndex: 1000
    }}>
      {isOpen ? (
        <div style={{
          backgroundColor: 'white',
          border: '1px solid #ccc',
          borderRadius: '8px',
          width: '350px',
          height: '500px',
          display: 'flex',
          flexDirection: 'column',
          boxShadow: '0 4px 12px rgba(0,0,0,0.15)'
        }}>
          <div style={{
            padding: '15px',
            borderBottom: '1px solid #eee',
            display: 'flex',
            justifyContent: 'space-between',
            alignItems: 'center'
          }}>
            <h3 style={{ margin: 0, fontSize: '16px' }}>Physical AI Chatbot</h3>
            <div style={{ display: 'flex', gap: '10px' }}>
              <select
                value={selectedSession?.id || ''}
                onChange={(e) => e.target.value ? loadChats(e.target.value) : setSelectedSession(null as any)}
                style={{
                  padding: '4px 8px',
                  fontSize: '14px',
                  borderRadius: '4px',
                  border: '1px solid #ccc'
                }}
              >
                <option value="">New Session</option>
                {sessions.map(session => (
                  <option key={session.id} value={session.id}>
                    {session.name}
                  </option>
                ))}
              </select>
              <button
                onClick={() => setIsOpen(false)}
                style={{
                  background: 'none',
                  border: 'none',
                  fontSize: '20px',
                  cursor: 'pointer',
                  padding: '0',
                  width: '24px',
                  height: '24px'
                }}
              >
                −
              </button>
            </div>
          </div>

          <div style={{
            flex: 1,
            overflowY: 'auto',
            padding: '15px',
            display: 'flex',
            flexDirection: 'column',
            gap: '10px'
          }}>
            {messages.map((message) => (
              <div
                key={message.id}
                style={{
                  alignSelf: message.role === 'user' ? 'flex-end' : 'flex-start',
                  backgroundColor: message.role === 'user' ? '#007cba' : '#f1f1f1',
                  color: message.role === 'user' ? 'white' : 'black',
                  padding: '8px 12px',
                  borderRadius: '8px',
                  maxWidth: '80%'
                }}
              >
                {message.text}
              </div>
            ))}
            <div ref={messagesEndRef} />
          </div>

          <div style={{
            padding: '10px',
            borderTop: '1px solid #eee',
            display: 'flex'
          }}>
            <textarea
              value={inputValue}
              onChange={(e) => setInputValue(e.target.value)}
              onKeyDown={handleKeyDown}
              placeholder="Ask about robotics concepts..."
              style={{
                flex: 1,
                padding: '8px',
                border: '1px solid #ccc',
                borderRadius: '4px',
                resize: 'none',
                fontSize: '14px'
              }}
              rows={2}
            />
            <button
              onClick={handleSendMessage}
              disabled={!inputValue.trim()}
              style={{
                marginLeft: '10px',
                padding: '8px 15px',
                backgroundColor: inputValue.trim() ? '#007cba' : '#ccc',
                color: 'white',
                border: 'none',
                borderRadius: '4px',
                cursor: inputValue.trim() ? 'pointer' : 'not-allowed'
              }}
            >
              Send
            </button>
          </div>
        </div>
      ) : (
        <button
          onClick={() => {
            setIsOpen(true);
            loadSessions();
          }}
          style={{
            width: '60px',
            height: '60px',
            borderRadius: '50%',
            backgroundColor: '#007cba',
            color: 'white',
            border: 'none',
            fontSize: '24px',
            cursor: 'pointer',
            display: 'flex',
            alignItems: 'center',
            justifyContent: 'center',
            boxShadow: '0 4px 12px rgba(0,0,0,0.15)'
          }}
        >
          💬
        </button>
      )}
    </div>
  );
};

export default ChatWidget;