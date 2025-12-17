import React from 'react';
import Layout from '@theme/Layout';
import ChatWidget from '../components/Chatbot/ChatWidget';

const ChatbotDemoPage: React.FC = () => {
  return (
    <Layout title="Chatbot Demo" description="Physical AI and Humanoid Robotics Chatbot">
      <div style={{ padding: '20px' }}>
        <h1>Physical AI and Humanoid Robotics Chatbot</h1>
        <p>
          This is a demonstration of the AI-powered chatbot integrated into the Physical AI and Humanoid Robotics textbook.
          The chatbot can answer questions about robotics concepts and provide personalized learning assistance.
        </p>

        <div style={{ margin: '20px 0' }}>
          <h2>Try it out!</h2>
          <p>Select some text on this page and ask the chatbot about it, or just type a question below.</p>
        </div>

        <div style={{ margin: '20px 0' }}>
          <h2>About this Chatbot</h2>
          <p>
            This chatbot is powered by a custom FastAPI backend with RAG (Retrieval Augmented Generation) capabilities,
            specifically designed for the Physical AI and Humanoid Robotics textbook. It can:
          </p>
          <ul>
            <li>Answer questions about robotics concepts</li>
            <li>Reference specific content from the textbook</li>
            <li>Adapt responses to your experience level (beginner, intermediate, advanced)</li>
            <li>Maintain conversation history across sessions</li>
            <li>Provide contextual responses based on selected text</li>
          </ul>
        </div>
      </div>

      {/* The chatbot widget will appear as a floating button on the page */}
      <ChatWidget />
    </Layout>
  );
};

export default ChatbotDemoPage;