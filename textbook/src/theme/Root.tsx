import React from 'react';
import ChatWidget from '@site/src/components/Chatbot/ChatWidget';

// Root component that wraps the entire app
const Root: React.FC<{ children: React.ReactNode }> = ({ children }) => {
  return (
    <>
      {children}
      <ChatWidget />
    </>
  );
};

export default Root;