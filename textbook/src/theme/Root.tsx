import React from 'react';

// Root component that wraps the entire app
const Root: React.FC<{ children: React.ReactNode }> = ({ children }) => {
  return (
    <>
      {children}
    </>
  );
};

export default Root;