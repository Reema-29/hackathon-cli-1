import React from 'react';
import Chatbot from '../Chatbot';

// This component wraps the entire site with the chatbot
export default function Root({ children }) {
  return (
    <>
      {children}
      <Chatbot title="Physical AI & Robotics Assistant" />
    </>
  );
}