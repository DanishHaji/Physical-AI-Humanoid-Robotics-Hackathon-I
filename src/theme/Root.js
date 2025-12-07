import React from 'react';
import { AuthProvider } from '../contexts/AuthContext';
import ChatbotWidget from '../components/ChatbotWidget';

// This component wraps your entire Docusaurus site
export default function Root({ children }) {
  return (
    <AuthProvider>
      {children}
      <ChatbotWidget />
    </AuthProvider>
  );
}
