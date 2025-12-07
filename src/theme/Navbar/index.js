import React from 'react';
import Navbar from '@theme-original/Navbar';
import { useAuth } from '@site/src/contexts/AuthContext';
import Link from '@docusaurus/Link';
import './styles.module.css';

export default function NavbarWrapper(props) {
  const { user, isAuthenticated, logout } = useAuth();

  return (
    <>
      <Navbar {...props} />
      {isAuthenticated() && user && (
        <div style={{
          position: 'fixed',
          top: '60px',
          right: '20px',
          zIndex: 1000,
          background: 'linear-gradient(135deg, #00FF88 0%, #00CC66 100%)',
          color: 'white',
          padding: '8px 16px',
          borderRadius: '8px',
          boxShadow: '0 2px 8px rgba(0, 255, 136, 0.3)',
          fontSize: '14px',
          fontWeight: '500',
          display: 'flex',
          alignItems: 'center',
          gap: '12px'
        }}>
          <span>Logged in as: <strong>{user.full_name || user.email}</strong></span>
          <button
            onClick={logout}
            style={{
              background: 'rgba(255, 255, 255, 0.2)',
              border: 'none',
              color: 'white',
              padding: '4px 12px',
              borderRadius: '4px',
              cursor: 'pointer',
              fontSize: '13px',
              fontWeight: '500',
              transition: 'background 0.2s'
            }}
            onMouseEnter={(e) => e.target.style.background = 'rgba(255, 255, 255, 0.3)'}
            onMouseLeave={(e) => e.target.style.background = 'rgba(255, 255, 255, 0.2)'}
          >
            Logout
          </button>
        </div>
      )}
    </>
  );
}
