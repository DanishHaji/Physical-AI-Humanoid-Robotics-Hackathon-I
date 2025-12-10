import React, { createContext, useState, useContext, useEffect } from 'react';

const API_URL = "https://physical-ai-textbook-api-i4ug.onrender.com";

const AuthContext = createContext(null);

export const useAuth = () => {
  const context = useContext(AuthContext);
  if (!context) {
    throw new Error('useAuth must be used within AuthProvider');
  }
  return context;
};

export const AuthProvider = ({ children }) => {
  const [user, setUser] = useState(null);
  const [profile, setProfile] = useState(null);
  const [token, setToken] = useState(null);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState(null);

  useEffect(() => {
    const storedToken = localStorage.getItem('auth_token');
    const storedUser = localStorage.getItem('auth_user');
    const storedProfile = localStorage.getItem('auth_profile');

    if (storedToken && storedUser && storedProfile) {
      try {
        setToken(storedToken);
        setUser(JSON.parse(storedUser));
        setProfile(JSON.parse(storedProfile));
      } catch (e) {
        console.error('Failed to load auth state:', e);
        clearAuthState();
      }
    }
    setLoading(false);
  }, []);

  const clearAuthState = () => {
    setUser(null);
    setProfile(null);
    setToken(null);
    localStorage.removeItem('auth_token');
    localStorage.removeItem('auth_user');
    localStorage.removeItem('auth_profile');
  };

  const saveAuthState = (authData) => {
    const { user, profile, token } = authData;
    setUser(user);
    setProfile(profile);
    setToken(token.access_token);
    localStorage.setItem('auth_token', token.access_token);
    localStorage.setItem('auth_user', JSON.stringify(user));
    localStorage.setItem('auth_profile', JSON.stringify(profile));
  };

  const signup = async (signupData) => {
    setLoading(true);
    setError(null);

    try {
      const response = await fetch(`${API_URL}/auth/signup`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(signupData),
      });

      const data = await response.json();

      if (!response.ok) {
        throw new Error(data.detail || 'Signup failed');
      }

      saveAuthState(data);
      setLoading(false);
      return { success: true };
    } catch (err) {
      setError(err.message);
      setLoading(false);
      return { success: false, error: err.message };
    }
  };

  const login = async (email, password) => {
    setLoading(true);
    setError(null);

    try {
      const response = await fetch(`${API_URL}/auth/login`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ email, password }),
      });

      const data = await response.json();

      if (!response.ok) {
        throw new Error(data.detail || 'Login failed');
      }

      saveAuthState(data);
      setLoading(false);
      return { success: true };
    } catch (err) {
      setError(err.message);
      setLoading(false);
      return { success: false, error: err.message };
    }
  };

  const logout = () => {
    clearAuthState();
  };

  const updateProfile = async (updates) => {
    if (!token) {
      return { success: false, error: 'Not authenticated' };
    }

    try {
      const response = await fetch(`${API_URL}/auth/profile`, {
        method: 'PUT',
        headers: {
          'Content-Type': 'application/json',
          'Authorization': `Bearer ${token}`,
        },
        body: JSON.stringify(updates),
      });

      const data = await response.json();

      if (!response.ok) {
        throw new Error(data.detail || 'Profile update failed');
      }

      setProfile(data);
      localStorage.setItem('auth_profile', JSON.stringify(data));
      return { success: true, profile: data };
    } catch (err) {
      return { success: false, error: err.message };
    }
  };

  const getAuthHeaders = () => {
    if (!token) return {};
    return { 'Authorization': `Bearer ${token}` };
  };

  const isAuthenticated = () => {
    return !!token && !!user;
  };

  const getDifficultyLevel = () => {
    if (!profile) return 'intermediate';
    if (profile.preferred_difficulty !== 'auto') {
      return profile.preferred_difficulty;
    }
    const experienceLevels = {
      'beginner': 1, 'intermediate': 2, 'advanced': 3,
      'none': 0, 'hobbyist': 1, 'student': 2, 'professional': 3
    };
    const levels = [
      experienceLevels[profile.software_experience] || 0,
      experienceLevels[profile.hardware_experience] || 0,
      experienceLevels[profile.robotics_experience] || 0
    ];
    const avgLevel = levels.reduce((a, b) => a + b, 0) / levels.length;
    if (avgLevel < 1.3) return 'beginner';
    if (avgLevel < 2.3) return 'intermediate';
    return 'advanced';
  };

  const value = {
    user, profile, token, loading, error,
    signup, login, logout, updateProfile,
    getAuthHeaders, isAuthenticated, getDifficultyLevel,
  };

  return <AuthContext.Provider value={value}>{children}</AuthContext.Provider>;
};

export default AuthContext;
