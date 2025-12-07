import React, { useState, useEffect } from 'react';
import Layout from '@theme/Layout';
import { useAuth } from '../contexts/AuthContext';
import { useHistory } from '@docusaurus/router';
import styles from './auth.module.css';

export default function Login() {
  const { login, isAuthenticated } = useAuth();
  const history = useHistory();
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState('');
  const [formData, setFormData] = useState({
    email: '',
    password: '',
  });

  // Redirect if already logged in
  useEffect(() => {
    if (isAuthenticated()) {
      history.push('/');
    }
  }, [isAuthenticated, history]);

  const handleInputChange = (e) => {
    const { name, value } = e.target;
    setFormData(prev => ({
      ...prev,
      [name]: value
    }));
  };

  const handleSubmit = async (e) => {
    e.preventDefault();
    setError('');
    setLoading(true);

    const result = await login(formData.email, formData.password);

    if (result.success) {
      history.push('/');
    } else {
      setError(result.error || 'Login failed');
      setLoading(false);
    }
  };

  return (
    <Layout title="Log In" description="Sign in to your account">
      <div className={styles.authContainer}>
        <div className={styles.authCard}>
          <h1 className={styles.authTitle}>Welcome Back</h1>
          <p className={styles.authSubtitle}>
            Log in to continue your learning journey
          </p>

          <form onSubmit={handleSubmit} className={styles.authForm}>
            {error && <div className={styles.error}>{error}</div>}

            <div className={styles.formGroup}>
              <label>Email</label>
              <input
                type="email"
                name="email"
                value={formData.email}
                onChange={handleInputChange}
                required
                placeholder="your@email.com"
                autoComplete="email"
              />
            </div>

            <div className={styles.formGroup}>
              <label>Password</label>
              <input
                type="password"
                name="password"
                value={formData.password}
                onChange={handleInputChange}
                required
                placeholder="Enter your password"
                autoComplete="current-password"
              />
            </div>

            <button type="submit" disabled={loading} className={styles.btnPrimary}>
              {loading ? 'Logging in...' : 'Log In'}
            </button>
          </form>

          <div className={styles.authFooter}>
            Don't have an account? <a href="/signup">Sign up</a>
          </div>
        </div>
      </div>
    </Layout>
  );
}
