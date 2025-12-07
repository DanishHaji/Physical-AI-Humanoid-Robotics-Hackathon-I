import React, { useState, useEffect } from 'react';
import Layout from '@theme/Layout';
import { useAuth } from '../contexts/AuthContext';
import { useHistory } from '@docusaurus/router';
import styles from './auth.module.css';

export default function Signup() {
  const { signup, isAuthenticated } = useAuth();
  const history = useHistory();
  const [step, setStep] = useState(1);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState('');

  // Form state
  const [formData, setFormData] = useState({
    // Step 1: Basic Info
    email: '',
    password: '',
    confirmPassword: '',
    username: '',
    // Step 2: Background Questionnaire
    software_experience: 'intermediate',
    hardware_experience: 'intermediate',
    programming_languages: [],
    robotics_experience: 'none',
    learning_goal: '',
    // Step 3: Preferences
    preferred_difficulty: 'auto',
    show_code_examples: true,
    show_advanced_topics: true,
    preferred_language: 'en',
  });

  // Redirect if already logged in
  useEffect(() => {
    if (isAuthenticated()) {
      history.push('/');
    }
  }, [isAuthenticated, history]);

  const handleInputChange = (e) => {
    const { name, value, type, checked } = e.target;
    setFormData(prev => ({
      ...prev,
      [name]: type === 'checkbox' ? checked : value
    }));
  };

  const handleLanguageToggle = (lang) => {
    setFormData(prev => {
      const languages = prev.programming_languages.includes(lang)
        ? prev.programming_languages.filter(l => l !== lang)
        : [...prev.programming_languages, lang];
      return { ...prev, programming_languages: languages };
    });
  };

  const validateStep1 = () => {
    if (!formData.email || !formData.password || !formData.username) {
      setError('Please fill in all required fields');
      return false;
    }
    if (formData.password.length < 8) {
      setError('Password must be at least 8 characters');
      return false;
    }
    if (formData.password !== formData.confirmPassword) {
      setError('Passwords do not match');
      return false;
    }
    if (!/[A-Z]/.test(formData.password)) {
      setError('Password must contain at least one uppercase letter');
      return false;
    }
    if (!/[a-z]/.test(formData.password)) {
      setError('Password must contain at least one lowercase letter');
      return false;
    }
    if (!/\d/.test(formData.password)) {
      setError('Password must contain at least one number');
      return false;
    }
    return true;
  };

  const nextStep = () => {
    setError('');
    if (step === 1 && !validateStep1()) {
      return;
    }
    setStep(step + 1);
  };

  const prevStep = () => {
    setError('');
    setStep(step - 1);
  };

  const handleSubmit = async (e) => {
    e.preventDefault();
    setError('');
    setLoading(true);

    // Remove confirmPassword from submission
    const { confirmPassword, ...signupData } = formData;

    const result = await signup(signupData);

    if (result.success) {
      history.push('/');
    } else {
      setError(result.error || 'Signup failed');
      setLoading(false);
    }
  };

  const programmingLanguages = [
    'Python', 'C++', 'JavaScript', 'Java', 'C', 'Rust', 'Go', 'TypeScript'
  ];

  return (
    <Layout title="Sign Up" description="Create your account">
      <div className={styles.authContainer}>
        <div className={styles.authCard}>
          <h1 className={styles.authTitle}>Create Account</h1>
          <p className={styles.authSubtitle}>
            Join us to learn Physical AI & Humanoid Robotics
          </p>

          {/* Progress indicator */}
          <div className={styles.progressBar}>
            <div className={`${styles.progressStep} ${step >= 1 ? styles.active : ''}`}>1</div>
            <div className={`${styles.progressLine} ${step >= 2 ? styles.active : ''}`}></div>
            <div className={`${styles.progressStep} ${step >= 2 ? styles.active : ''}`}>2</div>
            <div className={`${styles.progressLine} ${step >= 3 ? styles.active : ''}`}></div>
            <div className={`${styles.progressStep} ${step >= 3 ? styles.active : ''}`}>3</div>
          </div>

          <form onSubmit={handleSubmit} className={styles.authForm}>
            {error && <div className={styles.error}>{error}</div>}

            {/* Step 1: Basic Info */}
            {step === 1 && (
              <div className={styles.stepContainer}>
                <h2 className={styles.stepTitle}>Basic Information</h2>

                <div className={styles.formGroup}>
                  <label>Email *</label>
                  <input
                    type="email"
                    name="email"
                    value={formData.email}
                    onChange={handleInputChange}
                    required
                    placeholder="your@email.com"
                  />
                </div>

                <div className={styles.formGroup}>
                  <label>Username *</label>
                  <input
                    type="text"
                    name="username"
                    value={formData.username}
                    onChange={handleInputChange}
                    required
                    placeholder="Choose a username"
                    minLength={3}
                    maxLength={50}
                  />
                </div>

                <div className={styles.formGroup}>
                  <label>Password *</label>
                  <input
                    type="password"
                    name="password"
                    value={formData.password}
                    onChange={handleInputChange}
                    required
                    placeholder="Min 8 chars, 1 uppercase, 1 number"
                    minLength={8}
                  />
                </div>

                <div className={styles.formGroup}>
                  <label>Confirm Password *</label>
                  <input
                    type="password"
                    name="confirmPassword"
                    value={formData.confirmPassword}
                    onChange={handleInputChange}
                    required
                    placeholder="Re-enter password"
                  />
                </div>

                <button type="button" onClick={nextStep} className={styles.btnPrimary}>
                  Next →
                </button>
              </div>
            )}

            {/* Step 2: Background Questionnaire */}
            {step === 2 && (
              <div className={styles.stepContainer}>
                <h2 className={styles.stepTitle}>Your Background</h2>
                <p className={styles.stepDescription}>
                  Help us personalize your learning experience
                </p>

                <div className={styles.formGroup}>
                  <label>Software Experience *</label>
                  <select
                    name="software_experience"
                    value={formData.software_experience}
                    onChange={handleInputChange}
                    required
                  >
                    <option value="beginner">Beginner - New to programming</option>
                    <option value="intermediate">Intermediate - Some programming experience</option>
                    <option value="advanced">Advanced - Experienced developer</option>
                  </select>
                </div>

                <div className={styles.formGroup}>
                  <label>Hardware Experience *</label>
                  <select
                    name="hardware_experience"
                    value={formData.hardware_experience}
                    onChange={handleInputChange}
                    required
                  >
                    <option value="beginner">Beginner - Little to no hardware experience</option>
                    <option value="intermediate">Intermediate - Some electronics/hardware work</option>
                    <option value="advanced">Advanced - Extensive hardware experience</option>
                  </select>
                </div>

                <div className={styles.formGroup}>
                  <label>Programming Languages (Select all that apply)</label>
                  <div className={styles.languageGrid}>
                    {programmingLanguages.map(lang => (
                      <button
                        key={lang}
                        type="button"
                        className={`${styles.languageBtn} ${
                          formData.programming_languages.includes(lang) ? styles.selected : ''
                        }`}
                        onClick={() => handleLanguageToggle(lang)}
                      >
                        {lang}
                      </button>
                    ))}
                  </div>
                </div>

                <div className={styles.formGroup}>
                  <label>Robotics Experience *</label>
                  <select
                    name="robotics_experience"
                    value={formData.robotics_experience}
                    onChange={handleInputChange}
                    required
                  >
                    <option value="none">None - Complete beginner</option>
                    <option value="hobbyist">Hobbyist - Personal projects</option>
                    <option value="student">Student - Academic projects</option>
                    <option value="professional">Professional - Industry experience</option>
                  </select>
                </div>

                <div className={styles.formGroup}>
                  <label>What do you want to learn? (Optional)</label>
                  <textarea
                    name="learning_goal"
                    value={formData.learning_goal}
                    onChange={handleInputChange}
                    placeholder="e.g., Build autonomous robots, Learn ROS 2, Understand VLA systems..."
                    rows={3}
                  />
                </div>

                <div className={styles.btnGroup}>
                  <button type="button" onClick={prevStep} className={styles.btnSecondary}>
                    ← Back
                  </button>
                  <button type="button" onClick={nextStep} className={styles.btnPrimary}>
                    Next →
                  </button>
                </div>
              </div>
            )}

            {/* Step 3: Preferences */}
            {step === 3 && (
              <div className={styles.stepContainer}>
                <h2 className={styles.stepTitle}>Learning Preferences</h2>
                <p className={styles.stepDescription}>
                  Customize your learning experience
                </p>

                <div className={styles.formGroup}>
                  <label>Preferred Difficulty Level</label>
                  <select
                    name="preferred_difficulty"
                    value={formData.preferred_difficulty}
                    onChange={handleInputChange}
                  >
                    <option value="auto">Auto (Based on your background)</option>
                    <option value="beginner">Beginner - Simple explanations</option>
                    <option value="intermediate">Intermediate - Balanced approach</option>
                    <option value="advanced">Advanced - In-depth technical details</option>
                  </select>
                </div>

                <div className={styles.formGroup}>
                  <label className={styles.checkboxLabel}>
                    <input
                      type="checkbox"
                      name="show_code_examples"
                      checked={formData.show_code_examples}
                      onChange={handleInputChange}
                    />
                    Show code examples and implementations
                  </label>
                </div>

                <div className={styles.formGroup}>
                  <label className={styles.checkboxLabel}>
                    <input
                      type="checkbox"
                      name="show_advanced_topics"
                      checked={formData.show_advanced_topics}
                      onChange={handleInputChange}
                    />
                    Include advanced topics and research papers
                  </label>
                </div>

                <div className={styles.formGroup}>
                  <label>Preferred Language</label>
                  <select
                    name="preferred_language"
                    value={formData.preferred_language}
                    onChange={handleInputChange}
                  >
                    <option value="en">English</option>
                    <option value="ur">Urdu</option>
                  </select>
                </div>

                <div className={styles.btnGroup}>
                  <button type="button" onClick={prevStep} className={styles.btnSecondary}>
                    ← Back
                  </button>
                  <button type="submit" disabled={loading} className={styles.btnPrimary}>
                    {loading ? 'Creating Account...' : 'Create Account'}
                  </button>
                </div>
              </div>
            )}
          </form>

          <div className={styles.authFooter}>
            Already have an account? <a href="/login">Log in</a>
          </div>
        </div>
      </div>
    </Layout>
  );
}
