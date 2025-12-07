-- User Authentication & Profile Schema
-- Uses Neon PostgreSQL (already configured)

-- Users table (core authentication)
CREATE TABLE IF NOT EXISTS users (
    id SERIAL PRIMARY KEY,
    email VARCHAR(255) UNIQUE NOT NULL,
    password_hash VARCHAR(255) NOT NULL,
    username VARCHAR(100) UNIQUE NOT NULL,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    is_active BOOLEAN DEFAULT TRUE,
    email_verified BOOLEAN DEFAULT FALSE
);

-- User profiles (background & preferences)
CREATE TABLE IF NOT EXISTS user_profiles (
    id SERIAL PRIMARY KEY,
    user_id INTEGER REFERENCES users(id) ON DELETE CASCADE,

    -- Background questionnaire responses
    software_experience VARCHAR(50) NOT NULL, -- 'beginner', 'intermediate', 'advanced'
    hardware_experience VARCHAR(50) NOT NULL, -- 'beginner', 'intermediate', 'advanced'
    programming_languages TEXT[], -- ['python', 'cpp', 'javascript', etc.]
    robotics_experience VARCHAR(50), -- 'none', 'hobbyist', 'student', 'professional'
    learning_goal TEXT, -- Free text: what they want to learn

    -- Personalization preferences
    preferred_difficulty VARCHAR(50) DEFAULT 'auto', -- 'beginner', 'intermediate', 'advanced', 'auto'
    show_code_examples BOOLEAN DEFAULT TRUE,
    show_advanced_topics BOOLEAN DEFAULT TRUE,
    preferred_language VARCHAR(10) DEFAULT 'en', -- 'en', 'ur'

    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,

    UNIQUE(user_id)
);

-- User content edits (personalized changes to chapters)
CREATE TABLE IF NOT EXISTS user_edits (
    id SERIAL PRIMARY KEY,
    user_id INTEGER REFERENCES users(id) ON DELETE CASCADE,

    -- Content location
    chapter_path VARCHAR(500) NOT NULL, -- e.g., 'module-01-ros2/week-02-ros2-fundamentals'
    section_id VARCHAR(200), -- Section heading or ID

    -- Edit details
    original_content TEXT, -- Snapshot of original for reference
    edited_content TEXT NOT NULL, -- User's personalized version
    edit_type VARCHAR(50) DEFAULT 'replace', -- 'replace', 'append', 'prepend'

    -- Metadata
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,

    -- Ensure one edit per section per user
    UNIQUE(user_id, chapter_path, section_id)
);

-- User annotations (notes, highlights, bookmarks)
CREATE TABLE IF NOT EXISTS user_annotations (
    id SERIAL PRIMARY KEY,
    user_id INTEGER REFERENCES users(id) ON DELETE CASCADE,

    -- Content location
    chapter_path VARCHAR(500) NOT NULL,
    section_id VARCHAR(200),

    -- Annotation details
    annotation_type VARCHAR(50) NOT NULL, -- 'note', 'highlight', 'bookmark', 'question'
    content TEXT NOT NULL, -- The note/highlight text
    color VARCHAR(20), -- For highlights: 'yellow', 'green', 'blue', 'pink'
    position_start INTEGER, -- Character position in section
    position_end INTEGER,

    -- Metadata
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- User progress tracking
CREATE TABLE IF NOT EXISTS user_progress (
    id SERIAL PRIMARY KEY,
    user_id INTEGER REFERENCES users(id) ON DELETE CASCADE,
    chapter_path VARCHAR(500) NOT NULL,

    -- Progress metrics
    completion_percentage INTEGER DEFAULT 0, -- 0-100
    last_read_at TIMESTAMP,
    time_spent_seconds INTEGER DEFAULT 0,
    is_completed BOOLEAN DEFAULT FALSE,

    -- Metadata
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,

    UNIQUE(user_id, chapter_path)
);

-- Sessions table (for Better Auth)
CREATE TABLE IF NOT EXISTS sessions (
    id VARCHAR(255) PRIMARY KEY,
    user_id INTEGER REFERENCES users(id) ON DELETE CASCADE,
    expires_at TIMESTAMP NOT NULL,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Create indexes for performance
CREATE INDEX IF NOT EXISTS idx_user_edits_user_chapter ON user_edits(user_id, chapter_path);
CREATE INDEX IF NOT EXISTS idx_user_annotations_user_chapter ON user_annotations(user_id, chapter_path);
CREATE INDEX IF NOT EXISTS idx_user_progress_user ON user_progress(user_id);
CREATE INDEX IF NOT EXISTS idx_sessions_user ON sessions(user_id);
CREATE INDEX IF NOT EXISTS idx_sessions_expires ON sessions(expires_at);

-- Trigger to update updated_at timestamps
CREATE OR REPLACE FUNCTION update_updated_at_column()
RETURNS TRIGGER AS $$
BEGIN
    NEW.updated_at = CURRENT_TIMESTAMP;
    RETURN NEW;
END;
$$ language 'plpgsql';

CREATE TRIGGER update_users_updated_at BEFORE UPDATE ON users
    FOR EACH ROW EXECUTE FUNCTION update_updated_at_column();

CREATE TRIGGER update_user_profiles_updated_at BEFORE UPDATE ON user_profiles
    FOR EACH ROW EXECUTE FUNCTION update_updated_at_column();

CREATE TRIGGER update_user_edits_updated_at BEFORE UPDATE ON user_edits
    FOR EACH ROW EXECUTE FUNCTION update_updated_at_column();

CREATE TRIGGER update_user_annotations_updated_at BEFORE UPDATE ON user_annotations
    FOR EACH ROW EXECUTE FUNCTION update_updated_at_column();

CREATE TRIGGER update_user_progress_updated_at BEFORE UPDATE ON user_progress
    FOR EACH ROW EXECUTE FUNCTION update_updated_at_column();
