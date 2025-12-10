# Physical AI Textbook - Backend API

Complete authentication and personalization system for the AI-Native Physical AI & Humanoid Robotics textbook.

## Features Implemented

✅ **Complete Authentication System**
- JWT-based authentication with bcrypt password hashing
- User registration with comprehensive background questionnaire
- Login/Logout with token management
- Protected API endpoints with Bearer token authentication

✅ **User Personalization**
- Background questionnaire during signup (software/hardware experience, programming languages, robotics experience, learning goals)
- Automatic difficulty level detection
- Custom learning preferences

✅ **User-Specific Content**
- **Content Edits**: Personal versions of textbook content
- **Annotations**: Notes, highlights (yellow/green/blue/pink), bookmarks, questions
- **Progress Tracking**: Completion %, time spent, chapter completion status

## Setup Instructions

### 1. Prerequisites
- Python 3.9+
- Neon PostgreSQL account: https://neon.tech
- Qdrant Cloud account: https://cloud.qdrant.io
- Groq API key: https://console.groq.com

### 2. Install Dependencies
```bash
cd backend
pip install -r requirements.txt
```

### 3. Configure Environment

Create `.env` file in `backend/` directory:
```env
# Groq API (Free LLM)
GROQ_API_KEY=your-groq-api-key-here

# Qdrant Cloud (Free Vector Database)
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your-qdrant-api-key-here

# Neon PostgreSQL (Free Database)
NEON_DATABASE_URL=postgresql://user:password@host/database

# Authentication Secret (IMPORTANT: Change in production!)
SECRET_KEY=your-secret-key-min-32-chars-here
```

**Generate a secure SECRET_KEY:**
```bash
python -c "import secrets; print(secrets.token_urlsafe(32))"
```

### 4. Initialize Database
```bash
cd backend
python scripts/init_auth_db.py
```

Creates tables: users, user_profiles, user_edits, user_annotations, user_progress, sessions

### 5. Start the Backend
```bash
uvicorn app.main:app --reload --host 0.0.0.0 --port 8000
```

API: http://localhost:8000
Docs: http://localhost:8000/docs

## API Endpoints Overview

### Authentication
- `POST /auth/signup` - Create account with background questionnaire
- `POST /auth/login` - Login and get JWT token
- `GET /auth/me` - Get current user info
- `GET /auth/profile` - Get user profile
- `PUT /auth/profile` - Update profile & preferences

### Content Edits
- `POST /auth/edits` - Create/update personal edit
- `GET /auth/edits/{chapter_path}` - Get edits for chapter
- `DELETE /auth/edits/{edit_id}` - Delete edit

### Annotations
- `POST /auth/annotations` - Create annotation (note/highlight/bookmark/question)
- `GET /auth/annotations/{chapter_path}` - Get annotations for chapter
- `DELETE /auth/annotations/{annotation_id}` - Delete annotation

### Progress Tracking
- `POST /auth/progress` - Update chapter progress
- `GET /auth/progress/{chapter_path}` - Get progress for chapter
- `GET /auth/progress` - Get all progress
- `GET /auth/progress-stats` - Get overall statistics

Full API documentation: http://localhost:8000/docs

## Tech Stack

- **Backend**: FastAPI (Python 3.9+)
- **Auth**: JWT (python-jose) + bcrypt (passlib)
- **Database**: Neon PostgreSQL (cloud, serverless, FREE)
- **Vector DB**: Qdrant Cloud (1GB FREE)
- **LLM**: Groq API (llama-3.1-8b-instant, FREE)
- **Embeddings**: Local sentence-transformers (all-MiniLM-L6-v2, FREE)

**Total Cost**: $0/month (100% free tier)

## Security Features

- bcrypt password hashing with automatic salt
- JWT tokens signed with HS256
- Token expiration: 7 days (configurable)
- Password requirements: 8+ chars, 1 uppercase, 1 lowercase, 1 digit
- Protected endpoints require valid Bearer token
- Complete user data isolation
- CORS configured for frontend

## Development

### Project Structure
```
backend/
├── app/
│   ├── main.py           # FastAPI app
│   ├── config.py         # Settings
│   ├── auth/
│   │   ├── models.py     # Auth Pydantic models
│   │   ├── utils.py      # JWT & password utils
│   │   ├── database.py   # DB operations
│   │   └── router.py     # Auth API endpoints
│   └── database/
│       └── schema.sql    # Database schema
├── scripts/
│   └── init_auth_db.py   # DB initialization
└── requirements.txt
```

### Testing
```bash
pytest
```

## Deployment Checklist

- [ ] Generate new `SECRET_KEY` (never use default!)
- [ ] Set `DEBUG=false`
- [ ] Set `ENVIRONMENT=production`
- [ ] Use strong database passwords
- [ ] Enable SSL/TLS for database connections
- [ ] Configure proper CORS origins
- [ ] Set up HTTPS for API endpoints
- [ ] Enable monitoring and logging

## Free-Tier Limits

- **Groq**: 30 requests/min
- **Qdrant Cloud**: 1GB storage
- **Neon PostgreSQL**: 0.5GB storage, 100 compute hours/month
- **Embeddings**: Unlimited (local sentence-transformers)

## Troubleshooting

### Database Connection Issues
```bash
# Test Neon connection
psql $NEON_DATABASE_URL

# Verify tables exist
psql $NEON_DATABASE_URL -c "\dt"
```

### Token Validation Errors
- Ensure `SECRET_KEY` is consistent across all instances
- Check token expiration (default: 7 days)
- Verify Bearer token format: `Authorization: Bearer <token>`

### CORS Issues
Add frontend origin to `ALLOWED_ORIGINS` in `config.py`

## License

MIT License
