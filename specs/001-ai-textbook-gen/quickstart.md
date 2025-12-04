# Quickstart Guide: AI-Native Textbook Development

**Purpose**: Get the Physical AI & Humanoid Robotics textbook project running locally in <30 minutes.

**Target Audience**: Developers contributing to textbook content, RAG backend, or custom theme.

---

## Prerequisites

Ensure you have the following installed before proceeding:

| Tool | Version | Check Command | Installation |
|------|---------|---------------|--------------|
| **Node.js** | 18+ | `node --version` | [Download](https://nodejs.org/) |
| **npm** | 8+ | `npm --version` | Included with Node.js |
| **Python** | 3.11+ | `python --version` or `python3 --version` | [Download](https://www.python.org/) |
| **pip** | 23+ | `pip --version` or `pip3 --version` | Included with Python |
| **Git** | 2.30+ | `git --version` | [Download](https://git-scm.com/) |

**Optional** (for full RAG functionality):
- **Qdrant Cloud Account**: [Sign up](https://cloud.qdrant.io/) (free tier, no credit card required)
- **Neon Postgres Account**: [Sign up](https://neon.tech/) (free tier)
- **OpenAI API Key**: [Get API key](https://platform.openai.com/api-keys) (pay-as-you-go)

---

## Step 1: Clone Repository

```bash
git clone https://github.com/YOUR-USERNAME/Physical-AI-Humanoid-Robotics.git
cd Physical-AI-Humanoid-Robotics
```

---

## Step 2: Frontend Setup (Docusaurus)

### Install Dependencies

```bash
npm install
```

This will install:
- Docusaurus 3.x
- React 18
- Mermaid plugin
- Custom theme dependencies

### Start Development Server

```bash
npm start
```

**Expected Output**:
```
[INFO] Starting the development server...
[SUCCESS] Docusaurus website is running at: http://localhost:3000/

✔ Client
  Compiled successfully in 5.23s
```

**Verify**: Open browser to `http://localhost:3000`. You should see the textbook homepage.

### Preview Build (Optional)

```bash
npm run build
npm run serve
```

Build output will be in `build/` directory. Serve command runs production preview at `http://localhost:3000`.

---

## Step 3: Backend Setup (FastAPI + RAG)

### Navigate to Backend Directory

```bash
cd backend
```

### Create Python Virtual Environment

**Linux/macOS**:
```bash
python3 -m venv venv
source venv/bin/activate
```

**Windows (PowerShell)**:
```powershell
python -m venv venv
.\venv\Scripts\Activate.ps1
```

**Windows (Command Prompt)**:
```cmd
python -m venv venv
venv\Scripts\activate.bat
```

### Install Python Dependencies

```bash
pip install -r requirements.txt
```

This will install:
- FastAPI 0.104+
- Uvicorn (ASGI server)
- OpenAI Python SDK
- Qdrant Client
- psycopg2 (PostgreSQL adapter)
- Pydantic
- tiktoken (token counting)

### Create Environment Variables File

Create `.env` file in `backend/` directory:

```bash
# On Linux/macOS
cp .env.example .env

# On Windows
copy .env.example .env
```

Edit `.env` and fill in your credentials:

```env
# OpenAI API
OPENAI_API_KEY=sk-your-openai-api-key-here

# Qdrant Cloud
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your-qdrant-api-key-here

# Neon Postgres
DATABASE_URL=postgresql://user:password@your-neon-host.neon.tech/textbook_db?sslmode=require

# Server Config
ENVIRONMENT=development
ALLOWED_ORIGINS=http://localhost:3000,http://localhost:8000
RATE_LIMIT_PER_HOUR=100
```

### Run Database Migrations (First Time Only)

```bash
python app/init_db.py
```

**Expected Output**:
```
Creating tables in Neon Postgres...
✓ chapters table created
✓ chunks table created
✓ queries table created
✓ assessments table created
✓ diagrams table created
Database initialization complete!
```

### Start FastAPI Server

```bash
uvicorn app.main:app --reload --port 8000
```

**Expected Output**:
```
INFO:     Will watch for changes in these directories: ['/path/to/backend']
INFO:     Uvicorn running on http://127.0.0.1:8000 (Press CTRL+C to quit)
INFO:     Started reloader process [12345] using StatReload
INFO:     Started server process [12346]
INFO:     Waiting for application startup.
INFO:     Application startup complete.
```

**Verify**: Open `http://localhost:8000/docs` to see auto-generated API documentation (FastAPI Swagger UI).

---

## Step 4: Database Setup

### Qdrant Cloud Setup

1. Sign up at [Qdrant Cloud](https://cloud.qdrant.io/) (free tier)
2. Create new cluster (select free tier, 1GB storage)
3. Copy cluster URL and API key to `.env` file
4. Create collection:

```bash
python app/setup_qdrant.py
```

**Expected Output**:
```
Connecting to Qdrant Cloud...
✓ Connected successfully
Creating collection 'textbook_chunks'...
✓ Collection created with 1536-dim vectors (cosine distance, HNSW index)
Setup complete!
```

### Neon Postgres Setup

1. Sign up at [Neon](https://neon.tech/) (free tier)
2. Create new project → Create database named `textbook_db`
3. Copy connection string to `.env` file (ensure `?sslmode=require` is appended)
4. Database tables were created in Step 3 (`init_db.py`)

### Verify Database Connections

```bash
python app/health_check.py
```

**Expected Output**:
```
Checking service health...
✓ Qdrant: operational (latency: 45ms)
✓ Neon Postgres: operational (latency: 32ms)
✓ OpenAI API: operational (latency: 120ms)
All services healthy!
```

---

## Step 5: Run Locally and Verify

### Run Full Stack

**Terminal 1** (Frontend):
```bash
cd Physical-AI-Humanoid-Robotics
npm start
```

**Terminal 2** (Backend):
```bash
cd Physical-AI-Humanoid-Robotics/backend
source venv/bin/activate  # or venv\Scripts\activate on Windows
uvicorn app.main:app --reload --port 8000
```

### Test RAG Chatbot

**Option 1: Using API Documentation UI**

1. Open `http://localhost:8000/docs`
2. Expand `POST /query` endpoint
3. Click "Try it out"
4. Enter test query:
   ```json
   {
     "question": "What is ROS 2?",
     "mode": "auto"
   }
   ```
5. Click "Execute"
6. Verify response includes `answer`, `citations`, `mode_detected`, `response_time_ms`

**Option 2: Using curl**

```bash
curl -X POST "http://localhost:8000/query" \
  -H "Content-Type: application/json" \
  -d '{
    "question": "What is ROS 2?",
    "mode": "auto"
  }'
```

**Expected Response**:
```json
{
  "answer": "ROS 2 (Robot Operating System 2) is an open-source middleware framework for building robot applications...",
  "citations": [
    {
      "chapter": "ROS 2 Fundamentals",
      "section": "Introduction to ROS 2",
      "url": "/docs/module-01-ros2/week-01-ros2-intro#introduction-to-ros-2"
    }
  ],
  "mode_detected": "explain",
  "response_time_ms": 1850
}
```

**Option 3: Using Frontend Chatbot**

1. Open `http://localhost:3000`
2. Click chatbot icon in sidebar
3. Type "What is ROS 2?" and press Enter
4. Verify answer appears with citations

---

## Step 6: Common Troubleshooting

### Issue: `npm install` fails with "EACCES" permission error

**Solution (Linux/macOS)**:
```bash
sudo chown -R $USER ~/.npm
sudo chown -R $USER /usr/local/lib/node_modules
npm install
```

**Solution (Windows)**: Run terminal as Administrator

---

### Issue: Python module not found error

**Solution**: Ensure virtual environment is activated

```bash
# Linux/macOS
source venv/bin/activate

# Windows
venv\Scripts\activate
```

Then reinstall dependencies:
```bash
pip install -r requirements.txt
```

---

### Issue: `psycopg2` installation fails on macOS

**Solution**: Install PostgreSQL development libraries

```bash
brew install postgresql
pip install psycopg2-binary
```

---

### Issue: Qdrant connection timeout

**Causes**:
- Invalid API key → Check `.env` file
- Firewall blocking outbound HTTPS → Whitelist `*.qdrant.io`
- Cluster not created → Verify cluster exists in Qdrant Cloud dashboard

**Debug**:
```bash
python app/debug_qdrant.py
```

---

### Issue: Neon Postgres SSL error

**Solution**: Ensure connection string includes `?sslmode=require`:

```env
DATABASE_URL=postgresql://user:pass@host.neon.tech/db?sslmode=require
```

---

### Issue: OpenAI API rate limit error

**Causes**:
- Free tier rate limit exceeded (3 requests/min)
- Invalid API key

**Solution**:
- Wait 60 seconds between queries
- Upgrade to pay-as-you-go tier ($5 minimum balance)
- Verify API key at https://platform.openai.com/api-keys

---

### Issue: Frontend can't connect to backend (CORS error)

**Solution**: Ensure `ALLOWED_ORIGINS` in `.env` includes frontend URL:

```env
ALLOWED_ORIGINS=http://localhost:3000,http://localhost:8000
```

Restart FastAPI server after changing `.env`:
```bash
uvicorn app.main:app --reload --port 8000
```

---

### Issue: Mermaid diagrams not rendering

**Causes**:
- `@docusaurus/plugin-mermaid` not installed
- Mermaid syntax error

**Solution**:
1. Verify plugin in `package.json`:
   ```json
   {
     "dependencies": {
       "@docusaurus/plugin-mermaid": "^3.0.0"
     }
   }
   ```

2. Add to `docusaurus.config.js`:
   ```javascript
   module.exports = {
     plugins: ['@docusaurus/plugin-mermaid'],
     markdown: {
       mermaid: true,
     },
   };
   ```

3. Validate Mermaid syntax at [Mermaid Live Editor](https://mermaid.live/)

---

### Issue: Build fails with "Out of memory"

**Solution**: Increase Node.js memory limit

**Linux/macOS**:
```bash
export NODE_OPTIONS="--max-old-space-size=4096"
npm run build
```

**Windows (PowerShell)**:
```powershell
$env:NODE_OPTIONS="--max-old-space-size=4096"
npm run build
```

---

## Step 7: Next Steps

Once you have the project running locally:

### For Content Contributors

1. Read chapter writing guidelines: `.specify/templates/chapter-template.md`
2. Create new chapter: `docs/module-XX-name/week-YY-topic.md`
3. Follow 10-section structure (see constitution.md)
4. Add diagrams using Mermaid syntax
5. Test locally: `npm start`
6. Submit PR when ready

### For Backend Developers

1. Review API contracts: `specs/001-ai-textbook-gen/contracts/rag-api.openapi.yaml`
2. Implement new endpoints in `backend/app/main.py`
3. Add tests in `backend/tests/`
4. Run tests: `pytest`
5. Update OpenAPI docs automatically (FastAPI auto-generates from code)

### For Frontend Developers

1. Custom components: `src/components/`
2. Theme overrides: `src/theme/`
3. Custom CSS: `src/css/custom.css`
4. Test responsive design: Chrome DevTools (F12 → Device toolbar)
5. Verify accessibility: Lighthouse audit (Chrome DevTools → Lighthouse tab)

---

## Useful Commands

| Task | Command |
|------|---------|
| **Install frontend deps** | `npm install` |
| **Start frontend dev server** | `npm start` |
| **Build frontend for production** | `npm run build` |
| **Serve production build** | `npm run serve` |
| **Install backend deps** | `pip install -r requirements.txt` |
| **Start backend dev server** | `uvicorn app.main:app --reload --port 8000` |
| **Run backend tests** | `pytest` |
| **Format Python code** | `black app/` |
| **Lint Python code** | `ruff app/` |
| **Check TypeScript types** | `npm run typecheck` |
| **Run Lighthouse audit** | `npm run lighthouse` |
| **Generate API docs** | Navigate to `http://localhost:8000/docs` (auto-generated) |

---

## Project Structure Quick Reference

```
Physical-AI-Humanoid-Robotics/
├── docs/                    # Textbook chapters (Markdown)
├── src/                     # Custom React components + theme
├── static/                  # Images, diagrams, assets
├── backend/                 # FastAPI RAG server
│   ├── app/
│   │   ├── main.py         # API routes
│   │   ├── embeddings.py   # OpenAI embedding logic
│   │   ├── retrieval.py    # Qdrant + Neon queries
│   │   └── config.py       # Environment variables
│   ├── tests/              # pytest tests
│   └── requirements.txt    # Python dependencies
├── specs/                  # Feature specifications (SDD)
├── history/                # PHRs and ADRs
├── docusaurus.config.js    # Main Docusaurus config
├── package.json            # Frontend dependencies
└── .env.example            # Environment variables template
```

---

## Support

- **Documentation Issues**: Check `docs/` content, review constitution.md
- **Backend Issues**: Review API docs at `http://localhost:8000/docs`
- **Build Issues**: Check `npm run build` output, review GitHub Actions logs
- **General Questions**: See `README.md` or open GitHub Issue

---

**Quickstart Complete!** You should now have:
- ✅ Frontend running at `http://localhost:3000`
- ✅ Backend running at `http://localhost:8000`
- ✅ Databases connected (Qdrant + Neon + OpenAI)
- ✅ RAG chatbot functional

**Estimated Time**: 20-30 minutes (excluding account signups)

**Date**: 2025-12-04
