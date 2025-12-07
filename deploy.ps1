# =================================================================
# Automated Deployment Script for Physical AI Textbook (Windows)
# =================================================================
# PowerShell version for Windows users
# Run this after you've signed up for the required services
# =================================================================

$ErrorActionPreference = "Stop"

Write-Host "🚀 Physical AI Textbook - Automated Deployment Script" -ForegroundColor Cyan
Write-Host "======================================================" -ForegroundColor Cyan
Write-Host ""

# Check requirements
function Check-Requirements {
    Write-Host "Checking requirements..." -ForegroundColor Blue

    # Check Node.js
    try {
        $nodeVersion = node --version
        Write-Host "✓ Node.js installed: $nodeVersion" -ForegroundColor Green
    } catch {
        Write-Host "❌ Node.js is not installed. Please install from https://nodejs.org" -ForegroundColor Red
        exit 1
    }

    # Check npm
    try {
        $npmVersion = npm --version
        Write-Host "✓ npm installed: $npmVersion" -ForegroundColor Green
    } catch {
        Write-Host "❌ npm is not installed." -ForegroundColor Red
        exit 1
    }

    # Check Python
    try {
        $pythonVersion = python --version
        Write-Host "✓ Python installed: $pythonVersion" -ForegroundColor Green
    } catch {
        Write-Host "❌ Python is not installed. Please install from https://python.org" -ForegroundColor Red
        exit 1
    }

    # Check Git
    try {
        $gitVersion = git --version
        Write-Host "✓ Git installed: $gitVersion" -ForegroundColor Green
    } catch {
        Write-Host "❌ Git is not installed. Please install from https://git-scm.com" -ForegroundColor Red
        exit 1
    }

    Write-Host ""
}

# Install Vercel CLI
function Install-VercelCLI {
    Write-Host "Installing Vercel CLI..." -ForegroundColor Blue

    try {
        vercel --version | Out-Null
        Write-Host "✓ Vercel CLI already installed" -ForegroundColor Green
    } catch {
        npm install -g vercel
        Write-Host "✓ Vercel CLI installed" -ForegroundColor Green
    }

    Write-Host ""
}

# Setup API Keys
function Setup-APIKeys {
    Write-Host "================================================" -ForegroundColor Yellow
    Write-Host "API Keys Setup" -ForegroundColor Yellow
    Write-Host "================================================" -ForegroundColor Yellow
    Write-Host ""

    Write-Host "You need to get API keys from the following services:" -ForegroundColor White
    Write-Host ""
    Write-Host "1. Groq API (Free LLM)"
    Write-Host "   → Sign up: https://console.groq.com"
    Write-Host "   → Create API key in dashboard"
    Write-Host ""
    Write-Host "2. Qdrant Cloud (Free Vector Database)"
    Write-Host "   → Sign up: https://cloud.qdrant.io"
    Write-Host "   → Create free 1GB cluster"
    Write-Host "   → Copy cluster URL and API key"
    Write-Host ""
    Write-Host "3. Neon PostgreSQL (Free Database)"
    Write-Host "   → Sign up: https://neon.tech"
    Write-Host "   → Create new database"
    Write-Host "   → Copy connection string"
    Write-Host ""

    $hasKeys = Read-Host "Have you obtained all API keys? (y/n)"

    if ($hasKeys -ne "y") {
        Write-Host "Please get your API keys first, then run this script again." -ForegroundColor Red
        exit 1
    }

    Write-Host ""
    Write-Host "Enter your API keys:" -ForegroundColor Blue
    Write-Host ""

    $GROQ_API_KEY = Read-Host "Groq API Key"
    $QDRANT_URL = Read-Host "Qdrant URL"
    $QDRANT_API_KEY = Read-Host "Qdrant API Key"
    $DATABASE_URL = Read-Host "Neon Database URL"

    # Create .env file
    $envContent = @"
# Application Settings
APP_NAME="Physical AI Textbook RAG API"
APP_VERSION="3.0.0"
ENVIRONMENT="production"
DEBUG=false

# Groq API Configuration
GROQ_API_KEY="$GROQ_API_KEY"
GROQ_MODEL="llama-3.1-8b-instant"
GROQ_TEMPERATURE=0.1
GROQ_MAX_TOKENS=500

# Sentence Transformers Configuration
EMBEDDING_MODEL="all-MiniLM-L6-v2"
EMBEDDING_DEVICE="cpu"
VECTOR_SIZE=384

# Qdrant Cloud Configuration
QDRANT_URL="$QDRANT_URL"
QDRANT_API_KEY="$QDRANT_API_KEY"
QDRANT_COLLECTION_NAME="textbook_chunks"
QDRANT_DISTANCE="Cosine"

# Neon PostgreSQL Configuration
DATABASE_URL="$DATABASE_URL"

# RAG Configuration
RAG_TOP_K=5
RAG_SIMILARITY_THRESHOLD=0.7

# Rate Limiting
RATE_LIMIT_PER_MINUTE=20
RATE_LIMIT_PER_HOUR=100
RATE_LIMIT_PER_DAY=1000

# CORS (will be updated after deployment)
ALLOWED_ORIGINS=["http://localhost:3000"]
"@

    Set-Content -Path "backend\.env" -Value $envContent

    Write-Host "✓ .env file created" -ForegroundColor Green
    Write-Host ""
}

# Initialize databases
function Initialize-Databases {
    Write-Host "Initializing databases..." -ForegroundColor Blue

    Set-Location backend

    # Create virtual environment if it doesn't exist
    if (-not (Test-Path "venv")) {
        Write-Host "Creating Python virtual environment..."
        python -m venv venv
    }

    # Activate virtual environment
    & .\venv\Scripts\Activate.ps1

    # Install requirements
    Write-Host "Installing Python dependencies..."
    pip install -r requirements.txt -q

    # Initialize Neon database
    Write-Host "Initializing Neon database..."
    python scripts/init_neon.py

    # Ingest content to Qdrant
    Write-Host "Ingesting textbook content to Qdrant..."
    python scripts/ingest_to_qdrant.py

    Set-Location ..

    Write-Host "✓ Databases initialized and populated" -ForegroundColor Green
    Write-Host ""
}

# Deploy frontend to Vercel
function Deploy-Frontend {
    Write-Host "Deploying frontend to Vercel..." -ForegroundColor Blue

    # Install dependencies
    Write-Host "Installing npm dependencies..."
    npm install

    # Build the project
    Write-Host "Building Docusaurus site..."
    npm run build

    # Deploy to Vercel
    Write-Host "Deploying to Vercel..."
    Write-Host ""
    Write-Host "You will be prompted to login to Vercel." -ForegroundColor Yellow
    Write-Host "Follow the browser login flow." -ForegroundColor Yellow
    Write-Host ""

    vercel --prod

    Write-Host ""
    Write-Host "✓ Frontend deployed to Vercel" -ForegroundColor Green
    Write-Host ""
}

# Deploy backend to Render
function Deploy-Backend {
    Write-Host "Deploying backend to Render..." -ForegroundColor Blue
    Write-Host ""
    Write-Host "Render deployment requires manual setup:" -ForegroundColor Yellow
    Write-Host ""
    Write-Host "1. Go to: https://render.com/register"
    Write-Host "2. Sign up with your GitHub account"
    Write-Host "3. Click 'New +' → 'Web Service'"
    Write-Host "4. Select your repository"
    Write-Host "5. Configure:"
    Write-Host "   - Root Directory: backend"
    Write-Host "   - Build Command: pip install -r requirements.txt"
    Write-Host "   - Start Command: uvicorn app.main:app --host 0.0.0.0 --port `$PORT"
    Write-Host "6. Add environment variables from backend\.env"
    Write-Host "7. Click 'Create Web Service'"
    Write-Host ""
    Write-Host "See DEPLOYMENT.md for detailed instructions."
    Write-Host ""

    Read-Host "Press Enter when backend is deployed and you have the URL"

    $BACKEND_URL = Read-Host "Enter your Render backend URL"

    Write-Host ""
    Write-Host "✓ Backend URL saved: $BACKEND_URL" -ForegroundColor Green
    Write-Host ""

    # Update Vercel environment variable
    Write-Host "Updating frontend API URL..." -ForegroundColor Blue
    Write-Host "Enter the backend URL when prompted by Vercel CLI"
    vercel env add REACT_APP_API_URL production

    # Redeploy frontend
    Write-Host "Redeploying frontend with new API URL..."
    vercel --prod

    Write-Host "✓ Frontend redeployed with backend URL" -ForegroundColor Green
    Write-Host ""
}

# Test deployment
function Test-Deployment {
    Write-Host "Testing deployment..." -ForegroundColor Blue

    $FRONTEND_URL = Read-Host "Enter your Vercel frontend URL"
    $BACKEND_URL = Read-Host "Enter your Render backend URL"

    Write-Host ""
    Write-Host "Testing backend health endpoint..."
    try {
        $response = Invoke-WebRequest -Uri "$BACKEND_URL/health" -UseBasicParsing
        if ($response.StatusCode -eq 200) {
            Write-Host "✓ Backend is healthy" -ForegroundColor Green
        }
    } catch {
        Write-Host "❌ Backend health check failed" -ForegroundColor Red
    }

    Write-Host ""
    Write-Host "Testing frontend..."
    try {
        $response = Invoke-WebRequest -Uri $FRONTEND_URL -UseBasicParsing
        if ($response.Content -match "Physical AI") {
            Write-Host "✓ Frontend is live" -ForegroundColor Green
        }
    } catch {
        Write-Host "❌ Frontend check failed" -ForegroundColor Red
    }

    Write-Host ""
    Write-Host "================================================" -ForegroundColor Green
    Write-Host "🎉 Deployment Complete!" -ForegroundColor Green
    Write-Host "================================================" -ForegroundColor Green
    Write-Host ""
    Write-Host "Frontend: $FRONTEND_URL"
    Write-Host "Backend:  $BACKEND_URL"
    Write-Host ""
    Write-Host "Next steps:"
    Write-Host "1. Visit your frontend URL"
    Write-Host "2. Navigate to any chapter"
    Write-Host "3. Try the chatbot in the bottom-right corner"
    Write-Host "4. Ask: 'What is Physical AI?'"
    Write-Host ""
}

# Main execution
function Main {
    Write-Host ""
    Check-Requirements
    Install-VercelCLI
    Setup-APIKeys
    Initialize-Databases
    Deploy-Frontend
    Deploy-Backend
    Test-Deployment

    Write-Host "All done! 🚀" -ForegroundColor Green
    Write-Host ""
}

# Run main function
Main
