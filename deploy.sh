#!/bin/bash

# =================================================================
# Automated Deployment Script for Physical AI Textbook
# =================================================================
# This script automates the deployment to Vercel and Render
# Run this after you've signed up for the required services
# =================================================================

set -e  # Exit on any error

echo "🚀 Physical AI Textbook - Automated Deployment Script"
echo "======================================================"
echo ""

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Check if required CLIs are installed
check_requirements() {
    echo -e "${BLUE}Checking requirements...${NC}"

    # Check Node.js
    if ! command -v node &> /dev/null; then
        echo -e "${RED}❌ Node.js is not installed. Please install from https://nodejs.org${NC}"
        exit 1
    fi
    echo -e "${GREEN}✓ Node.js installed: $(node --version)${NC}"

    # Check npm
    if ! command -v npm &> /dev/null; then
        echo -e "${RED}❌ npm is not installed.${NC}"
        exit 1
    fi
    echo -e "${GREEN}✓ npm installed: $(npm --version)${NC}"

    # Check Python
    if ! command -v python &> /dev/null && ! command -v python3 &> /dev/null; then
        echo -e "${RED}❌ Python is not installed. Please install from https://python.org${NC}"
        exit 1
    fi

    if command -v python3 &> /dev/null; then
        PYTHON_CMD="python3"
        echo -e "${GREEN}✓ Python installed: $(python3 --version)${NC}"
    else
        PYTHON_CMD="python"
        echo -e "${GREEN}✓ Python installed: $(python --version)${NC}"
    fi

    # Check Git
    if ! command -v git &> /dev/null; then
        echo -e "${RED}❌ Git is not installed. Please install from https://git-scm.com${NC}"
        exit 1
    fi
    echo -e "${GREEN}✓ Git installed: $(git --version)${NC}"

    echo ""
}

# Install Vercel CLI
install_vercel_cli() {
    echo -e "${BLUE}Installing Vercel CLI...${NC}"
    if ! command -v vercel &> /dev/null; then
        npm install -g vercel
        echo -e "${GREEN}✓ Vercel CLI installed${NC}"
    else
        echo -e "${GREEN}✓ Vercel CLI already installed${NC}"
    fi
    echo ""
}

# Setup API Keys
setup_api_keys() {
    echo -e "${YELLOW}================================================${NC}"
    echo -e "${YELLOW}API Keys Setup${NC}"
    echo -e "${YELLOW}================================================${NC}"
    echo ""
    echo "You need to get API keys from the following services:"
    echo ""
    echo "1. Groq API (Free LLM)"
    echo "   → Sign up: https://console.groq.com"
    echo "   → Create API key in dashboard"
    echo ""
    echo "2. Qdrant Cloud (Free Vector Database)"
    echo "   → Sign up: https://cloud.qdrant.io"
    echo "   → Create free 1GB cluster"
    echo "   → Copy cluster URL and API key"
    echo ""
    echo "3. Neon PostgreSQL (Free Database)"
    echo "   → Sign up: https://neon.tech"
    echo "   → Create new database"
    echo "   → Copy connection string"
    echo ""

    read -p "Have you obtained all API keys? (y/n): " has_keys

    if [ "$has_keys" != "y" ]; then
        echo -e "${RED}Please get your API keys first, then run this script again.${NC}"
        exit 1
    fi

    echo ""
    echo -e "${BLUE}Enter your API keys:${NC}"
    echo ""

    read -p "Groq API Key: " GROQ_API_KEY
    read -p "Qdrant URL: " QDRANT_URL
    read -p "Qdrant API Key: " QDRANT_API_KEY
    read -p "Neon Database URL: " DATABASE_URL

    # Create .env file
    cat > backend/.env << EOF
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
EOF

    echo -e "${GREEN}✓ .env file created${NC}"
    echo ""
}

# Initialize databases
initialize_databases() {
    echo -e "${BLUE}Initializing databases...${NC}"

    cd backend

    # Create virtual environment if it doesn't exist
    if [ ! -d "venv" ]; then
        echo "Creating Python virtual environment..."
        $PYTHON_CMD -m venv venv
    fi

    # Activate virtual environment
    source venv/bin/activate || source venv/Scripts/activate

    # Install requirements
    echo "Installing Python dependencies..."
    pip install -r requirements.txt -q

    # Initialize Neon database
    echo "Initializing Neon database..."
    $PYTHON_CMD scripts/init_neon.py

    # Ingest content to Qdrant
    echo "Ingesting textbook content to Qdrant..."
    $PYTHON_CMD scripts/ingest_to_qdrant.py

    cd ..

    echo -e "${GREEN}✓ Databases initialized and populated${NC}"
    echo ""
}

# Deploy frontend to Vercel
deploy_frontend() {
    echo -e "${BLUE}Deploying frontend to Vercel...${NC}"

    # Install dependencies
    echo "Installing npm dependencies..."
    npm install

    # Build the project
    echo "Building Docusaurus site..."
    npm run build

    # Deploy to Vercel
    echo "Deploying to Vercel..."
    echo ""
    echo -e "${YELLOW}You will be prompted to login to Vercel.${NC}"
    echo -e "${YELLOW}Follow the browser login flow.${NC}"
    echo ""

    vercel --prod

    echo ""
    echo -e "${GREEN}✓ Frontend deployed to Vercel${NC}"
    echo ""
}

# Deploy backend to Render
deploy_backend() {
    echo -e "${BLUE}Deploying backend to Render...${NC}"
    echo ""
    echo -e "${YELLOW}Render deployment requires manual setup:${NC}"
    echo ""
    echo "1. Go to: https://render.com/register"
    echo "2. Sign up with your GitHub account"
    echo "3. Click 'New +' → 'Web Service'"
    echo "4. Select your repository"
    echo "5. Configure:"
    echo "   - Root Directory: backend"
    echo "   - Build Command: pip install -r requirements.txt"
    echo "   - Start Command: uvicorn app.main:app --host 0.0.0.0 --port \$PORT"
    echo "6. Add environment variables from backend/.env"
    echo "7. Click 'Create Web Service'"
    echo ""
    echo "See DEPLOYMENT.md for detailed instructions."
    echo ""

    read -p "Press Enter when backend is deployed and you have the URL..."

    read -p "Enter your Render backend URL: " BACKEND_URL

    echo ""
    echo -e "${GREEN}✓ Backend URL saved: $BACKEND_URL${NC}"
    echo ""

    # Update Vercel environment variable
    echo -e "${BLUE}Updating frontend API URL...${NC}"
    vercel env add REACT_APP_API_URL production
    # User will be prompted to enter the value

    # Redeploy frontend
    echo "Redeploying frontend with new API URL..."
    vercel --prod

    echo -e "${GREEN}✓ Frontend redeployed with backend URL${NC}"
    echo ""
}

# Test deployment
test_deployment() {
    echo -e "${BLUE}Testing deployment...${NC}"

    read -p "Enter your Vercel frontend URL: " FRONTEND_URL
    read -p "Enter your Render backend URL: " BACKEND_URL

    echo ""
    echo "Testing backend health endpoint..."
    curl -s "$BACKEND_URL/health" | grep -q "ok" && echo -e "${GREEN}✓ Backend is healthy${NC}" || echo -e "${RED}❌ Backend health check failed${NC}"

    echo ""
    echo "Testing frontend..."
    curl -s "$FRONTEND_URL" | grep -q "Physical AI" && echo -e "${GREEN}✓ Frontend is live${NC}" || echo -e "${RED}❌ Frontend check failed${NC}"

    echo ""
    echo -e "${GREEN}================================================${NC}"
    echo -e "${GREEN}🎉 Deployment Complete!${NC}"
    echo -e "${GREEN}================================================${NC}"
    echo ""
    echo "Frontend: $FRONTEND_URL"
    echo "Backend:  $BACKEND_URL"
    echo ""
    echo "Next steps:"
    echo "1. Visit your frontend URL"
    echo "2. Navigate to any chapter"
    echo "3. Try the chatbot in the bottom-right corner"
    echo "4. Ask: 'What is Physical AI?'"
    echo ""
}

# Main execution
main() {
    echo ""
    check_requirements
    install_vercel_cli
    setup_api_keys
    initialize_databases
    deploy_frontend
    deploy_backend
    test_deployment

    echo -e "${GREEN}All done! 🚀${NC}"
    echo ""
}

# Run main function
main
