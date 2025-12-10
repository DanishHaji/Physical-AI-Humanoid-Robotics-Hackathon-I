"""
Physical AI Textbook RAG API - Configuration (Cloud Free-Tier Stack)
Uses: Groq, Qdrant Cloud, Neon PostgreSQL, sentence-transformers
Cost: $0/month (100% free tier)
"""

from pydantic_settings import BaseSettings
from typing import List
import os
from pydantic import field_validator
import json


def parse_json_list(value: str) -> List[str]:
    """Parse a JSON array from string value"""
    if isinstance(value, str):
        if value.startswith('[') and value.endswith(']'):
            try:
                result = json.loads(value)
                if isinstance(result, list):
                    return result
            except json.JSONDecodeError:
                pass
        # If JSON parsing fails or it's not a list, split by comma
        return [item.strip() for item in value.split(',') if item.strip()]
    return value


class Settings(BaseSettings):
    # Application Settings
    APP_NAME: str = "Physical AI Textbook RAG API (Cloud Free-Tier)"
    APP_VERSION: str = "3.0.0"
    ENVIRONMENT: str = "development"
    DEBUG: bool = True
    HOST: str = "0.0.0.0"
    PORT: int = 8000
    RELOAD: bool = True

    # Groq API Configuration (Cloud LLM - FREE)
    # Sign up: https://console.groq.com
    # Free tier: 30 requests/min
    GROQ_API_KEY: str = ""  # Required - get from Groq console
    GROQ_MODEL: str = "llama-3.1-8b-instant"  # Free, fast (8B params)
    GROQ_TEMPERATURE: float = 0.1
    GROQ_MAX_TOKENS: int = 500
    GROQ_BASE_URL: str = "https://api.groq.com/openai/v1"

    # OpenAI API Configuration (Embeddings - FREE tier available)
    # Sign up: https://platform.openai.com/api-keys
    # Free tier: $5 credit (~10M tokens for embeddings)
    OPENAI_API_KEY: str = ""  # Required for embeddings - get from OpenAI dashboard

    # Groq Embeddings Configuration (API-based - FREE)
    # Using Groq API instead of local models to reduce RAM usage
    EMBEDDING_MODEL: str = "text-embedding-3-small"  # 1536 dimensions
    EMBEDDING_DEVICE: str = "api"  # Using API instead of local model
    VECTOR_SIZE: int = 1536  # Updated to match Groq embedding dimensions

    # Qdrant Cloud Configuration (Cloud Vector Database - FREE)
    # Sign up: https://cloud.qdrant.io
    # Free tier: 1GB cluster
    QDRANT_URL: str = ""  # Required - get from Qdrant Cloud dashboard
    QDRANT_API_KEY: str = ""  # Required - get from Qdrant Cloud dashboard
    QDRANT_COLLECTION_NAME: str = "textbook_chunks"
    QDRANT_DISTANCE: str = "Cosine"

    # Neon PostgreSQL Configuration (Cloud Database - FREE)
    # Sign up: https://neon.tech
    # Free tier: 0.5GB storage, 100 compute hours/month
    NEON_DATABASE_URL: str = ""  # Required - connection string from Neon dashboard
    NEON_MAX_CONNECTIONS: int = 10

    # Authentication Configuration
    # IMPORTANT: Generate a secure random key for production!
    # Example: python -c "import secrets; print(secrets.token_urlsafe(32))"
    SECRET_KEY: str = "your-secret-key-change-in-production-min-32-chars"  # Required for JWT tokens
    ACCESS_TOKEN_EXPIRE_MINUTES: int = 60 * 24 * 7  # 7 days

    # RAG Configuration
    RAG_TOP_K: int = 8
    RAG_SIMILARITY_THRESHOLD: float = 0.20
    RAG_CHUNK_SIZE: int = 1024
    RAG_CHUNK_OVERLAP: int = 100

    # Rate Limiting (protect free-tier quotas)
    RATE_LIMIT_PER_MINUTE: int = 20  # Conservative (Groq allows 30/min)
    RATE_LIMIT_PER_HOUR: int = 100
    RATE_LIMIT_PER_DAY: int = 1000

    # CORS Configuration
    ALLOWED_ORIGINS: List[str] = [
        "http://localhost:3000",
        "http://localhost:3001",  # Added for Docusaurus dev server
        "http://localhost:8000",
        "http://127.0.0.1:3000",
        "http://127.0.0.1:3001",  # Added for alternative format
        "http://127.0.0.1:8000",
        # Add your Vercel domain here - will be overridden by environment variable
    ]

    # Add Vercel domain from environment if available
    @property
    def all_allowed_origins(self) -> List[str]:
        origins = self.ALLOWED_ORIGINS.copy()
        vercel_domain = os.getenv("VERCEL_URL")
        if vercel_domain:
            origins.append(f"https://{vercel_domain}")
        frontend_url = os.getenv("FRONTEND_URL")
        if frontend_url and frontend_url not in origins:
            origins.append(frontend_url)
        return origins

    @field_validator("ALLOWED_ORIGINS", mode="before")
    @classmethod
    def validate_allowed_origins(cls, v):
        return parse_json_list(v)


    # Frontend API URL (for deployment)
    REACT_APP_API_URL: str = "http://localhost:8000"

    # Query Mode Detection Keywords
    CODE_KEYWORDS: List[str] = [
        "code", "example", "implement", "snippet", "syntax",
        "write", "create", "function", "class"
    ]
    EXAM_KEYWORDS: List[str] = [
        "quiz", "exam", "test", "mcq", "assessment",
        "question", "evaluate"
    ]
    EXPLAIN_KEYWORDS: List[str] = [
        "explain", "what is", "how does", "why", "describe",
        "tell me about", "define"
    ]

    class Config:
        env_file = ".env"
        case_sensitive = False


# Global settings instance
settings = Settings()


def get_settings() -> Settings:
    """Dependency injection for settings"""
    return settings


def is_production() -> bool:
    """Check if running in production environment"""
    return settings.ENVIRONMENT == "production"


def validate_cloud_credentials() -> dict:
    """
    Validate that all cloud API credentials are set
    Returns dict of credential status
    """
    status = {}

    # Check Groq API key
    status["groq"] = "configured" if settings.GROQ_API_KEY else "missing"

    # Check OpenAI API key (for embeddings)
    status["openai"] = "configured" if settings.OPENAI_API_KEY else "missing"

    # Check Qdrant Cloud credentials
    status["qdrant_url"] = "configured" if settings.QDRANT_URL else "missing"
    status["qdrant_key"] = "configured" if settings.QDRANT_API_KEY else "missing"

    # Check Neon database URL
    status["neon"] = "configured" if settings.NEON_DATABASE_URL else "missing"

    # Overall status
    all_configured = all(v == "configured" for v in status.values())
    status["all_services"] = "ready" if all_configured else "needs_setup"

    return status


def get_missing_credentials() -> List[str]:
    """
    Get list of missing credentials with setup instructions
    Returns list of missing credential names
    """
    missing = []

    if not settings.GROQ_API_KEY:
        missing.append("GROQ_API_KEY - Sign up at https://console.groq.com")

    if not settings.OPENAI_API_KEY:
        missing.append("OPENAI_API_KEY - Sign up at https://platform.openai.com/api-keys")

    if not settings.QDRANT_URL:
        missing.append("QDRANT_URL - Create cluster at https://cloud.qdrant.io")

    if not settings.QDRANT_API_KEY:
        missing.append("QDRANT_API_KEY - Get from Qdrant Cloud dashboard")

    if not settings.NEON_DATABASE_URL:
        missing.append("NEON_DATABASE_URL - Create database at https://neon.tech")

    return missing


# Print configuration on import (for debugging)
if settings.DEBUG:
    credential_status = validate_cloud_credentials()

    print(f"""
==========================================================
  Physical AI Textbook RAG - Cloud Free-Tier Stack
==========================================================
  LLM: Groq ({settings.GROQ_MODEL})
  Embeddings: OpenAI {settings.EMBEDDING_MODEL} ({settings.VECTOR_SIZE}d)
  Vector DB: Qdrant Cloud (1GB free)
  Database: Neon PostgreSQL (0.5GB free)
  Environment: {settings.ENVIRONMENT}
==========================================================
  Status: {credential_status['all_services'].upper()}
  - Groq API: {credential_status['groq']}
  - OpenAI API: {credential_status['openai']}
  - Qdrant URL: {credential_status['qdrant_url']}
  - Qdrant Key: {credential_status['qdrant_key']}
  - Neon DB: {credential_status['neon']}
==========================================================
    """)

    # Warn about missing credentials
    missing = get_missing_credentials()
    if missing:
        print("[WARNING] Missing credentials:")
        for cred in missing:
            print(f"   - {cred}")
        print("\nSee backend/.env.example for setup instructions\n")
