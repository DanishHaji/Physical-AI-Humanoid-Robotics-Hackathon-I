"""
Configuration module for Physical AI Textbook RAG Backend
Loads environment variables and provides typed config classes
"""

from pydantic_settings import BaseSettings
from typing import Optional
import os


class Settings(BaseSettings):
    """Application settings loaded from environment variables"""

    # Application
    APP_NAME: str = "Physical AI Textbook RAG API"
    APP_VERSION: str = "1.0.0"
    ENVIRONMENT: str = "development"
    DEBUG: bool = True

    # Server
    HOST: str = "0.0.0.0"
    PORT: int = 8000
    RELOAD: bool = True

    # OpenAI API
    OPENAI_API_KEY: str
    OPENAI_EMBEDDING_MODEL: str = "text-embedding-3-small"
    OPENAI_CHAT_MODEL: str = "gpt-4o-mini"
    OPENAI_MAX_TOKENS: int = 500
    OPENAI_TEMPERATURE: float = 0.1

    # Qdrant Vector Database
    QDRANT_URL: str
    QDRANT_API_KEY: Optional[str] = None
    QDRANT_COLLECTION_NAME: str = "textbook_chunks"
    QDRANT_VECTOR_SIZE: int = 1536  # text-embedding-3-small dimension
    QDRANT_DISTANCE: str = "Cosine"

    # Neon PostgreSQL
    DATABASE_URL: str
    DB_POOL_MIN_SIZE: int = 1
    DB_POOL_MAX_SIZE: int = 5

    # RAG Configuration
    RAG_TOP_K: int = 5
    RAG_SIMILARITY_THRESHOLD: float = 0.7
    RAG_CHUNK_SIZE: int = 1024
    RAG_CHUNK_OVERLAP: int = 100

    # Rate Limiting
    RATE_LIMIT_PER_HOUR: int = 100
    RATE_LIMIT_PER_DAY: int = 1000

    # CORS
    ALLOWED_ORIGINS: list[str] = [
        "http://localhost:3000",
        "http://localhost:8000",
        "https://your-username.github.io"
    ]

    # Query Routing Keywords
    CODE_KEYWORDS: list[str] = ["code", "example", "implement", "snippet", "syntax", "write", "create"]
    EXAM_KEYWORDS: list[str] = ["quiz", "exam", "test", "mcq", "assessment", "question"]
    EXPLAIN_KEYWORDS: list[str] = ["explain", "what is", "how does", "why", "describe"]

    class Config:
        env_file = ".env"
        env_file_encoding = "utf-8"
        case_sensitive = True


# Global settings instance
settings = Settings()


# Helper functions
def get_settings() -> Settings:
    """Dependency injection for FastAPI routes"""
    return settings


def is_production() -> bool:
    """Check if running in production environment"""
    return settings.ENVIRONMENT == "production"


def get_database_config() -> dict:
    """Get database connection configuration"""
    return {
        "url": settings.DATABASE_URL,
        "min_size": settings.DB_POOL_MIN_SIZE,
        "max_size": settings.DB_POOL_MAX_SIZE,
    }


def get_qdrant_config() -> dict:
    """Get Qdrant client configuration"""
    return {
        "url": settings.QDRANT_URL,
        "api_key": settings.QDRANT_API_KEY,
        "prefer_grpc": True,
    }


def get_openai_config() -> dict:
    """Get OpenAI API configuration"""
    return {
        "api_key": settings.OPENAI_API_KEY,
        "embedding_model": settings.OPENAI_EMBEDDING_MODEL,
        "chat_model": settings.OPENAI_CHAT_MODEL,
        "max_tokens": settings.OPENAI_MAX_TOKENS,
        "temperature": settings.OPENAI_TEMPERATURE,
    }
