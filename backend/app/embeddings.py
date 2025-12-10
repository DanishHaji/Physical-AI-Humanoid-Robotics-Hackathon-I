"""
Embeddings Module (Using Groq API)
Replaces local sentence-transformers with Groq API embeddings for Render deployment
"""
import os
from typing import List
import structlog
from groq import Groq
from .config import settings

logger = structlog.get_logger()

# Import tiktoken for token counting (lightweight alternative to transformers)
import tiktoken

# Initialize encoding for token counting
encoding = tiktoken.get_encoding("cl100k_base")  # Good for most text

def count_tokens(text: str) -> int:
    """
    Count the number of tokens in text using tiktoken

    Args:
        text: Input text

    Returns:
        int: Number of tokens
    """
    return len(encoding.encode(text))

class GroqEmbeddingsClient:
    def __init__(self):
        self.client = Groq(api_key=settings.GROQ_API_KEY)
        self.model = "text-embedding-3-small"  # Using the embedding model
        # Update vector size to match Groq's embedding dimensions
        settings.VECTOR_SIZE = 1536  # text-embedding-3-small produces 1536-dim vectors

    def get_embedding(self, text: str) -> List[float]:
        """
        Generate embedding for text using Groq API

        Args:
            text: Input text to embed

        Returns:
            List of float representing the embedding vector
        """
        try:
            response = self.client.embeddings.create(
                model=self.model,
                input=text
            )
            return response.data[0].embedding
        except Exception as e:
            logger.error("Error generating embedding with Groq", error=str(e))
            raise

    def get_embeddings_batch(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings for multiple texts using Groq API

        Args:
            texts: List of input texts to embed

        Returns:
            List of embedding vectors
        """
        try:
            response = self.client.embeddings.create(
                model=self.model,
                input=texts
            )
            return [item.embedding for item in response.data]
        except Exception as e:
            logger.error("Error generating embeddings batch with Groq", error=str(e))
            raise

# Global instance
_embeddings_client = None

def get_embeddings_client():
    global _embeddings_client
    if _embeddings_client is None:
        _embeddings_client = GroqEmbeddingsClient()
    return _embeddings_client

def generate_embedding(text: str) -> List[float]:
    """
    Generate embedding for a single text using Groq API

    Args:
        text: Input text to embed

    Returns:
        List of float representing the embedding vector
    """
    client = get_embeddings_client()
    return client.get_embedding(text)

def generate_embeddings_batch(texts: List[str]) -> List[List[float]]:
    """
    Generate embeddings for multiple texts using Groq API

    Args:
        texts: List of input texts to embed

    Returns:
        List of embedding vectors
    """
    client = get_embeddings_client()
    return client.get_embeddings_batch(texts)

def test_embedding_connection() -> bool:
    """
    Test if Groq embedding API is accessible

    Returns:
        bool: True if API is accessible, False otherwise
    """
    try:
        # Test with a simple text
        test_text = "test connection"
        embedding = generate_embedding(test_text)
        return len(embedding) > 0
    except Exception as e:
        logger.error("Embedding connection test failed", error=str(e))
        return False