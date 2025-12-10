"""
Embeddings Module (Using OpenAI API)
Uses OpenAI text-embedding-3-small for semantic search embeddings
"""
import os
from typing import List
import structlog
from openai import OpenAI
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

class OpenAIEmbeddingsClient:
    def __init__(self):
        # Use OpenAI API for embeddings (not Groq, as Groq doesn't provide embedding models)
        self.client = OpenAI(api_key=settings.OPENAI_API_KEY)
        self.model = "text-embedding-3-small"  # OpenAI embedding model
        # Verify vector size matches OpenAI's embedding dimensions
        if settings.VECTOR_SIZE != 1536:
            logger.warning(f"Vector size mismatch: config={settings.VECTOR_SIZE}, expected=1536")

    def get_embedding(self, text: str) -> List[float]:
        """
        Generate embedding for text using OpenAI API

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
            logger.error("Error generating embedding with OpenAI", error=str(e))
            raise

    def get_embeddings_batch(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings for multiple texts using OpenAI API

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
            logger.error("Error generating embeddings batch with OpenAI", error=str(e))
            raise

# Global instance
_embeddings_client = None

def get_embeddings_client():
    global _embeddings_client
    if _embeddings_client is None:
        _embeddings_client = OpenAIEmbeddingsClient()
    return _embeddings_client

def generate_embedding(text: str) -> List[float]:
    """
    Generate embedding for a single text using OpenAI API

    Args:
        text: Input text to embed

    Returns:
        List of float representing the embedding vector
    """
    client = get_embeddings_client()
    return client.get_embedding(text)

def generate_embeddings_batch(texts: List[str]) -> List[List[float]]:
    """
    Generate embeddings for multiple texts using OpenAI API

    Args:
        texts: List of input texts to embed

    Returns:
        List of embedding vectors
    """
    client = get_embeddings_client()
    return client.get_embeddings_batch(texts)

def test_embedding_connection() -> bool:
    """
    Test if OpenAI embedding API is accessible

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