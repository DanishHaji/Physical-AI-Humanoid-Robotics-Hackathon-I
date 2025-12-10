"""
Embeddings Module (Using sentence-transformers)
Uses local sentence-transformers model for semantic search embeddings
"""
import os
from typing import List
import structlog
from sentence_transformers import SentenceTransformer
import torch
from .config import settings

logger = structlog.get_logger()

def count_tokens(text: str) -> int:
    """
    Count the number of tokens in text (approximation for local models)
    For local models, we use a simple approximation based on word count

    Args:
        text: Input text

    Returns:
        int: Number of tokens (approximate)
    """
    # Simple token approximation: split by whitespace and punctuation
    import re
    words = re.findall(r'\b\w+\b', text.lower())
    return len(words)

class LocalEmbeddingsClient:
    def __init__(self):
        # Use local sentence-transformers model for embeddings
        logger.info(f"Loading embedding model: {settings.EMBEDDING_MODEL}")

        # Determine device based on configuration and availability
        if settings.EMBEDDING_DEVICE == "cuda" and torch.cuda.is_available():
            device = "cuda"
        else:
            device = "cpu"

        self.model = SentenceTransformer(settings.EMBEDDING_MODEL, device=device)
        self.device = device

        logger.info(f"Embedding model loaded on {device}")

        # Verify vector size matches the model's output
        sample_embedding = self.model.encode(["test"])
        actual_dim = len(sample_embedding[0]) if hasattr(sample_embedding[0], '__len__') else sample_embedding.shape[-1]

        if actual_dim != settings.VECTOR_SIZE:
            logger.warning(f"Vector size mismatch: config={settings.VECTOR_SIZE}, actual={actual_dim}")
            # Note: We don't update the global settings here to avoid import conflicts

    def get_embedding(self, text: str) -> List[float]:
        """
        Generate embedding for text using local sentence-transformers model

        Args:
            text: Input text to embed

        Returns:
            List of float representing the embedding vector
        """
        try:
            embedding = self.model.encode([text])[0]
            # Convert to list if it's a numpy array
            if hasattr(embedding, 'tolist'):
                return embedding.tolist()
            else:
                return list(embedding)
        except Exception as e:
            logger.error("Error generating embedding with local model", error=str(e))
            raise

    def get_embeddings_batch(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings for multiple texts using local sentence-transformers model

        Args:
            texts: List of input texts to embed

        Returns:
            List of embedding vectors
        """
        try:
            embeddings = self.model.encode(texts)
            # Convert to list of lists if needed
            if hasattr(embeddings, 'tolist'):
                return embeddings.tolist()
            else:
                return [list(embedding) for embedding in embeddings]
        except Exception as e:
            logger.error("Error generating embeddings batch with local model", error=str(e))
            raise

# Global instance
_embeddings_client = None

def get_embeddings_client():
    global _embeddings_client
    if _embeddings_client is None:
        _embeddings_client = LocalEmbeddingsClient()
    return _embeddings_client

def generate_embedding(text: str) -> List[float]:
    """
    Generate embedding for a single text using local sentence-transformers model

    Args:
        text: Input text to embed

    Returns:
        List of float representing the embedding vector
    """
    client = get_embeddings_client()
    return client.get_embedding(text)

def generate_embeddings_batch(texts: List[str]) -> List[List[float]]:
    """
    Generate embeddings for multiple texts using local sentence-transformers model

    Args:
        texts: List of input texts to embed

    Returns:
        List of embedding vectors
    """
    client = get_embeddings_client()
    return client.get_embeddings_batch(texts)

def test_embedding_connection() -> bool:
    """
    Test if local embedding model is accessible

    Returns:
        bool: True if model is accessible, False otherwise
    """
    try:
        # Test with a simple text
        test_text = "test connection"
        embedding = generate_embedding(test_text)
        return len(embedding) > 0
    except Exception as e:
        logger.error("Embedding connection test failed", error=str(e))
        return False