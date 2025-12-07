"""
Local Embedding Generation Module (Free Stack)
Generates vector embeddings using sentence-transformers (all-MiniLM-L6-v2)
No API keys required - runs completely offline on CPU/GPU
"""

from sentence_transformers import SentenceTransformer
from typing import List, Optional
import numpy as np
from .config import settings

# Initialize sentence-transformers model (loaded once, reused)
print(f"Loading embedding model: {settings.EMBEDDING_MODEL}...")
model = SentenceTransformer(
    settings.EMBEDDING_MODEL,
    device=settings.EMBEDDING_DEVICE
)
print(f"[OK] Embedding model loaded on {settings.EMBEDDING_DEVICE} (dimension: {settings.VECTOR_SIZE})")


def generate_embedding(text: str) -> List[float]:
    """
    Generate embedding vector for a single text using sentence-transformers

    Args:
        text: Input text to embed (no length limit, but performance degrades >512 tokens)

    Returns:
        List[float]: 384-dimensional embedding vector (all-MiniLM-L6-v2)

    Raises:
        Exception: If model inference fails
    """
    try:
        # Truncate very long texts for performance (sentence-transformers default: 512 tokens)
        if len(text) > 5000:  # ~1000 words
            text = text[:5000]
            print(f"[WARNING] Warning: Text truncated to 5000 characters for embedding")

        # Generate embedding (synchronous)
        embedding = model.encode(
            text,
            convert_to_numpy=True,
            show_progress_bar=False,
            normalize_embeddings=True  # L2 normalization for cosine similarity
        )

        # Convert numpy array to list
        embedding_list = embedding.tolist()

        # Validate dimension
        if len(embedding_list) != settings.VECTOR_SIZE:
            raise ValueError(
                f"Embedding dimension mismatch: expected {settings.VECTOR_SIZE}, "
                f"got {len(embedding_list)}"
            )

        return embedding_list

    except Exception as e:
        print(f"[ERROR] Error generating embedding: {str(e)}")
        raise


def generate_embeddings_batch(
    texts: List[str],
    batch_size: int = 32
) -> List[List[float]]:
    """
    Generate embeddings for multiple texts in batches (optimized for CPU/GPU)

    Args:
        texts: List of input texts to embed
        batch_size: Number of texts to process per batch (default: 32 for CPU)

    Returns:
        List[List[float]]: List of 384-dimensional embedding vectors

    Raises:
        Exception: If model inference fails
    """
    embeddings = []

    try:
        # Truncate very long texts
        truncated_texts = []
        for text in texts:
            if len(text) > 5000:
                text = text[:5000]
            truncated_texts.append(text)

        # Process in batches for memory efficiency
        for i in range(0, len(truncated_texts), batch_size):
            batch = truncated_texts[i:i + batch_size]

            # Generate embeddings for batch
            batch_embeddings = model.encode(
                batch,
                convert_to_numpy=True,
                show_progress_bar=False,
                normalize_embeddings=True,
                batch_size=batch_size
            )

            # Convert to list of lists
            batch_embeddings_list = batch_embeddings.tolist()
            embeddings.extend(batch_embeddings_list)

            print(f"[OK] Generated embeddings for batch {i // batch_size + 1}/{(len(texts) - 1) // batch_size + 1}")

        return embeddings

    except Exception as e:
        print(f"[ERROR] Error generating batch embeddings: {str(e)}")
        raise


def count_tokens(text: str) -> int:
    """
    Estimate token count for text (approximation for sentence-transformers)

    Note: sentence-transformers uses WordPiece tokenization (BERT-style).
    This is a rough estimate based on whitespace splitting.

    Args:
        text: Input text

    Returns:
        int: Approximate token count
    """
    # Rough approximation: 1 word ≈ 1.3 tokens for BERT models
    words = len(text.split())
    estimated_tokens = int(words * 1.3)
    return estimated_tokens


def truncate_text(text: str, max_chars: int = 5000) -> str:
    """
    Truncate text to a maximum number of characters

    Note: sentence-transformers models typically handle ~512 tokens max effectively.
    For all-MiniLM-L6-v2, 5000 chars ≈ 1000 words ≈ 1300 tokens.

    Args:
        text: Input text
        max_chars: Maximum number of characters (default: 5000)

    Returns:
        str: Truncated text
    """
    if len(text) <= max_chars:
        return text

    return text[:max_chars]


def test_embedding_connection() -> bool:
    """
    Test embedding model by generating a test embedding

    Returns:
        bool: True if model works, False otherwise
    """
    try:
        test_text = "This is a test sentence for embedding generation."
        embedding = generate_embedding(test_text)

        if len(embedding) == settings.VECTOR_SIZE:
            print(f"[OK] Sentence-transformers model working (dimension: {len(embedding)})")
            return True
        else:
            print(f"[ERROR] Embedding dimension mismatch: {len(embedding)}")
            return False

    except Exception as e:
        print(f"[ERROR] Embedding model test failed: {str(e)}")
        return False


def get_model_info() -> dict:
    """
    Get information about the loaded embedding model

    Returns:
        dict: Model metadata (name, dimension, device, max_seq_length)
    """
    return {
        "model_name": settings.EMBEDDING_MODEL,
        "vector_dimension": settings.VECTOR_SIZE,
        "device": settings.EMBEDDING_DEVICE,
        "max_seq_length": model.max_seq_length,
        "tokenizer": "WordPiece (BERT-style)",
        "normalization": "L2 (for cosine similarity)",
        "cost": "FREE (runs locally)"
    }


def compute_similarity(embedding1: List[float], embedding2: List[float]) -> float:
    """
    Compute cosine similarity between two embeddings

    Args:
        embedding1: First embedding vector
        embedding2: Second embedding vector

    Returns:
        float: Cosine similarity score (0-1, higher = more similar)
    """
    # Convert to numpy arrays
    vec1 = np.array(embedding1)
    vec2 = np.array(embedding2)

    # Cosine similarity (already normalized, so just dot product)
    similarity = np.dot(vec1, vec2)

    return float(similarity)


# Performance comparison with OpenAI (for reference)
"""
Performance Comparison:
┌─────────────────────┬─────────────────┬──────────────────────┐
│ Feature             │ OpenAI          │ sentence-transformers│
├─────────────────────┼─────────────────┼──────────────────────┤
│ Model               │ text-embed-3-s  │ all-MiniLM-L6-v2     │
│ Dimension           │ 1536            │ 384                  │
│ Cost                │ $0.02/1M tokens │ FREE                 │
│ Speed (single)      │ ~200ms (API)    │ ~20ms (CPU)          │
│ Speed (batch 100)   │ ~500ms          │ ~200ms (CPU)         │
│ Max tokens          │ 8191            │ 512 (optimal)        │
│ Quality             │ Very High       │ Good                 │
│ Privacy             │ Cloud (OpenAI)  │ 100% Local           │
│ Internet Required   │ Yes             │ No                   │
└─────────────────────┴─────────────────┴──────────────────────┘

For textbook RAG use case:
- sentence-transformers is sufficient (same papers, consistent terminology)
- 10x faster for local queries
- No rate limits, no costs
- Privacy: student queries stay on your server
"""
