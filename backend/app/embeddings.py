"""
OpenAI Embedding Generation Module
Generates vector embeddings for text using text-embedding-3-small
"""

from openai import AsyncOpenAI
from typing import List, Optional
import tiktoken
from .config import settings

# Initialize OpenAI client
client = AsyncOpenAI(api_key=settings.OPENAI_API_KEY)

# Initialize tokenizer for accurate token counting
encoding = tiktoken.get_encoding("cl100k_base")


async def generate_embedding(text: str) -> List[float]:
    """
    Generate embedding vector for a single text using OpenAI API

    Args:
        text: Input text to embed (will be truncated if >8191 tokens)

    Returns:
        List[float]: 1536-dimensional embedding vector

    Raises:
        Exception: If OpenAI API call fails
    """
    try:
        # Truncate text if it exceeds model's token limit
        tokens = encoding.encode(text)
        if len(tokens) > 8191:
            # Keep first 8191 tokens
            tokens = tokens[:8191]
            text = encoding.decode(tokens)
            print(f"⚠️ Warning: Text truncated to 8191 tokens for embedding")

        # Call OpenAI embedding API
        response = await client.embeddings.create(
            model=settings.OPENAI_EMBEDDING_MODEL,
            input=text,
            encoding_format="float"
        )

        # Extract embedding vector
        embedding = response.data[0].embedding

        # Validate dimension
        if len(embedding) != settings.QDRANT_VECTOR_SIZE:
            raise ValueError(
                f"Embedding dimension mismatch: expected {settings.QDRANT_VECTOR_SIZE}, "
                f"got {len(embedding)}"
            )

        return embedding

    except Exception as e:
        print(f"❌ Error generating embedding: {str(e)}")
        raise


async def generate_embeddings_batch(
    texts: List[str],
    batch_size: int = 100
) -> List[List[float]]:
    """
    Generate embeddings for multiple texts in batches

    Args:
        texts: List of input texts to embed
        batch_size: Number of texts to process per API call (max 2048 for OpenAI)

    Returns:
        List[List[float]]: List of 1536-dimensional embedding vectors

    Raises:
        Exception: If OpenAI API call fails
    """
    embeddings = []

    try:
        # Process in batches to respect API limits
        for i in range(0, len(texts), batch_size):
            batch = texts[i:i + batch_size]

            # Truncate texts in batch if needed
            truncated_batch = []
            for text in batch:
                tokens = encoding.encode(text)
                if len(tokens) > 8191:
                    tokens = tokens[:8191]
                    text = encoding.decode(tokens)
                truncated_batch.append(text)

            # Call OpenAI embedding API for batch
            response = await client.embeddings.create(
                model=settings.OPENAI_EMBEDDING_MODEL,
                input=truncated_batch,
                encoding_format="float"
            )

            # Extract embeddings from response
            batch_embeddings = [item.embedding for item in response.data]
            embeddings.extend(batch_embeddings)

            print(f"✅ Generated embeddings for batch {i // batch_size + 1}/{(len(texts) - 1) // batch_size + 1}")

        return embeddings

    except Exception as e:
        print(f"❌ Error generating batch embeddings: {str(e)}")
        raise


def count_tokens(text: str) -> int:
    """
    Count the number of tokens in a text using tiktoken

    Args:
        text: Input text

    Returns:
        int: Token count
    """
    tokens = encoding.encode(text)
    return len(tokens)


def truncate_text(text: str, max_tokens: int = 8191) -> str:
    """
    Truncate text to a maximum number of tokens

    Args:
        text: Input text
        max_tokens: Maximum number of tokens (default: 8191 for text-embedding-3-small)

    Returns:
        str: Truncated text
    """
    tokens = encoding.encode(text)
    if len(tokens) <= max_tokens:
        return text

    # Truncate and decode
    truncated_tokens = tokens[:max_tokens]
    return encoding.decode(truncated_tokens)


async def test_embedding_connection() -> bool:
    """
    Test OpenAI API connection by generating a test embedding

    Returns:
        bool: True if connection successful, False otherwise
    """
    try:
        test_text = "This is a test sentence for embedding generation."
        embedding = await generate_embedding(test_text)

        if len(embedding) == settings.QDRANT_VECTOR_SIZE:
            print(f"✅ OpenAI embedding connection successful (dimension: {len(embedding)})")
            return True
        else:
            print(f"❌ Embedding dimension mismatch: {len(embedding)}")
            return False

    except Exception as e:
        print(f"❌ OpenAI connection test failed: {str(e)}")
        return False


# Cost estimation helper
def estimate_embedding_cost(
    text_or_token_count: any,
    is_token_count: bool = False
) -> float:
    """
    Estimate cost for generating embeddings using text-embedding-3-small

    Args:
        text_or_token_count: Either text string or token count (int)
        is_token_count: If True, treats input as token count instead of text

    Returns:
        float: Estimated cost in USD
    """
    # text-embedding-3-small pricing: $0.02 per 1M tokens
    price_per_million_tokens = 0.02

    if is_token_count:
        token_count = text_or_token_count
    else:
        token_count = count_tokens(text_or_token_count)

    cost = (token_count / 1_000_000) * price_per_million_tokens
    return cost


def estimate_batch_cost(texts: List[str]) -> dict:
    """
    Estimate cost for embedding a batch of texts

    Args:
        texts: List of input texts

    Returns:
        dict: Contains total_tokens, cost_usd, and per_text_stats
    """
    total_tokens = 0
    per_text_tokens = []

    for text in texts:
        tokens = count_tokens(text)
        total_tokens += tokens
        per_text_tokens.append(tokens)

    cost_usd = estimate_embedding_cost(total_tokens, is_token_count=True)

    return {
        "total_tokens": total_tokens,
        "cost_usd": round(cost_usd, 6),
        "texts_count": len(texts),
        "avg_tokens_per_text": round(total_tokens / len(texts), 2),
        "min_tokens": min(per_text_tokens),
        "max_tokens": max(per_text_tokens)
    }
