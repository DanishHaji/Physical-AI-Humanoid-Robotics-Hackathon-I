#!/usr/bin/env python3
"""
Test script to verify Groq embeddings are working properly
"""
import sys
import os
from pathlib import Path

# Add the backend directory to the path
sys.path.insert(0, str(Path(__file__).parent))

from app.embeddings import generate_embedding, test_embedding_connection, count_tokens
from app.config import settings

def test_embeddings():
    """Test the embeddings functionality"""
    print("=" * 60)
    print("Testing OpenAI Embeddings API")
    print("=" * 60)

    # Check if OpenAI API key is configured
    if not settings.OPENAI_API_KEY:
        print("[ERROR] OPENAI_API_KEY not configured in environment!")
        print("Please set OPENAI_API_KEY in your .env file")
        print("Get your API key from: https://platform.openai.com/api-keys")
        return False

    print(f"[OK] OPENAI_API_KEY is configured")
    print(f"Embedding model: {settings.EMBEDDING_MODEL}")
    print(f"Vector size: {settings.VECTOR_SIZE}")

    # Test connection
    print("\n[*] Testing embedding connection...")
    connection_ok = test_embedding_connection()
    if not connection_ok:
        print("[ERROR] Embedding connection test failed!")
        return False
    print("[OK] Embedding connection test passed")

    # Test token counting
    test_text = "This is a test sentence for embedding generation."
    token_count = count_tokens(test_text)
    print(f"\n[*] Token count test...")
    print(f"[OK] Token count for '{test_text[:30]}...': {token_count}")

    # Test single embedding
    print(f"\n[*] Testing single embedding generation...")
    try:
        embedding = generate_embedding(test_text)
        print(f"[OK] Generated embedding with {len(embedding)} dimensions")

        # Check that it matches expected size
        if len(embedding) == settings.VECTOR_SIZE:
            print(f"[OK] Embedding dimension matches config ({settings.VECTOR_SIZE})")
        else:
            print(f"[ERROR] Embedding dimension mismatch: got {len(embedding)}, expected {settings.VECTOR_SIZE}")
            return False

        # Show a sample of the embedding values
        print(f"Sample values: {embedding[:5]}... (first 5 values)")

    except Exception as e:
        print(f"[ERROR] Failed to generate embedding: {str(e)}")
        return False

    print("\n" + "=" * 60)
    print("[SUCCESS] All embedding tests passed!")
    print("=" * 60)
    print("Embeddings are ready to use with:")
    print(f"- Model: {settings.EMBEDDING_MODEL}")
    print(f"- Dimensions: {settings.VECTOR_SIZE}")
    print(f"- Provider: OpenAI API")
    print(f"- Cost: Free tier ($5 credit, ~10M tokens)")
    return True

if __name__ == "__main__":
    success = test_embeddings()
    sys.exit(0 if success else 1)