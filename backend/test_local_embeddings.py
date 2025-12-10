#!/usr/bin/env python3
"""
Test script to verify local sentence-transformers embeddings are working properly
"""
import sys
import os
from pathlib import Path

# Add the backend directory to the path
sys.path.insert(0, str(Path(__file__).parent))

def test_embeddings():
    """Test the local embeddings functionality"""
    print("=" * 60)
    print("Testing Local Sentence-Transformers Embeddings")
    print("=" * 60)

    try:
        from app.embeddings import generate_embedding, test_embedding_connection, count_tokens
        from app.config import settings

        print(f"Embedding model: {settings.EMBEDDING_MODEL}")
        print(f"Vector size: {settings.VECTOR_SIZE}")
        print(f"Device: {settings.EMBEDDING_DEVICE}")

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

        print("\n" + "=" * 60)
        print("[SUCCESS] All embedding tests passed!")
        print("=" * 60)
        print("Local embeddings are ready to use with:")
        print(f"- Model: {settings.EMBEDDING_MODEL}")
        print(f"- Dimensions: {settings.VECTOR_SIZE}")
        print(f"- Provider: Local sentence-transformers (no API required)")
        print(f"- Cost: FREE (one-time download, then local)")
        return True

    except ImportError as e:
        print(f"[ERROR] Import error: {str(e)}")
        print("This might be because sentence-transformers is not installed yet.")
        print("Run: pip install -r requirements.txt")
        return False
    except Exception as e:
        print(f"[ERROR] Failed to test embeddings: {str(e)}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    success = test_embeddings()
    sys.exit(0 if success else 1)