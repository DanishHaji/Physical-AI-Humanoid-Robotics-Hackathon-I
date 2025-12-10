#!/usr/bin/env python3
"""
Test script to verify the RAG query process works properly
"""
import asyncio
import sys
from pathlib import Path

# Add the backend directory to the path
sys.path.insert(0, str(Path(__file__).parent))

async def test_rag_process():
    """Test the complete RAG process step by step"""
    print("=" * 60)
    print("Testing RAG Query Process")
    print("=" * 60)

    # Test 1: Check if embeddings work
    print("\n1. Testing embeddings...")
    try:
        from app.embeddings import generate_embedding, test_embedding_connection
        print("   - Testing embedding connection...")
        emb_test = test_embedding_connection()
        print(f"   - Embedding connection: {emb_test}")

        if emb_test:
            print("   - Generating test embedding...")
            test_emb = generate_embedding("What is Physical AI?")
            print(f"   - Generated embedding with {len(test_emb)} dimensions")
    except Exception as e:
        print(f"   - Embedding test failed: {e}")
        return False

    # Test 2: Check if Qdrant connection works
    print("\n2. Testing Qdrant connection...")
    try:
        from app.retrieval import test_qdrant_connection
        qdrant_test = await test_qdrant_connection()
        print(f"   - Qdrant connection: {qdrant_test}")
        if not qdrant_test:
            print("   - WARNING: Qdrant connection failed - this will cause the chatbot to hang")
    except Exception as e:
        print(f"   - Qdrant test failed: {e}")
        return False

    # Test 3: Check if Groq connection works
    print("\n3. Testing Groq connection...")
    try:
        from app.answer_generator import test_groq_connection
        groq_test = test_groq_connection()
        print(f"   - Groq connection: {groq_test}")
    except Exception as e:
        print(f"   - Groq test failed: {e}")
        return False

    # Test 4: Try a simple search (this might fail if no data is in Qdrant)
    print("\n4. Testing search functionality...")
    try:
        from app.embeddings import generate_embedding
        from app.retrieval import search_qdrant

        # Generate embedding for test query
        query_embedding = generate_embedding("What is Physical AI?")
        print(f"   - Generated query embedding: {len(query_embedding)} dimensions")

        # Try to search (this will return empty if no data in Qdrant)
        results = search_qdrant(query_embedding, top_k=3)
        print(f"   - Search results count: {len(results)}")
        print("   - This is expected to be 0 if no textbook content has been ingested")

    except Exception as e:
        print(f"   - Search test failed: {e}")
        import traceback
        traceback.print_exc()
        return False

    print("\n" + "=" * 60)
    print("[SUCCESS] RAG process tests completed!")
    print("=" * 60)
    print("\nNote: If search returns 0 results, you need to ingest textbook content first.")
    print("Run: python scripts/ingest_to_qdrant.py to add textbook content to Qdrant.")

    if qdrant_test:
        print("\nThe chatbot should work, but may return 'no context found' if no content is ingested.")
    else:
        print("\nThe chatbot will hang because Qdrant connection failed.")

    return True

if __name__ == "__main__":
    success = asyncio.run(test_rag_process())
    sys.exit(0 if success else 1)