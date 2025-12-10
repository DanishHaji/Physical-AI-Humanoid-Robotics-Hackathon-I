#!/usr/bin/env python3
"""
Test script to verify the complete query process works properly
"""
import asyncio
import sys
from pathlib import Path

# Add the backend directory to the path
sys.path.insert(0, str(Path(__file__).parent))

async def test_complete_query():
    """Test the complete query process"""
    print("=" * 60)
    print("Testing Complete Query Process")
    print("=" * 60)

    # Test the query process step by step
    print("\n1. Testing complete query flow...")
    try:
        from app.embeddings import generate_embedding
        from app.retrieval import search_qdrant, get_chunks_batch_from_qdrant
        from app.answer_generator import generate_answer

        # Step 1: Generate embedding
        print("   - Generating embedding for 'What is Physical AI?'")
        query_embedding = generate_embedding("What is Physical AI?")
        print(f"   - Generated embedding: {len(query_embedding)} dimensions")

        # Step 2: Search Qdrant
        print("   - Searching Qdrant for relevant chunks...")
        qdrant_results = search_qdrant(query_embedding, top_k=3)
        print(f"   - Found {len(qdrant_results)} results from Qdrant")

        if qdrant_results:
            # Step 3: Get chunk IDs and fetch full chunks
            chunk_ids = [chunk_id for chunk_id, score, metadata, content in qdrant_results]
            print(f"   - Chunk IDs: {chunk_ids[:3]}...")  # Show first 3

            print("   - Fetching full chunk data from Qdrant...")
            chunks = get_chunks_batch_from_qdrant(chunk_ids)
            print(f"   - Retrieved {len(chunks)} full chunks")

            # Step 4: Generate answer using Groq
            print("   - Generating answer with Groq...")
            answer = generate_answer(
                question="What is Physical AI?",
                context_chunks=chunks,
                mode="explain"
            )
            print(f"   - Answer generated: {len(answer)} characters")
            print(f"   - Answer preview: {answer[:100]}...")

        else:
            print("   - No chunks found in Qdrant, testing no-context response...")
            from app.answer_generator import generate_no_context_response
            answer = generate_no_context_response("What is Physical AI?", "explain")
            print(f"   - No-context response: {len(answer)} characters")
            print(f"   - Response preview: {answer[:100]}...")

    except Exception as e:
        print(f"   - Complete query test failed: {e}")
        import traceback
        traceback.print_exc()
        return False

    print("\n" + "=" * 60)
    print("[SUCCESS] Complete query process works!")
    print("=" * 60)
    print("\nThe chatbot should work properly now.")
    return True

if __name__ == "__main__":
    success = asyncio.run(test_complete_query())
    sys.exit(0 if success else 1)