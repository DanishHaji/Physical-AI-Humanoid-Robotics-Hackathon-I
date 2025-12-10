"""
Retrieval Module (Cloud Free-Tier Stack)
Vector search with Qdrant Cloud + query logging with Neon PostgreSQL
Uses cloud services - requires API credentials
"""

from typing import List, Dict, Optional, Tuple
from uuid import uuid4
import json
from datetime import datetime
from .config import settings
from .models import Citation
from .clients.qdrant_client import get_qdrant_client, QdrantClientWrapper
from .clients.neon_client import get_neon_client, NeonClient

# Global clients (initialized on startup)
qdrant_client: Optional[QdrantClientWrapper] = None
neon_client: Optional[NeonClient] = None


async def init_qdrant_client() -> QdrantClientWrapper:
    """
    Initialize Qdrant Cloud client

    Returns:
        QdrantClientWrapper instance
    """
    global qdrant_client

    try:
        qdrant_client = get_qdrant_client()
        qdrant_client.create_collection_if_not_exists()
        return qdrant_client

    except Exception as e:
        print(f"[ERROR] Failed to initialize Qdrant: {str(e)}")
        raise


async def init_neon_pool() -> NeonClient:
    """
    Initialize Neon PostgreSQL client

    Returns:
        NeonClient instance
    """
    global neon_client

    try:
        neon_client = await get_neon_client()
        await neon_client.init_tables()
        return neon_client

    except Exception as e:
        print(f"[ERROR] Failed to initialize Neon: {str(e)}")
        raise


async def close_connections():
    """Close Qdrant and Neon connections"""
    global neon_client

    # Qdrant client doesn't need explicit closing (HTTP-based)

    if neon_client:
        await neon_client.disconnect()
        print("[OK] Closed Neon connection")


def get_or_create_collection() -> QdrantClientWrapper:
    """
    Get or create Qdrant collection

    Returns:
        QdrantClientWrapper instance
    """
    global qdrant_client

    if not qdrant_client:
        # Use the synchronous getter instead of async init
        qdrant_client = get_qdrant_client()

    return qdrant_client


def upsert_chunks_to_qdrant(chunks: List[Dict]):
    """
    Upsert chunks with embeddings to Qdrant Cloud

    Args:
        chunks: List of dicts with keys: id, embedding, content, metadata
    """
    if not qdrant_client:
        get_or_create_collection()

    try:
        success = qdrant_client.upsert_chunks(chunks)
        if not success:
            raise RuntimeError("Failed to upsert chunks to Qdrant")

    except Exception as e:
        print(f"[ERROR] Error upserting to Qdrant: {str(e)}")
        raise


def search_qdrant(
    query_embedding: List[float],
    top_k: int = settings.RAG_TOP_K,
    filter_metadata: Optional[Dict] = None
) -> List[Tuple[str, float, Dict, str]]:
    """
    Search Qdrant Cloud for similar chunks

    Args:
        query_embedding: Query vector (384-dim)
        top_k: Number of results to return
        filter_metadata: Optional metadata filters (e.g., {"module": 1})

    Returns:
        List of tuples: (chunk_id, similarity_score, metadata, content)
    """
    if not qdrant_client:
        get_or_create_collection()

    try:
        search_results = qdrant_client.search(
            query_embedding=query_embedding,
            top_k=top_k,
            filter_metadata=filter_metadata,
            score_threshold=settings.RAG_SIMILARITY_THRESHOLD
        )

        return search_results

    except Exception as e:
        print(f"[ERROR] Error searching Qdrant: {str(e)}")
        raise


def get_chunk_from_qdrant(chunk_id: str) -> Optional[Dict]:
    """
    Retrieve chunk from Qdrant by ID

    Args:
        chunk_id: ID of the chunk

    Returns:
        Dict with chunk data or None if not found
    """
    if not qdrant_client:
        get_or_create_collection()

    try:
        return qdrant_client.get_chunk(chunk_id)

    except Exception as e:
        print(f"[ERROR] Error fetching chunk from Qdrant: {str(e)}")
        raise


def get_chunks_batch_from_qdrant(chunk_ids: List[str]) -> List[Dict]:
    """
    Retrieve multiple chunks from Qdrant

    Args:
        chunk_ids: List of chunk IDs

    Returns:
        List of dicts with chunk data
    """
    if not qdrant_client:
        get_or_create_collection()

    try:
        return qdrant_client.get_chunks_batch(chunk_ids)

    except Exception as e:
        print(f"[ERROR] Error fetching chunks batch from Qdrant: {str(e)}")
        raise


async def log_query_to_neon(
    question: str,
    mode: str,
    chunk_ids: List[str],
    answer: str,
    citations: List[Citation],
    response_time_ms: int,
    user_ip_hash: Optional[str] = None
) -> str:
    """
    Log query to Neon PostgreSQL for analytics

    Args:
        question: User's question
        mode: Query mode (explain, code, urdu, exam)
        chunk_ids: List of retrieved chunk IDs
        answer: Generated answer
        citations: List of Citation objects
        response_time_ms: Response time in milliseconds
        user_ip_hash: Hashed user IP (optional)

    Returns:
        UUID string of the logged query
    """
    if not neon_client:
        await init_neon_pool()

    try:
        # Convert Citation objects to dicts
        citations_dicts = [c.dict() for c in citations]

        query_id = await neon_client.log_query(
            question=question,
            mode=mode,
            chunk_ids=chunk_ids,
            answer=answer,
            citations=citations_dicts,
            response_time_ms=response_time_ms,
            user_ip_hash=user_ip_hash
        )

        return query_id

    except Exception as e:
        print(f"[WARNING] Warning: Failed to log query to Neon: {str(e)}")
        # Don't raise - logging failure shouldn't break the API
        return str(uuid4())


def extract_citations(chunks: List[Dict]) -> List[Citation]:
    """
    Extract citations from retrieved chunks

    Args:
        chunks: List of chunk dicts from ChromaDB

    Returns:
        List of Citation objects
    """
    citations = []
    seen = set()  # Avoid duplicate citations

    for chunk in chunks:
        chapter_title = chunk.get('chapter_title', 'Unknown Chapter')
        section = chunk.get('heading', 'Unknown Section')
        module = chunk.get('module', 1)
        week = chunk.get('week', 1)

        # Generate URL from module and week
        # Map module numbers to slugs
        module_slugs = {
            1: "module-01-ros2",
            2: "module-02-digital-twin",
            3: "module-03-isaac-sim",
            4: "module-04-vla"
        }
        module_slug = module_slugs.get(module, f"module-{module:02d}")

        # Map chapter titles to actual file slugs (matches actual .md file names)
        chapter_slug_map = {
            "Physical AI Introduction & Foundations": "physical-ai-intro",
            "ROS 2 Fundamentals": "ros2-fundamentals",
            "NVIDIA Isaac Sim": "isaac-sim",
            "VLA Systems Implementation": "vla-systems",
            "Capstone Integration Project": "capstone-project"
        }

        chapter_slug = chapter_slug_map.get(
            chapter_title,
            chapter_title.lower().replace(' ', '-').replace('&', '').replace('  ', '-')
        )

        week_slug = f"week-{week:02d}-{chapter_slug}"
        section_slug = section.lower().replace(' ', '-').replace('.', '').replace('>', '').replace(':', '').replace('(', '').replace(')', '')

        # Handle capstone (which is in docs/capstone/ not in a module folder)
        if week == 13 or chapter_title == "Capstone Integration Project":
            url = f"/docs/capstone/{week_slug}#{section_slug}"
        else:
            url = f"/docs/{module_slug}/{week_slug}#{section_slug}"

        # Create citation key to avoid duplicates
        citation_key = f"{chapter_title}|{section}"

        if citation_key not in seen:
            citations.append(Citation(
                chapter=chapter_title,
                section=section,
                url=url
            ))
            seen.add(citation_key)

    return citations


async def test_qdrant_connection() -> bool:
    """
    Test Qdrant Cloud connection

    Returns:
        bool: True if connected, False otherwise
    """
    try:
        if not qdrant_client:
            await init_qdrant_client()

        return qdrant_client.test_connection()

    except Exception as e:
        print(f"[ERROR] Qdrant connection failed: {str(e)}")
        return False


async def test_neon_connection() -> bool:
    """
    Test Neon PostgreSQL connection

    Returns:
        bool: True if connected, False otherwise
    """
    try:
        if not neon_client:
            await init_neon_pool()

        return await neon_client.test_connection()

    except Exception as e:
        print(f"[ERROR] Neon connection failed: {str(e)}")
        return False


def get_collection_stats() -> Dict:
    """
    Get statistics about the Qdrant collection

    Returns:
        Dict with collection stats
    """
    if not qdrant_client:
        get_or_create_collection()

    try:
        return qdrant_client.get_collection_stats()

    except Exception as e:
        print(f"[ERROR] Error getting collection stats: {str(e)}")
        return {
            "total_chunks": 0,
            "error": str(e)
        }


# Cloud Stack Benefits
"""
Cloud Stack Benefits (Qdrant + Neon vs ChromaDB + SQLite):
===========================================================

1. Vector Storage:
   - Qdrant Cloud: 1GB free cluster, auto-scaled, replicated
   - vs ChromaDB: Local storage, manual backups

2. Database:
   - Neon PostgreSQL: Serverless, auto-pause, 0.5GB free
   - vs SQLite: Single-file, limited concurrency

3. Performance:
   - Qdrant: ~100-200ms (API latency, but globally accessible)
   - Neon: ~10-30ms (cloud latency, connection pooling)
   - Overall: Slightly slower but enables serverless deployment

4. Deployment:
   - Cloud: Deploy anywhere (Vercel, Netlify, Render, Railway, Fly.io)
   - Local: Requires persistent disk, tied to single server

5. Cost:
   - 100% FREE (Groq 30 req/min + Qdrant 1GB + Neon 0.5GB + sentence-transformers)
   - No credit card required for any service

6. Scalability:
   - Qdrant: Auto-scales to handle traffic spikes
   - Neon: Auto-scales compute based on usage
   - Can upgrade to paid tiers when needed
"""
