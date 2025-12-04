"""
Retrieval Module
Vector search in Qdrant + metadata fetching from Neon PostgreSQL
"""

from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct, Filter, FieldCondition, MatchValue
from qdrant_client.models import ScoredPoint
import asyncpg
from typing import List, Dict, Optional, Tuple
from uuid import UUID, uuid4
from .config import settings, get_qdrant_config, get_database_config
from .models import Citation

# Global clients (initialized on startup)
qdrant_client: Optional[QdrantClient] = None
db_pool: Optional[asyncpg.Pool] = None


async def init_qdrant_client() -> QdrantClient:
    """
    Initialize Qdrant client with configuration

    Returns:
        QdrantClient instance
    """
    global qdrant_client

    config = get_qdrant_config()

    try:
        qdrant_client = QdrantClient(
            url=config['url'],
            api_key=config['api_key'],
            prefer_grpc=config.get('prefer_grpc', True)
        )

        print(f"✅ Connected to Qdrant at {config['url']}")
        return qdrant_client

    except Exception as e:
        print(f"❌ Failed to connect to Qdrant: {str(e)}")
        raise


async def init_neon_pool() -> asyncpg.Pool:
    """
    Initialize Neon PostgreSQL connection pool

    Returns:
        asyncpg.Pool instance
    """
    global db_pool

    config = get_database_config()

    try:
        db_pool = await asyncpg.create_pool(
            config['url'],
            min_size=config['min_size'],
            max_size=config['max_size']
        )

        print(f"✅ Connected to Neon PostgreSQL (pool size: {config['min_size']}-{config['max_size']})")
        return db_pool

    except Exception as e:
        print(f"❌ Failed to connect to Neon: {str(e)}")
        raise


async def close_connections():
    """Close Qdrant and Neon connections"""
    global qdrant_client, db_pool

    if qdrant_client:
        qdrant_client.close()
        print("✅ Closed Qdrant client")

    if db_pool:
        await db_pool.close()
        print("✅ Closed Neon pool")


async def create_qdrant_collection(
    collection_name: str = settings.QDRANT_COLLECTION_NAME,
    vector_size: int = settings.QDRANT_VECTOR_SIZE,
    distance: str = settings.QDRANT_DISTANCE
):
    """
    Create Qdrant collection with specified configuration

    Args:
        collection_name: Name of the collection
        vector_size: Dimension of vectors (default: 1536)
        distance: Distance metric (default: Cosine)
    """
    if not qdrant_client:
        raise RuntimeError("Qdrant client not initialized. Call init_qdrant_client() first.")

    try:
        # Check if collection exists
        collections = qdrant_client.get_collections().collections
        collection_names = [col.name for col in collections]

        if collection_name in collection_names:
            print(f"ℹ️  Collection '{collection_name}' already exists")
            return

        # Create collection
        qdrant_client.create_collection(
            collection_name=collection_name,
            vectors_config=VectorParams(
                size=vector_size,
                distance=Distance.COSINE if distance == "Cosine" else Distance.EUCLID
            )
        )

        print(f"✅ Created Qdrant collection: {collection_name} ({vector_size}-dim, {distance})")

    except Exception as e:
        print(f"❌ Error creating collection: {str(e)}")
        raise


async def upsert_chunks_to_qdrant(
    chunks: List[Dict],
    collection_name: str = settings.QDRANT_COLLECTION_NAME
):
    """
    Upsert chunks with embeddings to Qdrant

    Args:
        chunks: List of dicts with keys: id, embedding, metadata
        collection_name: Target collection name
    """
    if not qdrant_client:
        raise RuntimeError("Qdrant client not initialized")

    try:
        points = [
            PointStruct(
                id=str(chunk['id']),
                vector=chunk['embedding'],
                payload=chunk['metadata']
            )
            for chunk in chunks
        ]

        qdrant_client.upsert(
            collection_name=collection_name,
            points=points
        )

        print(f"✅ Upserted {len(chunks)} chunks to Qdrant")

    except Exception as e:
        print(f"❌ Error upserting to Qdrant: {str(e)}")
        raise


async def search_qdrant(
    query_embedding: List[float],
    top_k: int = settings.RAG_TOP_K,
    similarity_threshold: float = settings.RAG_SIMILARITY_THRESHOLD,
    filter_metadata: Optional[Dict] = None,
    collection_name: str = settings.QDRANT_COLLECTION_NAME
) -> List[Tuple[str, float, Dict]]:
    """
    Search Qdrant for similar chunks

    Args:
        query_embedding: Query vector (1536-dim)
        top_k: Number of results to return
        similarity_threshold: Minimum cosine similarity score (0-1)
        filter_metadata: Optional metadata filters (e.g., {"module": 1})
        collection_name: Collection to search

    Returns:
        List of tuples: (chunk_id, similarity_score, metadata)
    """
    if not qdrant_client:
        raise RuntimeError("Qdrant client not initialized")

    try:
        # Build filter if provided
        query_filter = None
        if filter_metadata:
            conditions = []
            for key, value in filter_metadata.items():
                conditions.append(
                    FieldCondition(
                        key=key,
                        match=MatchValue(value=value)
                    )
                )
            query_filter = Filter(must=conditions)

        # Search
        results = qdrant_client.search(
            collection_name=collection_name,
            query_vector=query_embedding,
            limit=top_k,
            query_filter=query_filter,
            score_threshold=similarity_threshold
        )

        # Extract results
        search_results = [
            (result.id, result.score, result.payload)
            for result in results
        ]

        print(f"✅ Found {len(search_results)} chunks in Qdrant (threshold: {similarity_threshold})")
        return search_results

    except Exception as e:
        print(f"❌ Error searching Qdrant: {str(e)}")
        raise


async def get_chunk_from_neon(chunk_id: str) -> Optional[Dict]:
    """
    Retrieve chunk metadata from Neon PostgreSQL

    Args:
        chunk_id: UUID of the chunk

    Returns:
        Dict with chunk data or None if not found
    """
    if not db_pool:
        raise RuntimeError("Neon pool not initialized")

    try:
        async with db_pool.acquire() as conn:
            row = await conn.fetchrow(
                """
                SELECT id, chapter_id, content, heading, position, token_count,
                       metadata, created_at
                FROM chunks
                WHERE id = $1
                """,
                UUID(chunk_id)
            )

            if row:
                return dict(row)
            return None

    except Exception as e:
        print(f"❌ Error fetching chunk from Neon: {str(e)}")
        raise


async def get_chunks_batch_from_neon(chunk_ids: List[str]) -> List[Dict]:
    """
    Retrieve multiple chunks from Neon PostgreSQL

    Args:
        chunk_ids: List of chunk UUIDs

    Returns:
        List of dicts with chunk data
    """
    if not db_pool:
        raise RuntimeError("Neon pool not initialized")

    try:
        async with db_pool.acquire() as conn:
            rows = await conn.fetch(
                """
                SELECT c.id, c.chapter_id, c.content, c.heading, c.position,
                       c.token_count, c.metadata, c.created_at,
                       ch.title as chapter_title, ch.module, ch.week
                FROM chunks c
                JOIN chapters ch ON c.chapter_id = ch.id
                WHERE c.id = ANY($1::uuid[])
                ORDER BY ch.week, c.position
                """,
                [UUID(cid) for cid in chunk_ids]
            )

            return [dict(row) for row in rows]

    except Exception as e:
        print(f"❌ Error fetching chunks batch from Neon: {str(e)}")
        raise


async def log_query_to_neon(
    question: str,
    mode: str,
    chunk_ids: List[str],
    answer: str,
    citations: List[Citation],
    response_time_ms: int,
    user_ip_hash: Optional[str] = None
) -> UUID:
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
        UUID of the logged query
    """
    if not db_pool:
        raise RuntimeError("Neon pool not initialized")

    try:
        async with db_pool.acquire() as conn:
            query_id = uuid4()

            await conn.execute(
                """
                INSERT INTO queries
                (id, question_text, mode, retrieved_chunk_ids, answer_text,
                 citations, response_time_ms, user_ip_hash, created_at)
                VALUES ($1, $2, $3, $4, $5, $6, $7, $8, NOW())
                """,
                query_id,
                question,
                mode,
                [UUID(cid) for cid in chunk_ids],
                answer,
                [c.dict() for c in citations],  # Convert to JSON
                response_time_ms,
                user_ip_hash
            )

            print(f"✅ Logged query to Neon (ID: {query_id})")
            return query_id

    except Exception as e:
        print(f"⚠️ Warning: Failed to log query to Neon: {str(e)}")
        # Don't raise - logging failure shouldn't break the API
        return uuid4()


def extract_citations(chunks: List[Dict]) -> List[Citation]:
    """
    Extract citations from retrieved chunks

    Args:
        chunks: List of chunk dicts from Neon

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
        # Example: /docs/module-01-ros2/week-02-ros2-fundamentals#section-slug
        module_slug = f"module-{module:02d}-ros2"  # TODO: Map module numbers to slugs
        week_slug = f"week-{week:02d}-{chapter_title.lower().replace(' ', '-')}"
        section_slug = section.lower().replace(' ', '-').replace('>', '-')

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
    Test Qdrant connection

    Returns:
        bool: True if connected, False otherwise
    """
    try:
        if not qdrant_client:
            await init_qdrant_client()

        collections = qdrant_client.get_collections()
        print(f"✅ Qdrant connection successful ({len(collections.collections)} collections)")
        return True

    except Exception as e:
        print(f"❌ Qdrant connection failed: {str(e)}")
        return False


async def test_neon_connection() -> bool:
    """
    Test Neon PostgreSQL connection

    Returns:
        bool: True if connected, False otherwise
    """
    try:
        if not db_pool:
            await init_neon_pool()

        async with db_pool.acquire() as conn:
            result = await conn.fetchval("SELECT 1")
            if result == 1:
                print("✅ Neon connection successful")
                return True
            return False

    except Exception as e:
        print(f"❌ Neon connection failed: {str(e)}")
        return False
