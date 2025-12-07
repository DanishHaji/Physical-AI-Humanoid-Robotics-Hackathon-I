"""
Qdrant Cloud Client for vector storage (Cloud Free-Tier)
Provides vector search using Qdrant Cloud's 1GB free cluster
Sign up: https://cloud.qdrant.io
"""

from qdrant_client import QdrantClient
from qdrant_client.models import (
    Distance, VectorParams, PointStruct, Filter,
    FieldCondition, MatchValue, ScoredPoint
)
from typing import List, Dict, Optional, Tuple
from uuid import uuid4
from ..config import settings


class QdrantClientWrapper:
    """
    Wrapper for Qdrant Cloud client with error handling and helper methods
    """

    def __init__(
        self,
        url: Optional[str] = None,
        api_key: Optional[str] = None
    ):
        """
        Initialize Qdrant Cloud client

        Args:
            url: Qdrant Cloud cluster URL (defaults to settings.QDRANT_URL)
            api_key: Qdrant Cloud API key (defaults to settings.QDRANT_API_KEY)

        Raises:
            ValueError: If credentials are not provided
        """
        self.url = url or settings.QDRANT_URL
        self.api_key = api_key or settings.QDRANT_API_KEY

        if not self.url or not self.api_key:
            raise ValueError(
                "Qdrant Cloud credentials not found. "
                "Sign up at https://cloud.qdrant.io and set QDRANT_URL and QDRANT_API_KEY in .env"
            )

        # Initialize Qdrant client
        self.client = QdrantClient(
            url=self.url,
            api_key=self.api_key,
            timeout=30  # 30 second timeout for cloud requests
        )

        self.collection_name = settings.QDRANT_COLLECTION_NAME
        self.vector_size = settings.VECTOR_SIZE

        print(f"[OK] Qdrant Cloud client initialized (collection: {self.collection_name})")

    def create_collection_if_not_exists(self) -> bool:
        """
        Create collection if it doesn't exist

        Returns:
            bool: True if created or already exists, False on error
        """
        try:
            # Check if collection exists
            collections = self.client.get_collections().collections
            collection_names = [c.name for c in collections]

            if self.collection_name in collection_names:
                print(f"[OK] Collection '{self.collection_name}' already exists")
                return True

            # Create collection
            self.client.create_collection(
                collection_name=self.collection_name,
                vectors_config=VectorParams(
                    size=self.vector_size,
                    distance=Distance.COSINE
                )
            )

            print(f"[OK] Created collection '{self.collection_name}'")
            return True

        except Exception as e:
            print(f"[ERROR] Error creating collection: {str(e)}")
            return False

    def upsert_chunks(
        self,
        chunks: List[Dict]
    ) -> bool:
        """
        Upsert chunks with embeddings to Qdrant Cloud

        Args:
            chunks: List of dicts with keys: id, embedding, content, metadata

        Returns:
            bool: True if successful, False otherwise
        """
        try:
            # Ensure collection exists
            self.create_collection_if_not_exists()

            # Convert chunks to PointStruct objects
            points = []
            for chunk in chunks:
                point = PointStruct(
                    id=str(chunk['id']),
                    vector=chunk['embedding'],
                    payload={
                        "content": chunk['content'],
                        **chunk['metadata']  # Flatten metadata into payload
                    }
                )
                points.append(point)

            # Upsert to Qdrant (batch operation)
            self.client.upsert(
                collection_name=self.collection_name,
                points=points
            )

            print(f"[OK] Upserted {len(chunks)} chunks to Qdrant Cloud")
            return True

        except Exception as e:
            print(f"[ERROR] Error upserting to Qdrant: {str(e)}")
            return False

    def search(
        self,
        query_embedding: List[float],
        top_k: int = 5,
        score_threshold: Optional[float] = None,
        filter_metadata: Optional[Dict] = None
    ) -> List[Tuple[str, float, Dict, str]]:
        """
        Search Qdrant Cloud for similar chunks

        Args:
            query_embedding: Query vector (384-dim for all-MiniLM-L6-v2)
            top_k: Number of results to return
            score_threshold: Minimum similarity score (0-1, higher = more similar)
            filter_metadata: Optional metadata filters (e.g., {"module": 1})

        Returns:
            List of tuples: (chunk_id, similarity_score, metadata, content)
        """
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

            # Search Qdrant using query_points (qdrant-client 1.16.1+)
            search_results: List[ScoredPoint] = self.client.query_points(
                collection_name=self.collection_name,
                query=query_embedding,
                limit=top_k,
                query_filter=query_filter,
                score_threshold=score_threshold or settings.RAG_SIMILARITY_THRESHOLD
            ).points

            # Convert to our format
            results = []
            for hit in search_results:
                chunk_id = hit.id
                score = hit.score  # Cosine similarity (0-1, higher = more similar)
                payload = hit.payload

                # Extract content and metadata
                content = payload.pop('content', '')
                metadata = payload  # Remaining payload is metadata

                results.append((chunk_id, score, metadata, content))

            print(f"[OK] Found {len(results)} chunks in Qdrant Cloud (threshold: {score_threshold or settings.RAG_SIMILARITY_THRESHOLD})")
            return results

        except Exception as e:
            print(f"[ERROR] Error searching Qdrant: {str(e)}")
            return []

    def get_chunk(self, chunk_id: str) -> Optional[Dict]:
        """
        Retrieve chunk from Qdrant by ID

        Args:
            chunk_id: ID of the chunk

        Returns:
            Dict with chunk data or None if not found
        """
        try:
            points = self.client.retrieve(
                collection_name=self.collection_name,
                ids=[chunk_id],
                with_vectors=True,
                with_payload=True
            )

            if points:
                point = points[0]
                payload = point.payload
                content = payload.pop('content', '')
                metadata = payload

                return {
                    "id": point.id,
                    "content": content,
                    "metadata": metadata,
                    "embedding": point.vector
                }

            return None

        except Exception as e:
            print(f"[ERROR] Error fetching chunk from Qdrant: {str(e)}")
            return None

    def get_chunks_batch(self, chunk_ids: List[str]) -> List[Dict]:
        """
        Retrieve multiple chunks from Qdrant

        Args:
            chunk_ids: List of chunk IDs

        Returns:
            List of dicts with chunk data
        """
        try:
            points = self.client.retrieve(
                collection_name=self.collection_name,
                ids=chunk_ids,
                with_payload=True
            )

            chunks = []
            for point in points:
                payload = point.payload
                content = payload.pop('content', '')

                chunks.append({
                    "id": point.id,
                    "content": content,
                    "metadata": payload,
                    "chapter_title": payload.get("chapter_title", "Unknown Chapter"),
                    "heading": payload.get("heading", "Unknown Section"),
                    "module": payload.get("module", 1),
                    "week": payload.get("week", 1)
                })

            # Sort by module and position
            chunks.sort(key=lambda x: (x['metadata'].get('module', 0), x['metadata'].get('position', 0)))

            return chunks

        except Exception as e:
            print(f"[ERROR] Error fetching chunks batch from Qdrant: {str(e)}")
            return []

    def get_collection_stats(self) -> Dict:
        """
        Get statistics about the Qdrant collection

        Returns:
            Dict with collection stats
        """
        try:
            collection_info = self.client.get_collection(self.collection_name)

            return {
                "collection_name": self.collection_name,
                "total_chunks": collection_info.points_count,
                "vector_dimension": collection_info.config.params.vectors.size,
                "distance_metric": collection_info.config.params.vectors.distance.name,
                "status": collection_info.status.name
            }

        except Exception as e:
            print(f"[ERROR] Error getting collection stats: {str(e)}")
            return {
                "collection_name": self.collection_name,
                "error": str(e)
            }

    def delete_collection(self) -> bool:
        """
        Delete the collection (use with caution!)

        Returns:
            bool: True if successful, False otherwise
        """
        try:
            self.client.delete_collection(self.collection_name)
            print(f"[OK] Deleted collection '{self.collection_name}'")
            return True

        except Exception as e:
            print(f"[ERROR] Error deleting collection: {str(e)}")
            return False

    def test_connection(self) -> bool:
        """
        Test Qdrant Cloud connection

        Returns:
            bool: True if connected, False otherwise
        """
        try:
            collections = self.client.get_collections()
            print(f"[OK] Qdrant Cloud connection successful ({len(collections.collections)} collections)")
            return True

        except Exception as e:
            print(f"[ERROR] Qdrant Cloud connection failed: {str(e)}")
            return False

    def get_client_info(self) -> Dict:
        """
        Get information about the Qdrant client configuration

        Returns:
            dict: Client metadata
        """
        return {
            "provider": "Qdrant Cloud",
            "url": self.url,
            "collection_name": self.collection_name,
            "vector_dimension": self.vector_size,
            "distance_metric": "Cosine",
            "cost": "FREE (1GB cluster)",
            "api_key_configured": bool(self.api_key)
        }


# Global instance (initialized when needed)
_qdrant_client: Optional[QdrantClientWrapper] = None


def get_qdrant_client() -> QdrantClientWrapper:
    """
    Get or create global Qdrant client instance

    Returns:
        QdrantClientWrapper instance

    Raises:
        ValueError: If credentials are not configured
    """
    global _qdrant_client

    if _qdrant_client is None:
        _qdrant_client = QdrantClientWrapper()

    return _qdrant_client


# Performance comparison (for reference)
"""
Qdrant Cloud vs ChromaDB Comparison:
┌─────────────────────┬─────────────────┬──────────────────────┐
│ Feature             │ ChromaDB        │ Qdrant Cloud         │
├─────────────────────┼─────────────────┼──────────────────────┤
│ Deployment          │ Local           │ Cloud (free tier)    │
│ Storage             │ SQLite (local)  │ 1GB (cloud)          │
│ Cost                │ FREE (local)    │ FREE (1GB cluster)   │
│ Search Speed        │ ~10-50ms        │ ~100-200ms (API)     │
│ Scalability         │ Limited (local) │ High (cloud)         │
│ API                 │ Python only     │ REST + gRPC          │
│ Persistence         │ Manual          │ Auto-replicated      │
│ Deployment          │ Same server     │ Separate service     │
└─────────────────────┴─────────────────┴──────────────────────┘

For textbook RAG:
- Qdrant Cloud enables serverless deployment (Vercel, Netlify, etc.)
- 1GB free tier is enough for ~1M chunks (with 384-dim embeddings)
- Auto-scaling and replication (no manual backups needed)
- Can deploy backend anywhere (not tied to vector DB location)
"""
