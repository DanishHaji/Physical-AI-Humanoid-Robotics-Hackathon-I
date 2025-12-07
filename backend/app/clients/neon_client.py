"""
Neon PostgreSQL Client for query logging (Cloud Free-Tier)
Provides async PostgreSQL access using Neon's serverless database
Sign up: https://neon.tech (0.5GB storage, 100 compute hours/month free)
"""

import asyncpg
from typing import List, Dict, Optional
from uuid import uuid4
from datetime import datetime
import json
from ..config import settings


class NeonClient:
    """
    Wrapper for Neon PostgreSQL client with connection pooling
    """

    def __init__(self, database_url: Optional[str] = None):
        """
        Initialize Neon client

        Args:
            database_url: Neon connection string (defaults to settings.NEON_DATABASE_URL)

        Raises:
            ValueError: If database URL is not provided
        """
        self.database_url = database_url or settings.NEON_DATABASE_URL

        if not self.database_url:
            raise ValueError(
                "Neon database URL not found. "
                "Sign up at https://neon.tech and set NEON_DATABASE_URL in .env"
            )

        self.pool: Optional[asyncpg.Pool] = None
        print(f"[OK] Neon client initialized")

    async def connect(self):
        """
        Create connection pool to Neon database

        Raises:
            Exception: If connection fails
        """
        if self.pool is not None:
            return  # Already connected

        try:
            self.pool = await asyncpg.create_pool(
                self.database_url,
                min_size=1,
                max_size=settings.NEON_MAX_CONNECTIONS,
                timeout=30,
                command_timeout=60
            )
            print(f"[OK] Connected to Neon PostgreSQL")

        except Exception as e:
            print(f"[ERROR] Failed to connect to Neon: {str(e)}")
            raise RuntimeError(
                f"Neon connection failed: {str(e)}. "
                "Check your NEON_DATABASE_URL in .env"
            )

    async def disconnect(self):
        """Close connection pool"""
        if self.pool:
            await self.pool.close()
            self.pool = None
            print(f"[OK] Disconnected from Neon PostgreSQL")

    async def init_tables(self):
        """
        Initialize database tables for query logging

        Creates:
        - queries table: stores user queries, responses, and citations
        """
        if not self.pool:
            await self.connect()

        try:
            async with self.pool.acquire() as conn:
                # Create queries table
                await conn.execute("""
                    CREATE TABLE IF NOT EXISTS queries (
                        id UUID PRIMARY KEY,
                        question_text TEXT NOT NULL,
                        mode VARCHAR(20) NOT NULL,
                        retrieved_chunk_ids JSONB NOT NULL,
                        answer_text TEXT NOT NULL,
                        citations JSONB NOT NULL,
                        response_time_ms INTEGER NOT NULL,
                        user_ip_hash VARCHAR(64),
                        created_at TIMESTAMP NOT NULL DEFAULT NOW()
                    )
                """)

                # Create index on created_at for analytics queries
                await conn.execute("""
                    CREATE INDEX IF NOT EXISTS idx_queries_created_at
                    ON queries(created_at DESC)
                """)

                # Create index on mode for filtering
                await conn.execute("""
                    CREATE INDEX IF NOT EXISTS idx_queries_mode
                    ON queries(mode)
                """)

                print(f"[OK] Initialized Neon database tables")

        except Exception as e:
            print(f"[ERROR] Error initializing tables: {str(e)}")
            raise

    async def log_query(
        self,
        question: str,
        mode: str,
        chunk_ids: List[str],
        answer: str,
        citations: List[Dict],
        response_time_ms: int,
        user_ip_hash: Optional[str] = None
    ) -> str:
        """
        Log query to Neon database

        Args:
            question: User's question
            mode: Query mode (explain, code, urdu, exam)
            chunk_ids: List of retrieved chunk IDs
            answer: Generated answer
            citations: List of citation dicts
            response_time_ms: Response time in milliseconds
            user_ip_hash: Hashed user IP (optional, for analytics)

        Returns:
            str: UUID of the logged query

        Raises:
            Exception: If database operation fails
        """
        if not self.pool:
            await self.connect()

        try:
            query_id = str(uuid4())

            async with self.pool.acquire() as conn:
                await conn.execute(
                    """
                    INSERT INTO queries
                    (id, question_text, mode, retrieved_chunk_ids, answer_text,
                     citations, response_time_ms, user_ip_hash, created_at)
                    VALUES ($1, $2, $3, $4, $5, $6, $7, $8, $9)
                    """,
                    query_id,
                    question,
                    mode,
                    json.dumps(chunk_ids),  # JSONB
                    answer,
                    json.dumps(citations),  # JSONB
                    response_time_ms,
                    user_ip_hash,
                    datetime.utcnow()
                )

            print(f"[OK] Logged query to Neon (ID: {query_id})")
            return query_id

        except Exception as e:
            print(f"[WARNING] Warning: Failed to log query to Neon: {str(e)}")
            # Don't raise - logging failure shouldn't break the API
            return str(uuid4())

    async def get_query_stats(self) -> Dict:
        """
        Get statistics about logged queries

        Returns:
            Dict with query statistics
        """
        if not self.pool:
            await self.connect()

        try:
            async with self.pool.acquire() as conn:
                # Total queries
                total = await conn.fetchval("SELECT COUNT(*) FROM queries")

                # Queries by mode
                mode_stats = await conn.fetch("""
                    SELECT mode, COUNT(*) as count
                    FROM queries
                    GROUP BY mode
                    ORDER BY count DESC
                """)

                # Average response time
                avg_time = await conn.fetchval("""
                    SELECT AVG(response_time_ms)::INTEGER
                    FROM queries
                """)

                # Queries in last 24 hours
                recent = await conn.fetchval("""
                    SELECT COUNT(*)
                    FROM queries
                    WHERE created_at > NOW() - INTERVAL '24 hours'
                """)

                return {
                    "total_queries": total,
                    "queries_24h": recent,
                    "avg_response_time_ms": avg_time or 0,
                    "queries_by_mode": {row['mode']: row['count'] for row in mode_stats}
                }

        except Exception as e:
            print(f"[ERROR] Error getting query stats: {str(e)}")
            return {
                "error": str(e)
            }

    async def get_recent_queries(self, limit: int = 10) -> List[Dict]:
        """
        Get recent queries from the database

        Args:
            limit: Maximum number of queries to return

        Returns:
            List of query dicts
        """
        if not self.pool:
            await self.connect()

        try:
            async with self.pool.acquire() as conn:
                rows = await conn.fetch("""
                    SELECT
                        id, question_text, mode, answer_text,
                        response_time_ms, created_at
                    FROM queries
                    ORDER BY created_at DESC
                    LIMIT $1
                """, limit)

                queries = []
                for row in rows:
                    queries.append({
                        "id": str(row['id']),
                        "question": row['question_text'],
                        "mode": row['mode'],
                        "answer": row['answer_text'][:200] + "..." if len(row['answer_text']) > 200 else row['answer_text'],
                        "response_time_ms": row['response_time_ms'],
                        "created_at": row['created_at'].isoformat()
                    })

                return queries

        except Exception as e:
            print(f"[ERROR] Error getting recent queries: {str(e)}")
            return []

    async def execute_raw(self, sql: str) -> None:
        """
        Execute raw SQL (for schema initialization, DDL, etc.)

        Args:
            sql: SQL statement to execute

        Raises:
            Exception: If execution fails
        """
        if not self.pool:
            await self.connect()

        try:
            async with self.pool.acquire() as conn:
                await conn.execute(sql)
            print(f"[OK] Executed raw SQL successfully")

        except Exception as e:
            print(f"[ERROR] Error executing raw SQL: {str(e)}")
            raise

    async def execute_query(self, sql: str, *args) -> List[Dict]:
        """
        Execute a query and return results as list of dicts

        Args:
            sql: SQL query to execute
            *args: Query parameters

        Returns:
            List of dicts with query results
        """
        if not self.pool:
            await self.connect()

        try:
            async with self.pool.acquire() as conn:
                rows = await conn.fetch(sql, *args)
                return [dict(row) for row in rows]

        except Exception as e:
            print(f"[ERROR] Error executing query: {str(e)}")
            raise

    async def test_connection(self) -> bool:
        """
        Test Neon database connection

        Returns:
            bool: True if connected, False otherwise
        """
        try:
            if not self.pool:
                await self.connect()

            async with self.pool.acquire() as conn:
                result = await conn.fetchval("SELECT 1")

                if result == 1:
                    print(f"[OK] Neon connection test successful")
                    return True
                return False

        except Exception as e:
            print(f"[ERROR] Neon connection test failed: {str(e)}")
            return False

    def get_client_info(self) -> Dict:
        """
        Get information about the Neon client configuration

        Returns:
            dict: Client metadata
        """
        return {
            "provider": "Neon PostgreSQL",
            "connection_url": self.database_url[:30] + "...",  # Masked for security
            "max_connections": settings.NEON_MAX_CONNECTIONS,
            "cost": "FREE (0.5GB storage, 100 compute hours/month)",
            "connected": self.pool is not None
        }


# Global instance (initialized when needed)
_neon_client: Optional[NeonClient] = None


async def get_neon_client() -> NeonClient:
    """
    Get or create global Neon client instance

    Returns:
        NeonClient instance

    Raises:
        ValueError: If database URL is not configured
    """
    global _neon_client

    if _neon_client is None:
        _neon_client = NeonClient()
        await _neon_client.connect()

    return _neon_client


# Performance comparison (for reference)
"""
Neon vs SQLite Comparison:
┌─────────────────────┬─────────────────┬──────────────────────┐
│ Feature             │ SQLite          │ Neon PostgreSQL      │
├─────────────────────┼─────────────────┼──────────────────────┤
│ Deployment          │ Local file      │ Cloud (free tier)    │
│ Storage             │ Local disk      │ 0.5GB (cloud)        │
│ Cost                │ FREE (local)    │ FREE (100h/month)    │
│ Query Speed         │ ~1-5ms          │ ~10-30ms (API)       │
│ Scalability         │ Single machine  │ Auto-scaling         │
│ Concurrency         │ Limited         │ High (pooling)       │
│ Backup              │ Manual          │ Auto-replicated      │
│ SQL Features        │ Basic           │ Full PostgreSQL      │
│ Deployment          │ Same server     │ Separate service     │
└─────────────────────┴─────────────────┴──────────────────────┘

For textbook RAG:
- Neon enables serverless deployment (no persistent disk needed)
- 0.5GB storage is enough for millions of query logs
- Auto-pause after inactivity (saves compute hours)
- PostgreSQL features (JSON, full-text search, analytics)
- Can deploy backend anywhere (not tied to database location)
"""
