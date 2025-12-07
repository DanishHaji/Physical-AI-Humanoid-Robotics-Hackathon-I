"""
Neon PostgreSQL Initialization Script (Cloud Free-Tier)
Creates query logging table for the textbook RAG system

Usage:
  python scripts/init_neon.py              # Initialize tables
  python scripts/init_neon.py --verify     # Verify tables
  python scripts/init_neon.py --reset      # Drop and recreate tables
"""

import sys
import asyncio
from pathlib import Path

# Add parent directory to path to import config
sys.path.insert(0, str(Path(__file__).parent.parent))

from app.config import settings, validate_cloud_credentials, get_missing_credentials
from app.clients.neon_client import NeonClient


async def create_tables(client: NeonClient):
    """Create all database tables"""

    print("[*] Creating Neon PostgreSQL tables...")

    try:
        # Initialize tables using the client's init_tables method
        await client.init_tables()

        print("\n[OK] Database initialization complete!")
        print(f"[INFO] Tables created:")
        print(f"  - queries (query logging and analytics)")
        print(f"  - Indexes: idx_queries_created_at, idx_queries_mode")

    except Exception as e:
        print(f"\n[ERROR] Error during database initialization: {str(e)}")
        raise


async def verify_tables(client: NeonClient):
    """Verify that all tables were created successfully"""

    print("\n[CHECK] Verifying tables...")

    try:
        # Get query stats to verify table exists and is accessible
        stats = await client.get_query_stats()

        if "error" in stats:
            print(f"  [ERROR] queries table: ERROR - {stats['error']}")
        else:
            print(f"  [OK] queries table: {stats['total_queries']} rows")
            print(f"     - Queries in last 24h: {stats['queries_24h']}")
            print(f"     - Avg response time: {stats['avg_response_time_ms']}ms")

            if stats['queries_by_mode']:
                print(f"     - Queries by mode:")
                for mode, count in stats['queries_by_mode'].items():
                    print(f"       {mode}: {count}")

        print("\n[OK] Verification complete!")

    except Exception as e:
        print(f"\n[ERROR] Error during verification: {str(e)}")


async def drop_all_tables(client: NeonClient):
    """
    Drop all tables (useful for resetting database)
    USE WITH CAUTION: This will delete all data!
    """

    print("[WARNING] Dropping all tables...")

    if not client.pool:
        await client.connect()

    try:
        async with client.pool.acquire() as conn:
            await conn.execute('DROP TABLE IF EXISTS queries CASCADE')

        print("[OK] All tables dropped")

    except Exception as e:
        print(f"[ERROR] Error dropping tables: {str(e)}")
        raise


async def add_sample_query(client: NeonClient):
    """Add a sample query for testing"""

    print("\n[*] Adding sample query...")

    try:
        from uuid import uuid4

        query_id = await client.log_query(
            question="What is ROS 2?",
            mode="explain",
            chunk_ids=["chunk-1", "chunk-2"],
            answer="ROS 2 is an open-source robotics middleware framework...",
            citations=[{
                "chapter": "ROS 2 Fundamentals",
                "section": "Introduction",
                "url": "/docs/module-01-ros2/week-02-ros2-fundamentals"
            }],
            response_time_ms=1234,
            user_ip_hash=None
        )

        print(f"[OK] Added sample query (ID: {query_id})")

    except Exception as e:
        print(f"[ERROR] Error adding sample query: {str(e)}")


async def main():
    """Main function"""

    print("=" * 60)
    print("Physical AI Textbook - Neon Database Initialization")
    print("=" * 60)
    print(f"\nCloud Stack: Neon PostgreSQL (Serverless)")
    print(f"Free tier: 0.5GB storage, 100 compute hours/month")
    print()

    # Validate cloud credentials
    print("[CHECK] Checking credentials...")
    cred_status = validate_cloud_credentials()

    if cred_status['neon'] != 'configured':
        print("\n[ERROR] Neon database URL not configured!")
        print("\nMissing credentials:")
        for cred in get_missing_credentials():
            print(f"  - {cred}")
        print("\nPlease update your .env file with Neon credentials.")
        print("See backend/.env.example for setup instructions.\n")
        return

    print("[OK] Neon credentials configured\n")

    # Initialize Neon client
    try:
        print("[CONNECT] Connecting to Neon PostgreSQL...")
        client = NeonClient()
        await client.connect()
        print("[OK] Connected to Neon\n")

    except Exception as e:
        print(f"\n[ERROR] Failed to connect to Neon: {str(e)}")
        print("\nTroubleshooting:")
        print("1. Check your NEON_DATABASE_URL in .env")
        print("2. Ensure the connection string is correct")
        print("3. Verify your Neon database is active (not paused)")
        print("4. Check network connectivity\n")
        return

    try:
        # Check for command line arguments
        if len(sys.argv) > 1:
            if sys.argv[1] == '--reset':
                print("[WARNING] RESETTING DATABASE - ALL DATA WILL BE LOST!")
                response = input("Type 'yes' to confirm: ")
                if response.lower() == 'yes':
                    await drop_all_tables(client)
                    await create_tables(client)
                    await verify_tables(client)
                else:
                    print("Reset cancelled.")
                    return

            elif sys.argv[1] == '--verify':
                await verify_tables(client)
                return

            elif sys.argv[1] == '--with-sample':
                await create_tables(client)
                await add_sample_query(client)
                await verify_tables(client)
            else:
                print(f"Unknown argument: {sys.argv[1]}")
                print("Usage: python scripts/init_neon.py [--verify|--reset|--with-sample]")
                return
        else:
            # Default: create tables and verify
            await create_tables(client)
            await verify_tables(client)

        print("\n" + "=" * 60)
        print("[OK] All done! Your Neon database is ready for use.")
        print("=" * 60)
        print("\nNext steps:")
        print("1. Ingest textbook chapters: python scripts/ingest_to_qdrant.py")
        print("2. Start the API server: uvicorn app.main:app --reload")
        print("3. Test the health endpoint: curl http://localhost:8000/health")
        print()
        print("Optional:")
        print("- Add sample data: python scripts/init_neon.py --with-sample")
        print("- Reset database: python scripts/init_neon.py --reset")
        print("- Verify tables: python scripts/init_neon.py --verify")
        print()

    finally:
        # Close connection
        await client.disconnect()


if __name__ == "__main__":
    # Run async main function
    asyncio.run(main())
