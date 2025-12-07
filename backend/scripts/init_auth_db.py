"""
Initialize Authentication Database Schema
Runs the schema.sql file to create all auth-related tables
"""

import asyncio
import sys
from pathlib import Path

# Add parent directory to path to import app modules
sys.path.insert(0, str(Path(__file__).parent.parent))

from app.clients.neon_client import get_neon_client


async def init_auth_schema():
    """
    Initialize authentication database schema
    Creates all tables for users, profiles, edits, annotations, progress
    """
    print("[INFO] Initializing authentication database schema...")

    # Read schema.sql file
    schema_file = Path(__file__).parent.parent / "app" / "database" / "schema.sql"

    if not schema_file.exists():
        print(f"[ERROR] Schema file not found: {schema_file}")
        return False

    print(f"[INFO] Reading schema from: {schema_file}")

    with open(schema_file, 'r', encoding='utf-8') as f:
        schema_sql = f.read()

    try:
        # Get Neon client
        print("[INFO] Connecting to Neon PostgreSQL...")
        neon_client = await get_neon_client()

        # Execute schema SQL statement by statement
        print("[INFO] Executing schema SQL...")

        # Split SQL by semicolons and execute each statement
        statements = [stmt.strip() for stmt in schema_sql.split(';') if stmt.strip()]

        async with neon_client.pool.acquire() as conn:
            for i, statement in enumerate(statements, 1):
                if statement:
                    try:
                        await conn.execute(statement)
                        print(f"[OK] ✓ Executed statement {i}/{len(statements)}")
                    except Exception as e:
                        # Ignore "already exists" errors
                        if "already exists" in str(e).lower():
                            print(f"[SKIP] Statement {i} - object already exists")
                        else:
                            raise

        print("\n[OK] ✓ Authentication schema initialized successfully!")

        # Verify tables were created
        print("\n[INFO] Verifying tables...")
        tables = await neon_client.execute_query(
            """
            SELECT table_name
            FROM information_schema.tables
            WHERE table_schema = 'public'
            AND table_name IN ('users', 'user_profiles', 'user_edits', 'user_annotations', 'user_progress', 'sessions')
            ORDER BY table_name;
            """
        )

        if tables:
            print(f"[OK] ✓ Created {len(tables)} tables:")
            for table in tables:
                print(f"  - {table['table_name']}")
        else:
            print("[WARNING] No auth tables found after initialization")

        # Disconnect
        await neon_client.disconnect()

        return True

    except Exception as e:
        print(f"[ERROR] Failed to initialize schema: {str(e)}")
        import traceback
        traceback.print_exc()
        return False


if __name__ == "__main__":
    success = asyncio.run(init_auth_schema())
    sys.exit(0 if success else 1)
