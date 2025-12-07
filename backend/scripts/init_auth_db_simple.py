"""
Simple Authentication Database Initialization
Uses psycopg2 to execute the schema.sql file
"""

import sys
from pathlib import Path
import psycopg2
from psycopg2.extensions import ISOLATION_LEVEL_AUTOCOMMIT

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from app.config import settings


def init_auth_schema():
    """Initialize authentication database schema using psycopg2"""
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
        # Connect to Neon PostgreSQL using psycopg2
        print("[INFO] Connecting to Neon PostgreSQL...")

        conn = psycopg2.connect(settings.NEON_DATABASE_URL)
        conn.set_isolation_level(ISOLATION_LEVEL_AUTOCOMMIT)

        print("[OK] ✓ Connected to Neon PostgreSQL")

        # Execute schema SQL (psycopg2 can handle multi-statement SQL)
        print("[INFO] Executing schema SQL...")

        cursor = conn.cursor()
        cursor.execute(schema_sql)

        print("\n[OK] ✓ Authentication schema initialized successfully!")

        # Verify tables were created
        print("\n[INFO] Verifying tables...")
        cursor.execute("""
            SELECT table_name
            FROM information_schema.tables
            WHERE table_schema = 'public'
            AND table_name IN ('users', 'user_profiles', 'user_edits', 'user_annotations', 'user_progress', 'sessions')
            ORDER BY table_name;
        """)

        tables = cursor.fetchall()

        if tables:
            print(f"[OK] ✓ Created {len(tables)} tables:")
            for table in tables:
                print(f"  - {table[0]}")
        else:
            print("[WARNING] No auth tables found after initialization")

        cursor.close()
        conn.close()

        return True

    except Exception as e:
        print(f"[ERROR] Failed to initialize schema: {str(e)}")
        import traceback
        traceback.print_exc()
        return False


if __name__ == "__main__":
    success = init_auth_schema()
    sys.exit(0 if success else 1)
