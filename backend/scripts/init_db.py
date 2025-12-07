"""
Database Initialization Script for SQLite (Free Stack)
Creates query logging table for the textbook RAG system

Note: ChromaDB handles chunk storage and metadata.
SQLite is only used for query logging and analytics.
"""

import sqlite3
from pathlib import Path
import sys
import os

# Add parent directory to path to import config
sys.path.insert(0, str(Path(__file__).parent.parent))

from app.config import settings


def create_tables(db_path: str = None):
    """Create all database tables"""

    if db_path is None:
        db_path = settings.DATABASE_PATH

    # Ensure directory exists
    db_file = Path(db_path)
    db_file.parent.mkdir(parents=True, exist_ok=True)

    print(f"🔧 Creating SQLite database at: {db_path}")

    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()

    try:
        print("🔧 Creating database tables...")

        # Create queries table (for query logging and analytics)
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS queries (
                id TEXT PRIMARY KEY,
                question_text TEXT NOT NULL CHECK (length(question_text) BETWEEN 1 AND 500),
                selected_text TEXT,
                mode TEXT NOT NULL CHECK (mode IN ('explain', 'code', 'urdu', 'exam')),
                retrieved_chunk_ids TEXT NOT NULL,
                answer_text TEXT NOT NULL,
                citations TEXT NOT NULL,
                response_time_ms INTEGER NOT NULL CHECK (response_time_ms > 0 AND response_time_ms < 10000),
                user_ip_hash TEXT,
                feedback TEXT CHECK (feedback IN ('helpful', 'not_helpful', 'report_issue')),
                created_at TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP
            )
        ''')
        print("✅ Created queries table")

        # Create indexes
        print("🔧 Creating indexes...")

        cursor.execute('CREATE INDEX IF NOT EXISTS idx_queries_created_at ON queries(created_at DESC)')
        cursor.execute('CREATE INDEX IF NOT EXISTS idx_queries_mode ON queries(mode)')
        cursor.execute('CREATE INDEX IF NOT EXISTS idx_queries_feedback ON queries(feedback)')

        print("✅ Created all indexes")

        conn.commit()

        print("\n✅ Database initialization complete!")
        print(f"📊 Table created: queries (for query logging)")
        print(f"📊 Indexes created: 3 indexes for optimized queries")
        print(f"\nNote: Chunk storage is handled by ChromaDB (not SQLite)")

    except Exception as e:
        print(f"\n❌ Error during database initialization: {str(e)}")
        conn.rollback()
        raise

    finally:
        conn.close()


def verify_tables(db_path: str = None):
    """Verify that all tables were created successfully"""

    if db_path is None:
        db_path = settings.DATABASE_PATH

    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()

    try:
        print("\n🔍 Verifying tables...")

        # Get list of tables
        cursor.execute('''
            SELECT name
            FROM sqlite_master
            WHERE type = 'table'
            AND name NOT LIKE 'sqlite_%'
            ORDER BY name
        ''')

        tables = [row[0] for row in cursor.fetchall()]

        expected_tables = ['queries']

        for table in expected_tables:
            if table in tables:
                # Get row count
                cursor.execute(f'SELECT COUNT(*) FROM {table}')
                count = cursor.fetchone()[0]
                print(f"  ✅ {table}: {count} rows")
            else:
                print(f"  ❌ {table}: NOT FOUND")

        # Get database size
        db_size = Path(db_path).stat().st_size
        db_size_mb = db_size / (1024 * 1024)
        print(f"\n📊 Database size: {db_size_mb:.2f} MB")

        print("\n✅ Verification complete!")

    except Exception as e:
        print(f"\n❌ Error during verification: {str(e)}")

    finally:
        conn.close()


def drop_all_tables(db_path: str = None):
    """
    Drop all tables (useful for resetting database)
    USE WITH CAUTION: This will delete all data!
    """

    if db_path is None:
        db_path = settings.DATABASE_PATH

    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()

    try:
        print("⚠️ Dropping all tables...")

        cursor.execute('DROP TABLE IF EXISTS queries')

        conn.commit()
        print("✅ All tables dropped")

    except Exception as e:
        print(f"❌ Error dropping tables: {str(e)}")
        conn.rollback()

    finally:
        conn.close()


def add_sample_query(db_path: str = None):
    """Add a sample query for testing"""

    if db_path is None:
        db_path = settings.DATABASE_PATH

    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()

    try:
        import json
        from uuid import uuid4
        from datetime import datetime

        query_id = str(uuid4())
        cursor.execute('''
            INSERT INTO queries
            (id, question_text, mode, retrieved_chunk_ids, answer_text,
             citations, response_time_ms, created_at)
            VALUES (?, ?, ?, ?, ?, ?, ?, ?)
        ''', (
            query_id,
            "What is ROS 2?",
            "explain",
            json.dumps(["chunk-1", "chunk-2"]),
            "ROS 2 is an open-source robotics middleware framework...",
            json.dumps([{
                "chapter": "ROS 2 Fundamentals",
                "section": "Introduction",
                "url": "/docs/module-01-ros2/week-02-ros2-fundamentals"
            }]),
            1234,
            datetime.utcnow().isoformat()
        ))

        conn.commit()
        print(f"✅ Added sample query (ID: {query_id})")

    except Exception as e:
        print(f"❌ Error adding sample query: {str(e)}")
        conn.rollback()

    finally:
        conn.close()


def main():
    """Main function"""

    print("=" * 60)
    print("Physical AI Textbook - Database Initialization (Free Stack)")
    print("=" * 60)
    print(f"\nDatabase: SQLite (local)")
    print(f"Path: {settings.DATABASE_PATH}")
    print(f"Vector storage: ChromaDB (separate)")
    print()

    # Create tables
    create_tables()

    # Verify tables
    verify_tables()

    # Optionally add sample query
    if len(sys.argv) > 1 and sys.argv[1] == '--with-sample':
        add_sample_query()
        verify_tables()

    print("\n" + "=" * 60)
    print("✅ All done! Your database is ready for use.")
    print("=" * 60)
    print("\nNext steps:")
    print("1. Initialize ChromaDB: python scripts/setup_chroma.py")
    print("2. Ingest chapters: python scripts/ingest_chapters.py")
    print("3. Start the API server: uvicorn app.main:app --reload")
    print()
    print("Optional:")
    print("- Add sample data: python scripts/init_db.py --with-sample")
    print("- Reset database: python scripts/init_db.py --reset")
    print()


def reset_database():
    """Reset database by dropping and recreating all tables"""
    print("⚠️ RESETTING DATABASE - ALL DATA WILL BE LOST!")
    input("Press Enter to confirm, or Ctrl+C to cancel...")

    drop_all_tables()
    create_tables()
    verify_tables()

    print("\n✅ Database reset complete!")


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == '--reset':
        reset_database()
    else:
        main()
