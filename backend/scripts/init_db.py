"""
Database Initialization Script for Neon PostgreSQL
Creates all necessary tables for the textbook RAG system
"""

import asyncio
import asyncpg
from dotenv import load_dotenv
import os

# Load environment variables
load_dotenv()

DATABASE_URL = os.getenv('DATABASE_URL')


async def create_tables():
    """Create all database tables"""

    conn = await asyncpg.connect(DATABASE_URL)

    try:
        print("🔧 Creating database tables...")

        # Enable UUID extension
        await conn.execute('CREATE EXTENSION IF NOT EXISTS "uuid-ossp";')
        print("✅ Enabled uuid-ossp extension")

        # Create chapters table
        await conn.execute('''
            CREATE TABLE IF NOT EXISTS chapters (
                id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
                title VARCHAR(200) NOT NULL,
                module INTEGER NOT NULL CHECK (module BETWEEN 1 AND 4),
                week INTEGER NOT NULL UNIQUE CHECK (week BETWEEN 1 AND 13),
                slug VARCHAR(100) NOT NULL UNIQUE,
                content TEXT NOT NULL CHECK (length(content) >= 2000),
                learning_objectives JSONB NOT NULL,
                code_examples JSONB,
                glossary_terms JSONB,
                status VARCHAR(20) NOT NULL DEFAULT 'draft' CHECK (status IN ('draft', 'review', 'published')),
                created_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
                updated_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
                author VARCHAR(100) DEFAULT 'Claude Code + Human Review',
                estimated_reading_time INTEGER GENERATED ALWAYS AS (
                    (length(content) / 5) / 200
                ) STORED
            );
        ''')
        print("✅ Created chapters table")

        # Create chunks table
        await conn.execute('''
            CREATE TABLE IF NOT EXISTS chunks (
                id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
                chapter_id UUID NOT NULL REFERENCES chapters(id) ON DELETE CASCADE,
                content TEXT NOT NULL CHECK (length(content) BETWEEN 100 AND 4096),
                metadata JSONB NOT NULL,
                heading VARCHAR(200) NOT NULL,
                position INTEGER NOT NULL CHECK (position >= 0),
                token_count INTEGER NOT NULL CHECK (token_count BETWEEN 512 AND 1024),
                created_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
                updated_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
                UNIQUE(chapter_id, position)
            );
        ''')
        print("✅ Created chunks table")

        # Create queries table
        await conn.execute('''
            CREATE TABLE IF NOT EXISTS queries (
                id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
                question_text VARCHAR(500) NOT NULL CHECK (length(question_text) BETWEEN 1 AND 500),
                selected_text TEXT,
                mode VARCHAR(20) NOT NULL CHECK (mode IN ('explain', 'code', 'urdu', 'exam')),
                retrieved_chunk_ids UUID[],
                answer_text TEXT NOT NULL,
                citations JSONB NOT NULL,
                response_time_ms INTEGER NOT NULL CHECK (response_time_ms > 0 AND response_time_ms < 10000),
                user_ip_hash CHAR(64),
                feedback VARCHAR(20) CHECK (feedback IN ('helpful', 'not_helpful', 'report_issue')),
                created_at TIMESTAMPTZ NOT NULL DEFAULT NOW()
            );
        ''')
        print("✅ Created queries table")

        # Create assessments table
        await conn.execute('''
            CREATE TABLE IF NOT EXISTS assessments (
                id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
                chapter_id UUID NOT NULL REFERENCES chapters(id) ON DELETE CASCADE,
                type VARCHAR(20) NOT NULL CHECK (type IN ('mcq', 'project', 'lab_exercise')),
                title VARCHAR(200) NOT NULL,
                problem_statement TEXT,
                requirements JSONB,
                starter_code TEXT,
                rubric JSONB,
                mcq_questions JSONB,
                point_value INTEGER NOT NULL CHECK (point_value > 0),
                estimated_time_minutes INTEGER,
                created_at TIMESTAMPTZ NOT NULL DEFAULT NOW()
            );
        ''')
        print("✅ Created assessments table")

        # Create diagrams table
        await conn.execute('''
            CREATE TABLE IF NOT EXISTS diagrams (
                id UUID PRIMARY KEY DEFAULT uuid_generate_v4(),
                chapter_id UUID NOT NULL REFERENCES chapters(id) ON DELETE CASCADE,
                type VARCHAR(20) NOT NULL CHECK (type IN ('architecture', 'workflow', 'concept', 'state_machine', 'sequence')),
                title VARCHAR(200) NOT NULL,
                mermaid_code TEXT NOT NULL,
                alt_text VARCHAR(500) NOT NULL,
                position INTEGER NOT NULL CHECK (position >= 0),
                created_at TIMESTAMPTZ NOT NULL DEFAULT NOW(),
                UNIQUE(chapter_id, position)
            );
        ''')
        print("✅ Created diagrams table")

        # Create indexes
        print("🔧 Creating indexes...")

        await conn.execute('CREATE INDEX IF NOT EXISTS idx_chapters_module ON chapters(module);')
        await conn.execute('CREATE INDEX IF NOT EXISTS idx_chapters_status ON chapters(status);')

        await conn.execute('CREATE INDEX IF NOT EXISTS idx_chunks_chapter_id ON chunks(chapter_id);')
        await conn.execute('CREATE INDEX IF NOT EXISTS idx_chunks_metadata_tags ON chunks USING GIN (metadata);')

        await conn.execute('CREATE INDEX IF NOT EXISTS idx_queries_created_at ON queries(created_at DESC);')
        await conn.execute('CREATE INDEX IF NOT EXISTS idx_queries_mode ON queries(mode);')
        await conn.execute('CREATE INDEX IF NOT EXISTS idx_queries_feedback ON queries(feedback) WHERE feedback IS NOT NULL;')

        await conn.execute('CREATE INDEX IF NOT EXISTS idx_assessments_chapter_id ON assessments(chapter_id);')
        await conn.execute('CREATE INDEX IF NOT EXISTS idx_assessments_type ON assessments(type);')

        await conn.execute('CREATE INDEX IF NOT EXISTS idx_diagrams_chapter_id ON diagrams(chapter_id);')

        print("✅ Created all indexes")

        # Create updated_at trigger for chapters and chunks
        await conn.execute('''
            CREATE OR REPLACE FUNCTION update_updated_at_column()
            RETURNS TRIGGER AS $$
            BEGIN
                NEW.updated_at = NOW();
                RETURN NEW;
            END;
            $$ language 'plpgsql';
        ''')

        await conn.execute('''
            CREATE TRIGGER update_chapters_updated_at BEFORE UPDATE ON chapters
            FOR EACH ROW EXECUTE FUNCTION update_updated_at_column();
        ''')

        await conn.execute('''
            CREATE TRIGGER update_chunks_updated_at BEFORE UPDATE ON chunks
            FOR EACH ROW EXECUTE FUNCTION update_updated_at_column();
        ''')

        print("✅ Created triggers for updated_at timestamps")

        print("\n✅ Database initialization complete!")
        print(f"📊 Tables created: chapters, chunks, queries, assessments, diagrams")
        print(f"📊 Indexes created: 11 indexes for optimized queries")

    except Exception as e:
        print(f"\n❌ Error during database initialization: {str(e)}")
        raise

    finally:
        await conn.close()


async def verify_tables():
    """Verify that all tables were created successfully"""

    conn = await asyncpg.connect(DATABASE_URL)

    try:
        print("\n🔍 Verifying tables...")

        # Get list of tables
        tables = await conn.fetch('''
            SELECT table_name
            FROM information_schema.tables
            WHERE table_schema = 'public'
            AND table_type = 'BASE TABLE'
            ORDER BY table_name;
        ''')

        table_names = [row['table_name'] for row in tables]

        expected_tables = ['chapters', 'chunks', 'queries', 'assessments', 'diagrams']

        for table in expected_tables:
            if table in table_names:
                # Get row count
                count = await conn.fetchval(f'SELECT COUNT(*) FROM {table}')
                print(f"  ✅ {table}: {count} rows")
            else:
                print(f"  ❌ {table}: NOT FOUND")

        print("\n✅ Verification complete!")

    except Exception as e:
        print(f"\n❌ Error during verification: {str(e)}")

    finally:
        await conn.close()


async def main():
    """Main function"""
    if not DATABASE_URL:
        print("❌ Error: DATABASE_URL environment variable not set")
        print("Please create a .env file with your Neon connection string")
        return

    print("=" * 60)
    print("Physical AI Textbook - Database Initialization")
    print("=" * 60)
    print(f"\nConnecting to: {DATABASE_URL.split('@')[1] if '@' in DATABASE_URL else 'database'}")
    print()

    await create_tables()
    await verify_tables()

    print("\n" + "=" * 60)
    print("✅ All done! Your database is ready for use.")
    print("=" * 60)
    print("\nNext steps:")
    print("1. Run document ingestion: python scripts/chunk_chapters.py")
    print("2. Start the API server: uvicorn app.main:app --reload")
    print()


if __name__ == "__main__":
    asyncio.run(main())
