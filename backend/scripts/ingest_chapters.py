"""
Document Ingestion Pipeline
Process markdown chapters, chunk, embed, and upload to Qdrant + Neon
"""

import asyncio
from pathlib import Path
import sys
import os
from uuid import uuid4
import json
from dotenv import load_dotenv

# Add parent directory to path
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from app.chunking import chunk_markdown_file, validate_chunks
from app.embeddings import generate_embeddings_batch, estimate_batch_cost
from app.retrieval import (
    init_qdrant_client,
    init_neon_pool,
    upsert_chunks_to_qdrant,
    close_connections
)
from app.config import settings
import asyncpg

# Load environment variables
load_dotenv()


async def ingest_chapter(
    chapter_path: Path,
    chapter_metadata: dict,
    neon_pool: asyncpg.Pool
) -> dict:
    """
    Ingest a single chapter: chunk → embed → upload

    Args:
        chapter_path: Path to markdown file
        chapter_metadata: Dict with chapter_id, title, module, week, slug
        neon_pool: Neon PostgreSQL connection pool

    Returns:
        Dict with ingestion statistics
    """
    print(f"\n📄 Processing: {chapter_path.name}")
    print(f"   Chapter: {chapter_metadata['title']} (Week {chapter_metadata['week']})")

    # Step 1: Chunk the chapter
    print("   🔧 Chunking...")
    chunks = chunk_markdown_file(
        str(chapter_path),
        min_tokens=settings.RAG_CHUNK_SIZE // 2,
        max_tokens=settings.RAG_CHUNK_SIZE,
        overlap_tokens=settings.RAG_CHUNK_OVERLAP
    )

    print(f"   ✅ Created {len(chunks)} chunks")

    # Validate chunks
    validation = validate_chunks(
        chunks,
        min_tokens=settings.RAG_CHUNK_SIZE // 2,
        max_tokens=settings.RAG_CHUNK_SIZE
    )
    print(f"   📊 Valid: {validation['valid_chunks']}/{validation['total_chunks']} ({validation['valid_percentage']}%)")

    # Step 2: Generate embeddings
    print("   🔧 Generating embeddings...")
    texts = [chunk.content for chunk in chunks]

    # Estimate cost
    cost_estimate = estimate_batch_cost(texts)
    print(f"   💰 Est. cost: ${cost_estimate['cost_usd']:.6f} ({cost_estimate['total_tokens']} tokens)")

    embeddings = await generate_embeddings_batch(texts, batch_size=100)
    print(f"   ✅ Generated {len(embeddings)} embeddings")

    # Step 3: Store in Neon PostgreSQL
    print("   🔧 Storing chunk metadata in Neon...")

    async with neon_pool.acquire() as conn:
        chunk_records = []

        for i, chunk in enumerate(chunks):
            chunk_id = uuid4()

            # Prepare metadata for Qdrant
            qdrant_metadata = {
                'chunk_id': str(chunk_id),
                'chapter_id': str(chapter_metadata['chapter_id']),
                'chapter_title': chapter_metadata['title'],
                'module': chapter_metadata['module'],
                'week': chapter_metadata['week'],
                'heading': chunk.heading,
                'position': chunk.position,
                'content_type': chunk.metadata.get('content_type', 'theory'),
                'concept_tags': chunk.metadata.get('concept_tags', []),
                'token_count': chunk.token_count
            }

            # Insert into Neon
            await conn.execute(
                """
                INSERT INTO chunks
                (id, chapter_id, content, metadata, heading, position, token_count)
                VALUES ($1, $2, $3, $4, $5, $6, $7)
                """,
                chunk_id,
                chapter_metadata['chapter_id'],
                chunk.content,
                json.dumps(chunk.metadata),
                chunk.heading,
                chunk.position,
                chunk.token_count
            )

            chunk_records.append({
                'id': chunk_id,
                'embedding': embeddings[i],
                'metadata': qdrant_metadata
            })

        print(f"   ✅ Stored {len(chunk_records)} chunks in Neon")

    # Step 4: Upload to Qdrant
    print("   🔧 Uploading to Qdrant...")
    await upsert_chunks_to_qdrant(chunk_records)
    print(f"   ✅ Uploaded {len(chunk_records)} vectors to Qdrant")

    return {
        'chapter': chapter_metadata['title'],
        'chunks': len(chunks),
        'valid_percentage': validation['valid_percentage'],
        'total_tokens': cost_estimate['total_tokens'],
        'cost_usd': cost_estimate['cost_usd']
    }


async def create_chapter_in_neon(
    neon_pool: asyncpg.Pool,
    title: str,
    module: int,
    week: int,
    slug: str,
    content: str
) -> str:
    """
    Create chapter entry in Neon PostgreSQL

    Args:
        neon_pool: Connection pool
        title: Chapter title
        module: Module number (1-4)
        week: Week number (1-13)
        slug: URL slug
        content: Full markdown content

    Returns:
        UUID of created chapter
    """
    async with neon_pool.acquire() as conn:
        chapter_id = uuid4()

        await conn.execute(
            """
            INSERT INTO chapters
            (id, title, module, week, slug, content, learning_objectives, status)
            VALUES ($1, $2, $3, $4, $5, $6, $7, $8)
            """,
            chapter_id,
            title,
            module,
            week,
            slug,
            content,
            json.dumps(["Learning objective 1", "Learning objective 2"]),  # TODO: Extract from chapter
            'published'
        )

        return chapter_id


async def ingest_all_chapters(chapters_dir: Path):
    """
    Ingest all chapters from a directory

    Args:
        chapters_dir: Path to docs/ directory with chapter markdown files
    """
    print("=" * 60)
    print("Physical AI Textbook - Document Ingestion")
    print("=" * 60)
    print()

    # Initialize connections
    print("🔧 Initializing connections...")
    await init_qdrant_client()
    neon_pool = await init_neon_pool()
    print()

    # Find all markdown files
    print("🔍 Scanning for chapter files...")
    markdown_files = list(chapters_dir.rglob("*.md"))
    markdown_files = [f for f in markdown_files if not f.name.startswith('_')]  # Skip hidden files

    print(f"   Found {len(markdown_files)} markdown files")
    print()

    # Process each chapter
    stats = []

    for md_file in markdown_files:
        # Extract metadata from path
        # Expected structure: docs/module-XX-name/week-YY-chapter.md
        parts = md_file.parts
        module_dir = next((p for p in parts if p.startswith('module-')), None)
        week_file = md_file.stem

        if not module_dir:
            print(f"⚠️  Skipping {md_file.name} (no module directory)")
            continue

        # Parse module and week
        try:
            module = int(module_dir.split('-')[1])
            week = int(week_file.split('-')[1]) if 'week-' in week_file else 0

            if week == 0:  # Skip intro and other non-week files
                print(f"ℹ️  Skipping {md_file.name} (not a weekly chapter)")
                continue

        except (ValueError, IndexError):
            print(f"⚠️  Skipping {md_file.name} (invalid naming)")
            continue

        # Read content
        with open(md_file, 'r', encoding='utf-8') as f:
            content = f.read()

        # Create title from filename
        title = ' '.join(week_file.split('-')[2:]).title().replace('_', ' ')
        slug = week_file

        # Create chapter in Neon
        chapter_id = await create_chapter_in_neon(
            neon_pool,
            title=title,
            module=module,
            week=week,
            slug=slug,
            content=content
        )

        # Prepare metadata
        chapter_metadata = {
            'chapter_id': chapter_id,
            'title': title,
            'module': module,
            'week': week,
            'slug': slug
        }

        # Ingest chapter
        try:
            result = await ingest_chapter(md_file, chapter_metadata, neon_pool)
            stats.append(result)
        except Exception as e:
            print(f"   ❌ Error ingesting chapter: {str(e)}")
            continue

    # Close connections
    print("\n🔧 Closing connections...")
    await close_connections()

    # Print summary
    print("\n" + "=" * 60)
    print("Ingestion Summary")
    print("=" * 60)

    if stats:
        total_chunks = sum(s['chunks'] for s in stats)
        total_cost = sum(s['cost_usd'] for s in stats)
        avg_valid = sum(s['valid_percentage'] for s in stats) / len(stats)

        print(f"\n📊 Chapters processed: {len(stats)}")
        print(f"📊 Total chunks: {total_chunks}")
        print(f"📊 Average valid chunks: {avg_valid:.1f}%")
        print(f"💰 Total cost: ${total_cost:.6f}")

        print("\nPer-chapter breakdown:")
        for stat in stats:
            print(f"  • {stat['chapter']}: {stat['chunks']} chunks (${stat['cost_usd']:.6f})")

        print("\n✅ All chapters ingested successfully!")
    else:
        print("\n⚠️  No chapters were processed")

    print("=" * 60)
    print("\nNext steps:")
    print("1. Start the API server: uvicorn app.main:app --reload")
    print("2. Test the chatbot: POST /query")
    print()


async def main():
    """Main function"""
    # Get chapters directory
    current_dir = Path(__file__).parent
    project_root = current_dir.parent.parent
    docs_dir = project_root / "docs"

    if not docs_dir.exists():
        print(f"❌ Error: docs/ directory not found at {docs_dir}")
        print("Please run this script from the backend/ directory")
        return

    await ingest_all_chapters(docs_dir)


if __name__ == "__main__":
    asyncio.run(main())
