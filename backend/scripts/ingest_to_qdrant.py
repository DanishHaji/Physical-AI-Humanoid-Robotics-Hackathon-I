"""
Chapter Ingestion Script (Cloud Free-Tier Stack)
Ingest markdown chapters into Qdrant Cloud with sentence-transformers embeddings

This script:
1. Reads markdown files from docs/ directory
2. Chunks content by headings (## sections)
3. Generates embeddings using sentence-transformers (local, free)
4. Stores chunks + embeddings in Qdrant Cloud (free 1GB cluster)

Usage:
  python scripts/ingest_to_qdrant.py                    # Ingest all chapters
  python scripts/ingest_to_qdrant.py /path/to/docs      # Custom docs directory
"""

import sys
from pathlib import Path
import re
from uuid import uuid4
from typing import List, Dict

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from app.config import settings, validate_cloud_credentials, get_missing_credentials
from app.embeddings import generate_embeddings_batch
from app.retrieval import init_qdrant_client, upsert_chunks_to_qdrant


def read_markdown_file(file_path: Path) -> str:
    """Read markdown file content"""
    with open(file_path, 'r', encoding='utf-8') as f:
        return f.read()


def chunk_markdown_by_headings(content: str, chapter_title: str, module: int, week: int) -> List[Dict]:
    """
    Chunk markdown content by headings (##)

    Args:
        content: Markdown content
        chapter_title: Title of the chapter
        module: Module number (1-4)
        week: Week number (1-13)

    Returns:
        List of chunk dicts with content and metadata
    """
    chunks = []

    # Split by ## headings
    sections = re.split(r'\n## ', content)

    for i, section in enumerate(sections):
        if not section.strip():
            continue

        # Extract heading and content
        lines = section.split('\n', 1)
        heading = lines[0].strip('#').strip()

        if len(lines) > 1:
            section_content = lines[1].strip()
        else:
            section_content = ""

        # Skip very short sections
        if len(section_content) < 100:
            continue

        # Limit chunk size
        if len(section_content) > settings.RAG_CHUNK_SIZE:
            # Split into smaller chunks
            for j in range(0, len(section_content), settings.RAG_CHUNK_SIZE - settings.RAG_CHUNK_OVERLAP):
                chunk_content = section_content[j:j + settings.RAG_CHUNK_SIZE]

                if len(chunk_content) < 100:
                    continue

                chunks.append({
                    'id': str(uuid4()),
                    'content': chunk_content,
                    'metadata': {
                        'chapter_title': chapter_title,
                        'heading': heading,
                        'module': module,
                        'week': week,
                        'position': len(chunks),
                        'sub_chunk': j // (settings.RAG_CHUNK_SIZE - settings.RAG_CHUNK_OVERLAP)
                    }
                })
        else:
            chunks.append({
                'id': str(uuid4()),
                'content': section_content,
                'metadata': {
                    'chapter_title': chapter_title,
                    'heading': heading,
                    'module': module,
                    'week': week,
                    'position': len(chunks),
                    'sub_chunk': 0
                }
            })

    return chunks


def ingest_chapter(file_path: Path, module: int, week: int, chapter_title: str):
    """
    Ingest a single chapter file

    Args:
        file_path: Path to markdown file
        module: Module number
        week: Week number
        chapter_title: Chapter title
    """
    print(f"\n📖 Processing: {chapter_title} (Module {module}, Week {week})")

    try:
        # Read file
        content = read_markdown_file(file_path)
        print(f"  [OK] Read {len(content)} characters")

        # Chunk content
        chunks = chunk_markdown_by_headings(content, chapter_title, module, week)
        print(f"  [OK] Created {len(chunks)} chunks")

        if len(chunks) == 0:
            print(f"  [WARNING] No chunks created - skipping")
            return 0

        # Generate embeddings (batch processing for speed)
        print(f"  [*] Generating embeddings...")
        texts = [chunk['content'] for chunk in chunks]
        embeddings = generate_embeddings_batch(texts, batch_size=32)

        # Add embeddings to chunks
        for chunk, embedding in zip(chunks, embeddings):
            chunk['embedding'] = embedding

        print(f"  [OK] Generated {len(embeddings)} embeddings")

        # Upsert to Qdrant Cloud
        print(f"  [*] Upserting to Qdrant Cloud...")
        upsert_chunks_to_qdrant(chunks)

        print(f"[OK] Successfully ingested: {chapter_title} ({len(chunks)} chunks)")
        return len(chunks)

    except Exception as e:
        print(f"[ERROR] Error ingesting {file_path}: {str(e)}")
        raise


def ingest_all_chapters(docs_dir: Path = None):
    """
    Ingest all chapters from the docs directory

    Args:
        docs_dir: Path to docs directory (defaults to ../../docs)
    """
    if docs_dir is None:
        # Default to project docs directory
        docs_dir = Path(__file__).parent.parent.parent / "docs"

    print("=" * 60)
    print("Physical AI Textbook - Chapter Ingestion (Cloud Free-Tier)")
    print("=" * 60)
    print(f"\nDocs directory: {docs_dir}")
    print(f"Chunk size: {settings.RAG_CHUNK_SIZE}")
    print(f"Chunk overlap: {settings.RAG_CHUNK_OVERLAP}")
    print(f"Embedding model: {settings.EMBEDDING_MODEL}")
    print()

    # Validate cloud credentials
    print("[CHECK] Checking credentials...")
    cred_status = validate_cloud_credentials()

    missing_services = []
    if cred_status['qdrant_url'] != 'configured':
        missing_services.append('qdrant_url')
    if cred_status['qdrant_key'] != 'configured':
        missing_services.append('qdrant_key')

    if missing_services:
        print("\n[ERROR] Qdrant Cloud credentials not configured!")
        print("\nMissing credentials:")
        for cred in get_missing_credentials():
            if 'QDRANT' in cred:
                print(f"  - {cred}")
        print("\nPlease update your .env file with Qdrant credentials.")
        print("See backend/.env.example for setup instructions.\n")
        return

    print("[OK] Qdrant credentials configured\n")

    # Initialize Qdrant Cloud
    print("[*] Initializing Qdrant Cloud...")
    try:
        import asyncio
        asyncio.run(init_qdrant_client())
        print("[OK] Connected to Qdrant Cloud\n")
    except Exception as e:
        print(f"\n[ERROR] Failed to connect to Qdrant Cloud: {str(e)}")
        print("\nTroubleshooting:")
        print("1. Check your QDRANT_URL in .env")
        print("2. Check your QDRANT_API_KEY in .env")
        print("3. Verify your Qdrant cluster is active")
        print("4. Check network connectivity\n")
        return

    # Define chapters to ingest
    chapters = [
        {
            'file': 'module-01-ros2/week-01-physical-ai-intro.md',
            'module': 1,
            'week': 1,
            'title': 'Physical AI Introduction & Foundations'
        },
        {
            'file': 'module-01-ros2/week-02-ros2-fundamentals.md',
            'module': 1,
            'week': 2,
            'title': 'ROS 2 Fundamentals'
        },
        {
            'file': 'module-01-ros2/week-03-ros2-programming.md',
            'module': 1,
            'week': 3,
            'title': 'ROS 2 Programming'
        },
        {
            'file': 'module-02-digital-twin/week-04-urdf-modeling.md',
            'module': 2,
            'week': 4,
            'title': 'URDF Robot Modeling'
        },
        {
            'file': 'module-02-digital-twin/week-05-digital-twin-intro.md',
            'module': 2,
            'week': 5,
            'title': 'Digital Twin Introduction'
        },
        {
            'file': 'module-02-digital-twin/week-06-gazebo-unity.md',
            'module': 2,
            'week': 6,
            'title': 'Gazebo & Unity Simulation'
        },
        {
            'file': 'module-03-isaac-sim/week-07-isaac-overview.md',
            'module': 3,
            'week': 7,
            'title': 'NVIDIA Isaac Overview'
        },
        {
            'file': 'module-03-isaac-sim/week-08-isaac-sim.md',
            'module': 3,
            'week': 8,
            'title': 'NVIDIA Isaac Sim'
        },
        {
            'file': 'module-03-isaac-sim/week-09-isaac-lab.md',
            'module': 3,
            'week': 9,
            'title': 'NVIDIA Isaac Lab'
        },
        {
            'file': 'module-04-vla/week-10-vla-intro.md',
            'module': 4,
            'week': 10,
            'title': 'Vision-Language-Action Introduction'
        },
        {
            'file': 'module-04-vla/week-11-vla-systems.md',
            'module': 4,
            'week': 11,
            'title': 'VLA Systems Implementation'
        },
        {
            'file': 'module-04-vla/week-12-deployment.md',
            'module': 4,
            'week': 12,
            'title': 'Real-World Deployment'
        },
        {
            'file': 'capstone/week-13-capstone-project.md',
            'module': 4,
            'week': 13,
            'title': 'Capstone Integration Project'
        },
    ]

    # Ingest each chapter
    total_ingested = 0
    total_chunks = 0

    for chapter in chapters:
        file_path = docs_dir / chapter['file']

        if not file_path.exists():
            print(f"[WARNING] Skipping (not found): {file_path}")
            continue

        try:
            chunk_count = ingest_chapter(
                file_path=file_path,
                module=chapter['module'],
                week=chapter['week'],
                chapter_title=chapter['title']
            )
            total_ingested += 1
            total_chunks += chunk_count
        except Exception as e:
            print(f"[ERROR] Failed to ingest {chapter['title']}: {str(e)}")
            continue

    print("\n" + "=" * 60)
    print("[OK] Ingestion complete!")
    print("=" * 60)
    print(f"\n[INFO] Summary:")
    print(f"  Chapters processed: {total_ingested}/{len(chapters)}")
    print(f"  Total chunks created: {total_chunks}")
    print(f"  Embedding model: {settings.EMBEDDING_MODEL} ({settings.VECTOR_SIZE}d)")
    print(f"  Storage: Qdrant Cloud (1GB free cluster)")
    print(f"  Cost: FREE (no API calls)")

    print(f"\nNext steps:")
    print("1. Start API server: uvicorn app.main:app --reload")
    print("2. Test chatbot: POST http://localhost:8000/query")
    print("3. Check health: GET http://localhost:8000/health")
    print()


def main():
    """Main function"""

    # Check if custom docs directory provided
    docs_dir = None
    if len(sys.argv) > 1:
        docs_dir = Path(sys.argv[1])
        if not docs_dir.exists():
            print(f"[ERROR] Error: Directory not found: {docs_dir}")
            return

    ingest_all_chapters(docs_dir)


if __name__ == "__main__":
    main()
