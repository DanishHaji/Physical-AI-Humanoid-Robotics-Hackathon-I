"""
ChromaDB Setup Script (Free Stack)
Initialize ChromaDB collection for textbook RAG system

Note: ChromaDB stores data locally in SQLite automatically.
No cloud setup or API keys required.
"""

import sys
from pathlib import Path

# Add parent directory to path to import modules
sys.path.insert(0, str(Path(__file__).parent.parent))

from app.config import settings
from app.retrieval import init_chroma_client, get_or_create_collection, get_collection_stats


def setup_chroma():
    """Initialize ChromaDB and create collection"""

    print("=" * 60)
    print("Physical AI Textbook - ChromaDB Setup (Free Stack)")
    print("=" * 60)
    print(f"\nPersistence directory: {settings.CHROMA_PERSIST_DIR}")
    print(f"Collection name: {settings.CHROMA_COLLECTION_NAME}")
    print(f"Vector dimension: {settings.VECTOR_SIZE}")
    print(f"Distance metric: {settings.CHROMA_DISTANCE}")
    print()

    try:
        # Initialize ChromaDB client
        print("🔧 Initializing ChromaDB client...")
        client = init_chroma_client()

        # Create or get collection
        print("🔧 Creating/verifying collection...")
        collection = get_or_create_collection()

        # Get collection stats
        stats = get_collection_stats()

        print("\n" + "=" * 60)
        print("✅ ChromaDB setup complete!")
        print("=" * 60)
        print(f"\n📊 Collection Stats:")
        print(f"  - Total chunks: {stats.get('total_chunks', 0)}")
        print(f"  - Modules: {stats.get('modules_count', 0)}")
        print(f"  - Chapters: {stats.get('chapters_count', 0)}")
        print(f"  - Vector dimension: {stats.get('vector_dimension', 'N/A')}")
        print(f"  - Distance metric: {stats.get('distance_metric', 'N/A')}")

        print(f"\nNext steps:")
        print("1. Initialize SQLite: python scripts/init_db.py")
        print("2. Ingest chapters: python scripts/ingest_chapters.py")
        print("3. Start API server: uvicorn app.main:app --reload")
        print()

        return True

    except Exception as e:
        print(f"\n❌ Error during ChromaDB setup: {str(e)}")
        return False


def verify_chroma():
    """Verify ChromaDB setup and show detailed info"""

    print("\n🔍 Verifying ChromaDB setup...")

    try:
        # Initialize client
        client = init_chroma_client()

        # List all collections
        collections = client.list_collections()

        print(f"\n✅ Found {len(collections)} collection(s):")
        for col in collections:
            print(f"  - {col.name}: {col.count()} chunks")

        # Get detailed stats for main collection
        if any(col.name == settings.CHROMA_COLLECTION_NAME for col in collections):
            stats = get_collection_stats()

            print(f"\n📊 Detailed stats for '{settings.CHROMA_COLLECTION_NAME}':")
            print(f"  Total chunks: {stats.get('total_chunks', 0)}")
            print(f"  Modules: {stats.get('modules_count', 0)}")
            print(f"  Chapters: {stats.get('chapters_count', 0)}")

        # Check disk usage
        chroma_path = Path(settings.CHROMA_PERSIST_DIR)
        if chroma_path.exists():
            # Calculate directory size
            total_size = sum(f.stat().st_size for f in chroma_path.rglob('*') if f.is_file())
            total_size_mb = total_size / (1024 * 1024)

            print(f"\n💾 Disk usage:")
            print(f"  Path: {chroma_path}")
            print(f"  Size: {total_size_mb:.2f} MB")

        print("\n✅ Verification complete!")
        return True

    except Exception as e:
        print(f"\n❌ Verification failed: {str(e)}")
        return False


def reset_chroma():
    """
    Reset ChromaDB by deleting all collections
    USE WITH CAUTION: This will delete all data!
    """

    print("⚠️ WARNING: This will delete all ChromaDB data!")
    confirmation = input("Type 'DELETE' to confirm: ")

    if confirmation != "DELETE":
        print("❌ Reset cancelled")
        return

    try:
        # Initialize client
        client = init_chroma_client()

        # Delete all collections
        collections = client.list_collections()

        for col in collections:
            print(f"🗑️ Deleting collection: {col.name}")
            client.delete_collection(col.name)

        print("\n✅ All collections deleted")
        print("Run 'python scripts/setup_chroma.py' to recreate")

    except Exception as e:
        print(f"\n❌ Error during reset: {str(e)}")


def main():
    """Main function"""

    if len(sys.argv) > 1:
        if sys.argv[1] == '--verify':
            verify_chroma()
        elif sys.argv[1] == '--reset':
            reset_chroma()
        else:
            print("Usage:")
            print("  python scripts/setup_chroma.py           # Setup ChromaDB")
            print("  python scripts/setup_chroma.py --verify  # Verify setup")
            print("  python scripts/setup_chroma.py --reset   # Reset (delete all data)")
    else:
        setup_chroma()


if __name__ == "__main__":
    main()
