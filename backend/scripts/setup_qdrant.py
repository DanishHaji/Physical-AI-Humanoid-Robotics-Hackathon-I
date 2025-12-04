"""
Qdrant Setup Script
Initialize Qdrant collection for textbook chunks
"""

import asyncio
from dotenv import load_dotenv
import sys
import os

# Add parent directory to path to import app modules
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from app.retrieval import init_qdrant_client, create_qdrant_collection
from app.config import settings

# Load environment variables
load_dotenv()


async def setup():
    """Setup Qdrant collection"""

    print("=" * 60)
    print("Physical AI Textbook - Qdrant Setup")
    print("=" * 60)
    print(f"\nQdrant URL: {settings.QDRANT_URL}")
    print(f"Collection: {settings.QDRANT_COLLECTION_NAME}")
    print(f"Vector Size: {settings.QDRANT_VECTOR_SIZE}")
    print(f"Distance: {settings.QDRANT_DISTANCE}")
    print()

    try:
        # Initialize Qdrant client
        print("🔧 Connecting to Qdrant Cloud...")
        await init_qdrant_client()

        # Create collection
        print("🔧 Creating collection...")
        await create_qdrant_collection()

        print("\n" + "=" * 60)
        print("✅ Qdrant setup complete!")
        print("=" * 60)
        print("\nNext steps:")
        print("1. Run document ingestion: python scripts/ingest_chapters.py")
        print("2. Start the API server: uvicorn app.main:app --reload")
        print()

    except Exception as e:
        print(f"\n❌ Error during Qdrant setup: {str(e)}")
        print("\nTroubleshooting:")
        print("1. Check that QDRANT_URL is correct in .env")
        print("2. Check that QDRANT_API_KEY is correct in .env")
        print("3. Verify your Qdrant Cloud account is active")
        print("4. Check internet connection")


if __name__ == "__main__":
    asyncio.run(setup())
