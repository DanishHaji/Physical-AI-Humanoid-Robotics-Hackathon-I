# Migration to Groq-Only Services

This document outlines the changes made to migrate the Physical AI Textbook RAG system from a mixed API approach (Groq + OpenAI) to a Groq-only approach that eliminates external API dependencies for embeddings.

## Problem Statement

The original system was configured to use:
- Groq API for LLM responses
- OpenAI API for embeddings
- Qdrant Cloud for vector storage
- Neon PostgreSQL for database

The issue was that Groq does not provide embedding models, so the system was trying to use OpenAI's `text-embedding-3-small` model, which caused errors in deployment.

## Solution

The system was modified to use local sentence-transformers for embeddings, eliminating the need for OpenAI API while maintaining the Groq LLM functionality.

## Changes Made

### 1. Configuration Changes (`app/config.py`)

- **Removed**: `OPENAI_API_KEY` configuration
- **Updated**: Embedding configuration to use local model
  - `EMBEDDING_MODEL`: Changed from `"text-embedding-3-small"` to `"all-MiniLM-L6-v2"`
  - `EMBEDDING_DEVICE`: Changed from `"api"` to `"cpu"` (with GPU support option)
  - `VECTOR_SIZE`: Changed from `1536` to `384` (to match sentence-transformers)
- **Updated**: Credential validation to remove OpenAI API check
- **Updated**: Status messages to reflect local embeddings

### 2. Embeddings Implementation (`app/embeddings.py`)

- **Replaced**: `OpenAIEmbeddingsClient` with `LocalEmbeddingsClient`
- **Updated**: Implementation to use `sentence_transformers.SentenceTransformer`
- **Added**: Local model loading with device detection (CPU/GPU)
- **Updated**: Token counting to use local approximation
- **Updated**: All functions to use local embeddings instead of API calls

### 3. Dependencies (`requirements.txt`)

- **Added**: `sentence-transformers>=2.6.0`
- **Added**: `torch>=2.0.0`
- **Added**: `transformers>=4.21.0`
- **Added**: `numpy>=1.21.0`
- **Removed**: OpenAI API dependency for embeddings
- **Updated**: Documentation to reflect local embeddings approach

### 4. Deployment Configuration (`render.yaml`)

- **No changes needed**: OpenAI API key was not required in original deployment config

## Benefits of the New Approach

1. **Single API Dependency**: Only Groq API is required for LLM responses
2. **No Embedding API Costs**: Local embeddings eliminate per-token costs
3. **Faster Embedding Generation**: Local models are typically faster than API calls
4. **Offline Capability**: Embeddings work without internet after initial download
5. **Consistent Vector Dimensions**: 384-dim vectors from sentence-transformers

## Model Comparison

| Aspect | Previous (OpenAI) | New (Local) |
|--------|------------------|-------------|
| Model | text-embedding-3-small | all-MiniLM-L6-v2 |
| Dimensions | 1,536 | 384 |
| Cost | $0.00002 per 1K tokens | Free after download |
| Latency | ~100-300ms (API) | ~10-50ms (local) |
| Dependencies | External API | Local model |

## Deployment Requirements

The system now requires:
- Groq API key for LLM responses
- Qdrant Cloud credentials for vector storage
- Neon PostgreSQL credentials for database
- No OpenAI API key (embeddings are local)

## Performance Considerations

- **Initial Load**: First startup will download the sentence-transformers model (~100MB)
- **Memory Usage**: Local model uses ~100-200MB RAM after loading
- **Vector Dimension**: Reduced from 1536 to 384 dimensions (smaller storage, faster search)
- **Similarity Threshold**: May need adjustment due to different embedding characteristics

## Testing the New System

```bash
# Test local embeddings
python test_local_embeddings.py

# Verify configuration
python -c "from app.config import settings; print(f'Embeddings: {settings.EMBEDDING_MODEL}')"

# Start the server
uvicorn app.main:app --reload
```

## Rollback Plan

If issues arise, revert to the previous version by:
1. Restoring the original `app/embeddings.py`
2. Reverting changes to `app/config.py`
3. Updating `requirements.txt` to remove sentence-transformers
4. Adding back OpenAI API dependencies

## Next Steps

1. Monitor embedding quality and similarity search performance
2. Fine-tune similarity thresholds if needed
3. Consider model optimization for production use
4. Update documentation to reflect the new architecture