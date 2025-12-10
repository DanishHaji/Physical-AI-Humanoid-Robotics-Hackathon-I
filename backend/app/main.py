"""
Physical AI Textbook RAG API - Main FastAPI Application
Provides retrieval-augmented generation endpoints for the textbook chatbot
"""

from fastapi import FastAPI, HTTPException, Request, Depends
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
from slowapi import Limiter, _rate_limit_exceeded_handler
from slowapi.util import get_remote_address
from slowapi.errors import RateLimitExceeded
import time
from datetime import datetime

from .config import settings, get_settings
from .models import (
    QueryRequest,
    QueryResponse,
    HealthResponse,
    ErrorResponse,
    Citation
)
from .embeddings import generate_embedding, test_embedding_connection
from .retrieval import (
    init_qdrant_client,
    init_neon_pool,
    close_connections,
    search_qdrant,
    get_chunks_batch_from_qdrant,
    log_query_to_neon,
    extract_citations,
    test_qdrant_connection,
    test_neon_connection
)
from .answer_generator import generate_answer, generate_no_context_response
from .auth.router import router as auth_router

# Initialize FastAPI app
app = FastAPI(
    title=settings.APP_NAME,
    version=settings.APP_VERSION,
    description="RAG API for AI-Native Physical AI & Humanoid Robotics Textbook",
    docs_url="/docs",
    redoc_url="/redoc"
)

# Include authentication router
app.include_router(auth_router)

# Initialize rate limiter
limiter = Limiter(key_func=get_remote_address)
app.state.limiter = limiter
app.add_exception_handler(RateLimitExceeded, _rate_limit_exceeded_handler)

# CORS middleware - allow origins from environment or default to development
# This allows both development and production origins
allowed_origins = settings.all_allowed_origins

app.add_middleware(
    CORSMiddleware,
    allow_origins=allowed_origins,
    allow_credentials=True,
    allow_methods=["GET", "POST", "PUT", "DELETE", "OPTIONS"],
    allow_headers=["*"],
)


# Startup event
@app.on_event("startup")
async def startup_event():
    """Initialize database connections and services"""
    print(f"[INFO] Starting {settings.APP_NAME} v{settings.APP_VERSION}")
    print(f"[INFO] Environment: {settings.ENVIRONMENT}")
    print(f"[INFO] Debug mode: {settings.DEBUG}")

    # Initialize Qdrant client
    try:
        await init_qdrant_client()
    except Exception as e:
        print(f"[WARNING]  Warning: Qdrant initialization failed: {str(e)}")

    # Initialize Neon PostgreSQL pool
    try:
        await init_neon_pool()
    except Exception as e:
        print(f"[WARNING]  Warning: Neon initialization failed: {str(e)}")

    # Validate sentence-transformers embedding model
    try:
        embedding_ok = test_embedding_connection()
        if not embedding_ok:
            print("[WARNING]  Warning: Embedding model test failed")
    except Exception as e:
        print(f"[WARNING]  Warning: Embedding model validation failed: {str(e)}")

    print("[OK] All services initialized successfully")


# Shutdown event
@app.on_event("shutdown")
async def shutdown_event():
    """Clean up database connections"""
    print("🛑 Shutting down...")

    # Close Qdrant client and Neon pool
    await close_connections()

    print("[OK] Cleanup complete")


# Health check endpoint
@app.get("/health", response_model=HealthResponse, tags=["Health"])
async def health_check():
    """
    Check if the API and its dependencies are operational

    Returns:
        HealthResponse: Service status and timestamp
    """
    services = {}

    # Check Qdrant
    try:
        qdrant_ok = test_qdrant_connection()
        services["qdrant"] = "operational" if qdrant_ok else "down"
    except:
        services["qdrant"] = "down"

    # Check Neon
    try:
        neon_ok = await test_neon_connection()
        services["neon"] = "operational" if neon_ok else "down"
    except:
        services["neon"] = "down"

    # Check Embedding Model
    try:
        embedding_ok = test_embedding_connection()
        services["embeddings"] = "operational" if embedding_ok else "down"
    except:
        services["embeddings"] = "down"

    status = "healthy" if all(v == "operational" for v in services.values()) else "degraded"

    return HealthResponse(
        status=status,
        timestamp=datetime.utcnow(),
        services=services
    )


# Main RAG query endpoint
@app.post("/query", response_model=QueryResponse, tags=["Query"])
@limiter.limit(f"{settings.RATE_LIMIT_PER_HOUR}/hour")
async def query_chatbot(
    request: Request,
    query: QueryRequest,
    config: dict = Depends(get_settings)
):
    """
    Submit a query to the RAG chatbot

    Args:
        request: FastAPI request object (for rate limiting)
        query: User question and optional selected text
        config: Application settings (dependency injection)

    Returns:
        QueryResponse: Generated answer with citations

    Raises:
        HTTPException: 400 (bad request), 429 (rate limit), 500 (server error)
    """
    start_time = time.time()

    try:
        # Step 1: Detect query mode (if mode == "auto")
        detected_mode = query.mode
        if query.mode == "auto":
            detected_mode = detect_query_mode(query.question)

        # Step 2: Generate embedding for the question
        embedding = generate_embedding(query.question)

        # Step 3: Retrieve top-k chunks from Qdrant
        qdrant_results = search_qdrant(
            query_embedding=embedding,
            top_k=settings.RAG_TOP_K
        )

        # Check if we found any relevant chunks
        if not qdrant_results:
            # No relevant context found
            answer = generate_no_context_response(query.question, detected_mode)
            citations = []
            response_time_ms = int((time.time() - start_time) * 1000)

            return QueryResponse(
                answer=answer,
                citations=citations,
                mode_detected=detected_mode,
                response_time_ms=response_time_ms
            )

        # Step 4: Fetch full chunk data from Qdrant
        chunk_ids = [chunk_id for chunk_id, score, metadata, content in qdrant_results]
        chunks = get_chunks_batch_from_qdrant(chunk_ids)

        # Step 5: Generate answer using Groq llama-3.1-8b-instant
        answer = generate_answer(
            question=query.question,
            context_chunks=chunks,
            mode=detected_mode,
            selected_text=query.selected_text
        )

        # Step 6: Extract citations from retrieved chunks
        citations = extract_citations(chunks)

        # Calculate response time
        response_time_ms = int((time.time() - start_time) * 1000)

        # Step 7: Log query to Neon PostgreSQL
        await log_query_to_neon(
            question=query.question,
            mode=detected_mode,
            chunk_ids=chunk_ids,
            answer=answer,
            citations=citations,
            response_time_ms=response_time_ms,
            user_ip_hash=None  # TODO: Hash user IP if needed
        )

        return QueryResponse(
            answer=answer,
            citations=citations,
            mode_detected=detected_mode,
            response_time_ms=response_time_ms
        )

    except Exception as e:
        print(f"[ERROR] Error processing query: {str(e)}")
        raise HTTPException(
            status_code=500,
            detail={
                "error": "Internal Server Error",
                "message": "An unexpected error occurred while processing your query",
                "code": "INTERNAL_ERROR"
            }
        )


# Helper function: Query mode detection
def detect_query_mode(question: str) -> str:
    """
    Detect query mode based on keywords (rule-based, zero-cost)

    Args:
        question: User's question text

    Returns:
        str: Detected mode ("explain", "code", "exam", or "urdu")
    """
    question_lower = question.lower()

    # Check for Urdu script
    if any('\u0600' <= char <= '\u06FF' for char in question):
        return "urdu"

    # Check for code keywords
    if any(keyword in question_lower for keyword in settings.CODE_KEYWORDS):
        return "code"

    # Check for exam keywords
    if any(keyword in question_lower for keyword in settings.EXAM_KEYWORDS):
        return "exam"

    # Default to explain mode
    return "explain"


# Root endpoint
@app.get("/", tags=["Root"])
async def root():
    """API root endpoint"""
    return {
        "name": settings.APP_NAME,
        "version": settings.APP_VERSION,
        "status": "operational",
        "docs": "/docs",
        "health": "/health"
    }


# Exception handler for validation errors
@app.exception_handler(422)
async def validation_exception_handler(request: Request, exc):
    """Custom handler for Pydantic validation errors"""
    return JSONResponse(
        status_code=400,
        content={
            "error": "Bad Request",
            "message": "Invalid request parameters",
            "code": "INVALID_REQUEST",
            "details": exc.errors()
        }
    )


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(
        "app.main:app",
        host=settings.HOST,
        port=settings.PORT,
        reload=settings.RELOAD
    )
