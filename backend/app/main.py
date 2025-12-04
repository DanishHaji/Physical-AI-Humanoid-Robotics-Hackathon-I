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

# Initialize FastAPI app
app = FastAPI(
    title=settings.APP_NAME,
    version=settings.APP_VERSION,
    description="RAG API for AI-Native Physical AI & Humanoid Robotics Textbook",
    docs_url="/docs",
    redoc_url="/redoc"
)

# Initialize rate limiter
limiter = Limiter(key_func=get_remote_address)
app.state.limiter = limiter
app.add_exception_handler(RateLimitExceeded, _rate_limit_exceeded_handler)

# CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.ALLOWED_ORIGINS,
    allow_credentials=True,
    allow_methods=["GET", "POST"],
    allow_headers=["*"],
)


# Startup event
@app.on_event("startup")
async def startup_event():
    """Initialize database connections and services"""
    print(f"🚀 Starting {settings.APP_NAME} v{settings.APP_VERSION}")
    print(f"📍 Environment: {settings.ENVIRONMENT}")
    print(f"🔧 Debug mode: {settings.DEBUG}")

    # TODO: Initialize Qdrant client
    # TODO: Initialize Neon PostgreSQL pool
    # TODO: Validate OpenAI API key

    print("✅ All services initialized successfully")


# Shutdown event
@app.on_event("shutdown")
async def shutdown_event():
    """Clean up database connections"""
    print("🛑 Shutting down...")

    # TODO: Close Qdrant client
    # TODO: Close Neon PostgreSQL pool

    print("✅ Cleanup complete")


# Health check endpoint
@app.get("/health", response_model=HealthResponse, tags=["Health"])
async def health_check():
    """
    Check if the API and its dependencies are operational

    Returns:
        HealthResponse: Service status and timestamp
    """
    # TODO: Implement actual health checks for Qdrant, Neon, OpenAI

    services = {
        "qdrant": "operational",  # TODO: ping Qdrant
        "neon": "operational",    # TODO: ping Neon
        "openai": "operational"   # TODO: validate API key
    }

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
        # TODO: Implement embedding generation
        # embedding = await generate_embedding(query.question)

        # Step 3: Retrieve top-k chunks from Qdrant
        # TODO: Implement vector search
        # chunks = await search_qdrant(embedding, top_k=settings.RAG_TOP_K)

        # Step 4: Filter chunks by similarity threshold
        # TODO: Implement filtering
        # filtered_chunks = filter_by_similarity(chunks, threshold=settings.RAG_SIMILARITY_THRESHOLD)

        # Step 5: Generate answer using OpenAI GPT-4o-mini
        # TODO: Implement answer generation
        # answer = await generate_answer(query.question, filtered_chunks, detected_mode)

        # Step 6: Extract citations from retrieved chunks
        # TODO: Implement citation extraction
        # citations = extract_citations(filtered_chunks)

        # TEMPORARY: Return mock response until RAG pipeline is implemented
        answer = "This is a placeholder response. The RAG system is not yet fully implemented. Please check back after the embedding, retrieval, and generation modules are complete."
        citations = [
            Citation(
                chapter="ROS 2 Fundamentals",
                section="Introduction",
                url="/docs/module-01-ros2/week-02-ros2-fundamentals#introduction"
            )
        ]

        # Calculate response time
        response_time_ms = int((time.time() - start_time) * 1000)

        # Step 7: Log query to Neon PostgreSQL
        # TODO: Implement query logging
        # await log_query(query, answer, citations, detected_mode, response_time_ms)

        return QueryResponse(
            answer=answer,
            citations=citations,
            mode_detected=detected_mode,
            response_time_ms=response_time_ms
        )

    except Exception as e:
        print(f"❌ Error processing query: {str(e)}")
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
