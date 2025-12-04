"""
Pydantic models for API request/response schemas
Matches the OpenAPI spec defined in specs/001-ai-textbook-gen/contracts/rag-api.openapi.yaml
"""

from pydantic import BaseModel, Field, field_validator
from typing import Optional, Literal
from datetime import datetime
from uuid import UUID


# Request Models
class QueryRequest(BaseModel):
    """Request schema for POST /query endpoint"""

    question: str = Field(
        ...,
        min_length=1,
        max_length=500,
        description="User's natural language question"
    )
    selected_text: Optional[str] = Field(
        None,
        max_length=2000,
        description="Optional text selected by user for 'Ask AI about this' feature"
    )
    mode: Literal["auto", "explain", "code", "urdu", "exam"] = Field(
        default="auto",
        description="Query mode: auto-detect or user-specified"
    )

    @field_validator("question")
    @classmethod
    def question_not_empty(cls, v: str) -> str:
        if not v.strip():
            raise ValueError("Question cannot be empty or whitespace only")
        return v.strip()


# Response Models
class Citation(BaseModel):
    """Citation schema for source references"""

    chapter: str = Field(..., description="Chapter title where information was found")
    section: str = Field(..., description="Specific section heading within the chapter")
    url: str = Field(..., description="Relative URL to the chapter section")


class QueryResponse(BaseModel):
    """Response schema for POST /query endpoint"""

    answer: str = Field(
        ...,
        description="Generated answer using retrieved textbook context"
    )
    citations: list[Citation] = Field(
        default_factory=list,
        description="List of chapter sections referenced in the answer"
    )
    mode_detected: Literal["explain", "code", "urdu", "exam"] = Field(
        ...,
        description="The mode used to generate the answer"
    )
    response_time_ms: int = Field(
        ...,
        ge=1,
        le=10000,
        description="Total pipeline latency in milliseconds"
    )


class HealthResponse(BaseModel):
    """Response schema for GET /health endpoint"""

    status: Literal["healthy", "degraded"] = Field(..., description="Overall service status")
    timestamp: datetime = Field(..., description="Current server timestamp")
    services: dict[str, Literal["operational", "down"]] = Field(
        ...,
        description="Status of individual services (qdrant, neon, openai)"
    )


# Error Response
class ErrorResponse(BaseModel):
    """Error response schema"""

    error: str = Field(..., description="HTTP error name")
    message: str = Field(..., description="Human-readable error message")
    code: Literal[
        "INVALID_QUESTION_LENGTH",
        "INVALID_MODE",
        "MISSING_QUESTION",
        "RATE_LIMIT_EXCEEDED",
        "INTERNAL_ERROR",
        "SERVICE_UNAVAILABLE",
        "OPENAI_API_ERROR",
        "QDRANT_ERROR",
        "NEON_ERROR"
    ] = Field(..., description="Machine-readable error code")
    retry_after_seconds: Optional[int] = Field(
        None,
        description="Seconds to wait before retrying (for rate limit errors)"
    )


# Database Models
class ChunkMetadata(BaseModel):
    """Metadata schema for chunks stored in Qdrant"""

    module: int = Field(..., ge=1, le=4, description="Module number")
    week: int = Field(..., ge=1, le=13, description="Week number")
    chapter_title: str = Field(..., max_length=200)
    heading: str = Field(..., max_length=200)
    concept_tags: list[str] = Field(default_factory=list)
    vla_tags: Optional[list[str]] = Field(default_factory=list)
    content_type: Literal["theory", "code", "lab", "troubleshooting", "assessment", "application"]


class Chunk(BaseModel):
    """Complete chunk schema (PostgreSQL + Qdrant)"""

    id: UUID
    chapter_id: UUID
    content: str = Field(..., min_length=100, max_length=4096)
    embedding: Optional[list[float]] = Field(None, min_items=1536, max_items=1536)
    metadata: ChunkMetadata
    heading: str = Field(..., max_length=200)
    position: int = Field(..., ge=0)
    token_count: int = Field(..., ge=512, le=1024)
    created_at: datetime
    updated_at: datetime


class Query(BaseModel):
    """Query log schema (stored in Neon PostgreSQL)"""

    id: UUID
    question_text: str = Field(..., min_length=1, max_length=500)
    selected_text: Optional[str] = None
    mode: Literal["explain", "code", "urdu", "exam"]
    retrieved_chunk_ids: list[UUID]
    answer_text: str
    citations: list[Citation]
    response_time_ms: int = Field(..., gt=0, lt=10000)
    user_ip_hash: Optional[str] = Field(None, max_length=64)
    feedback: Optional[Literal["helpful", "not_helpful", "report_issue"]] = None
    created_at: datetime
