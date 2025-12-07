"""
Authentication Models and Schemas
Pydantic models for user authentication, profiles, and personalization
"""

from datetime import datetime
from typing import Optional, List
from pydantic import BaseModel, EmailStr, Field, validator
import re


# ============================================================================
# REQUEST MODELS (Input Validation)
# ============================================================================

class SignupRequest(BaseModel):
    """
    User signup request with background questionnaire
    """
    # Basic auth
    email: EmailStr = Field(..., description="User email address")
    password: str = Field(..., min_length=8, description="Password (min 8 characters)")
    username: str = Field(..., min_length=3, max_length=50, description="Username (3-50 characters)")

    # Background questionnaire
    software_experience: str = Field(..., description="beginner | intermediate | advanced")
    hardware_experience: str = Field(..., description="beginner | intermediate | advanced")
    programming_languages: List[str] = Field(default=[], description="List of programming languages known")
    robotics_experience: Optional[str] = Field("none", description="none | hobbyist | student | professional")
    learning_goal: Optional[str] = Field(None, description="What user wants to learn")

    # Personalization preferences
    preferred_difficulty: str = Field("auto", description="beginner | intermediate | advanced | auto")
    show_code_examples: bool = Field(True, description="Show code examples in content")
    show_advanced_topics: bool = Field(True, description="Show advanced topics")
    preferred_language: str = Field("en", description="en | ur")

    @validator('password')
    def validate_password(cls, v):
        """
        Validate password strength
        - At least 8 characters
        - At least one uppercase letter
        - At least one lowercase letter
        - At least one digit
        """
        if len(v) < 8:
            raise ValueError('Password must be at least 8 characters')
        if not re.search(r'[A-Z]', v):
            raise ValueError('Password must contain at least one uppercase letter')
        if not re.search(r'[a-z]', v):
            raise ValueError('Password must contain at least one lowercase letter')
        if not re.search(r'\d', v):
            raise ValueError('Password must contain at least one digit')
        return v

    @validator('username')
    def validate_username(cls, v):
        """
        Validate username format
        - Only alphanumeric and underscores
        """
        if not re.match(r'^[a-zA-Z0-9_]+$', v):
            raise ValueError('Username can only contain letters, numbers, and underscores')
        return v

    @validator('software_experience', 'hardware_experience')
    def validate_experience_level(cls, v):
        """Validate experience level"""
        if v not in ['beginner', 'intermediate', 'advanced']:
            raise ValueError('Experience level must be: beginner, intermediate, or advanced')
        return v

    @validator('robotics_experience')
    def validate_robotics_experience(cls, v):
        """Validate robotics experience"""
        if v not in ['none', 'hobbyist', 'student', 'professional']:
            raise ValueError('Robotics experience must be: none, hobbyist, student, or professional')
        return v

    @validator('preferred_difficulty')
    def validate_difficulty(cls, v):
        """Validate preferred difficulty"""
        if v not in ['beginner', 'intermediate', 'advanced', 'auto']:
            raise ValueError('Preferred difficulty must be: beginner, intermediate, advanced, or auto')
        return v

    @validator('preferred_language')
    def validate_language(cls, v):
        """Validate preferred language"""
        if v not in ['en', 'ur']:
            raise ValueError('Preferred language must be: en or ur')
        return v


class LoginRequest(BaseModel):
    """
    User login request
    """
    email: EmailStr = Field(..., description="User email address")
    password: str = Field(..., description="User password")


class TokenRefreshRequest(BaseModel):
    """
    Token refresh request
    """
    refresh_token: str = Field(..., description="Refresh token")


class ProfileUpdateRequest(BaseModel):
    """
    Update user profile request
    """
    software_experience: Optional[str] = None
    hardware_experience: Optional[str] = None
    programming_languages: Optional[List[str]] = None
    robotics_experience: Optional[str] = None
    learning_goal: Optional[str] = None
    preferred_difficulty: Optional[str] = None
    show_code_examples: Optional[bool] = None
    show_advanced_topics: Optional[bool] = None
    preferred_language: Optional[str] = None

    @validator('software_experience', 'hardware_experience')
    def validate_experience_level(cls, v):
        """Validate experience level"""
        if v is not None and v not in ['beginner', 'intermediate', 'advanced']:
            raise ValueError('Experience level must be: beginner, intermediate, or advanced')
        return v

    @validator('robotics_experience')
    def validate_robotics_experience(cls, v):
        """Validate robotics experience"""
        if v is not None and v not in ['none', 'hobbyist', 'student', 'professional']:
            raise ValueError('Robotics experience must be: none, hobbyist, student, or professional')
        return v


class UserEditRequest(BaseModel):
    """
    Create or update user-specific content edit
    """
    chapter_path: str = Field(..., description="Path to chapter (e.g., module-01-ros2/week-02-ros2-fundamentals)")
    section_id: Optional[str] = Field(None, description="Section heading or ID")
    original_content: Optional[str] = Field(None, description="Snapshot of original content")
    edited_content: str = Field(..., description="User's personalized version")
    edit_type: str = Field("replace", description="replace | append | prepend")

    @validator('edit_type')
    def validate_edit_type(cls, v):
        """Validate edit type"""
        if v not in ['replace', 'append', 'prepend']:
            raise ValueError('Edit type must be: replace, append, or prepend')
        return v


class AnnotationRequest(BaseModel):
    """
    Create user annotation (note, highlight, bookmark)
    """
    chapter_path: str = Field(..., description="Path to chapter")
    section_id: Optional[str] = Field(None, description="Section heading or ID")
    annotation_type: str = Field(..., description="note | highlight | bookmark | question")
    content: str = Field(..., description="The note/highlight text")
    color: Optional[str] = Field(None, description="yellow | green | blue | pink")
    position_start: Optional[int] = Field(None, description="Character position in section")
    position_end: Optional[int] = Field(None, description="Character position in section")

    @validator('annotation_type')
    def validate_annotation_type(cls, v):
        """Validate annotation type"""
        if v not in ['note', 'highlight', 'bookmark', 'question']:
            raise ValueError('Annotation type must be: note, highlight, bookmark, or question')
        return v

    @validator('color')
    def validate_color(cls, v):
        """Validate highlight color"""
        if v is not None and v not in ['yellow', 'green', 'blue', 'pink']:
            raise ValueError('Color must be: yellow, green, blue, or pink')
        return v


class ProgressUpdateRequest(BaseModel):
    """
    Update user progress for a chapter
    """
    chapter_path: str = Field(..., description="Path to chapter")
    completion_percentage: Optional[int] = Field(None, ge=0, le=100, description="Completion percentage (0-100)")
    time_spent_seconds: Optional[int] = Field(None, ge=0, description="Time spent in seconds")
    is_completed: Optional[bool] = Field(None, description="Whether chapter is completed")


# ============================================================================
# RESPONSE MODELS (Output)
# ============================================================================

class UserResponse(BaseModel):
    """
    User data response (without sensitive info)
    """
    id: int
    email: str
    username: str
    created_at: datetime
    is_active: bool
    email_verified: bool

    class Config:
        from_attributes = True


class UserProfileResponse(BaseModel):
    """
    User profile response
    """
    id: int
    user_id: int
    software_experience: str
    hardware_experience: str
    programming_languages: List[str]
    robotics_experience: Optional[str]
    learning_goal: Optional[str]
    preferred_difficulty: str
    show_code_examples: bool
    show_advanced_topics: bool
    preferred_language: str
    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True


class TokenResponse(BaseModel):
    """
    JWT token response
    """
    access_token: str
    token_type: str = "bearer"
    expires_in: int = 60 * 60 * 24 * 7  # 7 days in seconds


class AuthResponse(BaseModel):
    """
    Authentication response (login/signup)
    """
    user: UserResponse
    profile: UserProfileResponse
    token: TokenResponse


class UserEditResponse(BaseModel):
    """
    User content edit response
    """
    id: int
    user_id: int
    chapter_path: str
    section_id: Optional[str]
    original_content: Optional[str]
    edited_content: str
    edit_type: str
    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True


class AnnotationResponse(BaseModel):
    """
    User annotation response
    """
    id: int
    user_id: int
    chapter_path: str
    section_id: Optional[str]
    annotation_type: str
    content: str
    color: Optional[str]
    position_start: Optional[int]
    position_end: Optional[int]
    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True


class ProgressResponse(BaseModel):
    """
    User progress response
    """
    id: int
    user_id: int
    chapter_path: str
    completion_percentage: int
    last_read_at: Optional[datetime]
    time_spent_seconds: int
    is_completed: bool
    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True


class ProgressStatsResponse(BaseModel):
    """
    Overall user progress statistics
    """
    total_chapters: int
    completed_chapters: int
    total_time_seconds: int
    average_completion: float
    chapters_in_progress: int
    last_activity: Optional[datetime]


# ============================================================================
# DATABASE MODELS (Internal)
# ============================================================================

class UserDB(BaseModel):
    """
    User database model (internal use)
    """
    id: int
    email: str
    password_hash: str
    username: str
    created_at: datetime
    updated_at: datetime
    is_active: bool
    email_verified: bool


class UserProfileDB(BaseModel):
    """
    User profile database model (internal use)
    """
    id: int
    user_id: int
    software_experience: str
    hardware_experience: str
    programming_languages: List[str]
    robotics_experience: Optional[str]
    learning_goal: Optional[str]
    preferred_difficulty: str
    show_code_examples: bool
    show_advanced_topics: bool
    preferred_language: str
    created_at: datetime
    updated_at: datetime
