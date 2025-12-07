"""
Authentication Module
Handles user authentication, authorization, and profile management
"""

from .models import (
    SignupRequest,
    LoginRequest,
    TokenRefreshRequest,
    ProfileUpdateRequest,
    UserEditRequest,
    AnnotationRequest,
    ProgressUpdateRequest,
    UserResponse,
    UserProfileResponse,
    TokenResponse,
    AuthResponse,
    UserEditResponse,
    AnnotationResponse,
    ProgressResponse,
    ProgressStatsResponse
)

from .utils import (
    hash_password,
    verify_password,
    create_access_token,
    verify_token,
    get_user_id_from_token
)

__all__ = [
    # Request models
    "SignupRequest",
    "LoginRequest",
    "TokenRefreshRequest",
    "ProfileUpdateRequest",
    "UserEditRequest",
    "AnnotationRequest",
    "ProgressUpdateRequest",
    # Response models
    "UserResponse",
    "UserProfileResponse",
    "TokenResponse",
    "AuthResponse",
    "UserEditResponse",
    "AnnotationResponse",
    "ProgressResponse",
    "ProgressStatsResponse",
    # Utilities
    "hash_password",
    "verify_password",
    "create_access_token",
    "verify_token",
    "get_user_id_from_token"
]
