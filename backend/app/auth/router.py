"""
Authentication Router
FastAPI routes for user authentication, profiles, edits, and annotations
"""

from fastapi import APIRouter, HTTPException, Depends, Header
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from typing import Optional, List
from datetime import timedelta

from .models import (
    SignupRequest,
    LoginRequest,
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
from .database import get_auth_db


router = APIRouter(prefix="/auth", tags=["Authentication"])
security = HTTPBearer()


# ============================================================================
# DEPENDENCY: Get Current User from Token
# ============================================================================

async def get_current_user(
    credentials: HTTPAuthorizationCredentials = Depends(security)
) -> int:
    """
    Extract and validate user ID from JWT token

    Args:
        credentials: HTTP Bearer token

    Returns:
        User ID

    Raises:
        HTTPException: If token is invalid or user not found
    """
    token = credentials.credentials

    # Verify token
    user_id = get_user_id_from_token(token)

    if user_id is None:
        raise HTTPException(
            status_code=401,
            detail="Invalid or expired token"
        )

    # Verify user exists and is active
    db = await get_auth_db()
    user = await db.get_user_by_id(user_id)

    if not user or not user.get('is_active'):
        raise HTTPException(
            status_code=401,
            detail="User not found or inactive"
        )

    return user_id


# ============================================================================
# AUTHENTICATION ENDPOINTS
# ============================================================================

@router.post("/signup", response_model=AuthResponse, status_code=201)
async def signup(signup_data: SignupRequest):
    """
    Create new user account with background questionnaire

    Returns user data, profile, and JWT token
    """
    db = await get_auth_db()

    # Hash password
    password_hash = hash_password(signup_data.password)

    # Create user
    user = await db.create_user(
        email=signup_data.email,
        password_hash=password_hash,
        username=signup_data.username
    )

    if user is None:
        raise HTTPException(
            status_code=400,
            detail="Email or username already exists"
        )

    # Create profile with questionnaire answers
    profile = await db.create_profile(
        user_id=user['id'],
        signup_data=signup_data
    )

    # Generate JWT token
    access_token = create_access_token(
        data={"sub": str(user['id'])}
    )

    return AuthResponse(
        user=UserResponse(**user),
        profile=UserProfileResponse(**profile),
        token=TokenResponse(access_token=access_token)
    )


@router.post("/login", response_model=AuthResponse)
async def login(login_data: LoginRequest):
    """
    Login user and return JWT token

    Returns user data, profile, and JWT token
    """
    db = await get_auth_db()

    # Get user by email
    user = await db.get_user_by_email(login_data.email)

    if not user:
        raise HTTPException(
            status_code=401,
            detail="Invalid email or password"
        )

    # Verify password
    if not verify_password(login_data.password, user['password_hash']):
        raise HTTPException(
            status_code=401,
            detail="Invalid email or password"
        )

    # Check if user is active
    if not user.get('is_active'):
        raise HTTPException(
            status_code=403,
            detail="Account is inactive"
        )

    # Get profile
    profile = await db.get_profile(user['id'])

    if not profile:
        raise HTTPException(
            status_code=500,
            detail="User profile not found"
        )

    # Generate JWT token
    access_token = create_access_token(
        data={"sub": str(user['id'])}
    )

    # Remove password_hash from response
    user_data = {k: v for k, v in user.items() if k != 'password_hash'}

    return AuthResponse(
        user=UserResponse(**user_data),
        profile=UserProfileResponse(**profile),
        token=TokenResponse(access_token=access_token)
    )


@router.get("/me", response_model=UserResponse)
async def get_current_user_info(user_id: int = Depends(get_current_user)):
    """
    Get current logged-in user information
    """
    db = await get_auth_db()
    user = await db.get_user_by_id(user_id)

    if not user:
        raise HTTPException(status_code=404, detail="User not found")

    return UserResponse(**user)


# ============================================================================
# PROFILE ENDPOINTS
# ============================================================================

@router.get("/profile", response_model=UserProfileResponse)
async def get_profile(user_id: int = Depends(get_current_user)):
    """
    Get user profile
    """
    db = await get_auth_db()
    profile = await db.get_profile(user_id)

    if not profile:
        raise HTTPException(status_code=404, detail="Profile not found")

    return UserProfileResponse(**profile)


@router.put("/profile", response_model=UserProfileResponse)
async def update_profile(
    updates: ProfileUpdateRequest,
    user_id: int = Depends(get_current_user)
):
    """
    Update user profile and preferences
    """
    db = await get_auth_db()
    profile = await db.update_profile(user_id, updates)

    if not profile:
        raise HTTPException(status_code=404, detail="Profile not found")

    return UserProfileResponse(**profile)


# ============================================================================
# USER EDITS ENDPOINTS
# ============================================================================

@router.post("/edits", response_model=UserEditResponse, status_code=201)
async def create_or_update_edit(
    edit_data: UserEditRequest,
    user_id: int = Depends(get_current_user)
):
    """
    Create or update user-specific content edit
    """
    db = await get_auth_db()
    edit = await db.create_or_update_edit(user_id, edit_data)

    return UserEditResponse(**edit)


@router.get("/edits/{chapter_path:path}", response_model=List[UserEditResponse])
async def get_edits_for_chapter(
    chapter_path: str,
    user_id: int = Depends(get_current_user)
):
    """
    Get all user edits for a specific chapter
    """
    db = await get_auth_db()
    edits = await db.get_edits_for_chapter(user_id, chapter_path)

    return [UserEditResponse(**edit) for edit in edits]


@router.delete("/edits/{edit_id}", status_code=204)
async def delete_edit(
    edit_id: int,
    user_id: int = Depends(get_current_user)
):
    """
    Delete a user edit
    """
    db = await get_auth_db()
    success = await db.delete_edit(user_id, edit_id)

    if not success:
        raise HTTPException(status_code=404, detail="Edit not found")

    return None


# ============================================================================
# ANNOTATIONS ENDPOINTS
# ============================================================================

@router.post("/annotations", response_model=AnnotationResponse, status_code=201)
async def create_annotation(
    annotation_data: AnnotationRequest,
    user_id: int = Depends(get_current_user)
):
    """
    Create user annotation (note, highlight, bookmark)
    """
    db = await get_auth_db()
    annotation = await db.create_annotation(user_id, annotation_data)

    return AnnotationResponse(**annotation)


@router.get("/annotations/{chapter_path:path}", response_model=List[AnnotationResponse])
async def get_annotations_for_chapter(
    chapter_path: str,
    user_id: int = Depends(get_current_user)
):
    """
    Get all annotations for a specific chapter
    """
    db = await get_auth_db()
    annotations = await db.get_annotations_for_chapter(user_id, chapter_path)

    return [AnnotationResponse(**ann) for ann in annotations]


@router.delete("/annotations/{annotation_id}", status_code=204)
async def delete_annotation(
    annotation_id: int,
    user_id: int = Depends(get_current_user)
):
    """
    Delete an annotation
    """
    db = await get_auth_db()
    success = await db.delete_annotation(user_id, annotation_id)

    if not success:
        raise HTTPException(status_code=404, detail="Annotation not found")

    return None


# ============================================================================
# PROGRESS TRACKING ENDPOINTS
# ============================================================================

@router.post("/progress", response_model=ProgressResponse)
async def update_progress(
    progress_data: ProgressUpdateRequest,
    user_id: int = Depends(get_current_user)
):
    """
    Update user progress for a chapter
    """
    db = await get_auth_db()
    progress = await db.update_progress(user_id, progress_data)

    return ProgressResponse(**progress)


@router.get("/progress/{chapter_path:path}", response_model=ProgressResponse)
async def get_progress_for_chapter(
    chapter_path: str,
    user_id: int = Depends(get_current_user)
):
    """
    Get progress for a specific chapter
    """
    db = await get_auth_db()
    progress = await db.get_progress(user_id, chapter_path)

    if not progress:
        # Return default progress if not found
        return ProgressResponse(
            id=0,
            user_id=user_id,
            chapter_path=chapter_path,
            completion_percentage=0,
            last_read_at=None,
            time_spent_seconds=0,
            is_completed=False,
            created_at=None,
            updated_at=None
        )

    return ProgressResponse(**progress)


@router.get("/progress", response_model=List[ProgressResponse])
async def get_all_progress(user_id: int = Depends(get_current_user)):
    """
    Get all progress for current user
    """
    db = await get_auth_db()
    progress_list = await db.get_all_progress(user_id)

    return [ProgressResponse(**p) for p in progress_list]


@router.get("/progress-stats", response_model=ProgressStatsResponse)
async def get_progress_stats(user_id: int = Depends(get_current_user)):
    """
    Get overall progress statistics
    """
    db = await get_auth_db()
    stats = await db.get_progress_stats(user_id)

    return ProgressStatsResponse(**stats)


# ============================================================================
# UTILITY ENDPOINTS
# ============================================================================

@router.get("/health")
async def health_check():
    """
    Health check endpoint
    """
    return {
        "status": "healthy",
        "service": "authentication"
    }
