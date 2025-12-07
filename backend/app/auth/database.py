"""
Authentication Database Service
Handles all database operations for users, profiles, edits, annotations, and progress
"""

from typing import Optional, List, Dict
from datetime import datetime
import json
from ..clients.neon_client import get_neon_client
from .models import (
    SignupRequest,
    ProfileUpdateRequest,
    UserEditRequest,
    AnnotationRequest,
    ProgressUpdateRequest,
    UserDB,
    UserProfileDB
)


class AuthDatabaseService:
    """
    Service for authentication database operations
    """

    def __init__(self):
        self.neon_client = None

    async def init(self):
        """Initialize database connection"""
        self.neon_client = await get_neon_client()

    # ========================================================================
    # USER OPERATIONS
    # ========================================================================

    async def create_user(
        self,
        email: str,
        password_hash: str,
        username: str
    ) -> Optional[Dict]:
        """
        Create a new user

        Args:
            email: User email
            password_hash: Hashed password
            username: Username

        Returns:
            User dict or None if email/username already exists
        """
        if not self.neon_client:
            await self.init()

        try:
            async with self.neon_client.pool.acquire() as conn:
                # Check if user already exists
                existing = await conn.fetchrow(
                    "SELECT id FROM users WHERE email = $1 OR username = $2",
                    email, username
                )

                if existing:
                    return None

                # Insert new user
                user = await conn.fetchrow(
                    """
                    INSERT INTO users (email, password_hash, username, created_at, updated_at, is_active, email_verified)
                    VALUES ($1, $2, $3, $4, $5, $6, $7)
                    RETURNING id, email, username, created_at, updated_at, is_active, email_verified
                    """,
                    email, password_hash, username, datetime.utcnow(), datetime.utcnow(), True, False
                )

                return dict(user)

        except Exception as e:
            print(f"[ERROR] Error creating user: {str(e)}")
            raise

    async def get_user_by_email(self, email: str) -> Optional[Dict]:
        """Get user by email (includes password_hash for auth)"""
        if not self.neon_client:
            await self.init()

        try:
            async with self.neon_client.pool.acquire() as conn:
                user = await conn.fetchrow(
                    """
                    SELECT id, email, password_hash, username, created_at, updated_at, is_active, email_verified
                    FROM users
                    WHERE email = $1
                    """,
                    email
                )

                return dict(user) if user else None

        except Exception as e:
            print(f"[ERROR] Error getting user by email: {str(e)}")
            raise

    async def get_user_by_id(self, user_id: int) -> Optional[Dict]:
        """Get user by ID (without password_hash)"""
        if not self.neon_client:
            await self.init()

        try:
            async with self.neon_client.pool.acquire() as conn:
                user = await conn.fetchrow(
                    """
                    SELECT id, email, username, created_at, updated_at, is_active, email_verified
                    FROM users
                    WHERE id = $1
                    """,
                    user_id
                )

                return dict(user) if user else None

        except Exception as e:
            print(f"[ERROR] Error getting user by ID: {str(e)}")
            raise

    # ========================================================================
    # USER PROFILE OPERATIONS
    # ========================================================================

    async def create_profile(
        self,
        user_id: int,
        signup_data: SignupRequest
    ) -> Optional[Dict]:
        """
        Create user profile from signup data

        Args:
            user_id: User ID
            signup_data: Signup request with questionnaire answers

        Returns:
            Profile dict
        """
        if not self.neon_client:
            await self.init()

        try:
            async with self.neon_client.pool.acquire() as conn:
                profile = await conn.fetchrow(
                    """
                    INSERT INTO user_profiles
                    (user_id, software_experience, hardware_experience, programming_languages,
                     robotics_experience, learning_goal, preferred_difficulty,
                     show_code_examples, show_advanced_topics, preferred_language, created_at, updated_at)
                    VALUES ($1, $2, $3, $4, $5, $6, $7, $8, $9, $10, $11, $12)
                    RETURNING *
                    """,
                    user_id,
                    signup_data.software_experience,
                    signup_data.hardware_experience,
                    signup_data.programming_languages,
                    signup_data.robotics_experience,
                    signup_data.learning_goal,
                    signup_data.preferred_difficulty,
                    signup_data.show_code_examples,
                    signup_data.show_advanced_topics,
                    signup_data.preferred_language,
                    datetime.utcnow(),
                    datetime.utcnow()
                )

                result = dict(profile)
                # Convert TEXT[] to List[str]
                result['programming_languages'] = list(result['programming_languages']) if result['programming_languages'] else []
                return result

        except Exception as e:
            print(f"[ERROR] Error creating profile: {str(e)}")
            raise

    async def get_profile(self, user_id: int) -> Optional[Dict]:
        """Get user profile by user ID"""
        if not self.neon_client:
            await self.init()

        try:
            async with self.neon_client.pool.acquire() as conn:
                profile = await conn.fetchrow(
                    "SELECT * FROM user_profiles WHERE user_id = $1",
                    user_id
                )

                if profile:
                    result = dict(profile)
                    # Convert TEXT[] to List[str]
                    result['programming_languages'] = list(result['programming_languages']) if result['programming_languages'] else []
                    return result
                return None

        except Exception as e:
            print(f"[ERROR] Error getting profile: {str(e)}")
            raise

    async def update_profile(
        self,
        user_id: int,
        updates: ProfileUpdateRequest
    ) -> Optional[Dict]:
        """
        Update user profile

        Args:
            user_id: User ID
            updates: Profile update request

        Returns:
            Updated profile dict
        """
        if not self.neon_client:
            await self.init()

        try:
            # Build dynamic UPDATE query based on provided fields
            update_fields = []
            values = []
            idx = 1

            for field, value in updates.dict(exclude_unset=True).items():
                if value is not None:
                    update_fields.append(f"{field} = ${idx}")
                    values.append(value)
                    idx += 1

            if not update_fields:
                return await self.get_profile(user_id)

            # Add updated_at
            update_fields.append(f"updated_at = ${idx}")
            values.append(datetime.utcnow())
            values.append(user_id)  # For WHERE clause

            query = f"""
                UPDATE user_profiles
                SET {', '.join(update_fields)}
                WHERE user_id = ${idx + 1}
                RETURNING *
            """

            async with self.neon_client.pool.acquire() as conn:
                profile = await conn.fetchrow(query, *values)

                if profile:
                    result = dict(profile)
                    result['programming_languages'] = list(result['programming_languages']) if result['programming_languages'] else []
                    return result
                return None

        except Exception as e:
            print(f"[ERROR] Error updating profile: {str(e)}")
            raise

    # ========================================================================
    # USER EDITS OPERATIONS
    # ========================================================================

    async def create_or_update_edit(
        self,
        user_id: int,
        edit_data: UserEditRequest
    ) -> Dict:
        """
        Create or update user content edit
        Uses UPSERT to handle conflicts
        """
        if not self.neon_client:
            await self.init()

        try:
            async with self.neon_client.pool.acquire() as conn:
                edit = await conn.fetchrow(
                    """
                    INSERT INTO user_edits
                    (user_id, chapter_path, section_id, original_content, edited_content, edit_type, created_at, updated_at)
                    VALUES ($1, $2, $3, $4, $5, $6, $7, $8)
                    ON CONFLICT (user_id, chapter_path, section_id)
                    DO UPDATE SET
                        edited_content = EXCLUDED.edited_content,
                        edit_type = EXCLUDED.edit_type,
                        updated_at = EXCLUDED.updated_at
                    RETURNING *
                    """,
                    user_id,
                    edit_data.chapter_path,
                    edit_data.section_id,
                    edit_data.original_content,
                    edit_data.edited_content,
                    edit_data.edit_type,
                    datetime.utcnow(),
                    datetime.utcnow()
                )

                return dict(edit)

        except Exception as e:
            print(f"[ERROR] Error creating/updating edit: {str(e)}")
            raise

    async def get_edits_for_chapter(
        self,
        user_id: int,
        chapter_path: str
    ) -> List[Dict]:
        """Get all user edits for a specific chapter"""
        if not self.neon_client:
            await self.init()

        try:
            async with self.neon_client.pool.acquire() as conn:
                edits = await conn.fetch(
                    """
                    SELECT * FROM user_edits
                    WHERE user_id = $1 AND chapter_path = $2
                    ORDER BY created_at DESC
                    """,
                    user_id, chapter_path
                )

                return [dict(edit) for edit in edits]

        except Exception as e:
            print(f"[ERROR] Error getting edits: {str(e)}")
            raise

    async def delete_edit(self, user_id: int, edit_id: int) -> bool:
        """Delete a user edit"""
        if not self.neon_client:
            await self.init()

        try:
            async with self.neon_client.pool.acquire() as conn:
                result = await conn.execute(
                    "DELETE FROM user_edits WHERE id = $1 AND user_id = $2",
                    edit_id, user_id
                )

                return result == "DELETE 1"

        except Exception as e:
            print(f"[ERROR] Error deleting edit: {str(e)}")
            raise

    # ========================================================================
    # ANNOTATIONS OPERATIONS
    # ========================================================================

    async def create_annotation(
        self,
        user_id: int,
        annotation_data: AnnotationRequest
    ) -> Dict:
        """Create user annotation (note, highlight, bookmark)"""
        if not self.neon_client:
            await self.init()

        try:
            async with self.neon_client.pool.acquire() as conn:
                annotation = await conn.fetchrow(
                    """
                    INSERT INTO user_annotations
                    (user_id, chapter_path, section_id, annotation_type, content, color,
                     position_start, position_end, created_at, updated_at)
                    VALUES ($1, $2, $3, $4, $5, $6, $7, $8, $9, $10)
                    RETURNING *
                    """,
                    user_id,
                    annotation_data.chapter_path,
                    annotation_data.section_id,
                    annotation_data.annotation_type,
                    annotation_data.content,
                    annotation_data.color,
                    annotation_data.position_start,
                    annotation_data.position_end,
                    datetime.utcnow(),
                    datetime.utcnow()
                )

                return dict(annotation)

        except Exception as e:
            print(f"[ERROR] Error creating annotation: {str(e)}")
            raise

    async def get_annotations_for_chapter(
        self,
        user_id: int,
        chapter_path: str
    ) -> List[Dict]:
        """Get all annotations for a chapter"""
        if not self.neon_client:
            await self.init()

        try:
            async with self.neon_client.pool.acquire() as conn:
                annotations = await conn.fetch(
                    """
                    SELECT * FROM user_annotations
                    WHERE user_id = $1 AND chapter_path = $2
                    ORDER BY created_at DESC
                    """,
                    user_id, chapter_path
                )

                return [dict(ann) for ann in annotations]

        except Exception as e:
            print(f"[ERROR] Error getting annotations: {str(e)}")
            raise

    async def delete_annotation(self, user_id: int, annotation_id: int) -> bool:
        """Delete an annotation"""
        if not self.neon_client:
            await self.init()

        try:
            async with self.neon_client.pool.acquire() as conn:
                result = await conn.execute(
                    "DELETE FROM user_annotations WHERE id = $1 AND user_id = $2",
                    annotation_id, user_id
                )

                return result == "DELETE 1"

        except Exception as e:
            print(f"[ERROR] Error deleting annotation: {str(e)}")
            raise

    # ========================================================================
    # PROGRESS TRACKING OPERATIONS
    # ========================================================================

    async def update_progress(
        self,
        user_id: int,
        progress_data: ProgressUpdateRequest
    ) -> Dict:
        """Update user progress for a chapter (UPSERT)"""
        if not self.neon_client:
            await self.init()

        try:
            async with self.neon_client.pool.acquire() as conn:
                # Build update fields
                updates = {}
                if progress_data.completion_percentage is not None:
                    updates['completion_percentage'] = progress_data.completion_percentage
                if progress_data.time_spent_seconds is not None:
                    updates['time_spent_seconds'] = progress_data.time_spent_seconds
                if progress_data.is_completed is not None:
                    updates['is_completed'] = progress_data.is_completed

                updates['last_read_at'] = datetime.utcnow()
                updates['updated_at'] = datetime.utcnow()

                # UPSERT
                progress = await conn.fetchrow(
                    """
                    INSERT INTO user_progress
                    (user_id, chapter_path, completion_percentage, last_read_at, time_spent_seconds,
                     is_completed, created_at, updated_at)
                    VALUES ($1, $2, $3, $4, $5, $6, $7, $8)
                    ON CONFLICT (user_id, chapter_path)
                    DO UPDATE SET
                        completion_percentage = COALESCE(EXCLUDED.completion_percentage, user_progress.completion_percentage),
                        time_spent_seconds = user_progress.time_spent_seconds + COALESCE(EXCLUDED.time_spent_seconds, 0),
                        is_completed = COALESCE(EXCLUDED.is_completed, user_progress.is_completed),
                        last_read_at = EXCLUDED.last_read_at,
                        updated_at = EXCLUDED.updated_at
                    RETURNING *
                    """,
                    user_id,
                    progress_data.chapter_path,
                    updates.get('completion_percentage', 0),
                    updates['last_read_at'],
                    updates.get('time_spent_seconds', 0),
                    updates.get('is_completed', False),
                    datetime.utcnow(),
                    updates['updated_at']
                )

                return dict(progress)

        except Exception as e:
            print(f"[ERROR] Error updating progress: {str(e)}")
            raise

    async def get_progress(self, user_id: int, chapter_path: str) -> Optional[Dict]:
        """Get progress for a specific chapter"""
        if not self.neon_client:
            await self.init()

        try:
            async with self.neon_client.pool.acquire() as conn:
                progress = await conn.fetchrow(
                    "SELECT * FROM user_progress WHERE user_id = $1 AND chapter_path = $2",
                    user_id, chapter_path
                )

                return dict(progress) if progress else None

        except Exception as e:
            print(f"[ERROR] Error getting progress: {str(e)}")
            raise

    async def get_all_progress(self, user_id: int) -> List[Dict]:
        """Get all progress for a user"""
        if not self.neon_client:
            await self.init()

        try:
            async with self.neon_client.pool.acquire() as conn:
                progress_list = await conn.fetch(
                    "SELECT * FROM user_progress WHERE user_id = $1 ORDER BY last_read_at DESC",
                    user_id
                )

                return [dict(p) for p in progress_list]

        except Exception as e:
            print(f"[ERROR] Error getting all progress: {str(e)}")
            raise

    async def get_progress_stats(self, user_id: int) -> Dict:
        """Get overall progress statistics for a user"""
        if not self.neon_client:
            await self.init()

        try:
            async with self.neon_client.pool.acquire() as conn:
                stats = await conn.fetchrow(
                    """
                    SELECT
                        COUNT(*) as total_chapters,
                        SUM(CASE WHEN is_completed THEN 1 ELSE 0 END) as completed_chapters,
                        SUM(time_spent_seconds) as total_time_seconds,
                        AVG(completion_percentage) as average_completion,
                        SUM(CASE WHEN completion_percentage > 0 AND NOT is_completed THEN 1 ELSE 0 END) as chapters_in_progress,
                        MAX(last_read_at) as last_activity
                    FROM user_progress
                    WHERE user_id = $1
                    """,
                    user_id
                )

                return {
                    "total_chapters": stats['total_chapters'] or 0,
                    "completed_chapters": stats['completed_chapters'] or 0,
                    "total_time_seconds": stats['total_time_seconds'] or 0,
                    "average_completion": float(stats['average_completion'] or 0),
                    "chapters_in_progress": stats['chapters_in_progress'] or 0,
                    "last_activity": stats['last_activity']
                }

        except Exception as e:
            print(f"[ERROR] Error getting progress stats: {str(e)}")
            raise


# Global instance
_auth_db_service: Optional[AuthDatabaseService] = None


async def get_auth_db() -> AuthDatabaseService:
    """Get or create global auth database service"""
    global _auth_db_service

    if _auth_db_service is None:
        _auth_db_service = AuthDatabaseService()
        await _auth_db_service.init()

    return _auth_db_service
