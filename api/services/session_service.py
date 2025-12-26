import asyncio
import uuid
from datetime import datetime
from typing import Dict, Any, Optional
from api.models.user import UserSession
from api.utils.logging import logger
from api.utils.errors import AIBotException


class SessionService:
    """Service for managing user sessions, including URDF-specific functionality."""

    def __init__(self):
        # In-memory storage for sessions (in production, use a database)
        self.sessions: Dict[str, UserSession] = {}
        self.user_sessions: Dict[str, list] = {}  # user_id -> [session_ids]

    async def create_session(self, user_id: Optional[str] = None, initial_location: Optional[Dict[str, str]] = None) -> UserSession:
        """Create a new user session."""
        session_id = str(uuid.uuid4())

        session = UserSession(
            id=session_id,
            user_id=user_id,
            start_time=datetime.now(),
            current_location=initial_location or {},
            progress=[],
            metadata={}
        )

        self.sessions[session_id] = session

        # Track user's sessions
        if user_id:
            if user_id not in self.user_sessions:
                self.user_sessions[user_id] = []
            self.user_sessions[user_id].append(session_id)

        logger.info(
            f"New session created",
            extra={
                "session_id": session_id,
                "user_id": user_id
            }
        )

        return session

    async def get_session(self, session_id: str) -> UserSession:
        """Get a user session by ID."""
        if session_id not in self.sessions:
            raise AIBotException(f"Session {session_id} not found", status_code=404)

        return self.sessions[session_id]

    async def update_session_location(self, session_id: str, location: Dict[str, str]) -> UserSession:
        """Update the current location in a session."""
        session = await self.get_session(session_id)
        session.current_location = location
        session.progress.append({
            "contentId": location.get("moduleId", "") + "/" + location.get("chapterId", ""),
            "completionStatus": "in_progress",
            "timeSpent": 0,  # Would be calculated in a real implementation
            "lastAccessed": datetime.now().isoformat()
        })

        logger.info(
            f"Session location updated",
            extra={
                "session_id": session_id,
                "location": location
            }
        )

        return session

    async def update_session_metadata(self, session_id: str, metadata: Dict[str, Any]) -> UserSession:
        """Update session metadata."""
        session = await self.get_session(session_id)

        # Update existing metadata with new values
        session.metadata.update(metadata)

        logger.info(
            f"Session metadata updated",
            extra={
                "session_id": session_id,
                "updated_fields": list(metadata.keys())
            }
        )

        return session

    async def end_session(self, session_id: str) -> UserSession:
        """End a user session."""
        session = await self.get_session(session_id)
        session.end_time = datetime.now()

        logger.info(
            f"Session ended",
            extra={
                "session_id": session_id,
                "duration": (session.end_time - session.start_time).total_seconds() if session.start_time else 0
            }
        )

        return session

    async def get_user_sessions(self, user_id: str) -> list:
        """Get all sessions for a specific user."""
        return self.user_sessions.get(user_id, [])


# Singleton instance
session_service = SessionService()