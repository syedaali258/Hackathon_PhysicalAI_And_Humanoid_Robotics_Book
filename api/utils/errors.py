from fastapi import HTTPException, status
from typing import Optional
import logging

logger = logging.getLogger(__name__)


class AIBotException(HTTPException):
    """Base exception class for the AI book application."""

    def __init__(self, detail: str, status_code: int = status.HTTP_400_BAD_REQUEST):
        super().__init__(status_code=status_code, detail=detail)
        logger.error(f"AIBotException: {detail} (Status: {status_code})")


class ContentNotFoundException(AIBotException):
    """Raised when requested content is not found."""

    def __init__(self, content_id: str, detail: Optional[str] = None):
        if detail is None:
            detail = f"Content with ID '{content_id}' not found"
        super().__init__(detail=detail, status_code=status.HTTP_404_NOT_FOUND)


class ChatSessionNotFoundException(AIBotException):
    """Raised when a chat session is not found."""

    def __init__(self, session_id: str):
        detail = f"Chat session with ID '{session_id}' not found"
        super().__init__(detail=detail, status_code=status.HTTP_404_NOT_FOUND)


class RAGException(AIBotException):
    """Raised when there's an issue with RAG (Retrieval Augmented Generation)."""

    def __init__(self, detail: str):
        super().__init__(detail=f"RAG Error: {detail}", status_code=status.HTTP_500_INTERNAL_SERVER_ERROR)


class OpenAIException(AIBotException):
    """Raised when there's an issue with the OpenAI API."""

    def __init__(self, detail: str):
        super().__init__(detail=f"OpenAI API Error: {detail}", status_code=status.HTTP_500_INTERNAL_SERVER_ERROR)


def handle_error(error: Exception, context: str = "") -> AIBotException:
    """
    Generic error handler that converts exceptions to appropriate AIBotException types.
    """
    error_msg = f"Error in {context}: {str(error)}" if context else str(error)

    if isinstance(error, AIBotException):
        # If it's already an AIBotException, just return it
        return error
    elif isinstance(error, HTTPException):
        # If it's already an HTTPException, wrap it in AIBotException
        return AIBotException(detail=error.detail, status_code=error.status_code)
    else:
        # For other exceptions, return a generic error
        logger.error(error_msg, exc_info=True)
        return AIBotException(
            detail=f"An unexpected error occurred: {str(error)}",
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR
        )