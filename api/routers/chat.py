from fastapi import APIRouter, HTTPException
from datetime import datetime
import uuid
from typing import Dict, Any
from ..models.chat import ChatRequest, ChatResponse, ChatSession
from ..services.rag_service import rag_service
from ..utils.logging import logger
from ..utils.errors import ChatSessionNotFoundException

router = APIRouter(prefix="/chat", tags=["chat"])


@router.post("/start", response_model=Dict[str, Any])
async def start_chat():
    """Start a new chat session."""
    session_id = str(uuid.uuid4())
    logger.info(f"New chat session started", extra={"session_id": session_id})

    return {
        "sessionId": session_id,
        "timestamp": datetime.now(),
        "message": "Hello! I'm your AI assistant for the Physical AI & Humanoid Robotics book. How can I help you with Module 1: The Robotic Nervous System?"
    }


@router.post("/{sessionId}/message", response_model=ChatResponse)
async def send_message(sessionId: str, request: ChatRequest):
    """Send a message to the chatbot and receive a response."""
    try:
        # Log the incoming message
        logger.info(
            f"Received message for session",
            extra={
                "session_id": sessionId,
                "message_length": len(request.message)
            }
        )

        # Add user message to chat history
        user_message = {
            "id": str(__import__('uuid').uuid4()),
            "sender": "user",
            "content": request.message,
            "timestamp": __import__('datetime').datetime.now().isoformat(),
            "sources": [],
            "context": request.context
        }

        if sessionId not in chat_sessions:
            chat_sessions[sessionId] = []
        chat_sessions[sessionId].append(user_message)

        # Generate response using RAG service
        response = await rag_service.generate_response(request, session_id)

        # Add assistant response to chat history
        assistant_message = {
            "id": str(__import__('uuid').uuid4()),
            "sender": "assistant",
            "content": response.response,
            "timestamp": response.timestamp.isoformat() if hasattr(response.timestamp, 'isoformat') else __import__('datetime').datetime.now().isoformat(),
            "sources": response.sources,
            "follow_up": response.follow_up
        }
        chat_sessions[sessionId].append(assistant_message)

        # Log successful response
        logger.info(
            f"Response generated successfully",
            extra={
                "session_id": sessionId,
                "response_length": len(response.response)
            }
        )

        return response

    except Exception as e:
        logger.error(
            f"Error processing chat message",
            extra={
                "session_id": sessionId,
                "error": str(e)
            },
            exc_info=True
        )
        raise HTTPException(status_code=500, detail=f"Error processing message: {str(e)}")


# In-memory storage for chat history (in production, use a database)
chat_sessions = {}

@router.get("/{sessionId}/history")
async def get_chat_history(sessionId: str, limit: int = 20, before: str = None):
    """Get chat session history."""
    try:
        logger.info(
            f"Retrieving chat history",
            extra={
                "session_id": sessionId,
                "limit": limit,
                "before": before
            }
        )

        # Check if session exists
        if sessionId not in chat_sessions:
            # Initialize an empty session if it doesn't exist
            chat_sessions[sessionId] = []

        # Get messages from the session
        session_messages = chat_sessions[sessionId]

        # Apply before filter if specified
        if before:
            import datetime
            try:
                before_time = datetime.datetime.fromisoformat(before.replace('Z', '+00:00'))
                session_messages = [
                    msg for msg in session_messages
                    if datetime.datetime.fromisoformat(msg['timestamp'].replace('Z', '+00:00')) < before_time
                ]
            except ValueError:
                raise HTTPException(status_code=400, detail="Invalid before timestamp format")

        # Apply limit and get the most recent messages
        if limit > 0:
            messages = session_messages[-limit:]
        else:
            messages = session_messages

        # Determine if there are more messages available
        has_more = len(session_messages) > len(messages)

        logger.info(
            f"Retrieved chat history",
            extra={
                "session_id": sessionId,
                "returned_count": len(messages),
                "total_count": len(session_messages)
            }
        )

        return {
            "messages": messages,
            "hasMore": has_more,
            "total": len(session_messages)
        }

    except HTTPException:
        # Re-raise HTTP exceptions
        raise
    except Exception as e:
        logger.error(
            f"Error retrieving chat history",
            extra={
                "session_id": sessionId,
                "error": str(e)
            },
            exc_info=True
        )
        raise HTTPException(status_code=500, detail=f"Error retrieving chat history: {str(e)}")


@router.post("/{sessionId}/end")
async def end_chat(sessionId: str):
    """End a chat session."""
    logger.info(f"Ending chat session", extra={"session_id": sessionId})

    return {
        "message": "Session ended successfully",
        "duration": 0
    }