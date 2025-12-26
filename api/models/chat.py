from pydantic import BaseModel
from typing import List, Optional, Dict, Any
from datetime import datetime
from enum import Enum


class SenderType(str, Enum):
    user = "user"
    assistant = "assistant"


class ChatMessage(BaseModel):
    id: str
    session_id: str
    timestamp: datetime
    sender: SenderType
    content: str
    source_chunks: Optional[List[Dict[str, Any]]] = []
    confidence: Optional[float] = None
    follow_up_suggestions: Optional[List[str]] = []


class ChatRequest(BaseModel):
    message: str
    context: Optional[str] = None


class ChatResponse(BaseModel):
    response: str
    sources: List[Dict[str, Any]]
    follow_up: List[str]
    timestamp: datetime
    confidence: float


class ChatSession(BaseModel):
    id: str
    user_id: Optional[str] = None
    created_at: datetime
    updated_at: datetime
    title: Optional[str] = None
    messages: List[ChatMessage] = []