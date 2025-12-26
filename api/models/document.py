from pydantic import BaseModel
from typing import List, Optional, Dict, Any
from datetime import datetime
from enum import Enum


class ContentType(str, Enum):
    module = "module"
    chapter = "chapter"
    section = "section"
    example = "example"
    exercise = "exercise"


class BookContent(BaseModel):
    id: str
    type: ContentType
    title: str
    slug: str
    content: str
    metadata: Dict[str, Any] = {}
    hierarchy: Dict[str, Any] = {}
    references: List[str] = []
    created_at: Optional[datetime] = None
    updated_at: Optional[datetime] = None


class ContentChunk(BaseModel):
    id: str
    content_id: str
    text: str
    embedding: Optional[List[float]] = []
    chunk_index: int
    metadata: Dict[str, Any] = {}
    created_at: Optional[datetime] = None