from pydantic import BaseModel
from typing import List, Optional, Dict, Any
from datetime import datetime


class UserSession(BaseModel):
    id: str
    user_id: Optional[str] = None
    start_time: datetime
    end_time: Optional[datetime] = None
    current_location: Optional[Dict[str, str]] = {}
    progress: List[Dict[str, Any]] = []
    metadata: Dict[str, Any] = {}