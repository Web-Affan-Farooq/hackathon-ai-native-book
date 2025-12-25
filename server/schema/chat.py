from pydantic import BaseModel , field_validator
from typing import Optional

class AskPayload(BaseModel):
    session_id: Optional[str] = None
    question: str
    user_selected_text: str

    @field_validator('question')
    def validate_question(cls, v):
        if not v.strip():
            raise ValueError('Question cannot be empty')
        return v

class GetChatsPayload(BaseModel):
    session_id: str

    @field_validator('session_id')
    def validate_session_id(cls, v):
        if not v.strip():
            raise ValueError('Session ID cannot be empty')
        return v