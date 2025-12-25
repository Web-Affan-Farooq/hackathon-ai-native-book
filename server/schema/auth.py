from pydantic import BaseModel , field_validator

class SignupPayload(BaseModel):
    name: str
    email: str
    password: str
    level_of_experience: int

    @field_validator('email')
    def validate_email(cls, v):
        if '@' not in v:
            raise ValueError('Email must be valid')
        return v

    @field_validator('level_of_experience')
    def validate_experience(cls, v):
        if v not in [1, 2, 3]:
            raise ValueError('Experience level must be 1, 2, or 3')
        return v

    @field_validator('password')
    def validate_password(cls, v):
        if len(v) < 6:
            raise ValueError('Password must be at least 6 characters')
        return v

class LoginPayload(BaseModel):
    email: str
    password: str

    @field_validator('email')
    def validate_email(cls, v):
        if '@' not in v:
            raise ValueError('Email must be valid')
        return v