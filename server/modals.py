import uuid
import datetime
from dotenv import load_dotenv
import os
from sqlalchemy import create_engine, Column, String, Integer, DateTime, ForeignKey
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker, relationship

load_dotenv()

# Database setup
DATABASE_URL = os.getenv("DATABASE_URL", os.getenv("NEON_DATABASE_URL", "postgresql://user:password@localhost/dbname"))
engine = create_engine(DATABASE_URL)

SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)
Base = declarative_base()

# JWT Configuration


# Database Models
class User(Base):
    __tablename__ = "users"

    id = Column(String, primary_key=True, default=lambda: str(uuid.uuid4()))
    name = Column(String, nullable=False)
    email = Column(String, unique=True, nullable=False)
    password = Column(String, nullable=False)
    last_login = Column(DateTime, default=datetime.datetime.utcnow())
    level_of_experience = Column(Integer, nullable=False)  # 1, 2, or 3

    # Relationship with sessions
    sessions = relationship("SessionDB", back_populates="user", cascade="all, delete-orphan")


class SessionDB(Base):
    __tablename__ = "sessions"

    id = Column(String, primary_key=True, default=lambda: str(uuid.uuid4()))
    user_id = Column(String, ForeignKey("users.id"), nullable=False)
    name = Column(String, nullable=False, default=lambda: f"Session {datetime.datetime.now().strftime('%Y-%m-%d %H:%M')}")
    created_at = Column(DateTime, default=datetime.datetime.utcnow())
    updated_at = Column(DateTime, default=datetime.datetime.utcnow(), onupdate=datetime.datetime.utcnow())

    # Relationship with user and chats
    user = relationship("User", back_populates="sessions")
    chats = relationship("Chat", back_populates="session", cascade="all, delete-orphan")


class Chat(Base):
    __tablename__ = "chats"

    id = Column(String, primary_key=True, default=lambda: str(uuid.uuid4()))
    session_id = Column(String, ForeignKey("sessions.id"), nullable=False)
    question = Column(String, nullable=False)
    answer = Column(String, nullable=False)
    created_at = Column(DateTime, default=datetime.datetime.utcnow())

    # Relationship with session
    session = relationship("SessionDB", back_populates="chats")

# Create tables
Base.metadata.create_all(bind=engine)