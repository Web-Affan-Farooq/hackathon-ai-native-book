import os
from datetime import datetime, timedelta
from fastapi import FastAPI, Response, Request, HTTPException, Depends
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
from sqlalchemy.orm import Session
import bcrypt
import jwt
from dotenv import load_dotenv
from schema import SignupPayload , LoginPayload , AskPayload ,GetChatsPayload
from modals import User , SessionDB , Chat , SessionLocal 


# Import the agent function
from agent import run_agent

load_dotenv()

COOKIE_NAME="humanoid-robotics"

# JWT Configuration
SECRET_KEY = os.getenv("JWT_SECRET_KEY", "your-secret-key-change-in-production")
ALGORITHM = "HS256"
ACCESS_TOKEN_EXPIRE_MINUTES = 60 * 24  # 24 hours

# JWT Utility Functions
def create_access_token(data: dict):
    to_encode = data.copy()
    expire = datetime.utcnow() + timedelta(minutes=ACCESS_TOKEN_EXPIRE_MINUTES)
    to_encode.update({"exp": expire})
    encoded_jwt = jwt.encode(to_encode, SECRET_KEY, algorithm=ALGORITHM)
    return encoded_jwt

def verify_token(token: str):
    try:
        payload = jwt.decode(token, SECRET_KEY, algorithms=[ALGORITHM])
        return payload
    except Exception as _:
        return None

# Password Utility Functions
def hash_password(password: str):
    salt = bcrypt.gensalt()
    hashed = bcrypt.hashpw(password.encode('utf-8'), salt)
    return hashed.decode('utf-8')

def verify_password(plain_password: str, hashed_password: str):
    return bcrypt.checkpw(plain_password.encode('utf-8'), hashed_password.encode('utf-8'))

# Dependency to get DB session
def get_db():
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()

# CORS setup
origins = [
    "http://localhost:3000",
    "http://localhost:3001",
    "http://localhost:8080",
]

app = FastAPI()

app.add_middleware(
    middleware_class=CORSMiddleware,
    allow_origins=origins,
    allow_credentials=True,
    allow_headers="*",
    allow_methods="*"
)

# Helper function to get token from cookies
def get_token_from_cookies(request: Request):
    token = request.cookies.get("access_token")
    if not token:
        raise HTTPException(status_code=401, detail="Please login to your account")
    return token

# Routes
@app.post("/signup")
def signup_user(payload: SignupPayload, response: Response, db: Session = Depends(get_db)):
    try:
        # Validate payload data
        if not payload.name.strip():
            return JSONResponse(
                status_code=400, 
                content = {
                    "message":"Name cannot be empty"
                    }
            )
        
        # Check if user already exists
        existing_user = db.query(User).filter(User.email == payload.email).first()

        if existing_user:
            # User exists, check if password matches
            if verify_password(payload.password, str(existing_user.password)):
                # Password matches, log user in
                token_data = {
                    "id": existing_user.id,
                    "email": existing_user.email
                }
                token = create_access_token(token_data)

                response.set_cookie(
                    key="access_token",
                    value=token,
                    httponly=True,
                    max_age=86400,  # 24 hours
                    secure=False,  # Set to True in production with HTTPS
                    samesite="lax"
                )

                # Update last login
                existing_user.last_login = datetime.utcnow()
                db.commit()

                return {"message": "welcome user", "user_id": existing_user.id}
            else:
                # Password doesn't match
                return JSONResponse(
                status_code=400, 
                content = {
                    "message":"Account already exists. Please enter correct password or login instead"
                    }
            )
        else:
            # User doesn't exist, create new user
            hashed_password = hash_password(payload.password)

            new_user = User(
                name=payload.name,
                email=payload.email,
                password=hashed_password,
                level_of_experience=payload.level_of_experience,
                last_login=datetime.utcnow()
            )

            db.add(new_user)
            db.commit()
            db.refresh(new_user)

            # Create JWT token
            token_data = {
                "id": new_user.id,
                "email": new_user.email
            }
            token = create_access_token(token_data)

            response.set_cookie(
                key="access_token",
                value=token,
                httponly=True,
                max_age=86400,  # 24 hours
                secure=False,  # Set to True in production with HTTPS
                samesite="lax"
            )
 
            return JSONResponse(
                status_code=200, 
                content = {"message": "welcome user", "user_id": new_user.id}
            )
    except HTTPException:
        raise  # Re-raise HTTP exceptions
    except Exception as e:
        print(f"Error during signup: {e}")
        return JSONResponse(
                status_code=500, 
                content = {
                    "message":"An error occurred during signup"
                    }
            )

@app.post("/login")
def login_user(payload: LoginPayload, response: Response, db: Session = Depends(get_db)):
    try:
        # Find user by email
        user = db.query(User).filter(User.email == payload.email).first()
        if not user:
            return JSONResponse(
                status_code=404,
                content={
                    "message":"User not found"
                }
            )
        # Verify password
        if not verify_password(payload.password, str(user.password)):
            return JSONResponse(
                status_code=401, 
                content = {
                    "message":"Invalid credentials"
                    }
            )

        # Create JWT token
        token_data = {
            "id": user.id,
            "email": user.email
        }
        token = create_access_token(token_data)
        response.set_cookie(
            key="access_token",
            value=token,
            httponly=True,
            max_age=86400,  # 24 hours
            secure=False,  # Set to True in production with HTTPS
            samesite="lax"
        )

        # Update last login
        user.last_login = datetime.utcnow()
        db.commit()
        return {"message": "welcome user", "user_id": user.id, "name": user.name}
    except HTTPException:
        raise  # Re-raise HTTP exceptions
    except Exception as e:
        print(f"Error during login: {e}")
        return JSONResponse(
                status_code=500, 
                content = {
                    "message":"An error occured during login"
                    }
            )

@app.post("/ask")
async def chatbot_logic(payload: AskPayload, request: Request, response: Response, db: Session = Depends(get_db)):
    try:
        # Check if user has valid token
        token = get_token_from_cookies(request)
        payload_token = verify_token(token)

        if not payload_token:
            return JSONResponse(
                status_code=401, 
                content = {
                    "message":"Please login to your account"
                    }
            )

        # Get user's experience level to customize the response
        user = db.query(User).filter(User.id == payload_token["id"]).first()
        if not user:
            return JSONResponse(
                status_code=404, 
                content = {
                    "message":"User not found"
                    }
            )
        
        experience_level = user.level_of_experience

        # Format the question with selected text context and user's experience level
        formatted_question = f"""Question: {payload.question}

Context from textbook: {payload.user_selected_text}

User Experience Level: {experience_level} (1=beginner, 2=intermediate, 3=advanced)
Please tailor your response to match the user's experience level, providing appropriate depth and complexity of explanation."""

        # Run the agent to get AI response
        ai_response = await run_agent(formatted_question)

        # Check if session_id is provided
        session_id = payload.session_id

        if not session_id:
            # Create a new session
            new_session = SessionDB(
                user_id=payload_token["id"],
                name=f"Session {datetime.now().strftime('%Y-%m-%d %H:%M')}"
            )
            db.add(new_session)
            db.commit()
            db.refresh(new_session)
            session_id = new_session.id

        # Insert chat record
        new_chat = Chat(
            session_id=session_id,
            question=payload.question,
            answer=ai_response
        )
        db.add(new_chat)
        db.commit()
        db.refresh(new_chat)

        return {
            "message": "success",
            "chat": {
                "id": new_chat.id,
                "session_id": new_chat.session_id,
                "question": new_chat.question,
                "answer": new_chat.answer
            }
        }
    except HTTPException:
        raise  # Re-raise HTTP exceptions
    except Exception as e:
        print(f"Error in chatbot logic: {e}")
        return JSONResponse(
                status_code=500, 
                content = {
                    "message":"An error occurred while processing your question"
                    }
            )

@app.get("/get-sessions")
def get_sessions(request: Request, db: Session = Depends(get_db)):
    try:
        # Get token from cookies
        token = request.cookies.get("access_token")
        if not token:
            return JSONResponse(
                status_code=401, 
                content = {
                    "message":"Please login first"
                    }
            )
        
        # Verify token
        payload = verify_token(token)
        if not payload:
            return JSONResponse(
                status_code=500, 
                content = {
                    "message":"An error occurred while processing your question"
                    }
            )

        # Get user ID from token
        user_id = payload["id"]

        # Query sessions for the user
        sessions = db.query(SessionDB).filter(SessionDB.user_id == user_id).all()

        return {
            "message": "Success",
            "sessions": [
                {
                    "id": session.id,
                    "user_id": session.user_id,
                    "name": session.name,
                    "created_at": session.created_at.isoformat() if session.created_at else None
                }
                for session in sessions
            ]
        }
    except HTTPException:
        raise  # Re-raise HTTP exceptions
    except Exception as e:
        print(f"Error retrieving sessions: {e}")
        return JSONResponse(
                status_code=500, 
                content = {
                    "message":"An error occurred while retrieving sessions"
                    }
            )
    
@app.post("/get-chats")
def get_chats(payload: GetChatsPayload, db: Session = Depends(get_db)):
    try:
        # Validate session ID
        if not payload.session_id:
            return JSONResponse(
                status_code=400, 
                content = {
                    "message":"Session ID is required"
                    }
            )

        # Query chats for the given session
        chats = db.query(Chat).filter(Chat.session_id == payload.session_id).all()

        return {
            "message": "Success",
            "chats": [
                {
                    "id": chat.id,
                    "session_id": chat.session_id,
                    "question": chat.question,
                    "answer": chat.answer,
                    "created_at": chat.created_at.isoformat() if chat.created_at else None
                }
                for chat in chats
            ]
        }
    except HTTPException:
        raise  # Re-raise HTTP exceptions
    except Exception as e:
        print(f"Error retrieving chats: {e}")
        return JSONResponse(
                status_code=500, 
                content = {
                    "message":"An error occured whiole retrieving chats"
                    }
            )

@app.get("/logout")
def logout(response:Response):
    response.set_cookie(
        key=COOKIE_NAME,
        value="",  # Value can be empty
        max_age=0,  # Expires the cookie immediately
        httponly=True,
        secure=True, # Should be True if you use HTTPS in production
        path="/", # Must match the path the original cookie was set with
        samesite="strict" # Must match the SameSite policy
    )
    return JSONResponse(
        status_code=200,
        content={
            "message":"Logout successfully"
        }
    )

@app.get("/")
def home():
    return {
        "message": "Physical AI and Humanoid Robotics Chatbot Backend API",
        "version": "1.0.0",
        "endpoints": [
            "POST /signup - User registration",
            "POST /login - User login",
            "POST /ask - Chat with the AI assistant",
            "GET /get-sessions - Get user's chat sessions",
            "POST /get-chats - Get chats for a session"
        ]
    }


@app.get("/health")
def health_check():
    """Health check endpoint to verify the API is running"""
    return {
        "status": "healthy",
        "message": "Physical AI and Humanoid Robotics Chatbot Backend is running",
        "timestamp": datetime.utcnow().isoformat()
    }