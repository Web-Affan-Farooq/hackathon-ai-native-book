/sp.plan Create: file sequence, section outline, example strategy, and validation checks for the chatbot backend.

Decisions to document:
- Database connection pooling strategy with Neon PostgreSQL
- JWT token expiration time (likely 24 hours for user sessions)
- Password hashing algorithm (bcrypt with 12 rounds)
- Session naming convention (auto-generated with timestamp)

Testing strategy:
- Verify all 5 endpoints work correctly with proper authentication
- Test database schema creation and relationships
- Validate JWT token handling and http-only cookie security
- Test AI integration with the existing agent.py functionality
- Ensure proper error handling with correct status codes

Technical details:
- Research-while-writing workflow for FastAPI best practices
- Phase order:
  1. Database models and schema setup
  2. JWT authentication utilities
  3. User registration and login endpoints
  4. Session and chat management endpoints
  5. AI integration for the /ask endpoint
  6. Testing and validation
- Use async/await for all database operations
- Implement proper password hashing with bcrypt
- Ensure secure cookie handling for JWT tokens