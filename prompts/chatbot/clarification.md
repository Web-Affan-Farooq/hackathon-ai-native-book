/sp.clarify Please review the specification for the chatbot backend for Physical AI and Humanoid Robotics textbook

1. Ambiguities
   - What should be the exact format and expiration time for JWT tokens?
   - How should the level_of_experience values (1, 2, 3) be used in the AI responses?
   - What specific error handling strategy should be used for database connection failures?

2. Missing assumptions
   - Expected concurrent user load for the chatbot system
   - Required data backup and recovery procedures for the chat history
   - Specific compliance requirements for user data storage

3. Incomplete requirements
   - Should the session name be auto-generated or user-defined?
   - How long should user chat history be retained?
   - Should there be rate limiting on the /ask endpoint?

4. Scope conflicts
   - How much AI context should be preserved in the chat history for continuity?
   - Should the system support file uploads for additional context?
   - What happens to existing sessions when a user deletes their account?

Identify gaps to resolve before planning generation.