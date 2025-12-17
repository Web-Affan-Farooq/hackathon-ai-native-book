

# 📘 Hackathon Master Document (For NotebookLM)
** Project: AI-Powered Book + Integrated RAG Chatbot**

## 1. Project Overview
This project requires building two major things:

### A. AI/Spec-Driven Book
- Write a full book using Docusaurus.
- Deploy it publicly on GitHub Pages.

### B. Embedded RAG Chatbot inside the book
The chatbot must:

- Use FastAPI, Qdrant Cloud, and OpenAI/ChatKit Agents SDKs.
- Answer questions about your book only.
- Support select-text mode:
- The user selects text → chatbot answers using only that selection + retrieved chunks.


### Bonus Marks
- Implement Claude Code Subagents.
- Implement Reusable Skills similar to Matrix “load helicopter program” idea.



## 2. Required Tech Stack
### Frontend / Book
- Docusaurus (React + Markdown book framework)
- GitHub Pages (hosting for book)
- React chat widget embedded in pages


### Backend
- FastAPI (Python backend + RAG pipeline)
- Qdrant Cloud Free Tier (vector DB)
- Python qdrant-client
- Embeddings: Any free LLM provider or local embedding model
- OpenAI Agents / ChatKit SDKs or Claude Code (recommended by project) for agent logic and subagents


### AI / RAG
- Document chunking (500–1000 tokens)
- Embedding generator
- Vector search via Qdrant
- Context packaging for the model
- Strict system prompts for “answer ONLY from book”


### DevOps
- GitHub Actions (deployment)
- Optional Docker

## 3. High-Level Architecture
                ┌────────────────────────────┐
                │        Docusaurus Book      │
                │ (hosted on GitHub Pages)    │
                └─────────────┬──────────────┘
                              │
                Embedded Chat Widget (React)
                              │
                              ▼
                 ┌────────────────────────────┐
                 │ FastAPI Backend (RAG API)  │
                 │   - /query endpoint        │
                 │   - selection mode         │
                 └─────────────┬──────────────┘
                               │
                               ▼
                    ┌──────────────────┐
                    │ Qdrant Cloud DB  │
                    │ (vector search)  │
                    └──────────────────┘
                               │
                               ▼
                 ┌────────────────────────────┐
                 │  LLM Agent (Claude/OpenAI) │
                 │  - Tools / Subagents       │
                 │  - Reusable Skills         │
                 └────────────────────────────┘


## 4. Detailed Steps You Must Build
### Step 1 — Write the Book


- Use Docusaurus.
- Chapters as Markdown.
- Can add React components inside chapters.
- Add a floating chat button UI on every page.


### Step 2 — Preprocess Book for RAG
- Write a Python script that:
- Reads Markdown files.
- Splits into 500–1000 token chunks.
- Generates embeddings.
- Pushes them into Qdrant.
- Run this script on every update (via GitHub Actions).

### Step 3 — FastAPI Backend
Endpoints: `POST /query`
Input:
```json
{
  "question": "What does chapter 3 say about X?",
  "selected_text": "optional user highlighted text",
  "page_id": "chapter-3"
}
```

Output:
```json
{
  "answer": "...",
  "sources": ["chapter-3", "chapter-2"]
}
```

Backend flow:
- Embed user question.
- If selected_text present → embed and boost in retrieval.
- Query Qdrant for relevant chunks.
- Call LLM agent with strict instructions.
- Return answer + citations.

## Step 4 — Chat Widget
Features:
- Floating button
- Message history UI
- Selection detector using: `window.getSelection().toString()`

- Sends selection + question to backend
- Display response with sources

## Step 5 — Agent Logic
The agent must:
- Answer only from provided context
- Reject questions outside book content

Use:
- OpenAI Agents/ChatKit Claude Code with tools/subagents


## 5. Subagents & Reusable Skills (Bonus)
Example Subagents
- Summarizer Agent
- Chapter Index Agent
- Example Generator Agent
- Citation Finder Agent


- Example Skill (Matrix-style)
A pre-defined JSON spec:
```json
{
  "skill": "chapter-summary",
  "input": "chapter name",
  "output": "short summary of chapter"
}
```
- Main agent loads skills as needed.

6. What NotebookLM Should Help You With

Once you upload this document, ask NotebookLM:
- “Explain how all parts of the architecture connect.”
- “Explain the RAG pipeline in simple words.”
- “Teach me how FastAPI talks to Qdrant.”
- “Teach me how to embed a chatbot inside Docusaurus.”
- “Give me step-by-step coding plan.”
- “Make me a learning path for the hackathon.”

7. Deliverable Checklist
- ✔ Docusaurus book
- ✔ Page-embedded chat widget
- ✔ FastAPI backend
- ✔ Qdrant vector store
- ✔ RAG pipeline (chunk → embed → retrieve → answer)
- ✔ Selection-based answering
- ✔ GitHub Pages deployment
- ✔ Bonus: subagents + skills
- ✔ Clean architecture diagram

8. Extra Notes
- Keep all API keys on the server side.
- Use free models during dev.
- Auto-deploy on every push.