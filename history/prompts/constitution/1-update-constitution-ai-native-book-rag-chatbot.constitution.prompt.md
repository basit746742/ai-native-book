---
id: 1
title: Update Constitution AI-Native Book RAG Chatbot
stage: constitution
date: 2025-12-28
surface: agent
model: Claude Sonnet 4.5
feature: none
branch: master
user: user
command: /sp.constitution
labels: ["constitution", "spec-first", "ai-native", "rag-chatbot"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
tests:
 - none
---

## Prompt

```text
Project: AI-Native Technical Book + Embedded RAG Chatbot

Goal:
Build and publish a spec-driven technical book (Docusaurus, GitHub Pages)
with an embedded RAG chatbot that answers strictly from book content,
including user-selected text.

Principles:
- Spec-first (Spec-Kit Plus is source of truth)
- AI-native authoring (Claude Code)
- Accuracy, reproducibility, deployability
- No hallucinations; clear separation of concerns

Deliverables:
- Docusaurus book, modular chapters, diagrams, illustrative code
- Embedded RAG chatbot

Stack:
- Frontend: Docusaurus
- Backend: FastAPI
- AI: OpenAI Agents / ChatKit
- DB: Neon Serverless Postgres
- Vector DB: Qdrant Cloud

Constraints:
- RAG answers must cite source sections
- Selected-text queries limited to that text only
- No external or unstated knowledge

Success:
- Public GitHub Pages site
- Specs traceable to implementation
- End-to-end demo works correctly
```

## Response snapshot

Updated the project constitution to reflect the specific requirements of the AI-Native Technical Book with Embedded RAG Chatbot project. The new constitution includes core principles focused on spec-first development, AI-native authoring with Claude Code, accuracy with no hallucinations, reproducibility, clear separation of concerns, and modular scalable architecture. The document also includes specific technology stack requirements, performance standards for RAG functionality, content quality standards, and governance procedures.

## Outcome

- ✅ Impact: Updated constitution now properly reflects project-specific requirements and principles
- 🧪 Tests: No tests required for constitution update
- 📁 Files: Updated .specify/memory/constitution.md with project-specific content
- 🔁 Next prompts: Ready to proceed with spec creation and planning
- 🧠 Reflection: Constitution now properly aligned with AI-Native Technical Book + RAG Chatbot project requirements

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A