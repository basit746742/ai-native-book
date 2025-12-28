<!--
Sync Impact Report:
- Version change: 1.0.0 → 2.0.0 (MAJOR version bump due to complete rewrite of principles and governance structure)
- Modified principles: All principles were replaced with project-specific ones
  - Old: "AI-Powered Book Creation" → New: "Spec-First Development (NON-NEGOTIABLE)"
  - Old: "User-Centric Design" → New: "AI-Native Authoring with Claude Code"
  - Old: "Test-First (NON-NEGOTIABLE)" → New: "Accuracy and No Hallucinations (NON-NEGOTIABLE)"
  - Old: "Content Quality Assurance" → New: "Reproducibility and Deployability"
  - Old: "Data Privacy and Security" → New: "Clear Separation of Concerns"
  - Old: "Scalable Architecture" → New: "Modular and Scalable Architecture"
- Added sections: Technology Stack Requirements (project-specific), Content Quality Standards, Performance Standards (RAG-specific)
- Removed sections: Generic technology requirements replaced with project-specific stack
- Templates requiring updates:
  - ✅ `.specify/templates/plan-template.md` - Constitution Check section will now reference new principles
  - ⚠️ `.specify/templates/spec-template.md` - No direct references to update
  - ⚠️ `.specify/templates/tasks-template.md` - No direct references to update
  - ⚠️ `.specify/templates/commands/sp.constitution.md` - Command file may need review
- Follow-up TODOs: None intentionally deferred
-->
# AI-Native Technical Book + Embedded RAG Chatbot Constitution

## Core Principles

### I. Spec-First Development (NON-NEGOTIABLE)
All development must follow the spec-driven approach with Spec-Kit Plus as the source of truth. All features, architecture decisions, and tasks must be traced back to specifications in the spec repository. No code implementation without corresponding spec documentation.

### II. AI-Native Authoring with Claude Code
The development process must leverage Claude Code for all implementation work. All features should be designed to work seamlessly with AI-assisted development workflows. Human developers focus on architecture, requirements, and validation while AI handles implementation details.

### III. Accuracy and No Hallucinations (NON-NEGOTIABLE)
All RAG chatbot responses must strictly cite source sections from the book content. No external knowledge or hallucinations are permitted. The system must clearly indicate when information is unavailable in the source materials. Selected-text queries must be limited to the specified text only.

### IV. Reproducibility and Deployability
All development artifacts must be reproducible across environments. The system must be deployable to GitHub Pages with clear deployment procedures. All dependencies and configurations must be documented and version-controlled.

### V. Clear Separation of Concerns
Architecture must maintain clear boundaries between frontend (Docusaurus), backend (FastAPI), AI services (OpenAI Agents/ChatKit), database (Neon Serverless Postgres), and vector database (Qdrant Cloud). Each component must have well-defined interfaces and responsibilities.

### VI. Modular and Scalable Architecture
The system must support modular chapters, diagrams, and illustrative code. The architecture should allow for easy addition of new content sections and maintain performance as the book grows. The RAG system must scale with increasing content volume.

## Additional Constraints

### Technology Stack Requirements
- Frontend: Docusaurus for documentation and GitHub Pages hosting
- Backend: FastAPI for API services and RAG functionality
- AI: OpenAI Agents/ChatKit for RAG processing
- Database: Neon Serverless Postgres for structured data
- Vector Database: Qdrant Cloud for content indexing and retrieval
- Deployment: GitHub Actions CI/CD pipeline to GitHub Pages

### Performance Standards
- RAG response times under 3 seconds for standard queries
- 99.9% uptime for chatbot functionality
- Support for concurrent users based on projected growth
- Page load times under 3 seconds on GitHub Pages
- Vector search response times under 1 second

### Content Quality Standards
- All book content must be technically accurate and well-documented
- Code examples must be tested and verified
- Diagrams must be clear and relevant to the content
- Source citations in RAG responses must be precise and verifiable

## Development Workflow

### Code Review Requirements
- All pull requests require at least one approval
- Automated tests must pass before review
- Code must follow established style guides for TypeScript/Python
- Security scanning must pass
- Spec compliance verification required
- Performance impact must be considered

### Quality Gates
- Minimum 80% test coverage for all components
- Code complexity within acceptable limits
- No security vulnerabilities in dependencies
- Performance benchmarks met for RAG responses
- User acceptance criteria validated against spec
- Spec-to-implementation traceability verified

## Governance

This constitution supersedes all other development practices. All team members must follow these principles. Amendments require documentation of rationale, team approval, and migration plan if needed. All PRs and reviews must verify compliance with these principles. Complexity must be justified with clear benefits to the user experience. All changes must maintain spec-driven development practices and ensure no hallucinations in RAG responses.

**Version**: 2.0.0 | **Ratified**: 2025-01-01 | **Last Amended**: 2025-12-28