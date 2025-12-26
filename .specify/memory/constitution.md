<!--
SYNC IMPACT REPORT
Version change: N/A (initial version) → 1.0.0
Modified principles: N/A (new principles added)
Added sections: All principles and sections (new constitution)
Removed sections: None (new file)
Templates requiring updates:
  - .specify/templates/plan-template.md ✅ updated (constitution check section will now reference new principles)
  - .specify/templates/spec-template.md ✅ updated (requirements will align with new principles)
  - .specify/templates/tasks-template.md ✅ updated (task categorization reflects new principles)
  - No command files to update
Follow-up TODOs: None
-->

# AI-native book on Physical AI & Humanoid Robotics with embedded RAG chatbot Constitution

## Core Principles

### Spec-driven, AI-native development
All development follows spec-driven methodology with AI-native tools; Every feature and change starts with a formal specification; AI tools like Claude Code must be integrated into the development workflow

### Technical accuracy from official sources
All content and code must be sourced from official documentation and verified sources; Technical claims must be reproducible and traceable to authoritative references; No hallucination or unverified information allowed

### Clear, modular content for AI/robotics learners
Content must be structured in a modular way that facilitates learning; Documentation should be accessible to AI/robotics learners at different skill levels; Clear separation of concepts with proper prerequisites

### Reproducible and traceable code and claims
All code examples must be runnable and produce documented results; Claims and statements must be traceable to source material; Changes must be version-controlled and documented

### Zero hallucination constraint
RAG chatbot must only respond based on book content or user-selected text; No generation of information outside the trained knowledge base; Strict adherence to source-based responses

### Deploy on GitHub Pages
All deliverables must be deployable on GitHub Pages; Deployment process must be automated and reliable; Static site generation required for accessibility

## Technology Stack Requirements

Built with Docusaurus via Spec-Kit Plus and Claude Code; Use OpenAI Agents/ChatKit, FastAPI, Neon Postgres, Qdrant; All technologies must support static deployment to GitHub Pages

## Success Criteria

Book deploys successfully to GitHub Pages; Chatbot answers accurately with zero hallucination; All content meets technical accuracy standards

## Governance

Constitution supersedes all other practices; Amendments require documentation and approval; All PRs/reviews must verify compliance with principles; Changes must maintain zero hallucination guarantee

**Version**: 1.0.0 | **Ratified**: 2025-12-20 | **Last Amended**: 2025-12-20