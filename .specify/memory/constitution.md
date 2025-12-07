<!-- Sync Impact Report:
Version change:  → 1.0.0
Modified principles: None
Added sections: Book Standards, RAG Chatbot Standards, Robotics Curriculum Constraints
Removed sections: None
Templates requiring updates:
- .specify/templates/plan-template.md ⚠ pending
- .specify/templates/spec-template.md ⚠ pending
- .specify/templates/tasks-template.md ⚠ pending
- .specify/templates/commands/sp.constitution.md ⚠ pending
- .specify/templates/commands/sp.adr.md ⚠ pending
- .specify/templates/commands/sp.analyze.md ⚠ pending
- .specify/templates/commands/sp.checklist.md ⚠ pending
- .specify/templates/commands/sp.clarify.md ⚠ pending
- .specify/templates/commands/sp.git.commit_pr.md ⚠ pending
- .specify/templates/commands/sp.implement.md ⚠ pending
- .specify/templates/commands/sp.phr.md ⚠ pending
- .specify/templates/commands/sp.plan.md ⚠ pending
- .specify/templates/commands/sp.specify.md ⚠ pending
- .specify/templates/commands/sp.tasks.md ⚠ pending
Follow-up TODOs: TODO(RATIFICATION_DATE): Clarify original ratification date if different from project start.
-->
# Book + Integrated RAG Chatbot on Physical AI & Humanoid Robotics Constitution

## Core Principles

### I. High Technical Accuracy
All content, especially regarding ROS 2, Gazebo, Unity, Isaac Sim, VLA, and RAG, must be technically accurate and verifiable.

### II. Clear Writing for Target Audience
Content must be written clearly and accessibly for Computer Science, AI, and Robotics students.

### III. Reproducible Code and Architectures
All code and architectural designs presented must be reproducible and runnable.

### IV. Source-Verified Facts & Originality
All factual claims must be source-verified with APA citations, ensuring 0% plagiarism.

## Book Standards

- Docusaurus v3, deployed to GitHub Pages
- 20k–30k words
- Required modules: ROS 2, Gazebo/Unity, Isaac Sim, VLA, Capstone
- Each chapter: Concepts → Architecture → Code → Lab → Quiz
- All code must run on: ROS 2 Humble, Gazebo Fortress, Unity 2022+, Isaac Sim

## RAG Chatbot Standards

- Stack: FastAPI + Qdrant Cloud + Neon Postgres + OpenAI Agents/ChatKit SDK
- Must support page-level and selected-text Q&A
- Retrieval-first, hallucination-resistant design
- Target: ≥85% retrieval accuracy, <2s latency

## Robotics Curriculum Constraints

- ROS 2 nodes/topics/services, URDF for humanoids
- Gazebo & Unity physics + sensor simulation
- Isaac Sim: perception, VSLAM, Nav2
- VLA: Whisper → L

## Governance

This constitution outlines the fundamental principles and standards for the project. Any amendments to this constitution require careful consideration, documentation, and approval from stakeholders. All project artifacts and development practices must align with these principles.

**Version**: 1.0.0 | **Ratified**: 2025-12-07 | **Last Amended**: 2025-12-07
