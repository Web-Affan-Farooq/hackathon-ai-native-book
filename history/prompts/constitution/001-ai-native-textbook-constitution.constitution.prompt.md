---
id: 001
title: AI-native textbook constitution
stage: constitution
date: 2025-12-07
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: main
user: affan
command: /sp.constitution
labels: [constitution, textbook, robotics, ai-native]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
  - .specify/templates/plan-template.md
  - .specify/templates/spec-template.md
  - .specify/templates/tasks-template.md
tests:
  - null
---

## Prompt

```
    ## Project: AI-native textbook on Physical AI & Humanoid Robotics

### Core principles:
- Technical depth with engineering accuracy across robotics, ROS2, simulation, and embodied AI
- AI-native workflow consistency using Claude Code CLI + Qwen Code API + SpecKit-Plus
- Modular, reusable content blocks aligned with the Docusaurus `/docs` structure
- Pedagogical clarity (beginner-to-intermediate engineering students)
- Hands-on orientation: every concept tied to code, simulation, or hardware practice

### Key standards:
- All robotics, ROS2, simulation, and hardware claims must be cross-verified with authoritative sources
  (ROS2 docs, Gazebo/Ignition docs, Isaac/Isaac Sim docs, NVIDIA technical papers, reputable robotics labs)
- Mathematical formulations must be accurate and unit-consistent
- Diagrams generated via AI tools must be checked for correctness before inclusion
- Code samples must be executable and tested (ROS2 Foxy/Humble standard, Gazebo/Ignition compatible)
- Terminology must match the `glossary` section vocabulary for global consistency
- Content should reference the folder/module where it belongs
  (e.g., ROS2 explanations → `/module-1-ros2/*`)

### Constraints:
- Writing tone: engineering clarity, instructor-style explanations, no conversational filler
- All modules must include at least one hands-on example, one diagram, and one exercise section
- Safety-critical topics (robot control loops, locomotion, torque limits) must include warnings
- Any LLM-generated technical content must be manually validated for correctness
- Format must remain compatible with Docusaurus (MDX-safe)

### Success criteria:
- Full coverage of every file and nested route in the `/docs` folder structure
- Zero technical inaccuracies in ROS2, Gazebo, physics simulation, Isaac Sim, VLA pipelines
- All exercises runnable by students with mid-tier hardware or cloud lab options
- Capstone architecture aligns with earlier modules (ROS2 → Simulation → Isaac → VLA)
- Glossary definitions remain consistent and cross-referenced
- Book passes internal technical review by at least one robotics engineer
```

## Response snapshot

Created a comprehensive project constitution for the AI-native textbook on Physical AI & Humanoid Robotics. The constitution includes:

- 5 core principles covering technical depth, AI-native workflow, modularity, pedagogical clarity, and hands-on orientation
- Technical and writing standards for content creation
- Success criteria with coverage, educational, and quality requirements
- Governance procedures for amendments and versioning

Updated the template files to align with the textbook project:
- Modified plan-template.md to include constitution checks for technical accuracy, content quality, and educational requirements
- Updated spec-template.md with learning journeys instead of user stories, educational/technical requirements, and educational success criteria
- Modified tasks-template.md to use educational terminology (learning journeys, educational assessments) instead of software development terms

## Outcome

- ✅ Impact: Created foundational constitution for the Physical AI & Humanoid Robotics textbook project
- 🧪 Tests: N/A (documentation task)
- 📁 Files: Updated 4 files (.specify/memory/constitution.md, .specify/templates/plan-template.md, .specify/templates/spec-template.md, .specify/templates/tasks-template.md)
- 🔁 Next prompts: Follow-up work on textbook modules based on this constitution
- 🧠 Reflection: The templates were successfully adapted from software development to educational content creation terminology

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Begin creating textbook modules based on the established constitution