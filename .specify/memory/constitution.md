<!-- Sync Impact Report:
Version change: N/A -> 1.0.0
Modified principles: N/A (new constitution)
Added sections: All sections
Removed sections: None
Templates requiring updates: N/A (new file)
Follow-up TODOs: None
-->

# Project Constitution

## Version and Ratification
- **Constitution Version:** 1.0.0
- **Ratification Date:** 2025-12-07
- **Last Amended Date:** 2025-12-07

## Core Principles

### Principle 1: Technical Depth and Engineering Accuracy
**Rule:** All robotics, ROS2, simulation, and hardware claims must be cross-verified with authoritative sources (ROS2 docs, Gazebo/Ignition docs, Isaac/Isaac Sim docs, NVIDIA technical papers, reputable robotics labs).
**Rationale:** Ensures the textbook maintains high technical standards and provides reliable information for engineering students.

### Principle 2: AI-Native Workflow Consistency
**Rule:** Maintain consistent workflow using Claude Code CLI + Qwen Code API + SpecKit-Plus throughout the project lifecycle.
**Rationale:** Creates predictable development patterns and leverages AI-native tools for maximum productivity and consistency.

### Principle 3: Modular and Reusable Content
**Rule:** All content must be modular and reusable, aligned with the Docusaurus `/docs` structure.
**Rationale:** Enables flexible content organization and supports the hierarchical documentation structure required by Docusaurus.

### Principle 4: Pedagogical Clarity
**Rule:** Content must be designed for beginner-to-intermediate engineering students with clear, instructor-style explanations.
**Rationale:** Ensures accessibility for the target audience while maintaining technical accuracy and educational effectiveness.

### Principle 5: Hands-On Orientation
**Rule:** Every concept must be tied to code, simulation, or hardware practice with executable examples.
**Rationale:** Promotes practical learning and ensures students can apply theoretical concepts in real-world scenarios.

## Key Standards

### Technical Standards
- Mathematical formulations must be accurate and unit-consistent
- Diagrams generated via AI tools must be checked for correctness before inclusion
- Code samples must be executable and tested (ROS2 Foxy/Humble standard, Gazebo/Ignition compatible)
- Terminology must match the glossary section vocabulary for global consistency
- Content should reference the specific folder/module where it belongs (e.g., ROS2 explanations → `/module-1-ros2/*`)

### Writing Standards
- Writing tone: engineering clarity, instructor-style explanations, no conversational filler
- All modules must include at least one hands-on example, one diagram, and one exercise section
- Safety-critical topics (robot control loops, locomotion, torque limits) must include warnings
- Any LLM-generated technical content must be manually validated for correctness
- Format must remain compatible with Docusaurus (MDX-safe)

## Success Criteria

### Coverage Requirements
- Full coverage of every file and nested route in the `/docs` folder structure
- Zero technical inaccuracies in ROS2, Gazebo, physics simulation, Isaac Sim, VLA pipelines

### Educational Requirements
- All exercises runnable by students with mid-tier hardware or cloud lab options
- Capstone architecture aligns with earlier modules (ROS2 → Simulation → Isaac → VLA)

### Quality Requirements
- Glossary definitions remain consistent and cross-referenced
- Book passes internal technical review by at least one robotics engineer

## Governance

### Amendment Procedure
Changes to this constitution require:
1. Technical review by subject matter expert
2. Approval from project maintainers
3. Update to version number according to semantic versioning

### Versioning Policy
- MAJOR: Backward incompatible governance/principle removals or redefinitions
- MINOR: New principle/section added or materially expanded guidance
- PATCH: Clarifications, wording, typo fixes, non-semantic refinements

### Compliance Review
Regular reviews should verify:
- All content aligns with established principles
- Technical accuracy is maintained
- Educational objectives are met
- Structural requirements are satisfied