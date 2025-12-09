```markdown
/sp.specify  Generate the full content for the "hardware-requirements" chapter of the textbook.
Markdown files to create inside:
`./textbook/docs/hardware-requirements`

Files:
- index.md
- workstation.md
- edge-ai-kit.md

Content requirements:
- Explain hardware requirements for the course’s ROS2, Gazebo, Isaac Sim, RL, and VLA modules.
- Provide clear specs, recommended builds, minimum vs. ideal requirements.
- Add tables, bullet points, and structured sections.
- Keep paragraphs short and instructional.
- Include guidance for students with low budgets or shared resources.
- Include GPU selection guidance (NVIDIA-focused), CPU, RAM, storage, peripherals.
- In edge-ai-kit.md, cover Jetson hardware, accelerators, sensors, and deployment workflow.
```

```markdown
/sp.plan   Create a concise execution plan for generating the hardware-requirements chapter:

1. Must use the `textbook-writer` sub agent.
6. Final validation: ensure all files exist, content is complete, and formatting is consistent.

```