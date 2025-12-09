```markdown
/sp.specify    Generate the full content for the "capstone" chapter of the textbook.

Markdown files to create inside:
`./textbook/docs/capstone`

Content requirements:
- The capstone is a full humanoid Voice-to-Action robot pipeline integrating all previous modules (ROS2, Gazebo, Isaac, VLA, RL where needed).
- Provide a clear high-level overview and learning outcomes in index.md.
- architecture.md: system diagram (described in text), ROS2 graph, data flows, compute layout (workstation + edge kit).
- perception.md: sensors, camera pipelines, SLAM optional, object detection using Isaac ROS or custom models, calibration steps.
- manipulation.md: controllers, planners, IK, motion execution, safety constraints.
- complete-build-guide.md: end-to-end steps for assembling, simulating, deploying, testing; include folder structures, commands, and checkpoints.

```

```markdown
/sp.plan      Create a concise execution plan for generating the capstone chapter:

1. Use `textbook-writer` to create `./textbook/docs/capstone`.
2. Draft index.md with overview, goals, grading rubric summary, and module dependencies.
3. Generate architecture.md describing all components and dataflow.
4. Generate perception.md with sensors, pipelines, calibration, and models.
5. Generate manipulation.md with controllers, IK, planning, and execution flow.
6. Generate complete-build-guide.md with step-by-step simulation + deployment workflow.
7. Validate file existence and consistency across all markdown content.

Keep the plan clear and short.

```