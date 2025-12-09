```markdown
/sp.specify    Generate the full content for the last "glossary" chapter of the textbook.

Markdown files to create inside:
`./textbook/docs/glossary`

Files:
- index.md
- robotics-terms.md
- ai-terms.md

Content requirements:
- index.md: short intro, how to use the glossary, links to sections.
- robotics-terms.md: concise definitions for ROS2, Gazebo, URDF/SDF, nodes/topics/services/actions, TF2, kinematics, perception, manipulation, SLAM, planning, controllers, actuators, sensors, etc.
- ai-terms.md: concise definitions for ML, DL, RL, transformers, VLA components, embeddings, inference, training, dataset terms, optimization concepts, etc.
- Definitions must be short, clear, and practical for beginners.
- Avoid long essays; one-paragraph or bullet-style explanations.
- Organize alphabetically within each file.

```

```markdown
/sp.plan    Create a concise execution plan for generating the glossary chapter:

1. Use `textbook-writer` to generate this chapter `./textbook/docs/glossary`.
2. Generate index.md with intro + navigation.
3. Generate robotics-terms.md with alphabetized list of robotics/ROS2/Gazebo terms.
4. Generate ai-terms.md with alphabetized list of AI/ML/RL/VLA terms.
5. Run final validation: ensure files exist and content is consistent and properly formatted.

Keep the plan short and clear.

```