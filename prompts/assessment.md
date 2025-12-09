```markdown
/sp.specify     Generate the full content for the "assessments" chapter of the textbook.

Write three Markdown files and place them inside:
`./textbook/docs/assessments`

Files to generate:
- index.md  
- ros2-project.md  
- capstone-project.md  

Content requirements:
- Match style, structure, and tone of earlier modules.
- Provide clear learning objectives, instructions, rubrics, and deliverables.
- Keep content beginner-friendly but technically correct.
- Use structured headings, tables, bullet points, and diagrams (ASCII allowed).
- Avoid overly long paragraphs.
- Fit logically after the weekly breakdown chapter.

Use the `textbook-writer` subagent to create and write files inside the correct directory.

```

```markdown
/sp.clarify    Before generating the assessments chapter, ask ONLY the essential clarifying questions needed to produce accurate content.

Focus questions on:
- Must use the `textbook-writer` subagent to generate the content 
- Level of difficulty for the ROS2 project
- Whether the capstone is individual or group-based
- Expected final deliverables (repo, report, demo video)
- Evaluation criteria format (points, pass/fail, rubric)
- Target hardware/sim (e.g., Gazebo, Isaac Sim, physical robot)
```