```markdown
/sp.specify   Weekly Breakdown Content (using textbook-writer)

Target audience: Students following the humanoid robotics textbook
Focus: A clear 6-week guided roadmap aligned with Modules 1–4 + capstone progression

Success criteria:
- Each week includes learning objectives, reading assignments, hands-on tasks, and checkpoints
- Tasks reference existing modules (ROS2, Digital Twin, Isaac, VLA)
- Workload is realistic and sequential
- Exercises point to relevant files in earlier chapters
- All markdown files must be generated inside ./textbook/docs/weekly-breakdown by the textbook-writer agent

Constraints:
- Word count per week: 300–600 words
- Format: Markdown for Docusaurus
- Structure: index.md + week-1.md … week-6.md
- Timeline: 6-week learning sprint format

Not building:
- Full lessons (those exist in modules)
- Advanced RL or large-model theory beyond weekly scope

```

```markdown
/sp.plan

Create: index structure, weekly layout template, task hierarchy, progress checkpoints, cross-links to module files.
Decisions needing documentation: depth of weekly tasks, number of hands-on items per week, checkpoint style.
Testing strategy: verify that Weekly Breakdown maps cleanly to Modules 1–4; ensure links match existing file paths; validate total workload realism.

Technical details:
- Output directory must be ./textbook/docs/weekly-breakdown
- Must call the subagent `textbook-writer` for for book writing
- Ensure consistent voice, formatting, and structure across weeks
```


