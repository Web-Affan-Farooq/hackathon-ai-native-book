---
id: 1
title: spec-writing-skill-creation
stage: general
date: '2025-12-16'
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: 001-assessments
user: affan
command: spec-writing-skill-creation
labels:
  - skill
  - specification
  - command
  - documentation
links:
  SPEC: null
  TICKET: null
  ADR: null
  PR: null
files:
  - .claude/commands/spec-wizard.md
  - .claude/skills/spec-writing.md
tests: []
---

# Spec-Writing Skill Creation

## Summary
Created a reusable skill and interactive command to help users write better specification commands following best practices.

## Details
- Created `.claude/skills/spec-writing.md` with guidance on creating comprehensive specifications
- Created `.claude/commands/spec-wizard.md` as an interactive wizard to guide users through the specification creation process
- Both files follow the successful pattern demonstrated with the ROS2 Foundations module specification
- The wizard provides step-by-step guidance for creating complete specifications with all necessary sections

## Files Created
- `.claude/commands/spec-wizard.md` - Interactive command for specification creation
- `.claude/skills/spec-writing.md` - Skill documentation with best practices

## Outcome
Users can now use `/spec-wizard` to be guided through creating complete specification commands that follow the successful pattern, or reference the skill documentation for best practices when creating specifications manually.

## Prompt Text
I want to create a reusable skill so that you can help me in writing better speckit-plus commands.

During our previous conversations , I've noticed some of the patterns :
- Complete specifications allow a brief overview of the features to be implemented .
- Clarification command allow us to catch pitfalls earlier .
- Planning command allow us to plan detailed and complete technical implementation .

Read the following specification command , this command is very successful and it created a good specs for the chapter `Module 1: ROS2 Foundations for Humanoid Robotics` of our docusaurus textbook `Physical AI and Humanoid Robotics ` .

```bash
/sp.specify   Module 1 — ROS2 Foundations for Humanoid Robotics

this is the first module from where the technical studies of our book `physical ai and humanoid robotics ` starts

Target audience: Students and practitioners preparing to work with ROS2 for humanoid robots.

Focus:
- Core ROS2 concepts (architecture, nodes, topics, services, actions)
- Building packages in Python
- Launch files + parameters
- URDF modeling tailored to humanoid robots
- Hands-on exercises to prepare for simulation modules

Success criteria:
- Generates 7 markdown files: index.md, ros2-architecture.md, nodes-topics-services-actions.md, ros2-packages-python.md, launch-files-params.md, urdf-for-humanoids.md, exercises.md in directory `./textbook/docs/module-1-ros-2`
- Explains concepts with clarity and relevance to humanoid systems
- Uses diagrams/flow descriptions in Markdown where helpful
- References upcoming modules (Digital Twin, Isaac Sim) without deep-diving

Constraints:
- Academic yet practical tone
- Concept-focused, no full project skeletons or long codebases
- All Markdown formatted for Docusaurus
- Limit scope to ROS2 basics and humanoid-specific URDF concepts

Not building:
- Detailed ROS2 C++ tutorials
- Full humanoid URDF or meshes
- Simulation pipelines (covered in later modules)
- Controllers, SLAM, Nav2, policy learning
```

Create a reusable skill called `spec-writing` which allow us to write complete specification command for the feature to be implemented . This skill will take a prompt from user through command line in which user will tell about the feature for which specify command should be created .
- Ask user for the feature name through command line .
- Ask user for the description of feature .
- Ask path to destination markdown file from user where you've to write the command of specification .

**Dont write a single word without the permission of user , must ask the above question**

## Response Text
Created two files to implement the spec-writing capability:
1. A reusable skill in `.claude/skills/spec-writing.md` with guidance on creating comprehensive specifications
2. An interactive command in `.claude/commands/spec-wizard.md` that guides users through the specification creation process step by step .