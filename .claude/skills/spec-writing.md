---
name: "specification-writer"
description: "Create complete specification commands following best practices."
version: "1.0.0"
---

# Spec generation skill :

## When to Use This Skill

- User asks to generate specification commmand  .
- When user mention explicit call .
- When user tell to edit the existing specs .

## How This Skill Works

1. **Get feature name from user**: Must sk feature name from user for which specc should be generated .
2. **Get feature description from user**: Must ask a small prompt from user as a feature description .
3. **Get destination markdown file path from user**: Topic sentence → evidence/citation → significance → transition
4. **Generate Specification in the destination file:** Generate specification command in the specified markdown file .

## Quality Criteria :
Command should be successfully generated in the destination markdown file .

## Required Information
To create a complete specification command, you'll need to ask 

1. **Feature Name**: The title of your feature/module
2. **Feature Description**: A detailed explanation of what the feature is about
2. **markdown file destination**: File path where the command has to be generated . 

from user , then generate command in the specified destination file .

## Example flow :
**Input**: I've successfully setup the docusaurus project in the `./textbook` directory . Use the `spec-writing` skill to generate the specification command 

**Output** Feature name ?
**Input**  Creation of a module` ROS2 fundamentals`.

**Output**: A small description about this feature ?
**Input** : You're required to create a new chapter in the docusaurus book which should be placed in `./textbook/docs/` directory . Get the overview of this book through this map .
```text
├─ module-1-ros-2/
│   ├─ index.md
│   ├─ ros2-architecture.md
│   ├─ nodes-topics-services-actions.md
│   ├─ ros2-packages-python.md
│   ├─ launch-files-params.md
│   ├─ urdf-for-humanoids.md
│   ├─ exercises.md
```
This book should be used by Students and practitioners praparing to work with ROS2 for humanoid robotics .
**Make sure**:
- Written for both technical and non-technical stakeholders
- Used clear, concise language .
- Created success criteria measurable and verifiable
- Defined clear boundaries to prevent scope creep
- Focused on user value and business needs rather than implementation details
- Formated all content for Docusaurus markdown compatibility
- Specified Clear target audience definition
- Specifically focusing areas relevant to the domain
- Designed clear constraints limiting scope
- Explicit "Not building" section to set boundaries  

**Output**:
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

