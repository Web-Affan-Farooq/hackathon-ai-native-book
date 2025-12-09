```markdown
/sp.specify    Module 2 — Digital Twin & Simulation Foundations

Target audience: Learners preparing to simulate humanoid robots using Gazebo and Digital Twin workflows.

Focus:
- Digital Twin principles
- Gazebo (Ignition) basics
- URDF → SDF conversion and model structuring
- Physics simulation essentials for humanoid behaviors
- Practical exercises bridging ROS2 and simulation

Success criteria:
- Generates 5 markdown files in `./textbook/docs/module-2-digital-twin-simulation-foundations`: index.md, gazebo-basics.md, urdf-sdf.md, physics-simulation.md, exercises.md
- Explains concepts clearly with humanoid robotics relevance
- Describes workflows (not full projects) using Markdown diagrams
- Connects to Module 1 content where appropriate

Constraints:
- Conceptual + practical balance
- No long code samples or full robot models
- Must use correct Gazebo/Ignition terminology
- Prepare reader for Module 3 (Isaac Sim + sim-to-real)

Not building:
- Detailed environment modeling
- Mesh authoring or CAD workflows
- ROS2 controller design
- Full Digital Twin pipelines with cloud systems

```

```markdown
/sp.clarify

My Module 2 simulation specification is at textbook/docs/module-2-digital-twin/spec.md  
Please analyze it for:

1. Ambiguous terms  
   - How deep should SDF schema explanations go?  
   - Expected level of physics detail?  

2. Missing assumptions  
   - Required prior knowledge (Module 1 ROS2 concepts?)  
   - Should Gazebo examples use Fortress, Harmonic, or generic Ignition?  

3. Incomplete requirements  
   - How detailed should index.md be?  
   - Should exercises depend on ROS2 nodes?  

4. Scope conflicts  
   - General simulation concepts vs humanoid-specific needs  
   - How much overlap with Module 3 should be avoided?

Identify what needs clarification before planning generation.

```

```markdown
/sp.plan    Create: content structure, file sequence, modeling workflow notes, and validation rules.

Decisions needing documentation:
- Level of detail for URDF→SDF conversion
- Whether physics examples use humanoid walking or generic rigid-body demos
- How many exercises and what progression

Testing strategy:
- Ensure all 5 markdown files are generated in destination folder  `./textbook/docs/module-2-digital-twin-simulation`
- Check internal cross-links to Module 1
- Validate accuracy of Gazebo/Ignition terminology
- Confirm physics explanations are correct and accessible

Technical details:
- Phased generation:
  1. index.md  
  2. gazebo-basics.md  
  3. urdf-sdf.md  
  4. physics-simulation.md  
  5. exercises.md  
- Research-while-writing workflow
- Markdown ready for Docusaurus navigation

```