/sp.constitution     ## Project: AI-native textbook on Physical AI & Humanoid Robotics

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

