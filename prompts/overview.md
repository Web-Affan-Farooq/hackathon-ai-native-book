## specifications :
```markdown
/sp.specify Physical AI & Humanoid Robotics — Course Overview Module

Target audience: Advanced undergraduates, graduate students, and industry professionals preparing for hands-on work in embodied AI, robotics simulation, and humanoid systems.

Focus:
- High-level, academically grounded introduction to Physical AI, Embodied Intelligence, Humanoid Robotics Landscape, and Sensor Systems.
- Establish foundational understanding before deeper modules (ROS2 → Digital Twins → Isaac Simulation → VLA policies).
- Ensure `course-overview/index.md` acts as the main landing page for the book.

Success criteria:
- Provides clear definitions of: Physical AI, Embodied Intelligence, Humanoid Robotics, and Sensor Systems.
- Includes distinctions between software-only AI and embodied systems with morphology + sensing.
- Outlines major players in humanoid robotics with a structured table (company, platform, focus).
- Explains proprioceptive vs exteroceptive sensors, including sensor fusion relevance.
- Highlights challenges and opportunities (e.g., balance, real-world interaction, sim-to-real).
- Uses 3–5 bolded terms per file for scannability.
- Tone: high-level, academic yet accessible.
- All content cleanly formatted in Markdown, ready for Docusaurus.

Constraints:
- Deliverables include full drafts of:  
  `index.md`, `physical-ai-intro.md`, `embodied-intelligence.md`, `humanoid-robotics-landscape.md`, `sensor-systems.md`
- The `index.md` must summarize the entire book and introduce Modules 1–4.
- Sensor-level content must reference later simulation chapters (digital twins, Isaac).
- No deep dives into ROS2, simulation APIs, controllers, or policy code (covered later).
- No equations-heavy robotics content; remain conceptual at this stage.

Not building:
- Full ROS2 tutorials
- Low-level control algorithms
- In-depth Isaac Sim pipeline
- VLA policy training guides
- Hardware-specific engineering documentation

```

## Clarification:

```markdown
/sp.clarify

My specification for the `course-overview` module is at textbook/docs/course-overview/spec.md  
Please analyze it for:

1. Ambiguous terms  
   - Are concepts like "high-level", "overview", or "accessible" defined clearly enough?
   - What does "motivational tone" specifically require?

2. Missing assumptions  
   - Expected length per file?
   - Expected reader prerequisites in AI/robotics?
   - Should terminology match glossary conventions (e.g., "visuomotor policies", "foundation models")?

3. Incomplete requirements  
   - Should societal impact (elder care, logistics, industrial automation) be included in intro chapters?
   - Should challenges section explicitly connect to Digital Twins (Module 2) and Sim-to-Real (Module 3)?
   - Is there a required format for the Humanoid Robotics table?

4. Scope conflicts  
   - Should this module remain purely conceptual, or include early hints of implementation?
   - How much historical context is appropriate before it becomes too academic?

What specification gaps must be addressed before planning the chapter structure?

```

## plan:
```markdown
/sp.plan   Create: chapter structure, file sequence, generation strategy, cross-module linking plan, quality validation checks.

Decisions needing documentation:
- Depth level of explanations (conceptual vs semi-technical)
- Which humanoid platforms to highlight in the comparison table (e.g., Boston Dynamics, Tesla, Figure AI)
- How many sensors per category (proprioceptive/exteroceptive)
- Tone balance between academic and practical
- Placement of cross-references to Modules 1–4 and Capstone

Testing strategy:
- Validate that each file meets success criteria from /sp.specify
- Confirm 3–5 bolded terms appear strategically in each file
- Ensure all definitions match terminology from the glossary system
- Check that `index.md` gives a roadmap of the full book and ends with a clean call-to-action toward Module 1 (ROS2)
- Confirm internal links follow Docusaurus paths
- Validate Markdown structure (tables, headings, lists)

Technical details:
- Use incremental, file-by-file generation (Structure → Core Concepts → Applied Context → Sensor Foundations)
- Follow research-concurrent approach (write while exploring concepts)
- Organize by phases:
  - Phase 1: Scaffold `index.md` including roadmap for all modules  
  - Phase 2: Build conceptual foundations (Physical AI, Embodied Intelligence)  
  - Phase 3: Create context-setting chapters (Humanoid Landscape, Sensor Systems)  
  - Phase 4: Insert cross-references to simulation, ROS2, VLA modules  
- Ensure Markdown is immediately compatible with Docusaurus sidebar + routing conventions

```