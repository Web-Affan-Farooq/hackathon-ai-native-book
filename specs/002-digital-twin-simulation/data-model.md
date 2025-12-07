# Data Model: Module 2 — Digital Twin & Simulation Foundations

## Educational Content Entities

### Module (Root Entity)
- **name**: String - "Module 2 — Digital Twin & Simulation Foundations"
- **target_audience**: String - "Learners preparing to simulate humanoid robots using Gazebo and Digital Twin workflows"
- **directory**: String - "./textbook/docs/module-2-digital-twin-simulation-foundations"
- **file_count**: Integer - 5
- **tone**: String - "Academic yet practical"
- **word_range**: Object - {min: 1000, max: 1500} per file

### ContentFile (5 instances)
- **filename**: String - One of ["index.md", "gazebo-basics.md", "urdf-sdf.md", "physics-simulation.md", "exercises.md"]
- **title**: String - Descriptive title for the content
- **focus**: String - Primary educational focus of the file
- **word_count**: Integer - Between 1000-1500 words
- **bold_terms**: Array - 3-5 bolded terms for scannability
- **diagrams**: Integer - At least 1 workflow diagram or figure
- **exercises**: Integer - At least 1 hands-on exercise
- **cross_references**: Array - References to upcoming modules (Isaac Sim, sim-to-real)

### Digital Twin Concept (Multiple per file)
- **name**: String - Name of the Digital Twin concept (e.g., "Digital Replica", "Bidirectional Sync", "Simulation-to-Real Transfer")
- **definition**: String - Clear definition of the concept
- **humanoid_application**: String - How the concept applies to humanoid robotics
- **workflow_example**: String - Markdown workflow diagram or process description
- **reference_to_real_world**: String - Connection to actual Digital Twin implementations

### Gazebo Concept (Multiple per file)
- **name**: String - Name of the Gazebo concept (e.g., "Physics Engine", "SDF Format", "Simulation Environment")
- **definition**: String - Clear definition of the concept
- **humanoid_application**: String - How the concept applies to humanoid robotics
- **configuration_example**: String - Example configuration or setup
- **troubleshooting_notes**: String - Common issues and solutions

### Simulation Concept (Multiple per file)
- **name**: String - Name of the simulation concept (e.g., "URDF to SDF", "Physics Parameters", "Collision Detection")
- **definition**: String - Clear definition of the concept
- **humanoid_application**: String - How the concept applies to humanoid robotics
- **conversion_example**: String - Example conversion or configuration
- **validation_method**: String - How to verify the simulation works correctly

### Exercise (Multiple per module)
- **type**: String - One of ["conceptual", "practical", "integration", "simulation"]
- **difficulty**: String - One of ["beginner", "intermediate", "advanced"]
- **prerequisites**: Array - List of concepts required to complete the exercise
- **instructions**: String - Clear step-by-step instructions
- **expected_outcome**: String - What the student should achieve
- **connection_to_future**: String - How this exercise relates to future modules

## Relationships
- Module contains 5 ContentFile entities
- Each ContentFile contains multiple Digital Twin, Gazebo, or Simulation Concept entities
- Each ContentFile contains multiple Exercise entities
- Digital Twin concepts connect to Gazebo and Simulation concepts
- Exercise entities connect to multiple concepts across different files
- All entities reference humanoid applications

## Validation Rules
- Each ContentFile must have 1000-1500 words
- Each ContentFile must include 3-5 bolded terms
- Each ContentFile must include at least 1 workflow diagram or figure
- Each ContentFile must include at least 1 hands-on exercise
- All terminology must be consistent across all files
- All content must maintain academic yet practical tone
- All content must be concept-focused without full robot models
- All content must reference upcoming modules without deep-diving
- Gazebo terminology must be accurate and current (Garden/Fortress standards)
- Physics concepts must be mathematically accurate and unit-consistent