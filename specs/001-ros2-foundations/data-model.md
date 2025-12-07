# Data Model: Module 1 — ROS2 Foundations for Humanoid Robotics

## Educational Content Entities

### Module (Root Entity)
- **name**: String - "Module 1 — ROS2 Foundations for Humanoid Robotics"
- **target_audience**: String - "Students and practitioners preparing to work with ROS2 for humanoid robots"
- **directory**: String - "./textbook/docs/module-1-ros-2"
- **file_count**: Integer - 7
- **tone**: String - "Academic yet practical"
- **word_range**: Object - {min: 1000, max: 1500} per file

### ContentFile (7 instances)
- **filename**: String - One of ["index.md", "ros2-architecture.md", "nodes-topics-services-actions.md", "ros2-packages-python.md", "launch-files-params.md", "urdf-for-humanoids.md", "exercises.md"]
- **title**: String - Descriptive title for the content
- **focus**: String - Primary educational focus of the file
- **word_count**: Integer - Between 1000-1500 words
- **bold_terms**: Array - 3-5 bolded terms for scannability
- **diagrams**: Integer - At least 1 flow description or diagram
- **exercises**: Integer - At least 1 hands-on exercise
- **cross_references**: Array - References to upcoming modules (Digital Twin, Isaac Sim)

### ROS2Concept (Multiple per file)
- **name**: String - Name of the ROS2 concept (e.g., "Node", "Topic", "Service", "Action")
- **definition**: String - Clear definition of the concept
- **humanoid_application**: String - How the concept applies to humanoid robotics
- **code_example**: String - Python code example demonstrating the concept
- **diagram_reference**: String - Reference to diagram or flow description

### Exercise (Multiple per module)
- **type**: String - One of ["conceptual", "practical", "integration"]
- **difficulty**: String - One of ["beginner", "intermediate", "advanced"]
- **prerequisites**: Array - List of concepts required to complete the exercise
- **instructions**: String - Clear step-by-step instructions
- **expected_outcome**: String - What the student should achieve
- **connection_to_future**: String - How this exercise relates to future modules

## Relationships
- Module contains 7 ContentFile entities
- Each ContentFile contains multiple ROS2Concept entities
- Each ContentFile contains multiple Exercise entities
- ROS2Concept entities reference humanoid applications
- Exercise entities connect to future module concepts

## Validation Rules
- Each ContentFile must have 1000-1500 words
- Each ContentFile must include 3-5 bolded terms
- Each ContentFile must include at least 1 diagram/flow description
- Each ContentFile must include at least 1 hands-on exercise
- All terminology must be consistent across all files
- All content must maintain academic yet practical tone
- All content must be concept-focused without full project skeletons
- All content must reference upcoming modules without deep-diving