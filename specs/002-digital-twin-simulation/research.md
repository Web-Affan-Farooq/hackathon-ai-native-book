# Research Summary: Module 2 — Digital Twin & Simulation Foundations

## Decision: Level of detail for URDF→SDF conversion
**Rationale**: For educational content targeting students and practitioners, the URDF to SDF conversion process should provide sufficient detail for understanding the key differences and conversion process without overwhelming learners with complex model details. The spec requires concept-focused content without full robot models.
**Decision**: Provide detailed explanation of the conversion process with practical examples showing key differences between URDF and SDF formats, highlighting the simulation-specific elements (inertial properties, collision models, visual elements) that need special attention during conversion. Focus on humanoid-specific considerations like joint limits and physics properties.
**Alternatives considered**:
- Minimal conversion details - insufficient for practical understanding
- Detailed full model conversion - too complex for learning, violates "no full robot models" constraint
- Detailed process with practical examples - optimal for educational goals while respecting constraints

## Decision: Physics examples (humanoid walking vs generic rigid-body demos)
**Rationale**: The module specifically focuses on "humanoid robotics applications" and aims to prepare students for humanoid robot simulation. Using humanoid-specific examples maintains consistency with the target audience and learning objectives.
**Decision**: Use humanoid walking and balancing examples for physics demonstrations, showing how gravity, friction, and dynamics apply specifically to bipedal robots. Include examples of stable walking gaits, balance control, and humanoid-specific physics challenges like center of mass management.
**Alternatives considered**:
- Generic rigid-body demos only - doesn't meet humanoid focus requirement
- Mixed examples - dilutes focus on humanoid applications
- Humanoid-specific examples with walking/balancing - aligns with educational objectives and target audience

## Decision: Exercises and progression
**Rationale**: The learning journeys are designed with progressive priorities (P1-P4), building foundational knowledge before advancing to complex topics. Exercises should reinforce this progression while remaining accessible.
**Decision**: Create 3-5 progressive exercises that build on concepts from earlier learning journeys, starting with basic Digital Twin concepts and Gazebo setup, progressing to URDF/SDF conversion, then physics simulation, and finally integration exercises that connect all concepts. Each exercise should be achievable independently if needed.
**Alternatives considered**:
- Fewer standalone exercises - doesn't leverage progressive learning
- Complex integrated exercises only - creates barriers for learners
- Progressive with independence - optimal for learning while maintaining flexibility

## Decision: Gazebo version and terminology
**Rationale**: The specification specifically mentions "Gazebo (Ignition)" and requires correct Gazebo/Ignition terminology. Modern Gazebo development has moved to Ignition Gazebo (now called Garden/Fortress).
**Decision**: Focus on modern Gazebo Garden/Fortress terminology and approaches, with references to legacy Gazebo Classic where necessary for historical context. Use correct Ignition/Garden terminology throughout the content.
**Alternatives considered**:
- Legacy Gazebo Classic only - outdated, doesn't match current standards
- Mixed legacy/modern - creates confusion with terminology
- Modern Gazebo Garden/Fortress with legacy notes - follows current best practices