# Research Summary: Physical AI & Humanoid Robotics Course Overview

## Decision: Depth Level of Explanations
**Rationale**: Based on the specification requirements for "academic yet accessible" content with 1000-1500 words per file and basic AI/ML prerequisites assumed, explanations should include semi-technical details but avoid complex mathematics or deep implementation code.
**Alternatives considered**:
- Purely conceptual (too shallow for target audience)
- Deep technical with mathematical formulations (too complex for overview module)
- Semi-technical with implementation hints (selected as it balances academic rigor with accessibility)

## Decision: Humanoid Platforms for Comparison Table
**Rationale**: To provide comprehensive coverage of the current landscape while meeting the requirement for at least 7 companies with columns for Company, Platform, Focus Area, Year Founded, Current Status, and Connection to Digital Twins/Sim-to-Real Challenges.
**Selected platforms**:
1. Boston Dynamics (Atlas, Spot) - Industrial and research applications
2. Tesla (Optimus) - Manufacturing and service applications
3. Figure AI (Figure 01) - Service and industrial applications
4. Honda (ASIMO, 3E concept robots) - Research and human assistance
5. Agility Robotics (Cassie, Digit) - Logistics and delivery
6. Sanctuary AI (Phoenix) - Industrial applications
7. Unitree (H1, G1) - Consumer and research applications

**Alternatives considered**:
- Fewer platforms (inadequate representation of field)
- More platforms (exceeds practical table size)
- Different selection (these represent diverse approaches and applications)

## Decision: Number of Sensors per Category
**Rationale**: To provide comprehensive coverage while maintaining focus on key sensor types relevant to humanoid robotics. The specification requires explaining proprioceptive vs exteroceptive sensors.
**Selected approach**:
- Proprioceptive sensors (5 types): Joint encoders, IMUs, force/torque sensors, motor current sensors, temperature sensors
- Exteroceptive sensors (7 types): Cameras (RGB, stereo), LiDAR, ultrasonic sensors, tactile sensors, microphones, GPS, environmental sensors

**Alternatives considered**:
- Fewer sensors (inadequate coverage)
- More sensors (excessive detail for overview)
- Equal number per category (unnatural as proprioceptive and exteroceptive systems serve different functions)

## Decision: Tone Balance Between Academic and Practical
**Rationale**: The specification calls for "high-level, academic yet accessible" content with appropriate technical depth. The tone should provide academic rigor while remaining practical and engaging for the target audience.
**Selected approach**: Engineering clarity with instructor-style explanations that include real-world applications and industry examples, while avoiding overly casual language.
**Alternatives considered**:
- Purely academic (too dry for engagement)
- Purely practical (lacks academic rigor)
- Balanced approach (selected as it meets specification requirements)

## Decision: Cross-Reference Placement
**Rationale**: To support the foundational role of this module in preparing students for deeper modules (ROS2 → Digital Twins → Isaac Simulation → VLA policies) while maintaining flow and readability.
**Selected approach**:
- In `index.md`: Overview of full course structure with forward references to all modules
- In `physical-ai-intro.md`: Connection to simulation and real-world applications
- In `embodied-intelligence.md`: References to Digital Twins and simulation modules
- In `humanoid-robotics-landscape.md`: Connections to ROS2 and simulation challenges
- In `sensor-systems.md`: Links to Sim-to-Real transfer challenges in future modules

**Alternatives considered**:
- No cross-references (violates specification requirement)
- Minimal references (insufficient connection to broader course)
- Dense references throughout (disrupts flow of individual modules)