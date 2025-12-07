I am preparing a book on `physical Ai and humanoid robotics` , this is an ai-native book should be using following .
- claude code cli setup with qwen code api (for book generation)
- speckit-plus (fork of github speckit)
- docusaurus or book content .

```
/docs
│
├─ intro.md
│
├─ course-overview/
│   ├─ index.md
│   ├─ physical-ai-intro.md
│   ├─ embodied-intelligence.md
│   ├─ humanoid-robotics-landscape.md
│   ├─ sensor-systems.md
│
├─ module-1-ros2/
│   ├─ index.md
│   ├─ ros2-architecture.md
│   ├─ nodes-topics-services-actions.md
│   ├─ ros2-packages-python.md
│   ├─ launch-files-params.md
│   ├─ urdf-for-humanoids.md
│   ├─ exercises.md
│
├─ module-2-digital-twin/
│   ├─ index.md
│   ├─ gazebo-basics.md
│   ├─ urdf-sdf.md
│   ├─ physics-simulation.md
│   ├─ sensor-simulation.md
│   ├─ unity-visualization.md
│   ├─ exercises.md
│
├─ module-3-nvidia-isaac/
│   ├─ index.md
│   ├─ isaac-sdk-overview.md
│   ├─ isaac-sim-setup.md
│   ├─ perception-and-manipulation.md
│   ├─ reinforcement-learning.md
│   ├─ sim-to-real-transfer.md
│   ├─ exercises.md
│
├─ module-4-vla/
│   ├─ index.md
│   ├─ voice-to-action.md
│   ├─ cognitive-planning.md
│   ├─ multimodal-perception.md
│   ├─ llm-ros-integration.md
│   ├─ exercises.md
│
├─ weekly-breakdown/
│   ├─ index.md
│   ├─ week-1.md
│   ├─ week-2.md
│   ├─ week-3.md
│   ├─ week-4.md
│   ├─ week-5.md
│   ├─ week-6.md
│   ├─ week-7.md
│   ├─ week-8.md
│   ├─ week-9.md
│   ├─ week-10.md
│   ├─ week-11.md
│   ├─ week-12.md
│   ├─ week-13.md
│
├─ assessments/
│   ├─ index.md
│   ├─ ros2-project.md
│   ├─ gazebo-sim-project.md
│   ├─ isaac-perception-pipeline.md
│   ├─ capstone-project.md
│
├─ hardware-requirements/
│   ├─ index.md
│   ├─ workstation.md
│   ├─ edge-ai-kit.md
│   ├─ robot-lab-options.md
│   ├─ cloud-lab.md
│
├─ capstone/
│   ├─ index.md
│   ├─ architecture.md
│   ├─ perception.md
│   ├─ navigation.md
│   ├─ manipulation.md
│   ├─ vla-integration.md
│   ├─ complete-build-guide.md
│
└─ glossary/
    ├─ index.md
    ├─ robotics-terms.md
    ├─ ai-terms.md
    ├─ simulation-terms.md
    ├─ hardware-terms.md
```

please create a contitution prompt for `/sp.constitution` prompt , take this ormat as an example .

```markdown
/sp.constitution

Project: Research paper on AI-native software development

Core principles:
- Accuracy through primary source verification
- Clarity for academic audience (computer science background)
- Reproducibility (all claims cited and traceable)
- Rigor (peer-reviewed sources preferred)

Key standards:
- All factual claims must be traceable to sources
- Citation format: APA style
- Source types: minimum 50% peer-reviewed articles
- Plagiarism check: 0% tolerance before submission
- Writing clarity: Flesch-Kincaid grade 10-12

Constraints:
- Word count: 5,000-7,000 words
- Minimum 15 sources
- Format: PDF with embedded citations

Success criteria:
- All claims verified against sources
- Zero plagiarism detected
- Passes fact-checking review
```