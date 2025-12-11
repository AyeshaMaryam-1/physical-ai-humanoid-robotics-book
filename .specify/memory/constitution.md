<!--
Sync Impact Report:
Version change: 0.0.0 -> 1.0.0 (MAJOR: Initial concrete content from user input)
Modified principles:
  - [PRINCIPLE_1_NAME] -> Educational Clarity
  - [PRINCIPLE_2_NAME] -> Engineering Accuracy
  - [PRINCIPLE_3_NAME] -> Practical Applicability
  - [PRINCIPLE_4_NAME] -> Ethical & Safe Operation
Added sections:
  - Standards
  - Structure and Constraints
  - Success Criteria
Removed sections:
  - PRINCIPLE_5_NAME, PRINCIPLE_6_NAME, PRINCIPLE__DESCRIPTION placeholders
Templates requiring updates:
  - .specify/templates/plan-template.md: ⚠ pending
  - .specify/templates/spec-template.md: ⚠ pending
  - .specify/templates/tasks-template.md: ⚠ pending
  - .specify/templates/commands/*.md: ⚠ pending
Follow-up TODOs: None
-->
# Spec-Driven Book — Physical AI, Humanoid Robotics, and Vision-Language-Action Systems Constitution

## Core Principles

### Educational Clarity
Concepts must support learners progressing from foundational robotics to advanced embodied AI.

### Engineering Accuracy
All robotics, AI, simulation, and VLA explanations must reflect real, industry-validated practices (ROS 2, Gazebo, Isaac, URDF/SDF).

### Practical Applicability
Every concept must translate into hands-on implementation with reproducible demos.

### Ethical & Safe Operation
Emphasize responsible physical AI, safe interaction, and hardware-aware risk notes.

## Standards

- Content must be original and grounded in authoritative sources.
- All code examples must be executable, version-accurate, and reproducible on ROS 2, Gazebo, Unity, and NVIDIA Isaac Sim.
- Robotics models must follow official formats and conventions (URDF/SDF, SLAM pipelines, sensor models).
- VLA and LLM integrations must reflect current multimodal AI capabilities.
- Diagrams and architecture flows must be clear, purposeful, and technically accurate.
- Tone: supportive, technical, instructor-to-student.

## Structure and Constraints

### Structure
- The book contains **4 modules**, each with **4 chapters** (16 total).
- Each module begins with a high-level specification.
- Each chapter includes:
    - Learning objectives
    - Core concepts
    - Step-by-step examples
    - Architecture diagrams
    - Runnable code samples
    - A short exercise
    - References
- Technical clarity: inputs, outputs, architecture, and safety notes.

### Constraints
- Chapter length: **1,000–2,500 words**
- Output format: Markdown/MDX for Docusaurus
- Must build and deploy cleanly to GitHub Pages
- Hardware assumptions must remain realistic (Jetson, RealSense, Unitree, or simulated humanoids)
- No hallucinated tools, APIs, or robot capabilities

## Success Criteria

- Students can design, simulate, and operate humanoid robots end-to-end.
- All examples run successfully in ROS 2, Gazebo, and Isaac environments.
- Zero plagiarism; all sources verifiable.
- Docusaurus build and GitHub Pages deployment error-free.
- Capstone VLA humanoid project is clearly achievable through the book.

## Governance

This constitution supersedes all other project practices. Amendments require a formal documentation process, explicit approval by project stakeholders, and a clear migration plan for any affected systems or guidelines. All pull requests and code reviews must verify compliance with these principles.

**Version**: 1.0.0 | **Ratified**: 2025-12-07 | **Last Amended**: 2025-12-07