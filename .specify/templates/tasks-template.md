---

description: "Task list template for feature implementation"
---

# Tasks: [FEATURE NAME]

**Input**: Design documents from `/specs/[###-feature-name]/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: The examples below include test tasks. Tests are OPTIONAL - only include them if explicitly requested in the feature specification.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Single project**: `src/`, `tests/` at repository root
- **Web app**: `backend/src/`, `frontend/src/`
- **Mobile**: `api/src/`, `ios/src/` or `android/src/`
- Paths shown below assume single project - adjust based on plan.md structure

<!--
  ============================================================================
  IMPORTANT: The tasks below are SAMPLE TASKS for illustration purposes only.

  The /sp.tasks command MUST replace these with actual tasks based on:
  - Learning journeys from spec.md (with their priorities P1, P2, P3...)
  - Educational and technical requirements from plan.md
  - Content structure from data-model.md
  - Module specifications from contracts/

  Tasks MUST be organized by learning journey so each journey can be:
  - Implemented independently
  - Tested independently
  - Delivered as an educational increment

  DO NOT keep these sample tasks in the generated tasks.md file.
  ============================================================================
-->

## Phase 1: Textbook Setup (Shared Infrastructure)

**Purpose**: Textbook project initialization and basic structure

- [ ] T001 Create textbook project structure per implementation plan in `../../textbook/docs` directory
- [ ] T002 Initialize Docusaurus project with proper configuration for textbook content
- [ ] T003 [P] Configure MDX compatibility and textbook-specific styling

---

## Phase 2: Foundational Content (Blocking Prerequisites)

**Purpose**: Core textbook infrastructure that MUST be complete before ANY learning journey can be implemented

**⚠️ CRITICAL**: No learning journey work can begin until this phase is complete

Examples of foundational tasks (adjust based on your project):

- [ ] T004 Setup glossary structure with consistent terminology definitions
- [ ] T005 [P] Implement content validation framework for technical accuracy
- [ ] T006 [P] Setup cross-reference system between modules and concepts
- [ ] T007 Create base content templates that all modules depend on
- [ ] T008 Configure educational assessment and exercise framework
- [ ] T009 Setup authoritative source verification system for technical claims

**Checkpoint**: Foundation ready - learning journey implementation can now begin in parallel

---

## Phase 3: Learning Journey 1 - [Title] (Priority: P1) 🎯 MVP

**Goal**: [Brief description of what this learning journey teaches students]

**Independent Test**: [How to verify students can learn this content independently]

### Educational Assessments for Learning Journey 1 (OPTIONAL - only if assessments requested) ⚠️

> **NOTE: Write these assessments FIRST, ensure they accurately measure learning objectives**

- [ ] T010 [P] [LJ1] Create hands-on exercise for [concept] in docs/module-[x]/exercises.md
- [ ] T011 [P] [LJ1] Create knowledge check questions for [learning objective] in docs/module-[x]/exercises.md

### Implementation for Learning Journey 1

- [ ] T012 [P] [LJ1] Create foundational concept explanation in docs/module-[x]/[concept].md
- [ ] T013 [P] [LJ1] Create practical tutorial content in docs/module-[x]/[tutorial].md
- [ ] T014 [LJ1] Implement code examples with ROS2/Humble compatibility in docs/module-[x]/[examples].md (depends on T012)
- [ ] T015 [LJ1] Add mathematical formulations with unit consistency in docs/module-[x]/[formulas].md
- [ ] T016 [LJ1] Add safety warnings for critical topics in docs/module-[x]/[safety].md
- [ ] T017 [LJ1] Add glossary cross-references for terminology consistency

**Checkpoint**: At this point, Learning Journey 1 should be fully educational and testable independently

---

## Phase 4: Learning Journey 2 - [Title] (Priority: P2)

**Goal**: [Brief description of what this learning journey teaches students]

**Independent Test**: [How to verify students can learn this content independently]

### Educational Assessments for Learning Journey 2 (OPTIONAL - only if assessments requested) ⚠️

- [ ] T018 [P] [LJ2] Create hands-on exercise for [concept] in docs/module-[x]/exercises.md
- [ ] T019 [P] [LJ2] Create knowledge check questions for [learning objective] in docs/module-[x]/exercises.md

### Implementation for Learning Journey 2

- [ ] T020 [P] [LJ2] Create foundational concept explanation in docs/module-[x]/[concept].md
- [ ] T021 [LJ2] Implement practical tutorial content in docs/module-[x]/[tutorial].md
- [ ] T022 [LJ2] Add cross-module references to Learning Journey 1 content (if needed)
- [ ] T023 [LJ2] Integrate with prerequisite knowledge from previous modules

**Checkpoint**: At this point, Learning Journeys 1 AND 2 should both work independently

---

## Phase 5: Learning Journey 3 - [Title] (Priority: P3)

**Goal**: [Brief description of what this learning journey teaches students]

**Independent Test**: [How to verify students can learn this content independently]

### Educational Assessments for Learning Journey 3 (OPTIONAL - only if assessments requested) ⚠️

- [ ] T024 [P] [LJ3] Create hands-on exercise for [concept] in docs/module-[x]/exercises.md
- [ ] T025 [P] [LJ3] Create knowledge check questions for [learning objective] in docs/module-[x]/exercises.md

### Implementation for Learning Journey 3

- [ ] T026 [P] [LJ3] Create foundational concept explanation in docs/module-[x]/[concept].md
- [ ] T027 [LJ3] Implement practical tutorial content in docs/module-[x]/[tutorial].md
- [ ] T028 [LJ3] Create capstone integration with earlier modules (ROS2 → Simulation → Isaac → VLA)

**Checkpoint**: All learning journeys should now be independently educational

---

[Add more learning journey phases as needed, following the same pattern]

---

## Phase N: Polish & Cross-Cutting Educational Concerns

**Purpose**: Improvements that affect multiple learning journeys

- [ ] TXXX [P] Educational content consistency review across all modules in docs/
- [ ] TXXX Technical accuracy verification and cross-validation with authoritative sources
- [ ] TXXX Educational flow optimization across all learning journeys
- [ ] TXXX [P] Additional exercises and assessments (if requested) in docs/*/exercises.md
- [ ] TXXX Accessibility improvements for diverse learning needs
- [ ] TXXX Run textbook validation and cross-reference checks

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all learning journeys
- **Learning Journeys (Phase 3+)**: All depend on Foundational phase completion
  - Learning journeys can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3)
- **Polish (Final Phase)**: Depends on all desired learning journeys being complete

### Learning Journey Dependencies

- **Learning Journey 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other journeys
- **Learning Journey 2 (P2)**: Can start after Foundational (Phase 2) - May integrate with LJ1 but should be independently learnable
- **Learning Journey 3 (P3)**: Can start after Foundational (Phase 2) - May integrate with LJ1/LJ2 but should be independently learnable

### Within Each Learning Journey

- Educational assessments (if included) MUST be written and validated before content implementation
- Foundational concepts before practical tutorials
- Tutorials before code examples
- Core content before integration
- Journey complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- Once Foundational phase completes, all learning journeys can start in parallel (if team capacity allows)
- All assessments for a learning journey marked [P] can run in parallel
- Content pieces within a journey marked [P] can run in parallel
- Different learning journeys can be worked on in parallel by different team members

---

## Parallel Example: Learning Journey 1

```bash
# Launch all educational assessments for Learning Journey 1 together (if assessments requested):
Task: "Create hands-on exercise for [concept] in docs/module-[x]/exercises.md"
Task: "Create knowledge check questions for [learning objective] in docs/module-[x]/exercises.md"

# Launch all content pieces for Learning Journey 1 together:
Task: "Create foundational concept explanation in docs/module-[x]/[concept].md"
Task: "Create practical tutorial content in docs/module-[x]/[tutorial].md"
```

---

## Educational Delivery Strategy

### MVP First (Learning Journey 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all journeys)
3. Complete Phase 3: Learning Journey 1
4. **STOP and VALIDATE**: Test Learning Journey 1 independently with students
5. Deploy/demo if ready

### Incremental Educational Delivery

1. Complete Setup + Foundational → Educational foundation ready
2. Add Learning Journey 1 → Test independently → Deploy/Demo (MVP!)
3. Add Learning Journey 2 → Test independently → Deploy/Demo
4. Add Learning Journey 3 → Test independently → Deploy/Demo
5. Each journey adds educational value without breaking previous learning

### Parallel Team Strategy

With multiple content developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: Learning Journey 1
   - Developer B: Learning Journey 2
   - Developer C: Learning Journey 3
3. Journeys complete and integrate independently

---

## Educational Notes

- [P] tasks = different files, no dependencies
- [LJ] label maps task to specific learning journey for traceability
- Each learning journey should be independently completable and learnable
- Verify educational assessments are valid before implementing content
- Commit after each task or logical group
- Stop at any checkpoint to validate learning journey independently
- Avoid: vague tasks, same file conflicts, cross-journey dependencies that break independent learning
