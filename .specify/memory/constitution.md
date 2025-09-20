<!--
Sync Impact Report
- Version change: 1.0.0 → 1.1.0
- Modified principles: (text condensed, intent preserved)
  - "Modularity (Package-per-HAL)" (clarified scope)
  - "ROS 2 & ros2-control Compatibility" (tightened phrasing)
  - "Safety & Determinism" → renamed "Safety & Fail-Safe Operation"
  - "Test-First & Hardware-in-the-Loop (HIL)" → renamed "Test-First & HIL"
  - "Documentation, ABI Stability & Versioning" → renamed "Versioned & Stable Contracts"
- Added sections: Acceptance Checklist (new minimal section)
- Removed sections: Additional Constraints (folded into principles), Development Workflow (scoped out for minimal form)
- Templates checked:
  - .specify/templates/plan-template.md ✅ version reference updated
  - .specify/templates/spec-template.md ✅ no changes needed
  - .specify/templates/tasks-template.md ✅ no changes needed
- Follow-up TODOs:
  - TODO(RATIFICATION_DATE): original adoption date still unknown; set when historical date established
  - Consider adding a lightweight CONTRIBUTING.md referencing this minimal constitution
-->

# My Steel Robot HAL Constitution (Minimal Form)

Purpose: Define the non‑negotiable rules for a modular ROS 2 (Humble) Hardware Abstraction Layer (HAL) project compatible with ros2-control. Keep surface area small; every rule must be enforceable.

## Core Principles

### 1. Modularity (Package-per-HAL)
Each hardware abstraction (HAL) MUST be a single ament package (`ament_cmake` for C++ HALs; `ament_python` only for tooling/tests). Public APIs MUST be narrow, documented, and independently buildable/installable. Cross-package coupling MUST occur only via ROS 2 interfaces (topics, services, params) or explicitly exported C++ headers.
Rationale: Isolated evolution of hardware drivers prevents cascading breakage and simplifies reuse and CI.

### 2. ROS 2 & ros2-control Compatibility
HAL packages that expose actuators or sensors MUST integrate (when applicable) through ros2-control hardware interfaces, exporting the correct state/command interfaces with stable naming. Parameters, lifecycle (if used), and namespaces MUST follow ROS 2 conventions. No custom control loop duplication when ros2-control suffices.
Rationale: Ensures controllers and higher-level stacks interoperate without bespoke adapters.

### 3. Safety & Fail-Safe Operation
Actuator-affecting code MUST implement: bounded timeouts, explicit enable/disable (E‑Stop path), and conservative startup defaults (motors off, safe torque/speed). Control/data paths used in real-time loops MUST avoid unbounded blocking calls (I/O, dynamic allocation spikes) and MUST document worst-case timing assumptions.
Rationale: Prevents undefined or hazardous hardware motion and supports predictable control latency.

### 4. Test-First & HIL
Any change to a public HAL API or hardware I/O logic MUST add or update tests before implementation is merged. Unit tests cover logic; integration or HIL tests cover on-wire / protocol / motion side effects. HIL tests MAY be conditionally skipped in generic CI, but MUST have a documented invocation path for hardware runners.
Rationale: Protects against regressions that could damage hardware or destabilize control loops.

### 5. Versioned & Stable Contracts
Public C++ headers, parameter schemas, message/service definitions, and ros2-control interface names form the stability surface. Changes follow semantic versioning (MAJOR = breaking/removal, MINOR = additive, PATCH = corrective). Breaking changes MUST include migration notes and appear only in a MAJOR bump.
Rationale: Downstream controllers and integration scripts require predictable upgrade paths.

## Acceptance Checklist (Per HAL Package)
All MUST be satisfied before a new HAL package is considered ready:
1. Independent build: `colcon build --packages-select <pkg>` succeeds.
2. Public API docs: README with purpose, exported interfaces, safety notes.
3. ros2-control (if applicable): Hardware interface registers required state & command interfaces; example controller config provided.
4. Safety: Documented enable/disable, timeouts, and startup defaults.
5. Tests: Unit + (integration or HIL) present; `colcon test --packages-select <pkg>` passes (HIL may be conditionally skipped with clear marker).
6. Version metadata: `package.xml` version updated + CHANGELOG entry.

## Governance
Amendment requires a PR describing the rationale and version bump justification. One maintainer approval suffices for clarifications (PATCH). Adding or materially redefining a principle requires maintainer consensus (MINOR or MAJOR). Removal or redefinition that weakens safety/modularity/compatibility requires a MAJOR bump and explicit migration note. 

Compliance: Each PR touching HAL logic MUST include a short "Constitution Check" summarizing adherence (or justified deviations). CI SHOULD gate on: build, tests, lint/format, and detection of missing Constitution Check text for relevant changes.

Versioning Policy (this document):
- MAJOR: Remove or fundamentally redefine a core principle.
- MINOR: Add a principle or expand enforceable scope.
- PATCH: Non-semantic clarifications (grammar, wording) only.

**Version**: 1.1.0 | **Ratified**: TODO(19-09-2025): set original adoption date | **Last Amended**: 2025-09-19

