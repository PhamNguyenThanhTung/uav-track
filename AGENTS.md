# Operating Contract for Coding Agents

This document defines the mandatory operating contract, behavioral rules, and development workflow for AI coding assistants (Agent, Codex, Gemini) working in this repository.

---

## 1. Project & Environment Roots

- **Project Root (Git Root)**:
  ```text
  /home/tungt/UAV_Project/project
  ```
  All project-owned source code, configuration, simulation assets, tests, and documentation reside here.

- **External Vendor & Software Dependencies**:
  ```text
  /home/tungt/UAV_Project/software
  ```
  Contains upstream/vendor software: PX4-Autopilot, ROS 2 dependency workspace (`ros2_ws`), Python virtual environments (`venvs/yolo26`), and QGroundControl.
  **STRICT RULE**: Do NOT modify, delete, rename, commit, or restructure anything under `../software` unless explicitly requested by the user.

---

## 2. Source-of-Truth Hierarchy

When code, runtime, and documentation disagree, establish current system state using this strict precedence:

1. **Runtime Evidence**: Observed topics, simulator state, execution logs.
2. **Current Source Code & Configuration**: What is actively compiled and executed.
3. **Current Workflow Documentation**: `docs/system_contract.md`, `docs/project_overview.md`, `PROGRESS.md`.
4. **Historical Milestone Documentation**: `docs/simulation/*.md` (archived records of past phases).

---

## 3. Mandatory Protocol Before Any Task

Before creating, modifying, or deleting any file, future agents MUST:

1. Read `AGENTS.md`.
2. Read `PROGRESS.md` to identify the current baseline and active scope.
3. Read `docs/project_overview.md` to understand system architecture.
4. Read `docs/system_contract.md` to respect topic interfaces, QoS, and boundary contracts.
5. Inspect all files directly relevant to the task (read code first, do not guess).
6. Check for existing implementation before writing new code to avoid duplication.
7. State explicitly which files will be created or modified.
8. Make the minimal necessary change scoped strictly to the user request.
9. Run relevant verification tests (unit tests, syntax checks, or simulation checks).
10. Update `PROGRESS.md` upon completion if project state changed.

---

## 4. Explicit Safety Rules

- **DO NOT delete or rename existing files** without explicit user approval.
- **DO NOT delete or replace convenience symlinks** (`benchmark_evaluator.py`, `metric_calculator.py`).
- **DO NOT modify external software** under `../software` (PX4 source, `px4_msgs`, MicroXRCEAgent, virtual environments).
- **DO NOT claim proposed functionality is implemented**. Distinguish `IMPLEMENTED` from `PROPOSED / NOT IMPLEMENTED`.
- **DO NOT modify historical milestone documents** (`docs/simulation/*.md`) unless specifically instructed.
- **DO NOT silently restructure packages** or change directory hierarchies.
- **DO NOT introduce unrequested infrastructure**: No Docker, CI/CD pipelines, devcontainers, Bazel, complex ADRs, or enterprise frameworks.
- **DO NOT expand task scope** beyond what the user asked.
- **DO NOT hardcode user session paths** (e.g. paths inside `.gemini/antigravity/brain/...`). Use repository-relative paths or environment variables.

---

## 5. Reporting Protocol

At the conclusion of every task, always report:

1. **Files inspected** (path and reason).
2. **Files created** (path and purpose).
3. **Files modified** (path, exact reason, summary of diff).
4. **Files intentionally left unchanged** (rationale).
5. **Tests and verifications performed** (command, exact output, PASS/FAIL).
6. **Remaining issues or limitations** discovered during the task.

---

## 6. Git Workflow

Follow this clean, sequential commit workflow:

```text
Inspect -> Modify -> Verify -> Review `git diff` -> Commit milestone
```

- Keep commits meaningful and atomic:
  - `docs: update system contract for perception topics`
  - `fix: correct QoS profile on vehicle_local_position subscription`
  - `feat: add target identity tracking filter`
- Never run destructive Git commands against user work (`git reset --hard`, `git clean -fd`, `git checkout .`).
