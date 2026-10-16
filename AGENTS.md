# Agent workflow

This repository contains a ROS 2, Gazebo, PX4 SITL, and perception project. Use this file for rules that apply to nearly every task; use the skills under `skills/` for detailed procedures.

## Scope and repository boundaries

- This directory is the Git root. `../software` contains external PX4, ROS 2, QGroundControl, and virtual environments. Do not modify it unless the user explicitly asks.
- Inspect the actual implementation and current working tree before changing anything. Existing user changes belong to the user.
- Read only task-relevant documentation. Use `PROGRESS.md`, `README.md`, and `docs/system_contract.md` when they affect the task.

## Change rules

- Check for existing functionality before creating code or workflow files.
- Prefer the smallest change that solves the request; do not expand scope silently.
- Preserve working behavior unless behavior change is requested.
- Do not delete, move, rename, or replace user files or convenience symlinks without explicit permission, unless the task itself requires it.
- Do not modify historical milestone reports under `docs/simulation/` unless requested.
- Do not add infrastructure, CI, containers, orchestration, or custom tools before repeated project pain justifies it.
- Never claim proposed or planned behavior is implemented.
- Avoid machine-specific absolute paths in source, scripts, and documentation; use repository-relative paths or environment variables.

## Verification and risk

- Select checks by risk: docs and mechanical edits need focused checks; logic changes need relevant tests; flight control, autonomous movement, safety, deletion, and deployment changes require a plan, strong review, and runtime or integration evidence.
- For high-risk robotics/control changes, verify relevant units, coordinate frames, time source / timestamps, loss/failsafe behavior, and runtime integration (refer to `skills/runtime-verification`).
- For bugs use: `REPRODUCE -> CAPTURE EVIDENCE -> FIX -> REPRODUCE AGAIN`. A passing unit test alone does not prove an end-to-end bug is fixed.
- For risky work use: `PLAN -> IMPLEMENT -> TEST -> RUNTIME VERIFY -> REVIEW -> ACCEPT`.
- When implementation changes the documented current behavior or interface, update the affected current source-of-truth documentation. Do not update unrelated docs or rewrite historical records. Use `skills/bug-fix`, `skills/runtime-verification`, and `skills/phase-review` when their procedures apply.

## Completion report

Report concisely:

- Changed: files and purpose.
- Created, modified, and intentionally unchanged files.
- Verification: checks performed with `PASS`, `FAIL`, or `NOT RUN`.
- Risks / remaining issues: meaningful items only.
- Next: only a clear next step.
