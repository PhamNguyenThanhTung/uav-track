# Phase review workflow

Use this skill for a milestone, risky change, or explicit review request. Review without rewriting code unless the user asks for fixes.

Inspect:

- **SCOPE** — Does the change stay inside the requested task?
- **BEHAVIOR** — Did existing behavior change unintentionally?
- **ARCHITECTURE** — Did it add unnecessary coupling or premature abstraction?
- **TESTS** — Do tests validate the requirement rather than mirror implementation details?
- **RUNTIME** — Is integration, simulation, or end-to-end evidence required and present?
- **SECURITY / SAFETY** — Are credentials, unsafe defaults, destructive actions, or sensitive control paths involved?
- **DOCS** — Do README, progress, and interface contracts describe current behavior?
- **REPOSITORY HYGIENE** — Are there junk, generated files, accidental absolute paths, or unrelated modifications?

Classify findings as `BLOCKER`, `HIGH`, `MEDIUM`, `LOW`, or `INFO`. Return a verdict: `PASS`, `PASS WITH NOTES`, or `CHANGES REQUIRED`, with file/location, issue, and recommended action for each finding.
