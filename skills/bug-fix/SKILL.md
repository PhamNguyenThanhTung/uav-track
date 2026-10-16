# Bug-fix workflow

Use this skill when the user reports incorrect behavior or asks to fix a defect.

1. Reproduce the bug through the user's actual path when possible.
2. Capture the error, logs, failing command, output, screenshot, or runtime state that demonstrates it.
3. Reduce the failure to the smallest reproducible condition.
4. Identify and state the root cause before editing.
5. Add regression coverage when it gives durable value.
6. Apply the smallest fix that addresses the cause.
7. Repeat the original reproduction and verify that the failure is gone.
8. Run nearby regression checks appropriate to the affected component.
9. Report before-and-after evidence, including any end-to-end reproduction that was not possible and why.

Do not declare a bug fixed only because a unit test passes. If the original path cannot be reproduced, say what evidence was used instead and what remains unverified.
