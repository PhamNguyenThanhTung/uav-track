# Runtime verification workflow

Use this skill when static inspection is insufficient to establish behavior.

Choose only checks relevant to the change and its risk. Possible checks include build, lint, unit or component tests, integration tests, runtime smoke tests, API or UI flows, simulation, logs, metrics, database state, and network interaction.

Use this order when applicable:

1. Define the behavior and acceptance evidence.
2. Run the smallest static or build check that can reject the change.
3. Run focused tests for the affected component.
4. Run an integration, smoke, or simulation check when components interact at runtime.
5. Inspect logs, outputs, metrics, or state for the expected result.
6. Record exact commands and classify each result as `PASS`, `FAIL`, or `NOT RUN`.

Do not run an expensive full-system check when a lower-cost check proves the relevant behavior. For this repository, changes involving PX4, flight control, gimbal motion, perception timing, or ROS/Gazebo interfaces normally need runtime or simulation evidence.
