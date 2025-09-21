# Repository Guidelines

## Project Structure & Module Organization

- `src/mecabridge_hardware/` houses the ROS 2 system interface, protocol library, configs, and launch assets.
- `firmware/mecabridge_pico/` contains Pico firmware sources plus the protocol data fixtures used by hardware tests.
- `robot/` packages simulation, teleop, and driver utilities; keep launch and config files in sync with hardware schema updates.
- `.docs/` and `specs/001-mecabridge-plan/` record architecture decisions, task statuses, and protocol definitions consulted by reviewers.
- `test/` stores standalone mecabridge harnesses and Docker-ready scripts, while `scripts/` wraps common workspace build flows.

## Build, Test, and Development Commands

- Launch the ROS Humble dev container with `./scripts/dev.sh`; manual alternative is `docker-compose up --build -d`.
- Inside the container, run `colcon build --symlink-install` (use `--packages-select drive_arduino mecabridge_hardware` when scoping changes).
- Execute unit and lint suites via `colcon test --packages-select mecabridge_hardware` followed by `colcon test-result --all --verbose`.
- Legacy mecabridge coverage still runs through `./test/run_tests.sh`, which sources ROS and prints verbose GTest summaries.


## Coding Style & Naming Conventions

- C++ targets C++17 with `-Wall -Wextra -Wpedantic`; prefer `PascalCase` for types, `snake_case` for functions and members with trailing underscores for storage fields.

- Keep protocol helpers exception-safe and align with patterns in `mecabridge_utils`; add comments only where control flow is non-obvious.
- Python utilities under `robot/` follow PEP 8 (4-space indents, explicit cleanup); annotate new GPIO-facing APIs with docstrings and type hints.
- Run `colcon test --packages-select mecabridge_hardware --ctest-args -R lint` before submitting to satisfy `ament_lint_auto` format checks.

## Testing Guidelines

- Place new GTest cases alongside existing suites in `src/mecabridge_hardware/test/mecabridge/`; wire them up with `ament_add_gtest` and reuse shared fixtures.
- Update the binary protocol fixtures in `firmware/mecabridge_pico/data/` when message layouts change; failing to sync breaks watchdog and CRC regression tests.
- Integration exercises assume the Docker environment; document hardware or serial-port dependencies in PRs and guard tests with skips when physical devices are needed.

## Commit & Pull Request Guidelines

- Favor concise, imperative commit subjects; Conventional Commit prefixes (`feat:`, `fix:`) match recent history and ease changelog generation.
- Reference supporting design docs (for example `specs/001-mecabridge-plan/tasks.md`) whenever you close a task or adjust requirements.
- Confirm `colcon build` and targeted tests locally; share command snippets or `colcon test-result` output in the PR description.
- PRs should outline configuration impacts (ports, watchdog timing) and attach screenshots or logs when altering runtime telemetry or dashboards.
