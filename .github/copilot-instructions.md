# Copilot instructions — ArduPilot repository

Purpose
-------
Short, actionable guidance for Copilot sessions: how to build, run tests, lint, and where to look for cross-file architecture and project conventions.

Quick commands (build, test, lint)
---------------------------------
Prerequisites:
- Ensure submodules: `git submodule update --init --recursive`
- Python >= 3.8 (root `./waf` enforces this)

Build (common examples):
- Configure SITL debug build:
  `./waf configure --board sitl --debug`
- Build a vehicle binary (SITL):
  `./waf build --target bin/arducopter`  # replace arducopter with ardup* binary
- Build all / default targets:
  `./waf all` or `./waf <vehicle>` (e.g. `./waf copter`, `./waf plane`)
- Build via CI-style wrapper (used by Actions):
  `Tools/scripts/build_ci.sh`

Tests
-----
Python tests (pytest):
- Run whole suite: `pytest tests/`
- Run a single test: `pytest tests/path/to/test_module.py::test_name -q`

Compiled / SITL tests (autotest):
- Build unit tests: `./waf configure --board linux --debug && ./waf tests`
- Or use autotest helper: `Tools/autotest/autotest.py build.unit_tests run.unit_tests`
- List available autotest steps: `Tools/autotest/autotest.py --list`
- List all subtests: `Tools/autotest/autotest.py --list-subtests`
- List subtests for a vehicle: `Tools/autotest/autotest.py --list-subtests-for-vehicle Copter`
- Run all Copter SITL tests: `Tools/autotest/autotest.py test.Copter`
- Run a specific SITL subtest: `Tools/autotest/autotest.py test.Copter.<SubTestName>` (e.g. `test.CopterTests1a` or `test.Copter.CPUFailsafe`)
- Run SITL with debug/gdb: `Tools/autotest/autotest.py --debug --gdb test.Copter`

Lint / style
------------
- Run pre-commit hooks (same as CI): `pre-commit run --all-files`
- Flake8 (project-specific checker): `Tools/scripts/run_flake8.py` (checks files containing the `AP_FLAKE8_CLEAN` marker)
- AStyle (C/C++ style): `Tools/scripts/run_astyle.py --dry-run` (remove `--dry-run` to apply)
- Other helpers: `Tools/CodeStyle/*`, `Tools/gittools/*`

High-level architecture (big-picture)
------------------------------------
- Top-level vehicle folders: `ArduCopter`, `ArduPlane`, `Rover`, `ArduSub`, `AntennaTracker` — vehicle-specific flight code.
- `libraries/` — shared libraries (AP_* components: math, sensors, drivers, etc.).
- `modules/` — third-party code and helpers (includes `modules/waf`, `mavlink/pymavlink`, gtest, etc.).
- `Tools/` — build/test helpers, SITL helpers, autotest framework, code-style scripts, packaging.
- `build/<config>/bin/<binary>` — out-of-tree build artifacts produced by `waf` (e.g. `build/sitl/bin/arducopter`).
- `tests/` — Python pytest-based tests. `Tools/autotest/` holds SITL/autotest infrastructure and tests.

Key repo conventions (non-obvious patterns)
------------------------------------------
- Build system: `./waf` (root wrapper). Always `./waf configure --board <board>` before builds. `modules/waf` is a submodule; the wrapper auto-inits it if missing.
- Board names: use `--board <board>`; `sitl` is the simulator target. Many board names are defined in hwdef directories (see Tools/scripts/configure_all.py).
- Build artifact locations: `build/<config>/bin/<name>`; SITL binaries are typically `bin/arducopter`, `bin/arduplane`, etc.
- Tests: two kinds — Python pytest tests under `tests/`, and compiled/SITL tests under `Tools/autotest/`.
- Autotest step naming: strings like `build.Copter`, `test.Copter`, or `test.Copter.<SubTest>`; list and run via `Tools/autotest/autotest.py` (use `--list` / `--list-subtests` flags).
- AP_FLAKE8_CLEAN: Python files containing this marker are expected flake8-clean; `Tools/scripts/run_flake8.py` checks only those files.
- Pre-commit and CI: CI uses pre-commit, ccache, Docker images `ardupilot/ardupilot-dev-*`. To reproduce CI locally, use `Tools/scripts/build_ci.sh` or the same container image.
- Style tools live in `Tools/CodeStyle/` (astyle settings) and pre-commit is configured in `.pre-commit-config.yaml` (note excluded paths).

Where to read more
------------------
- Root README.md and developer wiki: https://ardupilot.org/dev/
- Autotest internals: `Tools/autotest/autotest.py` and `Tools/autotest/*`

Notes for Copilot sessions
--------------------------
- Prioritize referencing: `README.md`, `.github/workflows/*` (CI commands), `Tools/scripts/build_ci.sh`, `Tools/autotest/autotest.py`, and `./waf` (root wrapper) when suggesting build/test/lint commands.
- For test-specific tasks, inspect `Tools/autotest/<vehicle>.py` to find subtest names and behaviors.
- For Python formatting/lint guidance, prefer `Tools/scripts/run_flake8.py` and the pre-commit config over generic rules.

Files checked while creating this guidance:
- README.md, .github/CONTRIBUTING.md, .pre-commit-config.yaml, Tools/scripts/*, Tools/autotest/autotest.py, modules/waf, .github/workflows/*

If you want this adjusted (more vehicle-specific examples, CI-to-local reproduction steps, or per-directory rules), say which area to expand.
