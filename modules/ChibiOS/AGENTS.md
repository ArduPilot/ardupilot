# Repository Guidelines

## Project Structure & Module Organization
- Operating system modules are under `./os`.
- Core STM32 HAL sources live under `os/hal/ports/STM32`.
- Shared STM32 infrastructure (RCC helpers, registry, limits) sits alongside in the same tree, while board examples and demos are under `demos/`.
- Tests and validation harnesses reside in the `test*` directories; use them to verify changes before submitting.

## Coding Style & Naming Conventions
- Follow existing style patterns: 2-space indentation for C sources, and `STM32_*` macro naming for register constants (see `hal_lld.h`/`stm32_rcc.h`).
- Keep functions `static` unless they are part of the public HAL API.
- Regenerate formatting by hand; clang-format is not used on this port—mirror the surrounding code style.
- General rule: line endings must be LF, except for externally provided files (non-ChibiOS copyright).

## Build & Test Hygiene
- Clean projects after test builds unless otherwise specified.

## Repository
- Repository is subversion, ignore .svn directory, do not use git commands.
- Ask for confirmation before touching non-versioned files.
