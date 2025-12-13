# TestBoardNoReadme

## Overview

This is a test board used to verify that the ArduPilot CI system correctly
enforces the requirement for README.md files in new board hwdef directories.

## Hardware

- **MCU:** STM32F427
- **Flash:** 2048KB
- **Board ID:** 1234

## Purpose

This board definition exists solely for testing the `test_new_boards.py` script
and should not be used for actual hardware.

## Testing

When this board is added with a README.md file, the CI check should pass.
When added without a README.md file (see the `pr-test/test-board-no-readme` branch),
the CI check should fail with an error message indicating the missing README.md.
