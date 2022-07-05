#!/bin/bash
# wrapper script for those that don't remember our numerous tools

set -ex

pre-commit run --all-files
