#!/usr/bin/env bash

set -euo pipefail

destination="${1:-build/renode}"
branch="pr-arudpilot-am32-perf"

if [[ ! -d "${destination}/.git" ]]; then
    git clone --branch "${branch}" --single-branch \
        https://github.com/ArduPilot/renode.git "${destination}"
fi

destination="$(cd "${destination}" && pwd)"
git -C "${destination}" submodule update --init --recursive
(
    cd "${destination}"
    ./perf_patches/apply.sh
    ./build.sh --net
)

echo "Renode executable: ${destination}/renode"
