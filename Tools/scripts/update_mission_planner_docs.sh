#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"

VEHICLE="${1:-Rover}"
DESTINATION="${2:-C:/ProgramData/Mission Planner}"

if command -v wslpath >/dev/null 2>&1 && [[ "${DESTINATION}" =~ ^[A-Za-z]:[\\/] ]]; then
    DESTINATION="$(wslpath -u "${DESTINATION}")"
fi

PARAM_TOOL="${REPO_ROOT}/Tools/autotest/param_metadata/param_parse.py"
GENERATED_XML="${REPO_ROOT}/apm.pdef.xml"
TARGET_XML="${DESTINATION}/${VEHICLE}.apm.pdef.xml"
TEMP_XML="${TARGET_XML}.tmp.$$"

if [[ ! -f "${PARAM_TOOL}" ]]; then
    echo "Error: parameter metadata generator not found: ${PARAM_TOOL}" >&2
    exit 1
fi

cd "${REPO_ROOT}"

echo "Validating ${VEHICLE} parameter metadata..."
python3 "${PARAM_TOOL}" --vehicle "${VEHICLE}" --no-emit

echo "Generating ${GENERATED_XML}..."
python3 "${PARAM_TOOL}" --vehicle "${VEHICLE}" --format xml

if [[ ! -s "${GENERATED_XML}" ]]; then
    echo "Error: generated XML is missing or empty: ${GENERATED_XML}" >&2
    exit 1
fi

if ! grep -Fq "<parameters name=\"${VEHICLE}\">" "${GENERATED_XML}"; then
    echo "Error: generated XML does not contain ${VEHICLE} parameters" >&2
    exit 1
fi

mkdir -p "${DESTINATION}"

cleanup() {
    rm -f -- "${TEMP_XML}"
}
trap cleanup EXIT

cp -- "${GENERATED_XML}" "${TEMP_XML}"
mv -f -- "${TEMP_XML}" "${TARGET_XML}"

trap - EXIT

echo "Mission Planner documentation updated:"
echo "  ${TARGET_XML}"
echo "Restart Mission Planner to reload the parameter descriptions."
