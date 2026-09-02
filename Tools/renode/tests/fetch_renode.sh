#!/usr/bin/env bash

set -euo pipefail

destination="${1:-build/renode}"
base_url="${RENODE_PACKAGE_BASE_URL:-https://firmware.ardupilot.org/Tools/Renode}"

if [[ "$(uname -s)" != "Linux" ]]; then
    echo "Only Linux Renode packages are supported by this helper" >&2
    exit 1
fi

case "$(uname -m)" in
    x86_64|amd64)
        architecture="x86_64"
        artifact="renode-linux-x86_64"
        ;;
    aarch64|arm64)
        architecture="aarch64"
        artifact="renode-linux-aarch64"
        ;;
    *)
        echo "No published Renode package for architecture $(uname -m)" >&2
        exit 1
        ;;
esac

destination_parent="$(dirname "${destination}")"
mkdir -p "${destination_parent}"
if [[ -e "${destination}" ]]; then
    echo "Renode destination already exists: ${destination}" >&2
    exit 1
fi
temporary="$(mktemp -d "${destination_parent}/.renode-download.XXXXXX")"
trap 'rm -rf -- "${temporary}"' EXIT

manifest="${temporary}/latest.json"
package_data="${temporary}/package-data"
curl --fail --location --retry 5 --retry-all-errors \
    --output "${manifest}" "${base_url}/latest.json"

python3 - "${manifest}" "${artifact}" "${architecture}" \
    "${RENODE_SOURCE_REVISION:-}" > "${package_data}" <<'PY'
import json
import pathlib
import re
import sys

manifest_path = pathlib.Path(sys.argv[1])
artifact_name = sys.argv[2]
architecture = sys.argv[3]
expected_revision = sys.argv[4]
manifest = json.loads(manifest_path.read_text(encoding="utf-8"))

if manifest.get("schema_version") != 1:
    raise SystemExit("unsupported Renode package manifest schema")

matches = [entry for entry in manifest.get("artifacts", []) if entry.get("artifact") == artifact_name]
if len(matches) != 1:
    raise SystemExit(f"manifest does not contain exactly one {artifact_name} artifact")

entry = matches[0]
expected_target = {
    "architecture": architecture,
    "platform": "linux",
    "runtime_identifier": "linux-arm64" if architecture == "aarch64" else "linux-x64",
}
if entry.get("target") != expected_target:
    raise SystemExit(f"unexpected Renode package target: {entry.get('target')!r}")

packages = entry.get("packages")
if not isinstance(packages, list) or len(packages) != 1:
    raise SystemExit("Linux Renode artifact must contain exactly one package")

package = packages[0]
filename = package.get("filename")
checksum = package.get("sha256")
size = package.get("size")
revision = manifest.get("source", {}).get("revision")
if not isinstance(filename, str) or not re.fullmatch(r"[A-Za-z0-9._+-]+\.tar\.gz", filename):
    raise SystemExit("unsafe or invalid Renode package filename")
if not isinstance(checksum, str) or not re.fullmatch(r"[0-9a-f]{64}", checksum):
    raise SystemExit("invalid Renode package SHA-256")
if not isinstance(size, int) or size <= 0:
    raise SystemExit("invalid Renode package size")
if not isinstance(revision, str) or not re.fullmatch(r"[0-9a-f]{40}", revision):
    raise SystemExit("invalid Renode source revision")
if expected_revision and revision != expected_revision:
    raise SystemExit(
        f"Renode source revision is {revision}, expected {expected_revision}"
    )

print(filename)
print(checksum)
print(size)
print(revision)
PY

mapfile -t package_fields < "${package_data}"
if [[ "${#package_fields[@]}" -ne 4 ]]; then
    echo "Failed to parse the Renode package manifest" >&2
    exit 1
fi

package_filename="${package_fields[0]}"
package_checksum="${package_fields[1]}"
package_size="${package_fields[2]}"
source_revision="${package_fields[3]}"
package_path="${temporary}/${package_filename}"
curl --fail --location --retry 5 --retry-all-errors \
    --output "${package_path}" "${base_url}/${package_filename}"

actual_size="$(stat --format=%s "${package_path}")"
if [[ "${actual_size}" != "${package_size}" ]]; then
    echo "Renode package size is ${actual_size}, expected ${package_size}" >&2
    exit 1
fi
printf '%s  %s\n' "${package_checksum}" "${package_path}" | sha256sum --check --status

extracted="${temporary}/extracted"
mkdir "${extracted}"
tar -xzf "${package_path}" --strip-components=1 -C "${extracted}"
if [[ ! -x "${extracted}/renode" ]]; then
    echo "Downloaded package has no Renode executable" >&2
    exit 1
fi
mv "${extracted}" "${destination}"

echo "Renode source revision: ${source_revision}"
echo "Renode executable: ${destination}/renode"
"${destination}/renode" --version
