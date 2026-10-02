#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)
# shellcheck source-path=SCRIPTDIR
# shellcheck source=common.sh
. "$SCRIPT_DIR/common.sh"

for command_name in \
    awk bash bison bzip2 cc c++ curl flex make makeinfo patch realpath sed \
    sha256sum sha512sum tar xz
do
    require_command "$command_name"
done

python_bin=${PYTHON:-python3}
require_command "$python_bin"

python_include=$($python_bin -c 'import sysconfig; print(sysconfig.get_path("include"))')
[[ -f $python_include/Python.h ]] || die \
    "Python development headers not found ($python_include/Python.h)"

probe_dir=$(mktemp -d)
trap 'rm -rf "$probe_dir"' EXIT

printf '%s\n' \
    '#include <gmp.h>' \
    '#include <mpfr.h>' \
    '#include <mpc.h>' \
    '#include <expat.h>' \
    'int main(void) { return 0; }' >"$probe_dir/host-libs.c"

if ! cc "$probe_dir/host-libs.c" -lgmp -lmpfr -lmpc -lexpat -o "$probe_dir/host-libs"; then
    die "GMP, MPFR, MPC, or Expat development files are missing; see README.md"
fi

note "Host prerequisites found"
note "Python: $($python_bin --version 2>&1) ($python_include)"
