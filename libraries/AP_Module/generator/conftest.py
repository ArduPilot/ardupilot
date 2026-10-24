# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Make generated headers available to generator compilation tests."""

import json
import os

import generate

OUTPUT = generate.ROOT.parents[1] / 'build/tests/AP_Module'
generate.write_outputs(
    generate.generate(json.loads((generate.ROOT / 'generator/host_api.json').read_text())),
    OUTPUT,
)
os.environ['CPLUS_INCLUDE_PATH'] = os.pathsep.join(
    value for value in (
        str(OUTPUT), str(OUTPUT.parent), str(generate.ROOT), str(generate.ROOT.parent),
        os.environ.get('CPLUS_INCLUDE_PATH'),
    ) if value
)
os.environ['AP_MODULE_SDK_GENERATED'] = str(OUTPUT / 'sdk/rust/src')
