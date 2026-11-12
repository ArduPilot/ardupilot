# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Bounded inclusion-polygon upload through the firmware fence loader."""

from native_templates import render

OWNER = 'AC_Fence'
POINTS_MAX = 21
POINTS_MIN = 3
POLICY = (f'v1:points-min{POINTS_MIN}-max{POINTS_MAX},lat-lng-i32-le-pairs-zero-pad,'
          'replaces-all-stored-fence-items,blocking-scheduler-semaphore,storage-write,'
          'no-retained-module-pointers')


def select(ir, first_id):
    owner = next((item for item in ir['singletons'] if item['name'] == OWNER), None)
    expected = dict(rename='fence', depends='AP_FENCE_ENABLED', get=None, literal=False, reference=False,
                    semaphore=False, scheduler_semaphore=False, semaphore_pointer=False)
    if owner is None or any(owner[key] != value for key, value in expected.items()):
        raise ValueError('unaudited AC_Fence metadata')
    return specification(first_id)


def specification(first_id):
    return dict(methods=[dict(
        id=first_id, name='fence.write_inclusion_polygon',
        arguments=[dict(type=f'u8,range{POINTS_MIN}-{POINTS_MAX}', name='count'),
                   dict(type=f'i32-le[{2 * POINTS_MAX}]', name='lat_lng')],
        result='bool-u8', policy=POLICY)])


def rust(spec, notice):
    method = spec['methods'][0]
    return notice + render('rust/fence.rs', points_max=POINTS_MAX, points_min=POINTS_MIN,
                           input_size=1 + 8 * POINTS_MAX, binding_id=method['id'])


def cpp(spec):
    method = spec['methods'][0]
    return [
        f'// {method["name"]}',
        'static __attribute__((noinline)) int32_t binding_fence_write_inclusion_polygon(',
        '    const uint8_t *input, uint32_t, uint8_t *output, uint32_t)',
        '{',
        '    (void)input;',
        '    output[0] = 0;',
        '    const uint8_t count = input[0];',
        f'    if (count < {POINTS_MIN} || count > {POINTS_MAX}) {{',
        '        return AP_MODULE_STATUS_INVALID;',
        '    }',
        '#if AP_FENCE_ENABLED && AP_SCHEDULER_ENABLED',
        f'    AC_PolyFenceItem items[{POINTS_MAX}] {{}};',
        f'    for (uint8_t i = 0; i < {POINTS_MAX}; i++) {{',
        '        const int32_t lat = int32_t(BindingWire::read(input + 1 + 8 * i, 4));',
        '        const int32_t lng = int32_t(BindingWire::read(input + 5 + 8 * i, 4));',
        '        if (i >= count) {',
        '            if (lat != 0 || lng != 0) {',
        '                return AP_MODULE_STATUS_INVALID;',
        '            }',
        '            continue;',
        '        }',
        '        if (!check_lat(lat) || !check_lng(lng)) {',
        '            return AP_MODULE_STATUS_INVALID;',
        '        }',
        '        items[i].type = AC_PolyFenceType::POLYGON_INCLUSION;',
        '        items[i].loc = Vector2l{lat, lng};',
        '        items[i].vertex_count = count;',
        '    }',
        '    auto *fence = AC_Fence::get_singleton();',
        '    auto *scheduler = AP_Scheduler::get_singleton();',
        '    if (fence == nullptr || scheduler == nullptr) {',
        '        return AP_MODULE_STATUS_UNAVAILABLE;',
        '    }',
        '    // fence uploads and fence checks run in the main loop',
        '    BindingLock lock(scheduler->get_semaphore());',
        '    output[0] = fence->polyfence().write_fence(items, count) ? 1 : 0;',
        '    return AP_MODULE_STATUS_OK;',
        '#else',
        '    return AP_MODULE_STATUS_UNAVAILABLE;',
        '#endif',
        '}',
        '',
    ]
