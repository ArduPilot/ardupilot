# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Owned, pointer-free CRSF scripted-menu bindings."""

from native_templates import render

OWNER = 'AP_CRSF_Telem'
MENU = 'CRSFMenu'
PARAMETER = 'CRSFParameter'
MENU_NAME_MAX = 15
PARAMETER_DATA_MAX = 255
EVENT_PAYLOAD_MAX = 57
POLICY = ('v1:module-context-owned-menus,opaque-generation-tokens,no-pointer-wire,'
          'cleanup-after-worker-join,lua-restart-isolation,queued-pointer-prune-before-free,'
          'menu-name-nul-free-bytes-max15,parameter-bytes-max255,event-payload-max57,'
          'head-event-owner-validation,response-owner-validation,blocking-locks')


def _field_shape(field):
    return (field['kind'], field['name'], field['nullable'], field['reference'], field['pointer'],
            field['skip_check'], tuple(field['range']) if field['range'] else None)


def _alias(owner, alias, target):
    matches = [item for item in owner['aliases'] if item['alias'] == alias]
    expected = dict(name=target, alias=alias, depends=None, manual=True)
    if matches != [expected]:
        raise ValueError(f'unaudited {owner["name"]}.{alias} declaration')


def select(ir, first_id):
    crsf = next((item for item in ir['singletons'] if item['name'] == OWNER), None)
    expected_crsf = dict(rename='crsf', depends='AP_CRSF_SCRIPTING_ENABLED', get=None, literal=False,
                         reference=False, semaphore=True, scheduler_semaphore=False,
                         semaphore_pointer=False)
    if crsf is None or any(crsf[key] != value for key, value in expected_crsf.items()):
        raise ValueError('unaudited AP_CRSF_Telem owner metadata')
    for alias, target in (('add_menu', 'lua_CRSF_add_root_menu'),
                          ('peek_menu_event', 'lua_CRSF_peek_menu_event'),
                          ('send_write_response', 'lua_CRSF_send_response')):
        _alias(crsf, alias, target)
    expected_methods = {
        'pop_menu_event': (('void', None, False, False, False, False, None), []),
        'send_response': (('boolean', None, False, False, False, False, None), []),
    }
    actual_methods = {
        method['name']: (_field_shape(method['return']), [_field_shape(arg) for arg in method['arguments']])
        for method in crsf['methods'] if method['name'] in expected_methods
    }
    if actual_methods != expected_methods:
        raise ValueError('unaudited AP_CRSF_Telem event methods')

    menu = next((item for item in ir['userdata'] if item['name'] == MENU), None)
    expected_menu = dict(rename=None, depends='AP_CRSF_SCRIPTING_ENABLED', get=None,
                         creation='lua_CRSF_new_menu', creation_manual=True, creation_args=1,
                         has_fields=False, has_enums=False, operations=0, operator_getter=None,
                         operators_manual=False, operator_names=[], literal=False, reference=False,
                         semaphore=False, scheduler_semaphore=False, semaphore_pointer=False)
    if menu is None or any(menu[key] != value for key, value in expected_menu.items()):
        raise ValueError('unaudited CRSFMenu metadata')
    _alias(menu, 'add_menu', 'lua_CRSF_add_menu')
    _alias(menu, 'add_parameter', 'lua_CRSF_add_parameter')
    expected_menu_methods = {
        'id': (('uint8_t', None, False, False, False, True, None), []),
        'name': (('string', None, False, False, False, False, None), []),
        'num_params': (('uint8_t', None, False, False, False, True, None), []),
    }
    actual_menu_methods = {
        method['name']: (_field_shape(method['return']), [_field_shape(arg) for arg in method['arguments']])
        for method in menu['methods'] if method['name'] in expected_menu_methods
    }
    if actual_menu_methods != expected_menu_methods:
        raise ValueError('unaudited CRSFMenu metadata methods')

    parameter = next((item for item in ir['userdata'] if item['name'] == PARAMETER), None)
    expected_parameter = dict(rename=None, depends='AP_CRSF_SCRIPTING_ENABLED', get=None,
                              creation=None, creation_manual=False, creation_args=0,
                              has_fields=False, has_enums=False, operations=0, operator_getter=None,
                              operators_manual=False, operator_names=[], literal=False, reference=False,
                              semaphore=False, scheduler_semaphore=False, semaphore_pointer=False)
    if parameter is None or any(parameter[key] != value for key, value in expected_parameter.items()):
        raise ValueError('unaudited CRSFParameter metadata')
    id_method = next((item for item in parameter['methods'] if item['name'] == 'id'), None)
    expected_id = ('uint8_t', None, False, False, False, True, None)
    if (id_method is None or id_method['rename'] is not None or id_method['depends'] is not None or
            id_method['deprecate'] is not None or _field_shape(id_method['return']) != expected_id or
            id_method['arguments']):
        raise ValueError('unaudited CRSFParameter.id declaration')
    _alias(parameter, 'data', 'lua_CRSF_param_data')
    return specification(first_id)


def specification(first_id):
    methods = [
        dict(id=first_id, name='crsf.add_menu', arguments=['menu-name'], result='optional-menu-token'),
        dict(id=first_id + 1, name='CRSFMenu.add_menu', arguments=['menu-token', 'menu-name'],
             result='optional-menu-token'),
        dict(id=first_id + 2, name='CRSFMenu.add_parameter',
             arguments=['menu-token', 'bytes-len-u8-max255-zero-pad255'], result='optional-parameter-token'),
        dict(id=first_id + 3, name='CRSFParameter.id', arguments=['parameter-token'], result='u8'),
        dict(id=first_id + 4, name='CRSFParameter.data', arguments=['parameter-token'],
             result='bytes-len-u8-max255-zero-pad255'),
        dict(id=first_id + 5, name='crsf.peek_menu_event', arguments=[],
             result='optional-owned-event-count-param-events-payload57'),
        dict(id=first_id + 6, name='crsf.pop_menu_event', arguments=['param-id', 'events'], result='bool-u8'),
        dict(id=first_id + 7, name='crsf.send_write_response',
             arguments=['bytes-len-u8-max255-zero-pad255'], result='bool-u8'),
        dict(id=first_id + 8, name='crsf.send_response', arguments=[], result='bool-u8'),
        dict(id=first_id + 9, name='crsf.get_menu_event', arguments=['event-mask-u8'],
             result='optional-owned-event-param-events-payload57'),
        dict(id=first_id + 10, name='CRSFMenu.id', arguments=['menu-token'], result='u8'),
        dict(id=first_id + 11, name='CRSFMenu.name', arguments=['menu-token'],
             result='utf8-len-u8-max15-zero-pad15'),
        dict(id=first_id + 12, name='CRSFMenu.num_params', arguments=['menu-token'], result='u8'),
    ]
    for method in methods:
        method['policy'] = POLICY
    return dict(methods=methods)


def rust(spec, notice):
    (add_root, add_menu, add_parameter, parameter_id, parameter_data,
     peek, pop, send_write, send, get_event, menu_id, menu_name, menu_num_params) = spec['methods']
    return notice + render(
        'rust/crsf.rs',
        menu_name_max=MENU_NAME_MAX,
        parameter_data_max=PARAMETER_DATA_MAX,
        event_payload_max=EVENT_PAYLOAD_MAX,
        add_root_id=add_root['id'],
        peek_id=peek['id'],
        pop_id=pop['id'],
        send_write_id=send_write['id'],
        send_id=send['id'],
        get_event_id=get_event['id'],
        menu_id_id=menu_id['id'],
        menu_name_id=menu_name['id'],
        menu_num_params_id=menu_num_params['id'],
        add_menu_id=add_menu['id'],
        add_parameter_id=add_parameter['id'],
        parameter_id_id=parameter_id['id'],
        parameter_data_id=parameter_data['id'],
    )
