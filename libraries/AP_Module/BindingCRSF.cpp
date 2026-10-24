// SPDX-License-Identifier: GPL-3.0-or-later
#include "BindingCRSF.h"

#if AP_MODULE_ENABLED
#include "BindingSupport.h"
#include <AP_Module/HostApi.h>
#include <AP_Common/AP_Common.h>
#include <AP_RCTelemetry/AP_CRSF_Telem.h>
#include <string.h>

namespace AP_Module
{

static constexpr uint8_t MENU_NAME_MAX = 15;
static constexpr uint16_t PARAMETER_DATA_MAX = 255;
static constexpr uint8_t EVENT_PAYLOAD_MAX = 57;
static constexpr uint8_t EVENT_READ = 1;
static constexpr uint8_t EVENT_WRITE = 2;
static constexpr uint8_t MENU_NAME_WIRE_SIZE = 1 + MENU_NAME_MAX;
static constexpr uint16_t PARAMETER_WIRE_SIZE = 4 + 1 + PARAMETER_DATA_MAX;
static constexpr uint8_t EVENT_WIRE_SIZE = 1 + 1 + 1 + 1 + 1 + EVENT_PAYLOAD_MAX;
static constexpr uint8_t FILTERED_EVENT_WIRE_SIZE = 1 + 1 + 1 + 1 + EVENT_PAYLOAD_MAX;

static bool valid_padding(const uint8_t *data, uint16_t used, uint16_t size)
{
    for (uint16_t i = used; i < size; i++) {
        if (data[i] != 0) {
            return false;
        }
    }
    return true;
}

static bool read_menu_name(const uint8_t *input, char (&name)[MENU_NAME_MAX + 1])
{
    const uint8_t length = input[0];
    if (length > MENU_NAME_MAX || !valid_padding(input + 1, length, MENU_NAME_MAX) ||
        memchr(input + 1, 0, length) != nullptr) {
        return false;
    }
    memcpy(name, input + 1, length);
    name[length] = 0;
    return true;
}

#if AP_CRSF_SCRIPTING_ENABLED
static constexpr uint8_t MAX_MENUS = 10;
static constexpr uint8_t MAX_PARAMETERS = 255;

struct MenuHandle {
    void *context;
    uint32_t token;
    AP_CRSF_Telem::ScriptedMenu *menu;
};

struct ParameterHandle {
    void *context;
    uint32_t token;
    AP_CRSF_Telem::ScriptedMenu *menu;
    uint8_t parameter_id;
};

static MenuHandle menu_handles[MAX_MENUS];
static ParameterHandle parameter_handles[MAX_PARAMETERS];
static uint32_t next_menu_token = 1;
static uint32_t next_parameter_token = 1;
static HAL_Semaphore registry_sem;

static bool token_in_use(const MenuHandle *handles, uint8_t count, uint32_t token)
{
    for (uint8_t i = 0; i < count; i++) {
        if (handles[i].menu != nullptr && handles[i].token == token) {
            return true;
        }
    }
    return false;
}

static bool token_in_use(const ParameterHandle *handles, uint16_t count, uint32_t token)
{
    for (uint16_t i = 0; i < count; i++) {
        if (handles[i].menu != nullptr && handles[i].token == token) {
            return true;
        }
    }
    return false;
}

template <typename Handle>
static uint32_t allocate_token(Handle *handles, uint16_t count, uint32_t &next)
{
    do {
        if (++next == 0) {
            next = 1;
        }
    } while (token_in_use(handles, count, next));
    return next;
}

static MenuHandle *resolve_menu(void *context, uint32_t token)
{
    for (auto &handle : menu_handles) {
        if (handle.context == context && handle.token == token && handle.menu != nullptr) {
            return &handle;
        }
    }
    return nullptr;
}

static ParameterHandle *resolve_parameter(void *context, uint32_t token)
{
    for (auto &handle : parameter_handles) {
        if (handle.context == context && handle.token == token && handle.menu != nullptr) {
            return &handle;
        }
    }
    return nullptr;
}

static AP_CRSF_Telem::ScriptedParameter *resolve_parameter(ParameterHandle &handle)
{
    for (uint8_t i = 0; i < handle.menu->num_params; i++) {
        auto &parameter = handle.menu->params[i];
        if (parameter.id == handle.parameter_id) {
            return &parameter;
        }
    }
    return nullptr;
}

static bool owns_parameter(void *context, uint8_t parameter_id)
{
    for (const auto &handle : parameter_handles) {
        if (handle.context == context && handle.menu != nullptr && handle.parameter_id == parameter_id) {
            return true;
        }
    }
    return false;
}

static uint8_t menu_count(AP_CRSF_Telem &crsf)
{
    uint8_t count = 0;
    for (auto *menu = crsf.scripted_menus.next_menu; menu != nullptr; menu = menu->next_menu) {
        count++;
    }
    return count;
}

static int32_t add_menu_handle(void *context, AP_CRSF_Telem::ScriptedMenu *menu, uint8_t *output)
{
    if (menu == nullptr) {
        return AP_MODULE_STATUS_OK;
    }
    for (auto &handle : menu_handles) {
        if (handle.menu == nullptr) {
            handle = {context, allocate_token(menu_handles, MAX_MENUS, next_menu_token), menu};
            BindingWire::write(output, handle.token, 4);
            return AP_MODULE_STATUS_OK;
        }
    }
    return AP_MODULE_STATUS_NO_MEMORY;
}

static int32_t crsf_instance(AP_CRSF_Telem *&crsf)
{
    crsf = AP::crsf_telem();
    return crsf == nullptr ? AP_MODULE_STATUS_UNAVAILABLE : AP_MODULE_STATUS_OK;
}
#endif

int32_t binding_crsf_add_menu(void *context, const uint8_t *input, uint32_t input_size,
                              uint8_t *output, uint32_t output_size)
{
    if (context == nullptr || input == nullptr || input_size != MENU_NAME_WIRE_SIZE ||
        output == nullptr || output_size != 4) {
        return AP_MODULE_STATUS_INVALID;
    }
    memset(output, 0, output_size);
    char name[MENU_NAME_MAX + 1];
    if (!read_menu_name(input, name)) {
        return AP_MODULE_STATUS_INVALID;
    }
#if AP_CRSF_SCRIPTING_ENABLED
    BindingLock registry_lock(registry_sem);
    AP_CRSF_Telem *crsf;
    const int32_t available = crsf_instance(crsf);
    if (available != AP_MODULE_STATUS_OK) {
        return available;
    }
    BindingLock crsf_lock(crsf->get_semaphore());
    if (menu_count(*crsf) >= MAX_MENUS) {
        return AP_MODULE_STATUS_NO_MEMORY;
    }
    return add_menu_handle(context, crsf->add_menu(name, context), output);
#else
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

int32_t binding_crsf_menu_add_menu(void *context, const uint8_t *input, uint32_t input_size,
                                   uint8_t *output, uint32_t output_size)
{
    if (context == nullptr || input == nullptr || input_size != 4 + MENU_NAME_WIRE_SIZE ||
        output == nullptr || output_size != 4) {
        return AP_MODULE_STATUS_INVALID;
    }
    memset(output, 0, output_size);
    char name[MENU_NAME_MAX + 1];
    if (!read_menu_name(input + 4, name)) {
        return AP_MODULE_STATUS_INVALID;
    }
#if AP_CRSF_SCRIPTING_ENABLED
    BindingLock registry_lock(registry_sem);
    auto *parent = resolve_menu(context, BindingWire::read(input, 4));
    AP_CRSF_Telem *crsf;
    const int32_t available = crsf_instance(crsf);
    if (parent == nullptr) {
        return AP_MODULE_STATUS_INVALID;
    }
    if (available != AP_MODULE_STATUS_OK) {
        return available;
    }
    BindingLock crsf_lock(crsf->get_semaphore());
    if (menu_count(*crsf) >= MAX_MENUS || parent->menu->num_params >= AP_CRSF_Telem::MAX_SCRIPTED_MENU_SIZE) {
        return AP_MODULE_STATUS_NO_MEMORY;
    }
    auto *menu = parent->menu->add_menu(name, 2, parent->menu->id, context);
    return add_menu_handle(context, menu, output);
#else
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

int32_t binding_crsf_menu_add_parameter(void *context, const uint8_t *input, uint32_t input_size,
                                        uint8_t *output, uint32_t output_size)
{
    if (context == nullptr || input == nullptr || input_size != PARAMETER_WIRE_SIZE ||
        output == nullptr || output_size != 4) {
        return AP_MODULE_STATUS_INVALID;
    }
    memset(output, 0, output_size);
    const uint8_t length = input[4];
    if (length == 0 || !valid_padding(input + 5, length, PARAMETER_DATA_MAX)) {
        return AP_MODULE_STATUS_INVALID;
    }
#if AP_CRSF_SCRIPTING_ENABLED
    BindingLock registry_lock(registry_sem);
    auto *menu = resolve_menu(context, BindingWire::read(input, 4));
    if (menu == nullptr) {
        return AP_MODULE_STATUS_INVALID;
    }
    ParameterHandle *free_handle = nullptr;
    for (auto &handle : parameter_handles) {
        if (handle.menu == nullptr) {
            free_handle = &handle;
            break;
        }
    }
    if (free_handle == nullptr || menu->menu->num_params >= AP_CRSF_Telem::MAX_SCRIPTED_MENU_SIZE) {
        return AP_MODULE_STATUS_NO_MEMORY;
    }
    auto *parameter = menu->menu->add_parameter(length, reinterpret_cast<const char *>(input + 5));
    if (parameter == nullptr) {
        return AP_MODULE_STATUS_OK;
    }
    *free_handle = {context, allocate_token(parameter_handles, MAX_PARAMETERS, next_parameter_token),
                    menu->menu, parameter->id};
    BindingWire::write(output, free_handle->token, 4);
    return AP_MODULE_STATUS_OK;
#else
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

int32_t binding_crsf_parameter_id(void *context, const uint8_t *input, uint32_t input_size,
                                  uint8_t *output, uint32_t output_size)
{
    if (context == nullptr || input == nullptr || input_size != 4 || output == nullptr || output_size != 1) {
        return AP_MODULE_STATUS_INVALID;
    }
    output[0] = 0;
#if AP_CRSF_SCRIPTING_ENABLED
    BindingLock lock(registry_sem);
    auto *handle = resolve_parameter(context, BindingWire::read(input, 4));
    if (handle == nullptr || resolve_parameter(*handle) == nullptr) {
        return AP_MODULE_STATUS_INVALID;
    }
    output[0] = handle->parameter_id;
    return AP_MODULE_STATUS_OK;
#else
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

int32_t binding_crsf_parameter_data(void *context, const uint8_t *input, uint32_t input_size,
                                    uint8_t *output, uint32_t output_size)
{
    if (context == nullptr || input == nullptr || input_size != 4 || output == nullptr ||
        output_size != 1 + PARAMETER_DATA_MAX) {
        return AP_MODULE_STATUS_INVALID;
    }
    memset(output, 0, output_size);
#if AP_CRSF_SCRIPTING_ENABLED
    BindingLock registry_lock(registry_sem);
    auto *handle = resolve_parameter(context, BindingWire::read(input, 4));
    if (handle == nullptr) {
        return AP_MODULE_STATUS_INVALID;
    }
    AP_CRSF_Telem *crsf;
    const int32_t available = crsf_instance(crsf);
    if (available != AP_MODULE_STATUS_OK) {
        return available;
    }
    BindingLock crsf_lock(crsf->get_semaphore());
    auto *parameter = resolve_parameter(*handle);
    if (parameter == nullptr || parameter->length > PARAMETER_DATA_MAX ||
        (parameter->length != 0 && parameter->data == nullptr)) {
        return AP_MODULE_STATUS_HOST_ERROR;
    }
    output[0] = uint8_t(parameter->length);
    if (parameter->length != 0) {
        memcpy(output + 1, parameter->data, parameter->length);
    }
    return AP_MODULE_STATUS_OK;
#else
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

int32_t binding_crsf_peek_menu_event(void *context, const uint8_t *input, uint32_t input_size,
                                     uint8_t *output, uint32_t output_size)
{
    if (context == nullptr || input != nullptr || input_size != 0 || output == nullptr ||
        output_size != EVENT_WIRE_SIZE) {
        return AP_MODULE_STATUS_INVALID;
    }
    memset(output, 0, output_size);
#if AP_CRSF_SCRIPTING_ENABLED
    BindingLock registry_lock(registry_sem);
    AP_CRSF_Telem *crsf;
    const int32_t available = crsf_instance(crsf);
    if (available != AP_MODULE_STATUS_OK) {
        return available;
    }
    BindingLock crsf_lock(crsf->get_semaphore());
    AP_CRSF_Telem::ScriptedParameterWrite request{};
    if (!crsf->select_scripted_event(crsf->inbound_params, context, request) || request.param == nullptr ||
        !owns_parameter(context, request.param->id)) {
        return AP_MODULE_STATUS_OK;
    }
    if (request.payload.payload_length > EVENT_PAYLOAD_MAX) {
        return AP_MODULE_STATUS_HOST_ERROR;
    }
    output[0] = 1;
    const uint32_t available_count = crsf->inbound_params.available();
    output[1] = available_count > UINT8_MAX ? UINT8_MAX : uint8_t(available_count);
    output[2] = request.param->id;
    output[3] = uint8_t(request.type);
    output[4] = request.payload.payload_length;
    memcpy(output + 5, request.payload.payload, request.payload.payload_length);
    return AP_MODULE_STATUS_OK;
#else
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

int32_t binding_crsf_pop_menu_event(void *context, const uint8_t *input, uint32_t input_size,
                                    uint8_t *output, uint32_t output_size)
{
    if (context == nullptr || input == nullptr || input_size != 2 || output == nullptr || output_size != 1 ||
        (input[1] != EVENT_READ && input[1] != EVENT_WRITE)) {
        return AP_MODULE_STATUS_INVALID;
    }
    output[0] = 0;
#if AP_CRSF_SCRIPTING_ENABLED
    BindingLock registry_lock(registry_sem);
    AP_CRSF_Telem *crsf;
    const int32_t available = crsf_instance(crsf);
    if (available != AP_MODULE_STATUS_OK) {
        return available;
    }
    BindingLock crsf_lock(crsf->get_semaphore());
    AP_CRSF_Telem::ScriptedParameterWrite request{};
    if (!crsf->select_scripted_event(crsf->inbound_params, context, request) || request.param == nullptr ||
        !owns_parameter(context, request.param->id) || request.param->id != input[0] ||
        uint8_t(request.type) != input[1] || !crsf->inbound_params.pop(request)) {
        return AP_MODULE_STATUS_OK;
    }
    output[0] = crsf->outbound_params.push(request);
    return AP_MODULE_STATUS_OK;
#else
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

int32_t binding_crsf_send_write_response(void *context, const uint8_t *input, uint32_t input_size,
                                         uint8_t *output, uint32_t output_size)
{
    if (context == nullptr || input == nullptr || input_size != 1 + PARAMETER_DATA_MAX ||
        output == nullptr || output_size != 1 || !valid_padding(input + 1, input[0], PARAMETER_DATA_MAX)) {
        return AP_MODULE_STATUS_INVALID;
    }
    output[0] = 0;
#if AP_CRSF_SCRIPTING_ENABLED
    BindingLock registry_lock(registry_sem);
    AP_CRSF_Telem *crsf;
    const int32_t available = crsf_instance(crsf);
    if (available != AP_MODULE_STATUS_OK) {
        return available;
    }
    BindingLock crsf_lock(crsf->get_semaphore());
    AP_CRSF_Telem::ScriptedParameterWrite request{};
    if (!crsf->select_scripted_event(crsf->outbound_params, context, request) || request.param == nullptr ||
        !owns_parameter(context, request.param->id) || !crsf->outbound_params.pop(request)) {
        return AP_MODULE_STATUS_OK;
    }
    const uint8_t length = input[0];
    switch (request.type) {
    case AP_CRSF_Telem::PARAMETER_READ:
        if (length > request.param->length) {
            char *data = static_cast<char *>(mem_realloc(const_cast<char *>(request.param->data),
                                                         request.param->length, length));
            if (data == nullptr) {
                return AP_MODULE_STATUS_NO_MEMORY;
            }
            request.param->data = data;
        }
        if (length != 0) {
            memcpy(const_cast<char *>(request.param->data), input + 1, length);
        }
        request.param->length = length;
        break;
    case AP_CRSF_Telem::PARAMETER_WRITE:
        request.payload.payload_length = length > EVENT_PAYLOAD_MAX ? EVENT_PAYLOAD_MAX : length;
        memcpy(request.payload.payload, input + 1, request.payload.payload_length);
        break;
    }
    output[0] = crsf->ready_params.push(request);
    return AP_MODULE_STATUS_OK;
#else
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

int32_t binding_crsf_send_response(void *context, const uint8_t *input, uint32_t input_size,
                                   uint8_t *output, uint32_t output_size)
{
    if (context == nullptr || input != nullptr || input_size != 0 || output == nullptr || output_size != 1) {
        return AP_MODULE_STATUS_INVALID;
    }
    output[0] = 0;
#if AP_CRSF_SCRIPTING_ENABLED
    BindingLock registry_lock(registry_sem);
    AP_CRSF_Telem *crsf;
    const int32_t available = crsf_instance(crsf);
    if (available != AP_MODULE_STATUS_OK) {
        return available;
    }
    BindingLock crsf_lock(crsf->get_semaphore());
    AP_CRSF_Telem::ScriptedParameterWrite request{};
    if (!crsf->select_scripted_event(crsf->outbound_params, context, request) || request.param == nullptr ||
        !owns_parameter(context, request.param->id) || !crsf->outbound_params.pop(request)) {
        return AP_MODULE_STATUS_OK;
    }
    output[0] = crsf->ready_params.push(request);
    return AP_MODULE_STATUS_OK;
#else
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

int32_t binding_crsf_get_menu_event(void *context, const uint8_t *input, uint32_t input_size,
                                    uint8_t *output, uint32_t output_size)
{
    if (context == nullptr || input == nullptr || input_size != 1 || output == nullptr ||
        output_size != FILTERED_EVENT_WIRE_SIZE) {
        return AP_MODULE_STATUS_INVALID;
    }
    memset(output, 0, output_size);
#if AP_CRSF_SCRIPTING_ENABLED
    BindingLock registry_lock(registry_sem);
    AP_CRSF_Telem *crsf;
    const int32_t available = crsf_instance(crsf);
    if (available != AP_MODULE_STATUS_OK) {
        return available;
    }
    BindingLock crsf_lock(crsf->get_semaphore());
    AP_CRSF_Telem::ScriptedParameterWrite request{};
    if (!crsf->select_scripted_event(crsf->inbound_params, context, request) || request.param == nullptr ||
        !owns_parameter(context, request.param->id)) {
        return AP_MODULE_STATUS_OK;
    }
    if (request.payload.payload_length > EVENT_PAYLOAD_MAX || !crsf->inbound_params.pop(request)) {
        return AP_MODULE_STATUS_HOST_ERROR;
    }
    if ((uint8_t(request.type) & input[0]) == 0) {
        crsf->ready_params.push(request);
        return AP_MODULE_STATUS_OK;
    }
    output[0] = 1;
    output[1] = request.param->id;
    output[2] = uint8_t(request.type);
    if (request.type == AP_CRSF_Telem::PARAMETER_WRITE) {
        output[3] = request.payload.payload_length;
        memcpy(output + 4, request.payload.payload, request.payload.payload_length);
    }
    crsf->outbound_params.push(request);
    return AP_MODULE_STATUS_OK;
#else
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

static int32_t menu_metadata(void *context, const uint8_t *input, uint32_t input_size,
                             uint8_t *output, uint32_t output_size, uint8_t field)
{
    const bool valid_size = (field == 1) ? output_size == MENU_NAME_WIRE_SIZE : output_size == 1;
    if (context == nullptr || input == nullptr || input_size != 4 || output == nullptr || !valid_size) {
        return AP_MODULE_STATUS_INVALID;
    }
    memset(output, 0, output_size);
#if AP_CRSF_SCRIPTING_ENABLED
    BindingLock registry_lock(registry_sem);
    auto *handle = resolve_menu(context, BindingWire::read(input, 4));
    if (handle == nullptr) {
        return AP_MODULE_STATUS_INVALID;
    }
    AP_CRSF_Telem *crsf;
    const int32_t available = crsf_instance(crsf);
    if (available != AP_MODULE_STATUS_OK) {
        return available;
    }
    BindingLock crsf_lock(crsf->get_semaphore());
    switch (field) {
    case 0:
        output[0] = handle->menu->id;
        break;
    case 1: {
        if (handle->menu->name == nullptr) {
            return AP_MODULE_STATUS_INVALID;
        }
        const size_t length = strnlen(handle->menu->name, MENU_NAME_MAX + 1);
        if (length > MENU_NAME_MAX) {
            return AP_MODULE_STATUS_HOST_ERROR;
        }
        output[0] = uint8_t(length);
        memcpy(output + 1, handle->menu->name, length);
        break;
    }
    case 2:
        output[0] = handle->menu->num_params;
        break;
    default:
        return AP_MODULE_STATUS_INVALID;
    }
    return AP_MODULE_STATUS_OK;
#else
    (void)field;
    return AP_MODULE_STATUS_UNAVAILABLE;
#endif
}

int32_t binding_crsf_menu_id(void *context, const uint8_t *input, uint32_t input_size,
                             uint8_t *output, uint32_t output_size)
{
    return menu_metadata(context, input, input_size, output, output_size, 0);
}

int32_t binding_crsf_menu_name(void *context, const uint8_t *input, uint32_t input_size,
                               uint8_t *output, uint32_t output_size)
{
    return menu_metadata(context, input, input_size, output, output_size, 1);
}

int32_t binding_crsf_menu_num_params(void *context, const uint8_t *input, uint32_t input_size,
                                     uint8_t *output, uint32_t output_size)
{
    return menu_metadata(context, input, input_size, output, output_size, 2);
}

void binding_crsf_cleanup(void *context)
{
#if AP_CRSF_SCRIPTING_ENABLED
    if (context == nullptr) {
        return;
    }
    WITH_SEMAPHORE(registry_sem);
    auto *crsf = AP::crsf_telem();
    if (crsf != nullptr) {
        crsf->clear_menus(context);
    }
    for (auto &handle : parameter_handles) {
        if (handle.context == context) {
            handle = {};
        }
    }
    for (auto &handle : menu_handles) {
        if (handle.context == context) {
            handle = {};
        }
    }
#else
    (void)context;
#endif
}

}
#endif
