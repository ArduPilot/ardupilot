@{from dronecan_dsdlc_helpers import *}@
@{indent = 0}@{ind = '    '*indent}@
#define CANARD_DSDLC_INTERNAL
#include <@(msg_header_file_name)>
@[    if msg_kind == "request"]@
#include <@(msg_resp_header_file_name)>
@[    end if]@
#include <string.h>

#ifdef CANARD_DSDLC_TEST_BUILD
#include <test_helpers.h>
#endif

uint32_t @(msg_underscored_name)_encode(@(msg_c_type)* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
) {
    uint32_t bit_ofs = 0;
    memset(buffer, 0, @(msg_define_name.upper())_MAX_SIZE);
    _@(msg_underscored_name)_encode(buffer, &bit_ofs, msg, 
#if CANARD_ENABLE_TAO_OPTION
    tao
#else
    true
#endif
    );
    return ((bit_ofs+7)/8);
}

/*
  return true if the decode is invalid
 */
bool @(msg_underscored_name)_decode(const CanardRxTransfer* transfer, @(msg_c_type)* msg) {
#if CANARD_ENABLE_TAO_OPTION
    if (transfer->tao && (transfer->payload_len > @(msg_define_name.upper())_MAX_SIZE)) {
        return true; /* invalid payload length */
    }
#endif
    uint32_t bit_ofs = 0;
    if (_@(msg_underscored_name)_decode(transfer, &bit_ofs, msg,
#if CANARD_ENABLE_TAO_OPTION
    transfer->tao
#else
    true
#endif
    )) {
        return true; /* invalid payload */
    }

    const uint32_t byte_len = (bit_ofs+7U)/8U;
#if CANARD_ENABLE_TAO_OPTION
    // if this could be CANFD then the dlc could indicating more bytes than
    // we actually have
    if (!transfer->tao) {
        return byte_len > transfer->payload_len;
    }
#endif
    return byte_len != transfer->payload_len;
}

#ifdef CANARD_DSDLC_TEST_BUILD
@(msg_c_type) sample_@(msg_underscored_name)_msg(void) {
@{indent += 1}@{ind = '    '*indent}@
@(ind)@(msg_c_type) msg;

@[  if msg_union]@
@(ind)msg.union_tag = random_range_unsigned_val(0, @(len(msg_fields)-1));

@(ind)switch(msg.union_tag) {
@{indent += 1}@{ind = '    '*indent}@
@[  end if]@
@[    for field in msg_fields]@
@[      if msg_union]@
@(ind)case @(msg_underscored_name.upper())_@(field.name.upper()): {
@{indent += 1}@{ind = '    '*indent}@
@[      end if]@
@[      if field.type.category == field.type.CATEGORY_COMPOUND]@
@(ind)msg.@(field.name) = sample_@(underscored_name(field.type))_msg();
@[      elif field.type.category == field.type.CATEGORY_PRIMITIVE]@
@[        if field.type.kind == field.type.KIND_FLOAT and field.type.bitlen == 16]@
@(ind)msg.@(field.name) = random_float16_val();
@[        elif field.type.kind == field.type.KIND_FLOAT]@
@(ind)msg.@(field.name) = random_float_val();
@[        else]@
@(ind)msg.@(field.name) = (@(dronecan_type_to_ctype(field.type)))random_bitlen_@('signed' if dronecan_type_is_signed(field.type) else 'unsigned')_val(@(field.type.bitlen));
@[        end if]@
@[      elif field.type.category == field.type.CATEGORY_ARRAY]@
@[        if field.type.mode == field.type.MODE_DYNAMIC]@
@(ind)msg.@(field.name).len = (@(c_array_len_type(field)))random_range_unsigned_val(0, @(field.type.max_size));
@(ind)for (size_t i=0; i < msg.@(field.name).len; i++) {
@[        else]@
@(ind)for (size_t i=0; i < @(field.type.max_size); i++) {
@[        end if]@
@{indent += 1}@{ind = '    '*indent}@
@[        if field.type.value_type.category == field.type.value_type.CATEGORY_PRIMITIVE]@
@[          if field.type.value_type.kind == field.type.value_type.KIND_FLOAT and field.type.value_type.bitlen == 16]@
@(ind)msg.@(field_get_data(field))[i] = random_float16_val();
@[          elif field.type.value_type.kind == field.type.value_type.KIND_FLOAT]@
@(ind)msg.@(field_get_data(field))[i] = random_float_val();
@[          else]@
@(ind)msg.@(field_get_data(field))[i] = (@(dronecan_type_to_ctype(field.type.value_type)))random_bitlen_@('signed' if dronecan_type_is_signed(field.type.value_type) else 'unsigned')_val(@(field.type.value_type.bitlen));
@[          end if]@
@[        elif field.type.value_type.category == field.type.value_type.CATEGORY_COMPOUND]@
@(ind)msg.@(field_get_data(field))[i] = sample_@(underscored_name(field.type.value_type))_msg();
@[        end if]@
@{indent -= 1}@{ind = '    '*indent}@
@(ind)}
@[      end if]@
@[      if msg_union]@
@(ind)break;
@{indent -= 1}@{ind = '    '*indent}@
@(ind)}
@[      end if]@
@[    end for]@
@[  if msg_union]@
@{indent -= 1}@{ind = '    '*indent}@
@(ind)}
@[  end if]@
@(ind)return msg;
@{indent -= 1}@{ind = '    '*indent}@
@(ind)}
#endif
