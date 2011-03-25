// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
//
// This is free software; you can redistribute it and/or modify it under
// the terms of the GNU Lesser General Public License as published by the
// Free Software Foundation; either version 2.1 of the License, or (at
// your option) any later version.
//

/// @file   AP_Var_menufuncs.cpp
/// @brief  Useful functions compatible with the menu system for
///         managing AP_Var variables.

#include <FastSerial.h>
#include <AP_Common.h>

void
AP_Var_print(AP_Var *vp)
{
    // try to print from variable types that we know
    if          (vp->meta_type_id() == AP_Var::k_typeid_float) {

        AP_Float    *v = (AP_Float *)vp;
        Serial.printf_P(PSTR("%f"), v->get());

    } else if   (vp->meta_type_id() == AP_Var::k_typeid_float16) {

        AP_Float16  *v = (AP_Float16 *)vp;
        Serial.printf_P(PSTR("%f"), v->get());

    } else if   (vp->meta_type_id() == AP_Var::k_typeid_int32) {

        AP_Int32    *v = (AP_Int32 *)vp;
        Serial.printf_P(PSTR("%ld"), v->get());

    } else if   (vp->meta_type_id() == AP_Var::k_typeid_int16) {

        AP_Int16    *v = (AP_Int16 *)vp;
        Serial.printf_P(PSTR("%d"),  v->get());

    } else if   (vp->meta_type_id() == AP_Var::k_typeid_int8) {

        AP_Int8    *v = (AP_Int8 *)vp;
        Serial.printf_P(PSTR("%d"), v->get());

    } else {
        Serial.print_P(PSTR("??"));
    }
}

int8_t
AP_Var_menu_set(uint8_t argc, const Menu::arg *argv)
{
    AP_Var  *vp;

    // check argument count
    if (argc != 3) {
        Serial.println_P(PSTR("missing name or value"));
        return -1;
    }
    Serial.printf_P(PSTR("%s: "), argv[1].str);

    // search for the variable
    vp = AP_Var::find(argv[1].str);
    if (NULL == vp) {
        Serial.println_P(PSTR("not found"));
        return -1;
    }

    // try to assign to variable types that we know
    if          (vp->meta_type_id() == AP_Var::k_typeid_float) {

        AP_Float    *v = (AP_Float *)vp;
        v->set(argv[2].f);

    } else if   (vp->meta_type_id() == AP_Var::k_typeid_float16) {

        AP_Float16  *v = (AP_Float16 *)vp;
        v->set(argv[2].f);

    } else if   (vp->meta_type_id() == AP_Var::k_typeid_int32) {

        AP_Int32    *v = (AP_Int32 *)vp;
        v->set(argv[2].i);

    } else if   (vp->meta_type_id() == AP_Var::k_typeid_int16) {

        AP_Int16    *v = (AP_Int16 *)vp;
        v->set(argv[2].i);

    } else if   (vp->meta_type_id() == AP_Var::k_typeid_int8) {

        AP_Int8     *v = (AP_Int8 *)vp;
        v->set(argv[2].i);

    } else {
        Serial.println_P(PSTR("unknown type"));
        return -1;
    }
    AP_Var_print(vp);
    Serial.println();
    return 0;
}

int8_t
AP_Var_menu_show(uint8_t argc, const Menu::arg *argv)
{
    AP_Var  *vp;

    // if no arguments, show all variables
    if (argc == 1) {
        for (vp = AP_Var::first(); NULL != vp; vp = vp->next()) {
            char    name_buffer[32];

            // groups should not be displayed
            if (vp->meta_type_id() == AP_Var::k_typeid_group) {
                continue;
            }

            // get a displayable name for the variable
            vp->copy_name(name_buffer, sizeof(name_buffer));
            if (name_buffer[0] == 0) {
                // without a name the variable is not displayable
                continue;
            }

            // print name and value
            Serial.printf_P(PSTR("%03u:%-32.32s: "), vp->key(), name_buffer);
            AP_Var_print(vp);
            Serial.println();
        }
        return 0;
    }

    // show variable by name
    vp = AP_Var::find(argv[1].str);
    if (NULL == vp) {
        Serial.println_P(PSTR("not found"));
        return -1;
    }
    Serial.printf_P(PSTR("%03u:%s: "), vp->key(), argv[1].str);
    AP_Var_print(vp);
    Serial.println();

    return 0;
}
