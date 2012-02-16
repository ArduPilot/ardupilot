//
// Unit tests for the AP_Meta_class and AP_Var classes.
//

#define USE_AP_VAR
#include <FastSerial.h>
#include <AP_Common.h>
#include <AP_Test.h>
#include <AP_Var.h>
#include <AP_Math.h>
#include <string.h>

// we need to do this, even though normally it's a bad idea
#pragma GCC diagnostic ignored "-Wfloat-equal"

FastSerialPort(Serial, 0);

//
// Unit tests
//
void
setup(void)
{
    Serial.begin(115200);
    Serial.println("AP_Var unit tests.\n");

    // MetaClass: test type ID
    {
        TEST(meta_type_id);

        AP_Float 	f1(0);
        AP_Float	f2(0);
        AP_Int8		i1(0);

        uint16_t	m1 = f1.meta_type_id();
        uint16_t	m2 = f2.meta_type_id();
        uint16_t	m3 = i1.meta_type_id();
        uint16_t    m4 = AP_Meta_class::meta_type_id<AP_Float>();

        REQUIRE(m1 != 0);
        REQUIRE(m1 == m2);
        REQUIRE(m1 != m3);
        REQUIRE(m1 == m4);
    }

    // MetaClass: meta_type_equivalent
    {
        TEST(meta_type_equivalent);

        AP_Float    f1;
        AP_Float    f2;
        AP_Int8     i1;

        REQUIRE(AP_Meta_class::meta_type_equivalent(&f1, &f2));
        REQUIRE(!AP_Meta_class::meta_type_equivalent(&f1, &i1));
    }


    // MetaClass: external handles
    {
        TEST(meta_handle);

        AP_Float	f(0);
        AP_Meta_class::Meta_handle	h = f.meta_get_handle();

        REQUIRE(0 != h);
        REQUIRE(NULL != AP_Meta_class::meta_validate_handle(h));
        REQUIRE(NULL == AP_Meta_class::meta_validate_handle(h + 1));
    }

    // MetaClass: test meta_cast
    {
        TEST(meta_cast);

        AP_Float	f(0);

        REQUIRE(NULL != AP_Meta_class::meta_cast<AP_Float>(&f));
        REQUIRE(NULL == AP_Meta_class::meta_cast<AP_Int8>(&f));
    }

    // MetaClass: ... insert tests here ...

    // AP_Var: constants
    {
        TEST(var_constants);

        REQUIRE(AP_Float_zero == 0);
        REQUIRE(AP_Float_unity == 1.0);
        REQUIRE(AP_Float_negative_unity = -1.0);
    }

    // AP_Var: type IDs
    {
        TEST(var_type_ids);

        AP_Float    f;
        AP_Float16   fs;
        AP_Int32    l;
        AP_Int16    s;
        AP_Int8     b;

        REQUIRE(f.meta_type_id()    == AP_Var::k_typeid_float);
        REQUIRE(fs.meta_type_id()   == AP_Var::k_typeid_float16);
        REQUIRE(l.meta_type_id()    == AP_Var::k_typeid_int32);
        REQUIRE(s.meta_type_id()    == AP_Var::k_typeid_int16);
        REQUIRE(b.meta_type_id()    == AP_Var::k_typeid_int8);

        REQUIRE(AP_Var::k_typeid_float != AP_Var::k_typeid_int32);
    }

    // AP_Var: initial value
    {
        TEST(var_initial_value);

        AP_Float    f1(12.345);
        AP_Float    f2;

        REQUIRE(f1 == 12.345);
        REQUIRE(f2 == 0);
    }

    // AP_Var: set, get, assignment
    {
        TEST(var_set_get);

        AP_Float	f(1.0);

        REQUIRE(f == 1.0);
        REQUIRE(f.get() == 1.0);

        f.set(10.0);
        REQUIRE(f == 10.0);
        REQUIRE(f.get() == 10.0);
    }

    // AP_Var: cast to type
    {
        TEST(var_cast_to_type);

        AP_Float	f(1.0);

        f *= 2.0;
        REQUIRE(f == 2.0);
        f /= 4;
        REQUIRE(f == 0.5);
        f += f;
        REQUIRE(f == 1.0);
    }

    // AP_Var: equality
    {
        TEST(var_equality);

        AP_Float	f1(1.0);
        AP_Float	f2(1.0);
        AP_Float	f3(2.0);

        REQUIRE(f1 == f2);
        REQUIRE(f2 != f3);
    }

    // AP_Var: naming
    {
        TEST(var_naming);

        AP_Float	f(0, AP_Var::k_key_none, PSTR("test"));
        char		name_buffer[16];

        f.copy_name(name_buffer, sizeof(name_buffer));
        REQUIRE(!strcmp(name_buffer, "test"));
    }

    // AP_Var: arrays
    {
        TEST(var_array);

        AP_VarA<float,4>    fa;

        fa[0] = 1.0;
        fa[1] = 10.0;
        fa.set(2, 100.0);
        fa[3] = -1000.0;

        REQUIRE(fa.get(0) == 1.0);
        REQUIRE(fa.get(1) == 10.0);
        REQUIRE(fa.get(2) == 100.0);
        REQUIRE(fa.get(3) == -1000.0);
    }

    // AP_Var: serialize
    // note that this presumes serialisation to the native in-memory format
    {
        TEST(var_serialize);

        float		b = 0;
        AP_Float	f(10.0);
        size_t		s;

        s = f.serialize(&b, sizeof(b));
        REQUIRE(s == sizeof(b));
        REQUIRE(b == 10.0);
    }

    // AP_Var: unserialize
    {
        TEST(var_unserialize);

        float		b = 10;
        AP_Float	f(0);
        size_t		s;

        s = f.unserialize(&b, sizeof(b));
        REQUIRE(s == sizeof(b));
        REQUIRE(f == 10);
    }

    // AP_Var: groups and names
    {
        TEST(group_names);

        AP_Var_group    group(AP_Var::k_key_none, PSTR("group_"));
        AP_Float        f(&group, 1, 1.0, PSTR("test"));
        char            name_buffer[16];

        f.copy_name(name_buffer, sizeof(name_buffer));
        REQUIRE(!strcmp(name_buffer, "group_test"));
    }

    // AP_Var: enumeration
    {
        TEST(empty_variables);

        REQUIRE(AP_Var::first() == NULL);
    }

    {
        TEST(enumerate_variables);

        AP_Float    f1;

        REQUIRE(AP_Var::first() == &f1);

        {
            AP_Var_group    group;
            AP_Var          f2(&group, 0, 0);
            AP_Var          f3(&group, 1, 0);
            AP_Var          *vp;

            vp = AP_Var::first();
            REQUIRE(vp == &group);      // XXX presumes FIFO insertion
            vp = vp->next();
            REQUIRE(vp == &f1);         // XXX presumes FIFO insertion
            vp = vp->next();
            REQUIRE(vp == &f2);         // first variable in the grouped list

            vp = AP_Var::first_member(&group);
            REQUIRE(vp == &f2);
            vp = vp->next_member();
            REQUIRE(vp == &f3);
        }
    }

    // AP_Var: save and load
    {
        TEST(var_save_load);

        AP_Float    f1(10.0, 1);
        AP_Float16  f2(1.23, 2);

        AP_Var::erase_all();
        REQUIRE(true == f1.save());
        REQUIRE(f1 == 10.0);
        f1 = 0;
        REQUIRE(true == f1.load());
        REQUIRE(f1 == 10.0);

        REQUIRE(true == f2.save());
        REQUIRE(f2 == 1.23);
        f2 = 0;
        REQUIRE(true == f2.load());
        REQUIRE(f2 == 1.23);

    }

    // AP_Var: reload
    {
        TEST(var_reload);

        AP_Float    f1(0, 1);

        REQUIRE(true == f1.load());
        REQUIRE(f1 == 10.0);

        AP_Var::erase_all();
    }

    // AP_Var: save/load all
    {
        TEST(var_save_load_all);

        AP_Float    f1(10.0, 1);
        AP_Float    f2(123.0, 2);
        AP_Int8     i(17, 3);

        REQUIRE(true == AP_Var::save_all());
        f1 = 0;
        f2 = 0;
        i = 0;
        REQUIRE(true == AP_Var::load_all());
        REQUIRE(f1 == 10.0);
        REQUIRE(f2 == 123.0);
        REQUIRE(i == 17);

        AP_Var::erase_all();
    }

    // AP_Var: group load/save
    {
        TEST(var_group_save_load);

        AP_Var_group    group(10);
        AP_Float        f1(&group, 0, 10.0);
        AP_Float        f2(&group, 1, 123.0);
        AP_Float        f3(-1.0);
        AP_Float16      f4(&group, 2, 7);

        REQUIRE(true == group.save());
        f1 = 0;
        f2 = 0;
        f3 = 0;
        f4 = 0;
        REQUIRE(true == group.load());
        REQUIRE(f1 == 10.0);
        REQUIRE(f2 == 123.0);
        REQUIRE(f3 == 0);
        REQUIRE(f4 == 7);

        AP_Var::erase_all();
    }


    Test::report();
}

void
loop(void)
{
}
