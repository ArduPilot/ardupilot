//
// Unit tests for the AP_Meta_class and AP_Var classes.
//

#include <FastSerial.h>
#include <AP_Common.h>
#include <string.h>

// we need to do this, even though normally it's a bad idea
#pragma GCC diagnostic ignored "-Wfloat-equal"

FastSerialPort(Serial, 0);

//
// Unit test framework
//
class Test
{
public:
    Test(const char *name) : _name(name), _fail(false) {}

    ~Test()	{
        Serial.printf("%s: %s\n", _fail ? "FAILED" : "passed", _name);
        if (_fail) {
            _failed++;
        } else {
            _passed++;
        }
    }

    void	require(bool expr, const char *source) {
        if (!expr) {
            _fail = true;
            Serial.printf("%s: fail: %s\n", _name, source);
        }
    }

    static void	report() {
        Serial.printf("\n%d passed  %d failed\n", _passed, _failed);
    }

private:
    const char	*_name;
    bool		_fail;
    static int	_passed;
    static int	_failed;
};

int Test::_passed = 0;
int Test::_failed = 0;

#define TEST(name)		Test _test(#name)
#define	REQUIRE(expr)	_test.require(expr, #expr)

//
// Unit tests
//
void
setup(void)
{
    Serial.begin(38400);

    // MetaClass: test type ID
    {
        TEST(meta_type_id);

        AP_Float 	f1(0);
        AP_Float	f2(0);
        AP_Int8		i1(0);

        uint16_t	m1 = f1.meta_type_id();
        uint16_t	m2 = f2.meta_type_id();
        uint16_t	m3 = i1.meta_type_id();

        REQUIRE(m1 != 0);
        REQUIRE(m1 == m2);
        REQUIRE(m1 != m3);
        REQUIRE(AP_Meta_class::meta_type_equivalent(&f1, &f2));
        REQUIRE(!AP_Meta_class::meta_type_equivalent(&f1, &i1));

        REQUIRE(NULL != AP_Meta_class::meta_cast<AP_Float>(&f1));
        REQUIRE(NULL == AP_Meta_class::meta_cast<AP_Int8>(&f1));
        REQUIRE(NULL == AP_Meta_class::meta_cast<AP_Float16>(&f1));
    }

    // MetaClass: test external handles
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

    // AP_Var: initial value
    {
        TEST(var_initial_value);

        AP_Float    f(12.345);

        REQUIRE(f == 12.345);
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

        AP_Float	f(0, AP_Var::k_no_key, PSTR("test"));
        char		name_buffer[16];

        f.copy_name(name_buffer, sizeof(name_buffer));
        REQUIRE(!strcmp(name_buffer, "test"));
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

    // AP_Var: enumeration
    // note that this test presumes the singly-linked list implementation of the list
    {
        TEST(var_enumeration);

        AP_Var	*v = AP_Var::lookup_by_index(0);

        // test basic enumeration
        AP_Float f1(0, AP_Var::k_no_key, PSTR("test1"));
        REQUIRE(AP_Var::lookup_by_index(0) == &f1);
        REQUIRE(AP_Var::lookup_by_index(1) == v);

        // test that new entries arrive in order
        {
            AP_Float f2(0, AP_Var::k_no_key, PSTR("test2"));
            REQUIRE(AP_Var::lookup_by_index(0) == &f2);
            REQUIRE(AP_Var::lookup_by_index(1) == &f1);
            REQUIRE(AP_Var::lookup_by_index(2) == v);

            {
                AP_Float f3(0, AP_Var::k_no_key, PSTR("test3"));
                REQUIRE(AP_Var::lookup_by_index(0) == &f3);
                REQUIRE(AP_Var::lookup_by_index(1) == &f2);
                REQUIRE(AP_Var::lookup_by_index(2) == &f1);
                REQUIRE(AP_Var::lookup_by_index(3) == v);
            }
        }

        // test that destruction removes from the list
        REQUIRE(AP_Var::lookup_by_index(0) == &f1);
        REQUIRE(AP_Var::lookup_by_index(1) == v);
    }

    // AP_Var: group names
    {
        TEST(group_names);

        AP_Var_group	group(AP_Var::k_no_key, PSTR("group_"));
        AP_Float		f(&group, 1, 1.0, PSTR("test"));
        char			name_buffer[16];

        f.copy_name(name_buffer, sizeof(name_buffer));
        REQUIRE(!strcmp(name_buffer, "group_test"));
    }

#if SAVE
    // AP_Var: load and save
    {
        TEST(var_load_save);

        AP_Float    f1(10, 4);
        AP_Float    f2(0, 4);

        f2.save();
        f2 = 1.0;
        f2.load();
        REQUIRE(f2 == 0);

        f1.save();
        f2.load();
        REQUIRE(f2 == 10);
    }

    // AP_Var: group load/save
    {
        TEST(var_group_loadsave);

        AP_Var_group	group(PSTR("group_"), 4);
        AP_Float    	f1(10.0, 8);
        AP_Float    	f2(1.0, 4, PSTR("var"), &group);

        f1.save();
        f1.load();
        REQUIRE(f1 == 10);

        f2.save();
        f2.load();
        REQUIRE(f2 == 1);

        f1.load();
        REQUIRE(f1 == 1);
    }
#endif

    // AP_Var: derived types
    {
        TEST(var_derived);

        AP_Float16	f(10.0, 20);

        f.save();
        f = 0;
        REQUIRE(f == 0);
        f.load();
        REQUIRE(f = 10.0);
    }


    Test::report();
}

void
loop(void)
{
}
