//
// Unit tests for the AP_MetaClass and AP_Var classes.
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

		AP_Float 	f1;
		AP_Float	f2;
		AP_Int8		i1;

		uint16_t	m1 = f1.meta_type_id();
		uint16_t	m2 = f2.meta_type_id();
		uint16_t	m3 = i1.meta_type_id();

		REQUIRE(m1 != 0);
		REQUIRE(m1 == m2);
		REQUIRE(m1 != m3);
		REQUIRE( AP_MetaClass::meta_type_equivalent(&f1, &f2));
		REQUIRE(!AP_MetaClass::meta_type_equivalent(&f1, &i1));
	}

	// MetaClass: test external handles
	{
		TEST(meta_handle);

		AP_Float					f;
		AP_MetaClass::AP_MetaHandle	h = f.meta_get_handle();

		REQUIRE(0 != h);
		REQUIRE(NULL != AP_MetaClass::meta_validate_handle(h));
		REQUIRE(NULL == AP_MetaClass::meta_validate_handle(h + 1));
	}

	// MetaClass: test meta_cast
	{
		TEST(meta_cast);

		AP_Float	f;

		REQUIRE(NULL != AP_MetaClass::meta_cast<AP_Float>(&f));
		REQUIRE(NULL == AP_MetaClass::meta_cast<AP_Int8>(&f));
	}

	// MetaClass: ... insert tests here ...

	// AP_Var: cast to type
	{
		TEST(var_cast_to_type);

		AP_Float	f(1.0);		REQUIRE(f == 1.0);
		f *= 2.0;				REQUIRE(f == 2.0);
		f /= 4;					REQUIRE(f == 0.5);
		f += f;					REQUIRE(f == 1.0);
	}

	// AP_Var: naming
	{
		TEST(var_naming);

		AP_Float	f(1.0, AP_Var::AP_VarNoAddress, PSTR("test"));
		char		name_buffer[16];

		f.copy_name(name_buffer, sizeof(name_buffer));
		REQUIRE(!strcmp(name_buffer, "test"));
	}

	// AP_Var: serialize
	{
		TEST(var_serialize);

		float		b = 0;
		AP_Float	f(10);
		size_t		s;

		s = f.serialize(&b, sizeof(b));
		REQUIRE(s == sizeof(b));
		REQUIRE(b == 10);
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

	// AP_Var: load and save
	{
		TEST(var_load_save);

		AP_Float	f1(10, 4);
		AP_Float	f2(0,  4);

		f2.save();
		f2.load();
		REQUIRE(f2 == 0);

		f1.save();
		f2.load();
		REQUIRE(f2 == 10);
	}

	// AP_Var: enumeration
	// note that this test presumes the singly-linked list implementation of the list
	{
		TEST(var_enumeration);

		// test basic enumeration
		AP_Float f1(1.0, AP_Var::AP_VarNoAddress, PSTR("test1"));
		REQUIRE(AP_Var::lookup(0) == &f1);
		REQUIRE(AP_Var::lookup(1) == NULL);

		// test that new entries arrive in order
		{
			AP_Float f2(2.0, AP_Var::AP_VarNoAddress, PSTR("test2"));
			REQUIRE(AP_Var::lookup(0) == &f2);
			REQUIRE(AP_Var::lookup(1) == &f1);
			REQUIRE(AP_Var::lookup(2) == NULL);
		}

		// test that destruction removes from the list
		REQUIRE(AP_Var::lookup(0) == &f1);
		REQUIRE(AP_Var::lookup(1) == NULL);
	}

	// AP_Var: scope names
	{
		TEST(var_scope_names);

		AP_VarScope		scope(PSTR("scope_"));
		AP_Float		f(1.0, AP_Var::AP_VarNoAddress, PSTR("test"), &scope);
		char			name_buffer[16];

		f.copy_name(name_buffer, sizeof(name_buffer));
		REQUIRE(!strcmp(name_buffer, "scope_test"));
	}

	// AP_Var: scope address offsets
	{
		TEST(var_scope_addressing);

		AP_Float	f1(10.0, 8);
		AP_VarScope	scope(PSTR("scope"), 4);
		AP_Float	f2(1.0, 4, PSTR("var"), &scope);

		f1.save();
		f1.load();
		REQUIRE(f1 == 10);

		f2.save();
		f2.load();
		REQUIRE(f2 == 1);

		f1.load();
		REQUIRE(f1 == 1);
	}


	Test::report();
}

void
loop(void)
{
}
