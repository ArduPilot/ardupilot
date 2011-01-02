//
// Unit tests for the AP_MetaClass and AP_Var classes.
//

#include <FastSerial.h>
#include <AP_Common.h>

FastSerialPort(Serial, 0);

#define TEST(name)		Serial.println("test: " #name)
#define	REQUIRE(expr)	if (!(expr)) Serial.println("FAIL: " #expr)

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

	// MetaClass: casting
	{
		TEST(meta_cast);

		AP_Float	f;

		REQUIRE(NULL != AP_MetaClass::meta_cast<AP_Float>(&f));
		REQUIRE(NULL == AP_MetaClass::meta_cast<AP_Int8>(&f));
	}

	Serial.println("done.");
}


void
loop(void)
{
}
