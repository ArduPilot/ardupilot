
#include <ruby.h>

static VALUE
m_hello (VALUE self)
{
	return rb_str_new2("Hello World");
}

void
Init_mytest_ext (void)
{
	VALUE mTest = rb_define_module ("MytestInt");

	rb_define_module_function (mTest, "hello", m_hello, 0);
}
