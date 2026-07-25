// Thomas Nagy, 2011-2016

#ifndef _FOO
#define _FOO

#include <QWidget>

class Foo : public QWidget {
	Q_OBJECT
	signals:
		void test();
	public:
		Foo();
};

#endif
