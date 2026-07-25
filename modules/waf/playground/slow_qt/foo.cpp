// Thomas Nagy, 2011

#include <QObject>

#include "foo.h"

Foo::Foo() : QWidget(NULL) {

}

class FooP : public QWidget {
	Q_OBJECT
	signals:
		void test();
	public:
		FooP();
};

FooP::FooP() {

}

#include "foo_cpp_moc.cpp"

