// Thomas Nagy, 2011

#include "foo.h"

Foo::Foo() : QWidget(NULL) {

}

/* WARNING: do not declare private classes in .cpp files (put them in header files) */

class Bar_private : public QWidget {
        Q_OBJECT
        signals:
                void test();
        public:
                Bar_private();
};

Bar_private::Bar_private() : QWidget(NULL) {
}

#if WAF
#include "foo.moc"
#include "foo.cpp.moc"
#endif
