// Thomas Nagy, 2011-2016

#include "foo.h"

Foo::Foo() : QWidget(NULL) {

}

class Bar_private : public QWidget {
        Q_OBJECT
        signals:
                void test();
        public:
                Bar_private();
};

Bar_private::Bar_private() : QWidget(NULL) {
}

#include "foo.moc"

