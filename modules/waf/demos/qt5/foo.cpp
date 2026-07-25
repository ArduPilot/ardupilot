// Thomas Nagy, 2011-2016

#include "foo.h"

Foo::Foo() : QWidget(NULL) {
	m_button = new QPushButton("Foo Button", this);
	m_button->setGeometry(QRect(QPoint(50, 60),
	QSize(120, 50)));
	connect(m_button, SIGNAL (released()), this, SLOT (handleButton()));
	myToggle = true;
}

void Foo::handleButton() {
	if (myToggle) {
		m_button->setText("Button Foo");
	} else {
		m_button->setText("Foo Button");
	}
	myToggle = !myToggle;
}

int Foo::FortyTwo() {
	return 42;
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

