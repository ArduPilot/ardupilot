// Thomas Nagy, 2011-2016

#ifndef _FOO
#define _FOO

#include <QWidget>
#include <QPushButton>

class Foo : public QWidget {
	Q_OBJECT
	signals:
		void test();
	private slots:
		void handleButton();
	public:
		Foo();
		int FortyTwo();
		QPushButton *m_button;
	public:
		bool myToggle;
};

#endif
