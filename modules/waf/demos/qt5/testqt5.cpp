// Example of Qt5 Unit test with QtTest library
// Federico Pellegrin, 2019 (fedepell)

#include "foo.h"
#include <QtTest/QtTest>

class TestQt5Test: public QObject {
	Q_OBJECT
	private:
		Foo myFoo;
	private slots:
		void testGui();
		void testFunc();
};

// Test of the UI by simulating a button click and button label reading
void TestQt5Test::testGui() {
	QCOMPARE(myFoo.m_button->text(), QString("Foo Button"));
	QTest::mouseClick(myFoo.m_button, Qt::LeftButton,Qt::NoModifier, QPoint(5,5), 0);
	QCOMPARE(myFoo.m_button->text(), QString("Button Foo"));
}

// Test of a normal function
void TestQt5Test::testFunc() {
	QCOMPARE(myFoo.FortyTwo(), 44); // this fails! 42 != 44
}

QTEST_MAIN(TestQt5Test)

#include "testqt5.moc"

