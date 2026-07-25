// Thomas Nagy, 2011

#include <QApplication>

#include "ui_but.h"

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
	QWidget window;
    Ui::Form ui;
    ui.setupUi(&window);
    return app.exec();
}
