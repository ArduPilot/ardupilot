// Thomas Nagy, 2016 (ita)

#include <QApplication>
//#include <QString>
//#include "mainwindow.h"
#include "ui_but.h"
#include "foo.h"

int main(int argc, char **argv)
{
    Q_INIT_RESOURCE(res);
    QApplication app(argc, argv);
    Foo window;
    Ui::Form ui;
    ui.setupUi(&window);
    window.show();
    return app.exec();
/*
    MainWindow window;
    if (argc == 2)
        window.openFile(argv[1]);
    else
        window.openFile(":/files/bubbles.svg");
    window.show();
    return app.exec();
*/
}
