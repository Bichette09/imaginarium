#include "mainwindow.h"
#include <QApplication>
#include <QDebug>
#include <cassert>

#include <zmq.h>


int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

	MainWindow w;
	w.show();

	a.exec();

	return 0;
}
