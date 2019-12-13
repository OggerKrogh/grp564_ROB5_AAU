/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <QLabel>
#include <iostream>
#include "../include/qtros/main_window.hpp"
#include <std_msgs/String.h>
#include <QTime>
#include <QTimer>
#include <QDateTime>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace qtros {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

    ReadSettings();
	setWindowIcon(QIcon(":/images/icon.png"));
    //ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).

    QObject::connect(ui.goButton, SIGNAL(clicked()), this, SLOT(goButton()));
    QObject::connect(ui.pinText, SIGNAL(returnPressed()), this, SLOT(pinText()));
   // QObject::connect(ui.cardRead, SIGNAL(clicked()), this, SLOT(setTextLabel()));

    /*********************
    ** Auto Start
    **********************/
    qnode.init();
    QObject::connect(&qnode,SIGNAL(valueChanged()), this, SLOT(colorChange()));
    QObject::connect(&qnode,SIGNAL(accessTimeOut()), this, SLOT(colorChangeRED()));
    //QObject::connect(&qnode,SIGNAL(commandSent()), this, SLOT(setTextLabel()));

    QObject::connect(&qnode,SIGNAL(stat1()), this, SLOT(setTextLabel1()));
    QObject::connect(&qnode,SIGNAL(stat2()), this, SLOT(setTextLabel2()));
    QObject::connect(&qnode,SIGNAL(stat3()), this, SLOT(setTextLabel3()));
    QObject::connect(&qnode,SIGNAL(stat4()), this, SLOT(setTextLabel4()));
    QObject::connect(&qnode,SIGNAL(stat5()), this, SLOT(setTextLabel5()));
    QObject::connect(&qnode,SIGNAL(stat6()), this, SLOT(setTextLabel6()));
    QObject::connect(&qnode,SIGNAL(stat7()), this, SLOT(setTextLabel7()));

    //QObject::connect(&qnode, &QTimer::timeout, this, SLOT(setTimer()));
    //QObject::connect(&qnode,SIGNAL(cardReadDenied()), this, SLOT(cardReadDeny()));
    //QObject::connect(&qnode,SIGNAL(cardReadAccepted()), this, SLOT(cardReadAccept()));
    //ui.statusLabel->setText("Status: Standing By");
    //ui.cardRead->setText("");
    //t.start();

    setPin = "1234";

}

MainWindow::~MainWindow() {}


/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
	QMessageBox msgBox;
	msgBox.setText("Couldn't find the ros master.");
	msgBox.exec();
}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */
void MainWindow::colorChange() {
    ui.goButton->setStyleSheet("* { background-color: rgb(51,165,50);color: rgb(0, 0, 0);}");
}

void MainWindow::colorChangeRED() {
    ui.goButton->setStyleSheet("* { background-color: rgb(207,20,43);color: rgb(0, 0, 0);}");
}


void MainWindow::pinText() {
   QString test = ui.pinText->text();
   word = test.toStdString();
   if(word==setPin) {
       colorChange();
       qnode.AccessHandler();
   } else{
       colorChangeRED();
       qnode.AccessHandlerDENY();
   }
}

void MainWindow::updateTimer() {
    QTimer *timer = new QTimer;
    //QObject::connect(timer, &QTimer::timeout, this, setTimer());
    timer->start(1000);




    setTimer();


}

void MainWindow::setTimer() {

    /*
    QTimer *timer = new QTimer(this);
    timer->start(1000);

    QTime = QTime::currentTime();
    QString text = time->toString("hh:mm");

   // QString timeText = timer->toString("hh:mm");
    */


    //int timeToS=t.elapsed()/1000;

    //ui.timeNumber->display(timeToS);
    //ROS_INFO_STREAM(timeToS);
    //ui.timeNumber.show();


}

/*

void MainWindow::cardReadDeny() {
    ui.cardRead->setText("Card Denied");
}

void MainWindow::cardReadAccept() {
    ui.cardRead->setText("Card Accepted");
}
*/


void MainWindow::setTextLabel1() {
    ui.goButton->setText("Start transport to CI");
}


void MainWindow::setTextLabel2() {
    ui.goButton->setText("Moving to CI");
}

void MainWindow::setTextLabel3() {
    ui.goButton->setText("Waiting for elevator");
}

void MainWindow::setTextLabel4() {
    ui.goButton->setText("*Elevator music*");
}

void MainWindow::setTextLabel5() {
    ui.goButton->setText("Please pick up transport");
}

void MainWindow::setTextLabel6() {
    ui.goButton->setText("Ready to return to CBD");
}


void MainWindow::setTextLabel7() {
    ui.goButton->setText("Returning to CBD");
}


void MainWindow::goButton() {
    //updateTimer();
    //setTimer();
    qnode.go();
}

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
    QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::ReadSettings() {
    QSettings settings("Qt-Ros Package", "qtros");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
    QString master_url = settings.value("master_url",QString("http://192.168.1.2:11311/")).toString();
    QString host_url = settings.value("host_url", QString("192.168.1.3")).toString();
    //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
    bool remember = settings.value("remember_settings", false).toBool();

    bool checked = settings.value("use_environment_variables", false).toBool();

    if ( checked ) {

    }
}

void MainWindow::WriteSettings() {
    QSettings settings("Qt-Ros Package", "qtros");

    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());

}

void MainWindow::closeEvent(QCloseEvent *event)
{
	WriteSettings();
	QMainWindow::closeEvent(event);
}

}  // namespace qtros

