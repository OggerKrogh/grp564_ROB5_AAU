/**
 * @file /include/qtros/main_window.hpp
 *
 * @brief Qt based gui for qtros.
 *
 * @date November 2010
 **/
#ifndef qtros_MAIN_WINDOW_H
#define qtros_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"
#include "std_msgs/String.h"
#include <QTime>

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace qtros {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();
    std::string setPin;
    QTime t; //Timer




public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
	void on_actionAbout_triggered();
    void pinText();
    void goButton();
    void colorChange();
    void colorChangeRED();
    void setTextLabel();
    void setTimer();
    void updateTimer();
    //void cardReadDeny();
    //void cardReadAccept();

    /******************************************
    ** Manual connections
    *******************************************/

private:
    Ui::MainWindowDesign ui;
	QNode qnode;
    std::string word;

};


}  // namespace qtros

#endif // qtros_MAIN_WINDOW_H
