/**
 * @file /include/qtros/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef qtros_QNODE_HPP_
#define qtros_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <string>
#include <QThread>
#include <QStringListModel>


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace qtros {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
    void go();
    void IDCallback(const std_msgs::String::ConstPtr& msg);
    void statusCallback(const std_msgs::Int32::ConstPtr& msg);
	void run();
    void AccessHandler();
    bool check();
    void AccessHandlerDENY();
    void getTime();

    bool AccGrant;

	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

    //QStringListModel* loggingModel() { return &logging_model; }
	void log( const LogLevel &level, const std::string &msg);

    Q_SIGNALS:
    //void loggingUpdated();
    void rosShutdown();
    void valueChanged();
    void accessTimeOut();
    void commandSent();
    void stat1();
    void stat2();
    void stat3();
    void stat4();
    void stat5();
    void stat6();
    void stat7();
    //void cardReadAccepted();
    //void cardReadDenied();

private:
	int init_argc;
	char** init_argv;
	ros::Publisher chatter_publisher;
    ros::Publisher time_publisher;
    ros::Subscriber IDreader;
    ros::Subscriber mirStatus;
    //QStringListModel logging_model;
};

}  // namespace qtros

#endif /* qtros_QNODE_HPP_ */
