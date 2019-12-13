/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/qtros/qnode.hpp"
#include "boost/date_time/posix_time/posix_time.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace qtros {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
    {

    //QObject::connect(&button, QPushButton::clicked, QNode::go(),srd::bind())
}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"qtros");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
	chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
    time_publisher = n.advertise<std_msgs::String>("timeLog", 1000);
    IDreader = n.subscribe("IDread", 1000, &QNode::IDCallback, this);
    AccGrant=false;
    start();
	return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
	ros::init(remappings,"qtros");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
	chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
    IDreader = n.subscribe("IDread", 1000, &QNode::IDCallback, this);
    AccGrant=false;
	start();
	return true;
}


void QNode::getTime() {
    boost::posix_time::ptime my_posix_time = ros::WallTime::now().toBoost();
    std::string iso_time_str = boost::posix_time::to_iso_extended_string(my_posix_time);


    /*
    char arrayFix[16]={};

       for(int i=0; i<16; i++) {
         //arrayFix[i]=msg->data.c_str()[i];
          if(i<10){
           arrayFix[i]=iso_time_str[i];
         }else if(i==10){
           arrayFix[i]=' ';
         } else {
           arrayFix[i]=iso_time_str[i];
         }
       }

       arrayFix[10]=" ";

    ROS_INFO_STREAM(arrayFix);
    */

    std_msgs::String msg;
    std::stringstream ss;
    ss << iso_time_str;
    msg.data = ss.str();
    time_publisher.publish(msg);
}


void QNode::go() {
    if(check()){
        std_msgs::String msg;
        std::stringstream ss;
        ss << "Go command";
        msg.data = ss.str();
        chatter_publisher.publish(msg);
        ros::spinOnce();
        AccGrant=false;
        getTime();
        Q_EMIT commandSent();
        Q_EMIT accessTimeOut();
    } else {
    std_msgs::String msg;
    std::stringstream ss;
    ss << "Access Denied";
    msg.data = ss.str();
    chatter_publisher.publish(msg);
    ros::spinOnce();
    }
}



bool QNode::check() {
    ros::spinOnce();
    if(AccGrant==true) {
        ros::spinOnce();
        return true;
    } else {
        return false;
   }
}
/*
void QThread::run() {
    for(int i=0; i<10; i++){
        ros::Rate loop_rate(1);
        loop_rate.sleep();
    }
    AccGrant=false;
    Q_EMIT accessTimeOut();//Change back to red
    quit();
} */


//When run grants access for 10 seconds
void QNode::AccessHandler(){
    AccGrant=true;
    //cardReadAccepted();
}

void QNode::AccessHandlerDENY(){
    AccGrant=false;
    //cardReadDenied();
}



//RFID card read
void QNode::IDCallback(const std_msgs::String::ConstPtr& msg) {
    std::string daniel;
    daniel= msg->data;

    if (daniel=="1") { //1=access grant, else not grant
        Q_EMIT valueChanged();
        ROS_INFO("Card Accepted");
        AccessHandler();
    } else {
        AccGrant=false;
        Q_EMIT accessTimeOut();
        ROS_INFO("Card Denied");
    }
}



void QNode::run() {
	ros::Rate loop_rate(1);
	while ( ros::ok() ) {
        ros::spinOnce();
		loop_rate.sleep();
	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}


}  // namespace qtros
