#include "ros/ros.h"
#include <sstream>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

//Libraries for RealSense
#include <opencv2/core.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
//#include <librealsense2/rs.hpp>

//arduino
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int8.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

cv_bridge::CvImagePtr cv_ptr;
cv_bridge::CvImagePtr cv_ptrDep;


geometry_msgs::Pose curPos;
double curXpose;
double curYpose;

double pose(int flag);
void onPoseSet(double x, double y, double theta);
typedef boost::array<double, 36> array;
array Covariance = {0.0};

int qtGo = 0;
int curFloor = 0;
int door = 0;


//Delay for a set time in seconds
void delaying(float t){

  ros::Time tidDelay = ros::Time::now()+ros::Duration(t);
  while(ros::Time::now()<tidDelay){
  }
}
//RGB image caalback
void imageCb(const sensor_msgs::ImageConstPtr msg){

  try
  {
    cv_ptr = cv_bridge::toCvCopy(*msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

}
//Depth image caalback
void depthCb(const sensor_msgs::ImageConstPtr msg){

  try
  {
    cv_ptrDep = cv_bridge::toCvCopy(*msg, sensor_msgs::image_encodings::TYPE_16UC1);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

}
//Send Command to arduino
void sendToArduino(int x, ros::Publisher pub1){

  std_msgs::Int8 arduinoMessage;
  arduinoMessage.data = x;
  pub1.publish(arduinoMessage);
}

// CALLBACK FUNCTION FOR onPoseSet
void moveBaseCallback(const geometry_msgs::PoseWithCovarianceStamped msg){

curXpose = msg.pose.pose.position.x;
curYpose = msg.pose.pose.position.y;
//ROS_INFO("x: %f, y: %f",curXpose,curYpose);
}
//MOVETOPOINT FUNCTION
void moveToPoint(double x, double y, double rot, int button, ros::Publisher pub1, MoveBaseClient& ac){

  //MoveBaseClient ac("move_base",true);
  //while(!ac.waitForServer(ros::Duration(3.0))){
    //ROS_INFO("Waiting for the move_base action server to come up");
  //}
  ac.waitForServer();
	ac.cancelAllGoals();


	move_base_msgs::MoveBaseGoal start;

  start.target_pose.header.frame_id = "map";
  start.target_pose.header.stamp = ros::Time::now();
  start.target_pose.pose.position.x = x;
  start.target_pose.pose.position.y = y;


  tf::Quaternion quat;
  quat.setRPY(0.0, 0.0, rot);
  tf::quaternionTFToMsg(quat, start.target_pose.pose.orientation);
	ros::Rate rate(1);

  ac.sendGoal(start);
  ac.waitForResult();

	while(ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED ){

		if (ac.getState() == actionlib::SimpleClientGoalState::ABORTED){
			ROS_INFO("The path is blocked, I am waiting");
switch (button) {
  case 1:
    sendToArduino(7,pub1);
    delaying(0.5);
    break;

  case 2:
    sendToArduino(6,pub1);
    delaying(0.5);
    break;

  default:
  break;
}
}
		ac.sendGoal(start, NULL, NULL, NULL);
		ac.waitForResult();

		rate.sleep();
  }

	//actionlib::SimpleClientGoalState state = ac.getState();
	//ROS_INFO("Action state: %s", state.toString().c_str() );
  //ac.stopTrackingGoal();
  //ac.cancelAllGoals();

  ROS_INFO("Hooray, the base moved to position");

}
//TELEPORT THE ROBOT ON INTERNAL MAP FUNCTION
void onPoseSet(double x, double y, double rot, ros::Publisher pub2){
    ros::Rate loop_rate(1);

      //Geometry message for initial pose
      geometry_msgs::PoseWithCovarianceStamped pose;

      std::string fixed_frame = "map";
      pose.header.frame_id = fixed_frame;
      pose.header.stamp = ros::Time::now();
      // set x,y coord
      pose.pose.pose.position.x = x;
      pose.pose.pose.position.y = y;
      pose.pose.pose.position.z = 0.0;
      //Convert rotation(radians) to Quaternion
      tf::Quaternion quat;
      quat.setRPY(0.0, 0.0, rot);
      tf::quaternionTFToMsg(quat, pose.pose.pose.orientation);
      //Covariance matrix with 0.0
      pose.pose.covariance = Covariance;
      // publish
      ROS_INFO("x: %f, y: %f, z: 0.0 rot: %f",x,y, rot);

    while( x-0.1 > curXpose || curXpose > x+0.1 ||  y-0.1 > curYpose || curYpose > y+0.1 ){

    ROS_INFO("current pos x: %f, y: %f,",curXpose,curYpose);

      pub2.publish(pose);
      loop_rate.sleep();
      ros::spinOnce();
  }
}

//Get RGB image
cv::Mat getImage() {
    cv::Mat img = cv::Mat(100,100, CV_8UC3); //RGB
    ros::Time tid = ros::Time::now()+ros::Duration(5.0);

    while (ros::ok()) {

      if(ros::Time::now()>tid){
          ROS_INFO("Did not get image. Abandonning");
          return img;
      }

    ros::spinOnce();
    if(cv_ptr){

      img = cv::Mat(cv_ptr->image.rows, cv_ptr->image.cols, CV_8UC3);
      img = cv_ptr->image;
      cv_ptr.reset();



      return img;
    }

  }
return img;
}
//Get depth image
cv::Mat getDepthImage() {

    cv::Mat imgD = cv::Mat(100,100, CV_16UC1); //Depth
    ros::Time tidD = ros::Time::now()+ros::Duration(5.0);

    while (ros::ok()) {

      if(ros::Time::now()>tidD){
          ROS_INFO("Did not get image. Abandonning");
          return imgD;
      }

    ros::spinOnce();
    if(cv_ptrDep){

      imgD = cv::Mat(cv_ptrDep->image.rows, cv_ptrDep->image.cols, CV_16UC1);
      imgD = cv_ptrDep->image;
      cv_ptrDep.reset();



      return imgD;
    }

  }
return imgD;
}

//Incoming messages from the QT GUI
void QTcallback(const std_msgs::String msg){

if(msg.data=="Go command")
qtGo = 1;
}
//Incomming messaages from the arduino
void ARDcallback(const std_msgs::Int8 msg){

curFloor = msg.data;

}
//Wait for the go command from the QT GUI
void waitForGo(){
  qtGo = 0;

  while (qtGo==0) {
    ROS_INFO("Waiting for GO...");
    ros::spinOnce();
    delaying(1.0);
  }
  qtGo==1;
}
//Request current floor and wait for the floor to be reached
void waitForFloor(int f, ros::Publisher pub1){

  while(curFloor != f){
    sendToArduino(4, pub1);//Request current floor NR
    ros::spinOnce();
    ROS_INFO("Current floor: %d",curFloor);
    delaying(1.0);
  }
}
//Check wich elevator door is open
int elevatorDoor(int floorNR){

  //How many pixels should left img be bigger than right
  int offset = 0;
  //Cutout from center in pixels
  int cutout = 100;
  //threshhold for open door
  double threshRight1 = 3340.0;
  double threshLeft1 =  3170.0;

  double threshRight2 = 3730.0;
  double threshLeft2 =  3790.0;

  cv::Mat colImg = getImage();
  cv::Mat dep = getDepthImage();
  cv::Mat dep2 = cv::Mat(dep.rows , dep.cols, CV_16UC1);
  cv::Mat depLeft = cv::Mat(dep.rows , ((dep.cols/2)+offset)-cutout, CV_16UC1);
  cv::Mat depRight = cv::Mat(dep.rows , ((dep.cols/2)-offset)-cutout, CV_16UC1);

  cv::medianBlur(dep, dep2, 5);

//left image
  for (int x = 0; x < ((dep2.cols/2)+offset)-cutout; x++) {
      for (int y = 0; y < dep2.rows-150; y++) {
        depLeft.at<unsigned short>(y, x) = dep2.at<unsigned short>(y, x);
  }
}
//right image
for (int x = ((dep2.cols/2)+offset)+cutout; x < dep2.cols; x++) {
    for (int y = 0; y < dep2.rows-150; y++) {
      depRight.at<unsigned short>(y, x-(((dep2.cols/2)+offset)+cutout)) = dep2.at<unsigned short>(y, x);
}
}
//Average left image
double avgCountLeft = 0;
double depTotalLeft = 0.0;
double avgDepLeft = 0.0;
for (int x = 0; x < depLeft.cols; x++) {
  for (int y = 0; y < depLeft.rows; y++) {
if (depLeft.at<unsigned short>(y, x)<10000 && depLeft.at<unsigned short>(y, x)>1){
  depTotalLeft+=(double)depLeft.at<unsigned short>(y, x);
  avgCountLeft++;
    }
  }
}
avgDepLeft = depTotalLeft/avgCountLeft;
//Average rigt image
double avgCountRight = 0;
double depTotalRight = 0.0;
double avgDepRight = 0.0;
for (int x = 0; x < depRight.cols; x++) {
  for (int y = 0; y < depRight.rows; y++) {
if (depRight.at<unsigned short>(y, x)<10000 && depRight.at<unsigned short>(y, x)>1){
  depTotalRight+=(double)depRight.at<unsigned short>(y, x);
  avgCountRight++;
    }
  }
}
avgDepRight = depTotalRight/avgCountRight;

  ROS_INFO("Left mean: %f",avgDepLeft);
  ROS_INFO("Right mean: %f",avgDepRight);

/*
  cv::imshow("color",colImg);
  cv::waitKey(100);
  cv::imshow("full",dep);
  cv::waitKey(100);
  cv::imshow("left",depLeft);
  cv::waitKey(100);
  cv::imshow("right",depRight);
  cv::waitKey(0);
*/
if (floorNR==1) {
  if(avgDepRight>threshRight1){
    return 1; //right door open
  }else if(avgDepLeft>threshLeft1){
    return 2;//left door open
    }else{
    return 0; //no door open
    }
  }else if(floorNR==2){
    if(avgDepRight>threshRight2){
      return 1; //right door open
    }else if(avgDepLeft>threshLeft2){
      return 2;//left door open
      }else{
      return 0; //no door open
      }
  }

}
//Calibrate arduino
void calArd(int x, ros::Publisher pub1){
while(curFloor != 8){
  sendToArduino(x, pub1);//Request current floor NR
  delaying(1.0);
  ros::spinOnce();
  ROS_INFO("Connect: %d",curFloor);
  }
}


//MAIN -------------------------------------------------------
int main(int argc, char** argv){
    //Initialize ros
    ros::init(argc, argv, "simple_navigation_goals");
    ros::start();
    ros::Rate loop_rate(1);

    ros::NodeHandle nh1;

    //Subscribe to camera rgb image
    ros::Subscriber image_sub_ = nh1.subscribe("/camera/color/image_raw", 1,imageCb);
    //Subscribe to camera depth
    ros::Subscriber depth_sub_ = nh1.subscribe("/camera/depth/image_rect_raw", 1,depthCb);
    //Subscribe to qt topic
    ros::Subscriber qt_sub = nh1.subscribe("chatter", 1, QTcallback);
    //Subscribe to feedback from arduino
    ros::Subscriber arduino_sub = nh1.subscribe("fromArduino", 64, ARDcallback);

    //Subscribe to the mir platform current position in the map
    ros::Subscriber sub_ = nh1.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose", 10, moveBaseCallback);
    //Publisher for the arduino
    ros::Publisher pubArd = nh1.advertise<std_msgs::Int8> ("/toArduino", 100);
    //Publisher for teleporting
    ros::Publisher pubPose = nh1.advertise<geometry_msgs::PoseWithCovarianceStamped> ("/initialpose", 1);

    MoveBaseClient ac("move_base",true);

    while(!ac.waitForServer(ros::Duration(3.0))){
      ROS_INFO("Waiting for the move_base action server to come up");
    }


    ac.cancelAllGoals();

    //door 1 = right door
    //door 2 = left door

    //Ard 1 = calibrate to floor 1
    //Ard 2 = calibrate to floor 2
    //Ard 3 = Start floor measurements
    //Ard 4 = request floor
    //Ard 5 button = Blue
    //Ard 6 button = red
    //Ard 7 button = yellow
    //Ard 8 button = green

//FINAL

while (ros::ok()) {

    //Floor 2
    ROS_INFO("Moving to CBD");
    moveToPoint(34.800, 42.450, 0.0, 0, pubArd, ac);

    ROS_INFO("Waiting for qt GO COMMAND");
    waitForGo(); //Wait for QT GUI

    ROS_INFO("Moving to elevator");
    moveToPoint(51.96, 29.9, 3.14, 0, pubArd, ac); //Front of elevator floor 2

    callElevator:

    ROS_INFO("Calling elevator");
    sendToArduino(7, pubArd); //Push the yellow button


    ROS_INFO("Waiting for elevator");
    ros::Time timeoutDoor = ros::Time::now()+ros::Duration(10.0);
    door = 0;

    while(door==0){
        if(ros::Time::now()>timeoutDoor){
            goto callElevator;
        }
        door = elevatorDoor(2); //Check elevators second floor
        delaying(0.5);
    }
    ROS_INFO("door: %i",door);

    //Right door
    if(door==1){
    ROS_INFO("Moving into elevator");
    moveToPoint(45.550, 31.100, 3.14, 1, pubArd, ac); //fully inside
    sendToArduino(3, pubArd);
    delaying(1.0);
    ROS_INFO("Done Moving into elevator");

    ROS_INFO("Pressing floor");
    calArd(2, pubArd); //Calibrate floor measurements
    delaying(1.0);
    sendToArduino(5, pubArd); //Push the blue button
    delaying(1.0);
    sendToArduino(5, pubArd); //Push the blue button
    delaying(1.0);
    sendToArduino(5, pubArd); //Push the blue button

    ROS_INFO("Change map");
    onPoseSet(14.500, 31.050, 3.14, pubPose);
    delaying(1.0);

    ROS_INFO("Wait for floor 1");
    waitForFloor(1, pubArd); //Wait for floor 1 to be reached

    ROS_INFO("Moving out");
    //moveToPoint(10.800, 28.600, 3.14, 0, pubArd, ac);
    }

    //left door
    if(door==2){
    ROS_INFO("Moving into elevator");
    moveToPoint(45.500, 28.450, 3.14, 1, pubArd, ac); //fully inside
    sendToArduino(3, pubArd);
    delaying(1.0);
    ROS_INFO("Done Moving into elevator");

    ROS_INFO("Pressing floor");
    calArd(2, pubArd); //Calibrate floor measurements
    delaying(1.0);
    sendToArduino(6, pubArd); //Push the red button
    delaying(1.0);
    sendToArduino(6, pubArd); //Push the blue button
    delaying(1.0);
    sendToArduino(6, pubArd); //Push the blue button

    ROS_INFO("Change map");
    onPoseSet(14.450, 28.100, 3.14, pubPose);
    delaying(1.0);

    ROS_INFO("Wait for floor 1");
    waitForFloor(1, pubArd); //Wait for floor 1 to be reached

    ROS_INFO("Moving out");
    //moveToPoint(10.800, 28.600, 3.14, 0, pubArd, ac);
    }


    //Floor 1
    ROS_INFO("Moving to CI");
    moveToPoint(6.1, 4.7, 3.14, 0, pubArd, ac);

    ROS_INFO("Waiting for qt GO COMMAND");
    waitForGo(); //Wait for QT GUI

    ROS_INFO("Moving to elevator");
    moveToPoint(9.750, 29.550, 0.0, 0, pubArd, ac); //Front of elevator floor 1

    callElevator:

    ROS_INFO("Calling elevator");
    sendToArduino(7, pubArd); //Push the yellow button


    ROS_INFO("Waiting for elevator");
    ros::Time timeoutDoor = ros::Time::now()+ros::Duration(10.0);
    door = 0;

    while(door==0){
      if(ros::Time::now()>timeoutDoor){
        goto callElevator;
      }
      door = elevatorDoor(1); //Check elevators first floor
      delaying(0.5);
    }
    ROS_INFO("door: %i",door);

    //left door
    if(door==2){
    ROS_INFO("Moving into elevator");
    moveToPoint(15.600, 31.050, 0.0, 1, pubArd, ac); //fully inside
    delaying(1.0);
    ROS_INFO("Done Moving into elevator");

    ROS_INFO("Pressing floor");
    calArd(1, pubArd); //Calibrate floor measurements
    delaying(1.0);
    sendToArduino(5, pubArd); //Push the blue button
    delaying(1.0);
    sendToArduino(5, pubArd); //Push the blue button
    delaying(1.0);
    sendToArduino(5, pubArd); //Push the blue button

    ROS_INFO("Change map");
    onPoseSet(46.550, 31.100, 0.0, pubPose);
    delaying(1.0);

    ROS_INFO("Wait for floor 2");
    waitForFloor(2, pubArd); //Wait for floor 1 to be reached

    ROS_INFO("Moving out");

    }

    //right door
    if(door==1){
    ROS_INFO("Moving into elevator");
    moveToPoint(15.600, 28.10, 0.0, 1, pubArd, ac); //fully inside
    delaying(1.0);
    ROS_INFO("Done Moving into elevator");

    ROS_INFO("Pressing floor");
    calArd(2, pubArd); //Calibrate floor measurements
    delaying(1.0);
    sendToArduino(6, pubArd); //Push the red button
    delaying(1.0);
    sendToArduino(6, pubArd); //Push the red button
    delaying(1.0);
    sendToArduino(6, pubArd); //Push the red button

    ROS_INFO("Change map");
    onPoseSet(46.550, 28.450, 0.0, pubPose);
    delaying(1.0);

    ROS_INFO("Wait for floor 2");
    waitForFloor(2, pubArd); //Wait for floor 1 to be reached

    ROS_INFO("Moving out");

    }
}

   return 0;
}
