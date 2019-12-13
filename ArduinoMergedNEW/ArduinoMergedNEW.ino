#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Char.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int8.h>
#include <SPI.h>
#include <MFRC522.h>
#include <Servo.h>   
#include "GY91.h"

//Pins for controlling buttons
int pinOne = 44;
int pinTwo = 42;
int pinThree = 40;

//Servo object and variables
Servo myservo;
int pos = 0;    // variable to store the servo position
int lockStat=3;
#define SERVO_PIN       8          // Servo attached to pin 8
#define RST_PIN         49          // Configurable, see typical pin layout above
#define SS_PIN          53         // Configurable, see typical pin layout above

//RFID
MFRC522 mfrc522(SS_PIN, RST_PIN);  // Create MFRC522 instance

ros::NodeHandle  nh;

GY91 GY;

std_msgs::String str_msg;
std_msgs::String msg;
//std_msgs::String qt_msg;
std_msgs::Int8 answer;
std_msgs::Int8 sender;


//FUNCTION DECLARATIONS//

void RFID();

void openLock();
void closeLock();

//ROS
void messageCb( const std_msgs::Int8& inc_msg);
void QtCb(const std_msgs::String& qt_msg);
void pub(std_msgs::Int8 msg);

//Publisher
ros::Publisher fromArduino("fromArduino", &answer);
//RFID
ros::Publisher IDread("IDread", &str_msg);
ros::Publisher logger("logger", &msg);

//Subscriber
ros::Subscriber<std_msgs::String> chatter("chatter", QtCb );
ros::Subscriber<std_msgs::Int8> toArduino("toArduino", &messageCb );

int floorNr = 5;

void setup()
{
  //Serial.begin(9600);
  //Attach servo
  myservo.attach(8);  // attaches the servo on pin 8 to the servo object

  //RFID
   SPI.begin();      // Init SPI bus
  mfrc522.PCD_Init();   // Init MFRC522
  delay(4);       // Optional delay. Some board do need more time after init to be ready, see Readme
  mfrc522.PCD_DumpVersionToSerial();  // Show details of PCD - MFRC522 Card Reader details

  GY.init();
  GY.resetBMP();
  GY.calAcc(); 
  GY.resetGY91();
  GY.setFloor(2);//Set current floor
  
  //Ros 
  nh.initNode();
  
  nh.subscribe(chatter);
  nh.subscribe(toArduino);
  
  nh.advertise(fromArduino);
  nh.advertise(IDread);
  nh.advertise(logger);

  

  //Pins for buttons
  pinMode(pinOne, OUTPUT);
  pinMode(pinTwo, OUTPUT);
  pinMode(pinThree, OUTPUT);

  digitalWrite(pinOne, LOW);
  digitalWrite(pinTwo, LOW);
  digitalWrite(pinThree, LOW);

  closeLock();
}

//Main loop
void loop(){
  
   if (floorNr != 1 && floorNr != 2){
     floorNr = GY.hasReachedFloor();
  }
  //Serial.println(floorNr);
  RFID();
  nh.spinOnce();
  delay(10);
}
   

//send strings back to the computer
void pub(std_msgs::Int8 msg){
  
  answer.data = msg.data;
  fromArduino.publish( &answer );
  nh.spinOnce();
  delay(100); 
  }

void RFID(){
   // Reset the loop if no new card present on the sensor/reader. This saves the entire process when idle.
  if ( ! mfrc522.PICC_IsNewCardPresent()) {
    return;
  }

  // Select one of the cards
  if ( ! mfrc522.PICC_ReadCardSerial()) {
    return;
  }

  String content = "";
  byte letter;
  for (byte i = 0; i < mfrc522.uid.size; i++)
  {
    content.concat(String(mfrc522.uid.uidByte[i] < 0x10 ? " 0" : " "));
    content.concat(String(mfrc522.uid.uidByte[i], HEX));
  }
  content.toUpperCase();

  if (content.substring(1) == "42 01 D0 D2" || content.substring(1) == "A2 1D D0 D2" || content.substring(1) == "02 76 D0 D2" || content.substring(1) == "A2 BA CF D2" || content.substring(1) == "02 BB D2 D2") //change here the UID of the card/cards that you want to give access
  {
    openLock();
    str_msg.data = "1";
    IDread.publish( &str_msg );
    nh.spinOnce();

    String cardNr = content.substring(1);

    char arrayFix[11]={};

    for(int i=0; i<11; i++) {
      arrayFix[i]=cardNr[i];
    }

   msg.data=arrayFix;
    
    logger.publish(&msg);

    nh.spinOnce();
    delay(1000);
  }

  else   {

   
    closeLock();
    str_msg.data = "0";
    IDread.publish( &str_msg );
    nh.spinOnce();
    delay(1000);
  }
  
}


//Servo control
void closeLock(){
  if (lockStat!=0){
    for (pos = 60; pos >= 10; pos -= 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(5);                       // waits 15ms for the servo to reach the position
    }
  lockStat = 0;
  }
}

void openLock(){
  if (lockStat!=1){
    for (pos = 10; pos <= 60; pos += 1) { // goes from 180 degrees to 0 degrees
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
    delay(5);                       // waits 15ms for the servo to reach the position
    }
  lockStat = 1; 
  }  
}

void QtCb( const std_msgs::String& qt_msg){
   //logger.publish(&qt_msg);
  if(qt_msg.data[0]=='G'){
    closeLock();
    }  
  }

//Callback when recieving a message
void messageCb( const std_msgs::Int8& inc_msg){

  switch(inc_msg.data) {
    case 1 :        //calibrate to current floor 1
    
    floorNr = 5;
    GY.setFloor(1);
    GY.resetBMP();
    GY.calAcc(); 
    GY.resetGY91();
    sender.data=8; 
    pub(sender);
        
             break; 
                  
    case 2 :        // Calibrate to current floor 2
    
    floorNr = 5;
    GY.setFloor(2);
    GY.resetBMP();
    GY.calAcc(); 
    GY.resetGY91();
    sender.data=8; 
    pub(sender);
             break;
             
    case 3 :       
    floorNr = 0; 
             break;

                            
    case 4 :        // Request current floor
    sender.data=floorNr; 
    pub(sender);
             break;

    case 5 :
  //Write 1
  digitalWrite(pinOne, HIGH);
  digitalWrite(pinTwo, LOW);
  digitalWrite(pinThree, LOW);
  delay(50);

  //Write 0
  digitalWrite(pinOne, LOW);
  digitalWrite(pinTwo, LOW);
  digitalWrite(pinThree, LOW);
      break;

    case 6 :
  //Write 2
  digitalWrite(pinOne, LOW);
  digitalWrite(pinTwo, HIGH);
  digitalWrite(pinThree, LOW);
  delay(50);

  //Write 0
  digitalWrite(pinOne, LOW);
  digitalWrite(pinTwo, LOW);
  digitalWrite(pinThree, LOW);
      break;

      
    case 7 :
  //Write 3
  digitalWrite(pinOne, HIGH);
  digitalWrite(pinTwo, HIGH);
  digitalWrite(pinThree, LOW);
  delay(50);

  //Write 0
  digitalWrite(pinOne, LOW);
  digitalWrite(pinTwo, LOW);
  digitalWrite(pinThree, LOW);
      break;
  
    case 8 :
  //Write 4
  digitalWrite(pinOne, LOW);
  digitalWrite(pinTwo, LOW);
  digitalWrite(pinThree, HIGH);
  delay(50);

  //Write 0
  digitalWrite(pinOne, LOW);
  digitalWrite(pinTwo, LOW);
  digitalWrite(pinThree, LOW);
      break;
                 
    default:
    sender.data=120;
    pub(sender);  
             break;
    }
}
