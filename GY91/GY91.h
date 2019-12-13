#ifndef GY91_H
#define GY91_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include "Filter.h"

class GY91
{
public:
      GY91();

      void init();

      int checkFloor();
      int hasReachedFloor();
      int isElevatorMoving();

      void resetBMP();
      void calAcc();
      void setFloor(int);
      void resetGY91();

      double zAcc();
      double avgAcc();
      double filterAccZ();
      double readBMP();

      //Variables to change
      int calIte = 30; // number of readings for acceleration calibration
      double accTrigger = 0.5; //acceleration or deaccelerating for determening elevator stopping
      double HeightUp = 1.6; //height for floor when going up to floor 2
      double HeightDown = -1.2; //height for floor when going down to floor 1

    //  double HeightUp = -5.0;
    //  double HeightDown = 5.0;

private:

      const int MPU=0x68;
      const double gravity = 9.816;

      double currentBMP;
      Adafruit_BMP280 bmp;

      int16_t AcX,AcY,AcZ;
      double BaseAcZ = 0;

      double calAccZ;
      double velocity = 0.0;

      int curFloor = 0;
      int elevatorFlag = 3;
      int Decel = 0;

      unsigned long timeStamp;

};


#endif
