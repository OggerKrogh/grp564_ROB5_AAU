#include "GY91.h"

const int RunningAverageCount = 20;
double RunningAverageBuffer[RunningAverageCount];
int NextRunningAverage;

ExponentialFilter<float> ADCFilter(10, 0);

GY91::GY91(){}

void GY91::init(){

    //Init MPU
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  //Serial.begin(9600);

  //Initialize the BMP
  bmp.begin(BMP280_ADDRESS_ALT);

  // Default settings from datasheet.
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     //Operating Mode.
                  Adafruit_BMP280::SAMPLING_X2,     //Temp. oversampling
                  Adafruit_BMP280::SAMPLING_X16,    //Pressure oversampling
                  Adafruit_BMP280::FILTER_X16,      //Filtering.
                  Adafruit_BMP280::STANDBY_MS_500); //Standby time.

}

double GY91::readBMP(){

    return bmp.readAltitude(1006.6500)-currentBMP;
}

double GY91::filterAccZ(){

  ADCFilter.Filter(zAcc());
  double accZ = (ADCFilter.Current()-BaseAcZ)*calAccZ;
  return accZ;
}

//Make current height measurement new zero
void GY91::resetBMP(){
 currentBMP = bmp.readAltitude(1006.6500);
}

int GY91::isElevatorMoving(){
  double Ac = avgAcc();

 if( curFloor== 1){

  if(Ac > accTrigger){
    elevatorFlag = 1; //Elevator is accelerating upwards
    return elevatorFlag; // Elevator is still moving
    }
  else if(Ac < -accTrigger){
    elevatorFlag = 0; // Elevator is deaccelerating, will stand still
    return elevatorFlag;
    }
  else if(Ac<0.3 && Ac>-0.3 ){
    elevatorFlag = 2;
    return elevatorFlag;
    }
  }

 if(curFloor == 2){

  if(Ac < -accTrigger){
    elevatorFlag = 1; //Elevator is accelerating downwards
    return elevatorFlag; // Elevator is still moving
    }
  else if(Ac>accTrigger){
    elevatorFlag = 0; // Elevator is deaccelerating, will stand still
    return elevatorFlag;
    }
    else if(Ac<0.3 && Ac>-0.3 ){
      elevatorFlag = 2;
      return elevatorFlag;
      }
  }
}

int GY91::hasReachedFloor(){
  if (isElevatorMoving()== 0 && Decel == 0){
     Decel = 1;
    }

    if(Decel ==1){
     if (isElevatorMoving() == 2){
       Decel = 0;
       return checkFloor();
     }
   }

   return 4;
}

// Accelerometer functions
double GY91::zAcc(){

  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,12,true);

  AcX = (uint16_t) Wire.read() << 8;  // first read MSB
  AcX |= Wire.read();                 // then LSB

  AcY = (uint16_t) Wire.read() << 8;  // first read MSB
  AcY |= Wire.read();                 // then LSB

  AcZ = (uint16_t) Wire.read() << 8;  // first read MSB
  AcZ |= Wire.read();   // then LSB

  return AcZ;
}
/*
void GY91::calAcc(){
  timeStamp = millis();

  while(millis()<timeStamp+5000){
    ADCFilter.Filter(zAcc());
    }
    BaseAcZ = ADCFilter.Current();
    calAccZ = gravity/BaseAcZ;
}
*/

void GY91::calAcc(){

    double acAcZ=0.0;

    for(int j = 0; j<calIte; j++){
      acAcZ += zAcc();
      //Serial.println(acAcZ);
     }

   BaseAcZ = acAcZ/double(calIte);
   //Serial.print("base acz: ");
   //Serial.println(BaseAcZ);

   if(BaseAcZ == 0){

   }else{
   calAccZ = gravity/BaseAcZ;
   }

}

double GY91::avgAcc()
{
  double RawAcc = (zAcc()-BaseAcZ)*calAccZ;


  RunningAverageBuffer[NextRunningAverage++] = RawAcc;
  if (NextRunningAverage >= RunningAverageCount)
  {
    NextRunningAverage = 0;
  }
  double RunningAverageAcc = 0;
  for(int i=0; i< RunningAverageCount; ++i)
  {
    RunningAverageAcc += RunningAverageBuffer[i];
  }
  RunningAverageAcc /= RunningAverageCount;

  delay(1);
  return RunningAverageAcc;
}


//check what the current floor
int GY91::checkFloor(){

 double altChange;
  if (curFloor == 1){
   altChange = readBMP();
    if (altChange > HeightUp){
      return 2;
    }else{
    return 3;
    }
  }

  if(curFloor == 2){
    altChange = readBMP();
    if (altChange < HeightDown){
      return 1;
    }else{
    return 3;
    }
  }
}

void GY91::setFloor(int x){

  curFloor = x;

}

void GY91::resetGY91(){

  elevatorFlag = 3;
  Decel = 0;

}
