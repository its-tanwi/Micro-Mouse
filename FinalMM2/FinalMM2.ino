#include "globals.h"
#include "treamux.h"
// #include "Api.h"
const uint8_t rows = 16, cols = 16;
// const uint8_t rows = 16, cols = 16;

bool flag = 0;

struct cell PathArray[rows * cols]; // Define PathArray

// uint8_t targetCellsGoal[1] = {12},targetCellStart[1] = {20}, startCell = 20, startDir = 0;

uint8_t targetCellsGoal = 4, targetCellStart = 247, startCell = 247, startDir = 0;
// uint8_t targetCellsGoal = 119, targetCellStart = 240, startCell = 240, startDir = 0;

// int sensorValue[4];



void setup() {
  delay(2000);
  Serial.begin(19200);
  pinMode(13,OUTPUT);
  //  /hc06.begin(9600);
  digitalWrite(13, 0);
  for (int i=0 ;i<4;i++){
    sensorValueLow[i]=analogRead(sense[i]);
  }
  // log("Starting....");
  initialise();
}


void loop() {

    if(currentCell != targetCellsGoal) {
      updateWalls();
      treamux();
      
      // log(String(currentCell));
      // updateTargetCell();
      goToTargetCell();
     }
    else{
      stop_();
    }

}
