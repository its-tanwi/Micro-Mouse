#include "globals.h"
#include "treamux.h"
// #include "Api.h"
#include "src/Stack/Stack.h"




static bool isNeighbourValid(uint8_t location, uint8_t direction);
static uint8_t getTargetAbsoluteDirection(uint8_t target);
static uint8_t getTargetRelativeDirection(uint8_t target);
static bool isDestination(uint8_t location);
static bool isEnclosed(uint8_t location);
Stack st(255);

// setColorcell(int x, int y, int visited){
//   if(visited == 1){
//     setColor(y,4-x,'g');
//   }
//   else if(visited == 2){
//     setColor(y,4-x,'r');
//   }
// }

// setColorcell(int x, int y, int visited){
//   if(visited == 1){
//     setColor(y,15-x,'g');
//   }
//   else if(visited == 2){
//     setColor(y,15-x,'r');
//   }
// }


uint8_t currentCell, targetCell;
uint8_t leftDir, currentDir, rightDir, nextLeftDir, nextDir, nextRightDir;

uint8_t readingCellLoc, readingCellDistance, targetRelativeDirection, neighbourCellLoc;

uint8_t distanceFromTarget = 1;

short cellDirectionAddition[4] = { -rows, 1, rows, -1 };

void treamux(){
  if(isDeadEnd(currentCell)){
          // log("Dead End");
          PathArray[currentCell].visited = 2;

          // setColorcell(delineariseRow(currentCell),delineariseCol(currentCell),2);

          targetCell = getNeighbourLocation(currentCell,(currentDir+2)%4);
          // log("target cell: "+String(targetCell));
          getTarget();

          
          

  }
  else if(isJunction(currentCell)){
        if(PathArray[currentCell].visited == 0){

          // setColorcell(delineariseRow(currentCell),delineariseCol(currentCell),1);

          // log("Junction New");
          st.push(currentDir);
          // log("Pushed: "+String(currentDir));
         
          PathArray[currentCell].visited = 1;

          // setColorcell(delineariseRow(currentCell),delineariseCol(currentCell),1);

          for (uint8_t i = 0; i < 4; i++) {
            if (!wallExists(currentCell, (i + currentDir) % 4)) {
              readingCellLoc = getNeighbourLocation(currentCell, (i + currentDir) % 4);
              // log("Reading Cell Location: " + String(readingCellLoc));
              if (isUnvisited(readingCellLoc)) {
                // log("is unvisited");
                targetCell = readingCellLoc;
                break;
              }
            }
          }
          // log("target cell: "+String(targetCell));
          getTarget();
          // log("relative dir: "+String(targetRelativeDirection));

        }
        // else if(PathArray[currentCell].visited == 1 and PathArray[getNeighbourLocation(currentCell,(currentDir+2)%4)].visited == 1){
        //   targetRelativeDirection = south;
        //   updateDirection(&nextLeftDir,targetRelativeDirection);
        //   updateDirection(&nextDir, targetRelativeDirection);
        //   updateDirection(&nextRightDir, targetRelativeDirection);

        //   distanceFromTarget = 1;

        // }
        else{
          // log("Junction visited");

          uint8_t minVisitedValue = 255; 
          for(uint8_t i = 0; i < 4; i++) {
              if(!wallExists(currentCell,(i+currentDir)%4)) {
                  uint8_t neighborCell = getNeighbourLocation(currentCell,(i+currentDir)%4);
                  if(PathArray[neighborCell].visited < minVisitedValue) {
                      minVisitedValue = PathArray[neighborCell].visited;
                      targetCell = neighborCell;
                  }
              }

          }
          
          if(minVisitedValue == 1){
            PathArray[currentCell].visited = 2;

            // setColorcell(delineariseRow(currentCell),delineariseCol(currentCell),2);

            int temp = *st.peek();
            // log("temp: "+String(temp));
            st.pop();
            targetCell = getNeighbourLocation(currentCell,(temp+2)%4);
          }
          // log("target cell: "+String(targetCell));
          getTarget();
          }

        }
  else{
        // log("Normal Cell");

      PathArray[currentCell].visited++;
      // setColorcell(delineariseRow(currentCell),delineariseCol(currentCell),PathArray[currentCell].visited);
      bool flag = 0;
      for(uint8_t i = 0; i < 4; i++) {
          if(!wallExists(currentCell, (i+currentDir)%4)) {
              
              uint8_t neighborCell = getNeighbourLocation(currentCell, (i+currentDir)%4);
              // log("Neighbor Cell: "+ String(neighborCell) + " " + String(PathArray[neighborCell].visited));
              if(PathArray[neighborCell].visited < PathArray[currentCell].visited) {
                  targetCell = neighborCell;
                  flag=0;
                  break;
              }
              else if(PathArray[neighborCell].visited >= PathArray[currentCell].visited){
                  flag=1;
                  targetCell = getNeighbourLocation(currentCell, (currentDir+2)%4);
              }
          }
      }
      if(flag){
        PathArray[currentCell].visited++;
        // setColorcell(delineariseRow(currentCell),delineariseCol(currentCell),PathArray[currentCell].visited);
      }
      // log("target cell: "+String(targetCell));
      getTarget();
    }
    
  
  

}



void getTarget(){
  targetRelativeDirection = getTargetRelativeDirection(targetCell);

  updateDirection(&nextLeftDir,targetRelativeDirection);
  updateDirection(&nextDir, targetRelativeDirection);
  updateDirection(&nextRightDir, targetRelativeDirection);

  distanceFromTarget = 1;
}

void goToTargetCell() {
  if (targetRelativeDirection == north) {
  } else if (targetRelativeDirection == east) {
    turn(70,90); //right
  } else if (targetRelativeDirection == south) {
    turn(90,90); //right
    turn(90,90); //right
  } else if (targetRelativeDirection == west) {
    turn(-50,90); //left
  }
  
  moveForward(distanceFromTarget);
  

  updateDirection(&leftDir, targetRelativeDirection);
  updateDirection(&currentDir, targetRelativeDirection);
  updateDirection(&rightDir, targetRelativeDirection);
  
  currentCell = targetCell;
}


// void updateWalls() {
//   PathArray[currentCell].neighbours = 0;
//   if (wallLeft()) {
//     markWall(currentCell, leftDir);
//     if (isNeighbourValid(currentCell, leftDir)) {
//       markWall(getNeighbourLocation(currentCell, leftDir), (leftDir + 2) % 4);
//     }
//     // log("Wall Left");
//   }
//   if (wallFront()) {
//     markWall(currentCell, currentDir);
//     if (isNeighbourValid(currentCell, currentDir)) {
//       markWall(getNeighbourLocation(currentCell, currentDir), (currentDir + 2) % 4);
//     }
//     // log("Wall Front");
//   }
//   if (wallRight()) {
//     markWall(currentCell, rightDir);
//     if (isNeighbourValid(currentCell, rightDir)) {
//       markWall(getNeighbourLocation(currentCell, rightDir), (rightDir + 2) % 4);
//     }
//     // log("Wall Right");
//   }
//   log("Walls Updated "+String(PathArray[currentCell].neighbours));
// }

//real
void updateWalls() {
  readWall();
  if (sensorValue[0]<wallThreshold) {
    markWall(currentCell, leftDir);
    if (isNeighbourValid(currentCell, leftDir)) {
      markWall(getNeighbourLocation(currentCell, leftDir), (leftDir + 2) % 4);
    }
  }
  if (sensorValue[1] <wallThreshold || sensorValue[2] <wallThreshold) {
    markWall(currentCell, currentDir);
    if (isNeighbourValid(currentCell, currentDir)) {
      markWall(getNeighbourLocation(currentCell, currentDir), (currentDir + 2) % 4);
    }
  }
  if (sensorValue[3]<wallThreshold) {
    markWall(currentCell, rightDir);
    if (isNeighbourValid(currentCell, rightDir)) {
      markWall(getNeighbourLocation(currentCell, rightDir), (rightDir + 2) % 4);
    }
  }
}



bool isDeadEnd(uint8_t location) {
  uint8_t count1 = 0;
  for (uint8_t i = 0; i < 4; i++) {
    if(i==(currentDir+2)%4) continue;
    if (wallExists(location, i)) count1++;
  }
  return count1 == 3;
}

bool isJunction(uint8_t location) {
  uint8_t count = 0;
  for (uint8_t i = 0; i < 4; i++) {
    if (wallExists(location, i)) count++;
  }
  // log("Count: "+String(count));
  return count < 2 ;
}

bool isUnvisited(uint8_t location) {
  return PathArray[location].visited == 0;
}

static bool isNeighbourValid(uint8_t location, uint8_t direction) {
  if (direction == north) return delineariseRow(location) > 0;
  else if (direction == east) return delineariseCol(location) < (cols - 1);
  else if (direction == south) return delineariseRow(location) < (rows - 1);
  else if (direction == west) return delineariseCol(location) > 0;
}

static uint8_t getTargetAbsoluteDirection(uint8_t target) {
  short diff = (short)target - (short)currentCell;
  if (diff == -rows) return north;
  if (diff == 1) return east;
  if (diff == rows) return south;
  if (diff == -1) return west;
}

static uint8_t   getTargetRelativeDirection(uint8_t target) {
  return (getTargetAbsoluteDirection(target) + 4 - currentDir) % 4;
}

static bool isDestination(uint8_t location) {
  return currentCell==targetCellsGoal;
}

static bool isEnclosed(uint8_t location) {
  // 15 is 00001111 in binary, which means that there are walls in 4 all 4 directions of the cell
  return PathArray[location].neighbours == 15;
}

void initialise() {
  currentCell = startCell;
  currentDir = startDir;
  leftDir = (currentDir + 3) % 4;
  rightDir = (currentDir + 1) % 4;
  nextLeftDir = leftDir;
  nextDir = currentDir;
  nextRightDir = rightDir;
}
