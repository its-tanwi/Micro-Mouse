#include <SparkFun_TB6612.h>
#include<Encoder.h>
#include <SoftwareSerial.h>

SoftwareSerial hc06(0,1);

#define PWMA 5
#define AIN1 6
#define AIN2 7
#define BIN1 8
#define BIN2 9
#define PWMB 10


Motor Motor_R = Motor(AIN1, AIN2, PWMA);
Motor Motor_L = Motor(BIN1, BIN2, PWMB);

#define ENCA1 2
#define ENCA2 4
#define ENCB1 3
#define ENCB2 11
Encoder Enc_R(ENCA1 ,ENCA2);
Encoder Enc_L(ENCB1, ENCB2);

int sense[4]={A0,A1,A2,A3};

int newPosition_R = 0;
int newPosition_L = 0;

int encStart_L = 0;
int encStart_R = 0;

float Kp_R = 0.05;
float Kp_L = 0.12;

float Kd = 0.05;
float Ki = 0.0;

float kp_diff = 0.2;
float kd_diff = 0.2;

float kp_front = 4.05;
float kp_wall = 1;
float kp_turn = 0.001;
float kd_turn = 0.0;

int steps = 700;

int max_control_R = 100;
int max_control_L = 100;

int min_control_R = 30;
int min_control_L = 30;

int sensorValue[4] = {0};
int sensorValueLow[4] = {0};

void encUpdate() {
  newPosition_R = -Enc_R.read();
  newPosition_L =  Enc_L.read();
  
}

void readWall(){
  delay(10);
  digitalWrite(13, 1);
  for (int i=0 ;i<4;i++){
    sensorValue[i]=analogRead(sense[i]) - sensorValueLow[i];
  }
}

float PID_Control(int setpoint, unsigned long *last_time, int sensed_output, float *last_error, double *int_error, int *min_control, int *max_control, float Kp){
  
  float control_signal;

  unsigned long current_time = millis();
  if ((*last_time)==0){
    *last_time=current_time;
  }

    int delta_time = current_time - (*last_time); 


    double error = setpoint - sensed_output;
    // Serial.println(error);
    *int_error += error;
    double delta_error = error - (*last_error);

    if (*last_time==current_time) (control_signal = Kp*error);

    else (control_signal = Kp*error +  (Kd/delta_time*1e-3)*delta_error + (Ki*delta_time*1e-3)*(*int_error));
    // else (control_signal = Kp*error);
    // Serial.print("  error :");
    // Serial.println(error);

    if (control_signal >= *max_control) control_signal = *max_control;
    else if (abs(control_signal) <= *min_control) control_signal = (*min_control)*(abs(control_signal)/control_signal);

    *last_error = error;
    *last_time = current_time;

    return control_signal;
}


int moveForward(int blocks) {
  encStart_R = -Enc_R.read();
  encStart_L = Enc_L.read();

  unsigned long last_time_l = 0;
  unsigned long last_time_r = 0;
  float last_error_l = 0.0;
  float last_error_r = 0.0;
  double total_error_l = 0.0;
  double total_error_r = 0.0;

  float last_diff_error = 0.0;
  float wallError = 0;

  while (abs(newPosition_R - encStart_R) < blocks * steps || abs(newPosition_L - encStart_L) < blocks * steps) {
    encUpdate();
    readWall();

    if ((sensorValue[1] < 80) || (sensorValue[2] < 80) ) {
      forward(Motor_R, 0, Motor_L, 0);
      return 1;
    }

    float error_diff = (newPosition_R - encStart_R) - (newPosition_L - encStart_L);
    float PWM_EncR = PID_Control(blocks * steps, &last_time_r, newPosition_R - encStart_R, &last_error_r, &total_error_r, &min_control_R, &max_control_R, Kp_R);
    float PWM_EncL = PID_Control(blocks * steps, &last_time_l, newPosition_L - encStart_L, &last_error_l, &total_error_l, &min_control_L, &max_control_L, Kp_L);

    if (sensorValue[0] <= 160 && sensorValue[3] <= 160) {
      wallError = kp_wall * (sensorValue[0] - sensorValue[3]);
    } else if (sensorValue[0] < 160 && sensorValue[3] > 160) {
      wallError = kp_wall * (sensorValue[0] - 60);
    } else if (sensorValue[0] > 160 && sensorValue[3] < 60) {
      wallError = kp_wall * (60 - sensorValue[3]);
    } else {
      wallError = kp_wall * (sensorValue[0] - sensorValue[3]);
    }

    float frontWallError = kp_front * (200 - (sensorValue[1] + sensorValue[2]) / 2);

    if (frontWallError >= max_control_R) frontWallError = max_control_R;

    int PWM_R = PWM_EncR + wallError - frontWallError;
    int PWM_L = PWM_EncL - wallError - frontWallError;

    //   int PWM_R = PWM_EncR - (error_diff * kp_diff + kd_diff * (error_diff - last_diff_error)) - frontWallError;
    // int PWM_L = PWM_EncL + (error_diff * kp_diff + kd_diff * (error_diff - last_diff_error))  - frontWallError;


    //   int PWM_R = PWM_EncR - frontWallError;
    // int PWM_L = PWM_EncL  - frontWallError;

    hc06.print(PWM_R);
    hc06.print(" ");
    hc06.println(PWM_L);

    if (PWM_R > max_control_R) PWM_R = max_control_R;
    if (PWM_L > max_control_R) PWM_L = max_control_R;

    if (abs(PWM_R) < min_control_R) PWM_R = min_control_R * (abs(PWM_R) / PWM_R);
    if (abs(PWM_L) < min_control_L) PWM_L = min_control_L * (abs(PWM_L) / PWM_L);

    forward(Motor_R, PWM_R, Motor_L, PWM_L);
    last_diff_error = error_diff;
    delay(100);
  }

  forward(Motor_R, 0, Motor_L, 0);
  delay(300);
  return 0;
}


void turn(int angle, int speed) {
  delay(1000);
  float currSpeed = 90;
  long endpos;
  int steps_dt = abs(angle) * 7.2;

  encStart_R = -Enc_R.read();
  encStart_L = Enc_L.read();
  if (angle > 0) endpos = encStart_L + steps_dt;
  else endpos = encStart_L - steps_dt;

  float last_diff = 0.0;

  while (newPosition_L != endpos) {
    encUpdate();
    float error = abs(newPosition_L - encStart_L) - abs(newPosition_R - encStart_R);

    if (abs(endpos - newPosition_L) < 404) {
      if (currSpeed > 50) currSpeed -= 0.5;
    } else {
      currSpeed += 0.05;
      if (currSpeed > speed) currSpeed = speed;
    }

    int PWM_R = currSpeed + (error * kp_turn + kd_turn * (error - last_diff));
    int PWM_L = currSpeed - (error * kp_turn + kd_turn * (error - last_diff));
    if (PWM_R > 150) PWM_R = 150;
    if (PWM_L > 150) PWM_L = 150;

    last_diff = error;

    if (angle > 0) {
      forward(Motor_R, -PWM_R, Motor_L, PWM_L);
    } else {
      forward(Motor_R, PWM_R, Motor_L, -PWM_L);
    }
  }

  forward(Motor_R, 0, Motor_L, 0);
}


void stop_(){
  forward(Motor_R, 0, Motor_L, 0);
}


// void setup() {
//   hc06.begin(9600);
//   digitalWrite(13, 0);
//   for (int i=0 ;i<4;i++){
//     sensorValueLow[i]=analogRead(sense[i]);
//   }

//   moveForward(1);
//   moveForward(1);
//   turn(-60,90);
//   moveForward(1);
//   turn(-60,90);
//   moveForward(1);
//   moveForward(1);
// }

// void loop() {
// }
