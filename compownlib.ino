#include "complementary.hpp"
#include "nonBlockingMillis.h"
#include "motor.hpp"
#include "pidLib.h"
#include "imuCalc.hpp"
#include "Romi.h"
#include "bsp.h"
#include "kinematics.h"

#define _SPEED_ 45

#define KP_VAL 45.0  // increase response time, but it'll increase oscillation also
#define KI_VAL 0.30   // minimise the total error 
#define KD_VAL 1.0   // if there is a huge changing

#define SQUARE_DISTANCE 500

#define SEND_DATA 1

LSM6 imu;
ComplementFilter<LSM6> cmp(imu);
imuCalculator imclc;
Kinematics_c knm;
//Bluetooth instance
HardwareSerial& bthc05(Serial1);

//These variables for handling the finite state machine
uint8_t current_state;
uint8_t next_state = IDLE_STATE;
uint32_t blocking_time {0};

//right and left motor instances
myMotor leftMotor(L_DIR_PIN, L_PWM_PIN);
myMotor rightMotor(R_DIR_PIN, R_PWM_PIN);

PID pidMotor(KP_VAL, KI_VAL, KD_VAL);

PID H_PID(KP_VAL, KI_VAL, KD_VAL);
PID L_PID(KP_VAL, KI_VAL, KD_VAL);
PID R_PID(KP_VAL, KI_VAL, KD_VAL);

int8_t turnRight = 1;
int8_t turnLeft  = 1;

uint8_t countTask{ 0 };

float bearing {0};

float driveStraight(float theta_demand) {
  return H_PID.updateValue(0 , (atan((theta_demand - imclc.rotation) * PI / 180.0)));
}

//square implementation
bool taskHandler() {
  if (countTask == 0) {
    if (imclc.Xnew <= SQUARE_DISTANCE) {
      bearing = driveStraight(0);
      return false;
    }
    else {
      countTask++;
      return true;
    }
  } else if (countTask == 1) {
    if (imclc.Ynew >= -SQUARE_DISTANCE) {
      bearing = driveStraight(-90);
      return false;
    }
    else {
      countTask++;
      return true;
    }
  } else if (countTask == 2) {
    if (imclc.Xnew >= 0) {
      bearing = driveStraight(-180);
      return false;
    }
    else {
      countTask++;
      return true;
    }
  } else if (countTask == 3) {
    if (imclc.Ynew <= 0) {
      bearing = driveStraight(-270);
      return false;
    }
    else {
      countTask++;
      return true;
    }
  }
}

//create the function which is called periodically
void imuTask(void);
taskInsert imuTaskIns(imuTask, 10);

void imuTask(void) {
  static int staticTime = 0;

  //update the filter values
  cmp.updateFilter();

  //get the coordinate using IMU sensor parameters
  imclc.getCoordinate(cmp.getFilteredZ(), cmp.getFilteredY());

#if SEND_DATA

  //Send values trough bluetooth
  bthc05.print(knm.x); bthc05.print("\t");
  bthc05.print(knm.y); bthc05.print("\t");
  bthc05.print(knm.theta * 180 / PI); bthc05.print("\t");
  //Send values trough bluetooth
  bthc05.print(imclc.Ynew); bthc05.print("\t");
  bthc05.print(imclc.Xnew); bthc05.print("\t");
  bthc05.print(imclc.rotation); bthc05.println("\t");

#endif

  leftMotor.motorControl(turnLeft * -_SPEED_ + bearing);
  rightMotor.motorControl(turnRight * -_SPEED_ - bearing);

}



void exampleTask(void);
taskInsert exampleTaskIns(exampleTask, 10);

void exampleTask(void) {

  static int staticTime = 0;

  //update the filter values
  cmp.updateFilter();

  //get the coordinate using IMU sensor parameters
  
  //imclc.updateRotation(cmp.getFilteredZ());
  imclc.turnUpdate(imu);
  imclc.getCoordinate(imclc.turnRotation, cmp.getFilteredY());
  //Serial.println(imclc.turnRotation);

  bool readPin = digitalRead(30);
  staticTime++;
  if (staticTime > 200 && staticTime < 400) {
    leftMotor.motorControl(-_SPEED_ + H_PID.updateValue(0 , (atan((0 - imclc.turnRotation) * PI / 180.0))));
    rightMotor.motorControl(-_SPEED_ - H_PID.updateValue(0 , (atan((0 - imclc.turnRotation) * PI / 180.0))));
  } else {
    leftMotor.motorControl(0);
    rightMotor.motorControl(0);
    imclc.velocity = 0;
  }
  Serial.print(" V: "); Serial.print( cmp.getFilteredY());
  Serial.print(" turnRot: "); Serial.println(imclc.turnRotation);
  if (readPin == LOW) {
    staticTime = 0;
  }

}

void setup() {
  // put your setup code here, to run once:

  bsp_ctor();
  pidMotor.reset();
  //Let the user place the robot to calibrate values
  delay(2000);

  cmp.cal_compl_params(2000);
  imclc.turnSetup(imu, 1024);

  bthc05.begin(9600);
  GO_HANDLE(IDLE_STATE); // start with handling IDLE state

}

void loop() {

  //Update the time for non-blocking tasks
  taskInsert::executeTasks();
  knm.update(count_e0, count_e1);
  switch (current_state) {

    case IDLE_STATE: {
        /*Start the state machine with the idle state (do nothing here)
          GO_HANDLE macro and the others are defined in the Romi.h file.
          They are used to navigate the flow of control. */

        exampleTaskIns.callMyTask();

        GO_HANDLE(IDLE_STATE);
        break;
      }

    case PATH_TRACING: {
        if (!taskHandler()) {
          turnLeft = 1;
          turnRight = 1;
          imuTaskIns.callMyTask();
          GO_HANDLE(PATH_TRACING);
        } else if (countTask >= 4) {
          BREAK_AND_GO(STOP_SYSTEM)
        } else {
          //Reset the accel. value and motor speed
          imclc.acceleration = 0.0;
          leftMotor.motorControl(0);
          rightMotor.motorControl(0);
          GO_HANDLE(WAIT_BEFORE_TURNING);
        }

        break;
      }

    case WAIT_BEFORE_TURNING: {
        WAIT_NONBLOCKING_SAME_MS(500, WAIT_BEFORE_TURNING);
        GO_HANDLE(TURN_ROMI_STATE);
        break;
      }

    case TURN_ROMI_STATE: {
        if (!imclc.turnDegree(-90)) {
          //turn motor right
          turnLeft = 1;
          turnRight = -1;
          imuTaskIns.callMyTask();
          GO_HANDLE(TURN_ROMI_STATE);
        } else {
          imclc.acceleration = 0.0;
          leftMotor.motorControl(0);
          rightMotor.motorControl(0);
          GO_HANDLE(MOTOR_STOP);
        }
        break;
      }

    case MOTOR_STOP: {
        cmp.updateFilter();//stabilize its z axis value
        WAIT_NONBLOCKING_SAME_MS(500, MOTOR_STOP);
        leftMotor.motorControl(0);
        rightMotor.motorControl(0);
        GO_HANDLE(PATH_TRACING);
        break;
      }

    case STOP_SYSTEM: {
        leftMotor.motorControl(0);
        rightMotor.motorControl(0);
        break;
      }
    default : {
        GO_HANDLE(IDLE_STATE);
        break;
      }

  }
}



