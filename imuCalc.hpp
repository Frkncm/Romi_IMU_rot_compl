#ifndef _IMU_CALC_
#define _IMU_CALC_

#include "bsp.h"

#define VEL_GAIN   10000
#define OFFSET_DOWN_VAL 1.0
#define OFFSET_UP_VAL 5.0
#define ANG_COEF 0.175
#define ROT_MEAN 86.6
#define ROT_LIMIT 0

class imuCalculator {

  private:
    uint64_t timer;
    float accOffsetDown{OFFSET_DOWN_VAL};
    float accOffsetUp{OFFSET_UP_VAL};

    uint16_t gyroLastUpdate = 0;

  public:
    // This constant represents a turn of 45 degrees.
    const int32_t turnAngle45 = 0x20000000;

    // This constant represents a turn of 90 degrees.
    const int32_t turnAngle90 = turnAngle45 * 2;

    // This constant represents a turn of approximately 1 degree.
    const int32_t turnAngle1 = (turnAngle45 + 22) / 45;
    int16_t turnRate;
    int32_t turnAngle = 0;
    int32_t turnRotation = 0;

    float acceleration{0};
    float distance{0};
    float velocity{0};
    float rotation{0};
    float Xnew{0}, Ynew {0}; // Y - coordinate
    int16_t gyroOffset;

    imuCalculator() {
      //default ctor.
    }

    imuCalculator(float offsetUp, float offsetDown) :
      accOffsetUp{offsetUp}, accOffsetDown{offsetDown}  {

    }

    auto getAcc()->float {
      return acceleration;
    }

    void getCoordinate (float rot, float acc) {
      //calculate x - y position
      static float last_distance {0};
      calcVelDist(acc);
      Ynew += (distance - last_distance) * sin(rotation * PI / 180);
      Xnew += (distance - last_distance) * cos(rotation * PI / 180);
      last_distance = distance;
    }

    void calcVelDist(float acc_new) {
      static float veloc_new = 0.0f;
      static float veloc_old = 0.0f;
      static float accel_old = 0.0f;
      static int number_of_sample = 1;
      static bool loc_trigger{ false };

      // Calculate delta time
      double dt_ = (double)(micros() - timer) / 1000000.0;
      timer = micros();

      //Eliminate under the offset value (noisy signal)
      if ((acc_new > accOffsetDown)) {
        loc_trigger = true;
        //Trapezoidal integration for getting velocity
        veloc_new +=  (accel_old + (acc_new - accel_old) / 2.0) * dt_ * 100;
        accel_old = acc_new;
        number_of_sample++;
      } else {
        if (loc_trigger) {
          loc_trigger = false;
          veloc_new /= (float)number_of_sample;
          if (veloc_new > velocity)
            velocity = veloc_new;
          number_of_sample = 1;
        }
      }
      //Trapezoidal integration for getting distance
      distance += ( veloc_old + (veloc_new - veloc_old) / 2.0) * dt_;
      veloc_old = velocity;
    }

    boolean turnDegree(float desiredAngle) {
      static float initialAngle{ 0 };
      static bool  enterTrigger{ true };

      if (enterTrigger) {
        initialAngle = rotation;
        enterTrigger = false;
      }

      float locAngle = (rotation - initialAngle);

      if (desiredAngle > 0) {
        //compare the angle difference for positive values
        if (desiredAngle > locAngle) {
          return false;
        } else {
          enterTrigger = true;
          return true;
        }
      } else if (desiredAngle < 0) {
        //compare the angle difference for positive values
        if (desiredAngle < locAngle) {
          return false;
        }
        else {
          enterTrigger = true;
          return true;
        }
      }
      return false;
    }

    void turnSetup(LSM6 const & imuIns, uint16_t cal_rot_rate)
    {
      // Calibrate the gyro.
      int32_t total = 0;
      for (uint16_t i = 0; i < cal_rot_rate; i++)
      {
        // Wait for new data to be available, then read it.
        while (!imuIns.readReg(LSM6::STATUS_REG) & 0x08);
        imuIns.read();

        // Add the Z axis reading to the total.
        total += imu.g.z;
      }

      gyroOffset = total / cal_rot_rate;

      // Display the angle (in degrees from -180 to 180) until the
      gyroLastUpdate = micros();
      turnAngle = 0;
    }

    void turnUpdate(LSM6 const & imuIns)
    {
      // Read the measurements from the gyro.
      imuIns.readGyro();
      turnRate = imuIns.g.z - gyroOffset;

      // Figure out how much time has passed since the last update (dt)
      uint16_t m = micros();
      uint16_t dt = m - gyroLastUpdate;
      gyroLastUpdate = m;

      // Multiply dt by turnRate in order to get an estimation of how
      // much the robot has turned since the last update.
      // (angular change = angular velocity * time)
      int32_t d = (int32_t)turnRate * dt;

      // The units of d are gyro digits times microseconds.  We need
      // to convert those to the units of turnAngle, where 2^29 units
      // represents 45 degrees.  The conversion from gyro digits to
      // degrees per second (dps) is determined by the sensitivity of
      // the gyro: 0.035 degrees per second per digit.
      //
      // (0.035 dps/digit) * (1/1000000 s/us) * (2^29/45 unit/degree)
      // = 7340032/17578125 unit/(digit*us)
      turnAngle += (int64_t)d * 7340032 / 17578125;
      turnRotation = turnAngle / turnAngle1;
    }

};

#endif _IMU_CALC_


