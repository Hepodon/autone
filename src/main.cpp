#include "main.h"
#include "pros/misc.h"
#include "pros/motor_group.hpp"
#include "pros/motors.h"

using namespace pros;

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>

typedef struct {
  int32_t wheels[9];
  double prong;
  uint8_t last;
} ReplayStep;

inline void write_replay(ReplayStep *steps, int count, const char *filename) {
  FILE *f = fopen(filename, "wb");
  if (!f)
    return;

  for (int i = 0; i < count; i++) {
    fwrite(&steps[i], sizeof(ReplayStep), 1, f);
    if (steps[i].last == 1)
      break;
  }

  fclose(f);
}

inline ReplayStep *read_replay(const char *filename, int *outCount) {
  FILE *f = fopen(filename, "rb");
  if (!f)
    return NULL;

  fseek(f, 0, SEEK_END);
  size_t size = ftell(f);
  fseek(f, 0, SEEK_SET);

  int count = size / sizeof(ReplayStep);
  *outCount = count;

  ReplayStep *replay = (ReplayStep *)malloc(size);
  fread(replay, sizeof(ReplayStep), count, f);

  fclose(f);
  return replay;
}

MotorGroup aright({-7, 5, 10});
MotorGroup aleft({19, -21, -6});

Motor intake(18);
Motor middle(3);
Motor top(4);

Controller userInput(E_CONTROLLER_MASTER);

IMU i1(15);
// IMU i2(9);

Rotation leftVert(-17);
Rotation rightVert(9);

float applySlew(int current, int target, float rate) {
  int diff = target - current;
  if (abs(diff) > rate)
    return current + rate * (diff > 0 ? 1 : -1);
  return target;
}

float midHorzDiamIn = 1.25;
float midHorzRadiusIn = 10;
float midHorzCircIn = midHorzDiamIn * M_PI;

float vertDiamIn = 1.25;
float vertWheelBaseIn = 5.5;
float vertCircIn = vertDiamIn * M_PI;

// float getOrien() {}

float getOrienNOR() { return (i1.get_rotation() /*+ i2.get_rotation()) / 2*/); }

float getDistTravIn(bool reset = true) {
  if (reset) {
    leftVert.reset();
    rightVert.reset();
  }

  return (((static_cast<float>(leftVert.get_position()) / 360.0f) *
           vertCircIn) +
          ((static_cast<float>(rightVert.get_position()) / 360.0f) *
           vertCircIn)) /
         2;
}

void turnTo(float theta) {

  bool turning = true;
  int power = 0;
  int slew = 12;
  int orientation = getOrienNOR();
  float kP = 0.5;

  while (turning) {

    if (orientation > (theta - 0.5) && orientation < (theta + 0.5)) {
      aleft.brake();
      aright.brake();
      turning = false;
      break;
    } else if (orientation < theta) {

      power = 0;
      while (orientation < theta) {
        power = applySlew(power, ((theta - orientation) * kP + 20), slew);

        aleft.move(power);
        aright.move(-power);

        orientation = getOrienNOR();
        delay(15);
      }
    } else if (orientation > theta) {

      power = 0;
      while (orientation > theta) {
        power = applySlew(power, ((theta - orientation) * kP - 20), slew);

        aleft.move(power);
        aright.move(-power);

        orientation = getOrienNOR();
        delay(15);
      }
    }
    orientation = getOrienNOR();
    delay(15);
  }
}

void driveFor(int inches) {
  int power = 0;
  int slew = 12;

  float kP = 0.7; // distance P
  float kH = 2.8; // heading P (kkP * kP)

  int startOrien = getOrienNOR();
  float travelled = getDistTravIn();

  while (true) {

    travelled = getDistTravIn(false);
    float error = inches - travelled;

    // If within acceptable range → stop
    if (fabs(error) <= 0.5) {
      aleft.brake();
      aright.brake();
      break;
    }

    // Base power from distance P
    int targetPower = error * kP;

    // Add minimum power to overcome friction
    if (targetPower > 0)
      targetPower += 20;
    else
      targetPower -= 20;

    // Slew limit it
    power = applySlew(power, targetPower, slew);

    // Heading correction
    float headingError = getOrienNOR() - startOrien;
    int correction = headingError * kH;

    int Lpower = power - correction;
    int Rpower = power + correction;

    aleft.move(Lpower);
    aright.move(Rpower);

    delay(15);
  }
}

void init() {
  i1.reset(false);
  // i2.reset(true);
  leftVert.reset();
  rightVert.reset();
  delay(300);
}

bool comp = false;

void ballmangement() {
  while (true) {
    if (userInput.get_digital(DIGITAL_R2)) {
      intake.move(-127);
    } else if (userInput.get_digital(DIGITAL_R1)) {
      intake.move(127);
    } else {
      intake.brake();
    }
    if (userInput.get_digital(DIGITAL_L1) ||
        (userInput.get_digital(DIGITAL_L2))) {
      middle.move(-127);
    } else if (userInput.get_digital(DIGITAL_R1)) {
      middle.move(127);
    } else {
      middle.brake();
    }
    if (userInput.get_digital(DIGITAL_L2)) {
      if (userInput.get_digital(DIGITAL_L1)) {
        top.move(127);
      } else {
        top.move(-127);
      }
    } else if (userInput.get_digital(DIGITAL_R1)) {
      top.move(-127);
    } else {
      top.brake();
    }
  }
}

#define PRONG_PORT 10

uint8_t wheels[9] = {7, 5, 10, 19, 21, 6, 18, 3, 4};

void initialize() {}

void disabled() {}

void competition_initialize() {}

void autonomous() {
  int count = 0;
  ReplayStep *replay = read_replay("/usd/rec", &count);
  if (!replay)
    return;

  for (int i = 0; i < count; i++) {
    if (replay[i].last == 1)
      break;

    for (int ii = 0; ii < 9; ii++) {
      c::motor_move_velocity(wheels[ii], replay[i].wheels[ii]);
    }

    c::motor_move_velocity(PRONG_PORT, replay[i].prong);
    delay(2);
  }

  free(replay);
}

void opcontrol() {
  Task balls(ballmangement);

  bool recording = false;
  int replay_step = 0;
  ReplayStep *replay = (ReplayStep *)malloc(sizeof(ReplayStep) * 30000);

  while (true) {
    if (recording) {
      for (int i = 0; i < 9; i++) {
        replay[replay_step].wheels[i] = c::motor_get_actual_velocity(wheels[i]);
      }
      replay[replay_step].prong = c::motor_get_actual_velocity(PRONG_PORT);
      replay[replay_step].last = 0;

      replay_step++;

      if (replay_step >= 30000) {
        replay_step--;
        replay[replay_step].last = 1;
        write_replay(replay, replay_step + 1, "/usd/rec");
        recording = false;
      }
    }
    int fwdpwr = userInput.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int trnpwr = userInput.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) * 0.6;
    aleft.move(fwdpwr + trnpwr);
    aright.move(fwdpwr - trnpwr);
  }
}
