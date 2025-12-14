#include "main.h"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motor_group.hpp"

#include <cstdint>
#include <cstdio>
#include <cstdlib>

using namespace pros;



constexpr bool USE_IMU_CORRECTION = false; 
constexpr float HEADING_kP = 2.5;          
constexpr int MAX_REPLAY_STEPS = 30000;



MotorGroup aright({-7, 5, 10});
MotorGroup aleft({19, -21, -6});

Motor intake(18);
Motor middle(3);
Motor top(4);

Controller userInput(E_CONTROLLER_MASTER);
adi::Pneumatics match('a', false);

IMU i1(15);



typedef struct {
  int16_t fwd;
  int16_t trn;

  int16_t intake;
  int16_t middle;
  int16_t top;

  float heading;
  uint16_t dt;
  uint8_t last;
} ReplayStep;


inline void write_replay(ReplayStep *steps, int count, const char *filename) {
  FILE *f = fopen(filename, "wb");
  if (!f)
    return;

  fwrite(steps, sizeof(ReplayStep), count, f);
  fclose(f);
}

inline ReplayStep *read_replay(const char *filename, int *outCount) {
  FILE *f = fopen(filename, "rb");
  if (!f)
    return nullptr;

  fseek(f, 0, SEEK_END);
  size_t size = ftell(f);
  rewind(f);

  int count = size / sizeof(ReplayStep);
  ReplayStep *data = (ReplayStep *)malloc(size);
  fread(data, sizeof(ReplayStep), count, f);

  fclose(f);
  *outCount = count;
  return data;
}



void apply_step(const ReplayStep *s) {
  float left = s->fwd + s->trn;
  float right = s->fwd - s->trn;

  if (USE_IMU_CORRECTION) {
    float err = i1.get_rotation() - s->heading;
    while (err > 180)
      err -= 360;
    while (err < -180)
      err += 360;

    float correction = err * HEADING_kP;
    if (correction > 30)
      correction = 30;
    if (correction < -30)
      correction = -30;

    left -= correction;
    right += correction;
  }

  left = std::clamp(left, -127.0f, 127.0f);
  right = std::clamp(right, -127.0f, 127.0f);

  aleft.move((int)left);
  aright.move((int)right);

  intake.move(s->intake);
  middle.move(s->middle);
  top.move(s->top);
}



void ballTask() {
  while (true) {
    intake.move(userInput.get_digital(DIGITAL_R2)   ? -127
                : userInput.get_digital(DIGITAL_R1) ? 127
                                                    : 0);

    middle.move(userInput.get_digital(DIGITAL_L2) ||
                        userInput.get_digital(DIGITAL_L1)
                    ? -127.
                : userInput.get_digital(DIGITAL_R1) ? 127
                                                    : 0);

    top.move(userInput.get_digital(DIGITAL_L1) ? 127
             : userInput.get_digital(DIGITAL_R1) ||
                     userInput.get_digital(DIGITAL_L2)
                 ? -127
                 : 0);

    delay(10);
  }
}



void initialize() {
  if (USE_IMU_CORRECTION)
    i1.reset(true);
}



void autonomous() {
  if (USE_IMU_CORRECTION)
    i1.reset(true);
  int count = 0;
  ReplayStep *replay = read_replay("/usd/rec", &count);
  if (!replay)
    return;

  for (int i = 0; i < count; i++) {
    if (replay[i].last)
      break;

    uint32_t start = millis();
    while (millis() - start < replay[i].dt) {
      apply_step(&replay[i]);
      delay(5);
    }
  }

  free(replay);
}



void opcontrol() {
  Task balls(ballTask);

  ReplayStep *replay =
      (ReplayStep *)malloc(sizeof(ReplayStep) * MAX_REPLAY_STEPS);

  bool recording = false;
  int step = 0;
  uint32_t lastTime = millis();

  while (true) {

    if (userInput.get_digital_new_press(DIGITAL_X)) {
      recording = true;
      step = 0;
      lastTime = millis();
    }

    if (recording && userInput.get_digital_new_press(DIGITAL_B)) {
      replay[step].last = 1;
      write_replay(replay, step + 1, "/usd/rec");
      recording = false;
    }

    int fwd = userInput.get_analog(ANALOG_LEFT_Y);
    int trn = userInput.get_analog(ANALOG_RIGHT_X) * 0.6;

    aleft.move(fwd + trn);
    aright.move(fwd - trn);

    if (recording && step < MAX_REPLAY_STEPS) {
      uint32_t now = millis();
      replay[step] = {(int16_t)fwd,
                      (int16_t)trn,
                      (int16_t)intake.get_actual_velocity(),
                      (int16_t)middle.get_actual_velocity(),
                      (int16_t)top.get_actual_velocity(),
                      static_cast<float>(i1.get_rotation()),
                      (uint16_t)(now - lastTime),
                      0};
      lastTime = now;
      step++;
    }

    if (userInput.get_digital_new_press(DIGITAL_A))
      match.toggle();

    delay(10);
  }
}
