#include "main.h"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motor_group.hpp"

#include <cstdint>
#include <cstdio>
#include <cstdlib>

using namespace pros;

constexpr int MAX_REPLAY_STEPS = 30000;

MotorGroup aright({-7, 5, 10});
MotorGroup aleft({19, -21, -6});

Motor intake(18);
Motor middle(3);
Motor top(4);

Controller userInput(E_CONTROLLER_MASTER);
adi::Pneumatics match('a', false);

typedef struct {
  int16_t fwd;
  int16_t trn;

  int16_t intake;
  int16_t middle;
  int16_t top;

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
  int left = s->fwd + s->trn;
  int right = s->fwd - s->trn;

  aleft.move(left);
  aright.move(right);

  intake.move(s->intake);
  middle.move(s->middle);
  top.move(s->top);
}

void ballTask() {
  while (true) {
    intake.move(userInput.get_digital(DIGITAL_R2)   ? -127
                : userInput.get_digital(DIGITAL_R1) ? 127
                                                    : 0);

    middle.move(userInput.get_digital(DIGITAL_L1) ||
                        userInput.get_digital(DIGITAL_L2)
                    ? -127
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

void autonomous() {
  int count = 0;
  ReplayStep *replay = read_replay("/usd/rec", &count);
  if (!replay)
    return;

  for (int i = 0; i < count; i++) {
    if (replay[i].last)
      break;
    apply_step(&replay[i]);
    delay(replay[i].dt);
  }

  aleft.brake();
  aright.brake();
  intake.brake();
  middle.brake();
  top.brake();

  free(replay);
}

void opcontrol() {
  Task balls(ballTask);

  ReplayStep *replay =
      (ReplayStep *)malloc(sizeof(ReplayStep) * MAX_REPLAY_STEPS);

  bool recording = false;
  int step = 0;

  constexpr uint32_t FRAME_MS = 10;
  uint32_t nextFrame = millis();

  while (true) {

    uint32_t now = millis();
    if ((int32_t)(nextFrame - now) > 0)
      delay(nextFrame - now);
    nextFrame += FRAME_MS;

    int fwd = userInput.get_analog(ANALOG_LEFT_Y);
    int trn = userInput.get_analog(ANALOG_RIGHT_X) * 0.6;

    aleft.move(fwd + trn);
    aright.move(fwd - trn);

    if (recording && step < MAX_REPLAY_STEPS) {
      replay[step++] = {(int16_t)fwd,
                        (int16_t)trn,

                        (int16_t)(userInput.get_digital(DIGITAL_R2)   ? -127
                                  : userInput.get_digital(DIGITAL_R1) ? 127
                                                                      : 0),

                        (int16_t)((userInput.get_digital(DIGITAL_L1) ||
                                   userInput.get_digital(DIGITAL_L2))
                                      ? -127
                                  : userInput.get_digital(DIGITAL_R1) ? 127
                                                                      : 0),

                        (int16_t)(userInput.get_digital(DIGITAL_L1) ? 127
                                  : userInput.get_digital(DIGITAL_R1) ||
                                          userInput.get_digital(DIGITAL_L2)
                                      ? -127
                                      : 0),

                        FRAME_MS,
                        0};
    }

    if (userInput.get_digital_new_press(DIGITAL_X)) {
      recording = true;
      step = 0;
    }

    if (recording && userInput.get_digital_new_press(DIGITAL_B)) {
      replay[step].last = 1;
      write_replay(replay, step + 1, "/usd/rec");
      recording = false;
    }

    if (userInput.get_digital_new_press(DIGITAL_A))
      match.toggle();
  }
}