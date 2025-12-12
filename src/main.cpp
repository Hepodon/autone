#include "main.h"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motor_group.hpp"
#include "pros/motors.h"

using namespace pros;

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>

typedef struct {
  int16_t fwd;    // fwd stick value (left Y)
  int16_t trn;    // turn value after scaling (right X * 0.6)
  int16_t intake; // intake motor command (-127..127)
  int16_t middle; // middle motor command (-127..127)
  int16_t top;    // top motor command (-127..127)
  int16_t prong;  // prong motor command (-127..127) - if you have analog, else
                  // button-derived
  float heading;  // IMU heading at this frame (degrees)
  uint32_t dt;    // milliseconds elapsed since previous frame (usually 10)
  uint8_t last;   // end marker
} ReplayStep;

static const int REPLAY_DT_MS = 10; // fixed loop dt for record/playback (ms)
static const int INITIAL_REPLAY_CAP = 4000; // initial capacity (~40s at 10ms)
static const int MAX_REPLAY_STEPS = 20000;  // safety cap (~200s at 10ms)

inline bool write_replay_file(const char *filename, ReplayStep *steps,
                              uint32_t count) {
  FILE *f = fopen(filename, "wb");
  if (!f)
    return false;
  // write count
  fwrite(&count, sizeof(uint32_t), 1, f);
  // write steps
  fwrite(steps, sizeof(ReplayStep), count, f);
  fclose(f);
  return true;
}

inline ReplayStep *read_replay_file(const char *filename, uint32_t *outCount) {
  FILE *f = fopen(filename, "rb");
  if (!f)
    return NULL;
  uint32_t count = 0;
  if (fread(&count, sizeof(uint32_t), 1, f) != 1) {
    fclose(f);
    return NULL;
  }
  if (count == 0 || count > MAX_REPLAY_STEPS) {
    fclose(f);
    return NULL;
  }
  ReplayStep *arr = (ReplayStep *)malloc(sizeof(ReplayStep) * count);
  if (!arr) {
    fclose(f);
    return NULL;
  }
  size_t read = fread(arr, sizeof(ReplayStep), count, f);
  fclose(f);
  if (read != count) {
    free(arr);
    return NULL;
  }
  *outCount = count;
  return arr;
}

// clamp helper
static inline int16_t clamp_i16(int val) {
  if (val > 127)
    return 127;
  if (val < -127)
    return -127;
  return (int16_t)val;
}

MotorGroup aright({-7, 5, 10});
MotorGroup aleft({19, -21, -6});

Motor intake(18);
Motor middle(3);
Motor top(4);

Controller userInput(E_CONTROLLER_MASTER);

adi::Pneumatics match('a', false);

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
      top.move(-127);
    } else if ((userInput.get_digital(DIGITAL_L1))) {
      top.move(127);
    } else if (userInput.get_digital(DIGITAL_R1)) {
      top.move(-127);
    } else {
      top.brake();
    }
    delay(10);
  }
}

#define PRONG_PORT 11

uint8_t wheels[9] = {7, 5, 10, 19, 21, 6, 18, 3, 4};

static void capture_controls_into_step(ReplayStep *s, uint32_t dt_ms) {
  // read controller analogs/buttons and convert to same commands used in
  // opcontrol
  int fwdpwr = userInput.get_analog(ANALOG_LEFT_Y);  // -127..127
  int trnraw = userInput.get_analog(ANALOG_RIGHT_X); // -127..127
  // opcontrol multiplies by 0.6 for turn:
  int trnpwr = static_cast<int>(trnraw * 0.6f);

  // intake / middle / top logic (mirror your ballmangement logic)
  int intake_cmd = 0;
  if (userInput.get_digital(DIGITAL_R2))
    intake_cmd = -127;
  else if (userInput.get_digital(DIGITAL_R1))
    intake_cmd = 127;
  else
    intake_cmd = 0;

  int middle_cmd = 0;
  if (userInput.get_digital(DIGITAL_L1) || userInput.get_digital(DIGITAL_L2))
    middle_cmd = -127;
  else if (userInput.get_digital(DIGITAL_R1))
    middle_cmd = 127;
  else
    middle_cmd = 0;

  int top_cmd = 0;
  if (userInput.get_digital(DIGITAL_L2))
    top_cmd = -127;
  else if (userInput.get_digital(DIGITAL_L1))
    top_cmd = 127;
  else if (userInput.get_digital(DIGITAL_R1))
    top_cmd = -127;
  else
    top_cmd = 0;

  // prong: if you have buttons mapping to it, record them. Here we read motor
  // voltage as fallback:
  int prong_cmd = c::motor_get_voltage(PRONG_PORT);
  // motor_get_voltage returns mV-ish; normalize to -127..127 approximate: pros
  // stores volts as ??? To keep things robust, we clamp to -127..127 after
  // scaling. If you have a stable prong mapping, replace this with the direct
  // command value logic. We'll map typical motor voltage range (-12000..12000)
  // to -127..127:
  prong_cmd = (prong_cmd * 127) / 12000;

  // fill step
  s->fwd = clamp_i16(fwdpwr);
  s->trn = clamp_i16(trnpwr);
  s->intake = clamp_i16(intake_cmd);
  s->middle = clamp_i16(middle_cmd);
  s->top = clamp_i16(top_cmd);
  s->prong = clamp_i16(prong_cmd);
  s->heading = i1.get_rotation(); // record heading in degrees
  s->dt = dt_ms < 3 ? 3 : dt_ms;  // obey a minimum dt
  s->last = 0;
}

static void apply_step_playback(const ReplayStep *s, float kH_playback) {
  // recompute motor outputs from recorded fwd/trn (exact same mixing as
  // opcontrol) left = fwd + trn; right = fwd - trn
  float left = static_cast<float>(s->fwd) + static_cast<float>(s->trn);
  float right = static_cast<float>(s->fwd) - static_cast<float>(s->trn);

  // IMU correction: compare current IMU heading to recorded heading and nudge
  // steering
  float currentHeading = i1.get_rotation();
  float headingError = currentHeading - s->heading;
  // normalize heading error to [-180,180]
  while (headingError > 180.0f)
    headingError -= 360.0f;
  while (headingError < -180.0f)
    headingError += 360.0f;

  float correction =
      headingError * kH_playback; // units: percent power per degree
  // Apply same convention as in drive code: L = power - correction; R = power +
  // correction
  left = left - correction;
  right = right + correction;

  // clamp
  if (left > 127.0f)
    left = 127.0f;
  if (left < -127.0f)
    left = -127.0f;
  if (right > 127.0f)
    right = 127.0f;
  if (right < -127.0f)
    right = -127.0f;

  // set drivetrain motors (use move with percent)
  aleft.move(static_cast<int>(left));
  aright.move(static_cast<int>(right));

  // set other motors (intake, middle, top, prong)
  intake.move(s->intake);
  middle.move(s->middle);
  top.move(s->top);
  // prong might be an individually addressed motor; use c::motor_move or create
  // a Motor object if needed We'll try the pros Motor API if possible: If
  // PRONG_PORT is not declared as pros::Motor earlier, fallback to c API to be
  // safe Try pros::Motor
  static bool prongMotorCreated = false;
  static Motor *prongMotor = nullptr;
  if (!prongMotorCreated) {
    // attempt to dynamically create; if fails, will still use c API below
    try {
      prongMotor = new Motor(PRONG_PORT);
    } catch (...) {
      prongMotor = nullptr;
    }
    prongMotorCreated = true;
  }
  if (prongMotor)
    prongMotor->move(s->prong);
  else
    c::motor_move(PRONG_PORT, s->prong);
}

void initialize() {}

void disabled() {}

void competition_initialize() {}

void autonomous() {
  // read a replay and play it back with IMU-assist
  uint32_t count = 0;
  ReplayStep *replay = read_replay_file("/usd/rec", &count);
  if (!replay)
    return;

  // playback parameters:
  const float kH_playback = 0.6f; // tune this 0.2..1.2 to taste
  for (uint32_t i = 0; i < count; i++) {
    // if end marker, break
    if (replay[i].last == 1)
      break;

    uint32_t start = pros::millis();
    // apply this step repeatedly for its dt (keeps same timing as recording)
    uint32_t targetDt = replay[i].dt;
    while ((uint32_t)(pros::millis() - start) < targetDt) {
      apply_step_playback(&replay[i], kH_playback);
      delay(5); // small sleep - ensures we yield and still hit dt
    }
  }

  // safety: stop everything
  aleft.brake();
  aright.brake();
  intake.brake();
  middle.brake();
  top.brake();
  // prong stop
  try {
    Motor pr(PRONG_PORT);
    pr.brake();
  } catch (...) {
    c::motor_brake(PRONG_PORT);
  }

  free(replay);
}

static bool g_in_opcontrol = false;

void opcontrol() {
  // start your ball management only here (not during replay)
  g_in_opcontrol = true;
  Task balls(ballmangement);

  bool recording = false;
  bool recorded = false;
  uint32_t replay_step = 0;

  // dynamic buffer for recorded steps
  uint32_t capacity = INITIAL_REPLAY_CAP;
  ReplayStep *replay = (ReplayStep *)malloc(sizeof(ReplayStep) * capacity);
  if (!replay) {
    // allocation failure - fallback to not recording
    capacity = 0;
  }

  uint32_t lastTime = pros::millis();

  while (true) {
    // handle starting recording
    if (userInput.get_digital_new_press(DIGITAL_X) && capacity > 0) {
      recording = true;
      replay_step = 0;
      recorded = false;
      lastTime = pros::millis();
    }

    // stop recording and save
    if (recording && userInput.get_digital_new_press(DIGITAL_B)) {
      if (replay_step > 0) {
        // mark last
        replay[replay_step - 1].last = 1;
        // write count
        write_replay_file("/usd/rec", replay, replay_step);
        recorded = true;
      }
      recording = false;
    }

    // handle match toggle
    if (userInput.get_digital_new_press(DIGITAL_A)) {
      match.toggle();
    }

    // drive mixing (same as before)
    int fwdpwr = userInput.get_analog(ANALOG_LEFT_Y);
    int trnpwr = static_cast<int>(userInput.get_analog(ANALOG_RIGHT_X) * 0.6f);
    aleft.move(fwdpwr + trnpwr);
    aright.move(fwdpwr - trnpwr);

    // record a frame if active
    if (recording && capacity > 0) {
      uint32_t now = pros::millis();
      uint32_t dt = (now - lastTime);
      if (dt < 1)
        dt = 1;
      lastTime = now;

      // expand buffer if needed
      if (replay_step >= capacity) {
        // realloc doubling
        if (capacity >= MAX_REPLAY_STEPS) {
          // reached max; stop recording and mark last
          recording = false;
          if (capacity > 0)
            replay[capacity - 1].last = 1;
        } else {
          uint32_t newcap = capacity * 2;
          if (newcap > MAX_REPLAY_STEPS)
            newcap = MAX_REPLAY_STEPS;
          ReplayStep *tmp =
              (ReplayStep *)realloc(replay, sizeof(ReplayStep) * newcap);
          if (tmp) {
            replay = tmp;
            capacity = newcap;
          } else {
            // realloc failed: stop recording gracefully
            recording = false;
          }
        }
      }

      if (recording) {
        // capture into replay[replay_step]
        capture_controls_into_step(&replay[replay_step], dt);
        replay_step++;
      }
    }

    delay(REPLAY_DT_MS);
  }

  // free if loop ever exits (it shouldn't)
  if (replay)
    free(replay);
  g_in_opcontrol = false;
}