#include "main.h"
#include "pros/adi.hpp"
#include "pros/gps.hpp"
#include "pros/imu.hpp"
#include "pros/misc.h"
#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <vector>

using namespace pros;

MotorGroup aright({-7, 5, 10});
MotorGroup aleft({19, -21, -6});
Motor intake(18);
Motor middle(3);
Motor top(4);

Controller userInput(E_CONTROLLER_MASTER);
adi::Pneumatics match('a', false);

IMU inertial(11);
GPS gps(12);
Rotation leftEncoder(13);
Rotation rightEncoder(14);

struct SensorConfig {
  bool useGPS = false;
  bool useEncoders = true;
  bool useIMU = true;
  bool gpsForCorrection = false;
};

SensorConfig sensorConfig;

constexpr double WHEEL_DIAMETER = 2.75;
constexpr double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * M_PI;
constexpr double TRACK_WIDTH = 12.0;
constexpr double LOOKAHEAD_DISTANCE = 12.0;
constexpr int MAX_WAYPOINTS = 5000;
constexpr uint32_t RECORD_INTERVAL_MS = 10;

struct Pose {
  double x;
  double y;
  double theta;
  uint32_t timestamp;
};

struct Waypoint {
  double x;
  double y;
  double theta;
  uint32_t timestamp;
  int16_t intake_cmd;
  int16_t middle_cmd;
  int16_t top_cmd;
};

class Odometry {
private:
  Pose pose;
  double lastLeftPos;
  double lastRightPos;

public:
  Odometry() : pose{0, 0, 0, 0}, lastLeftPos(0), lastRightPos(0) {}

  void reset(double x = 0, double y = 0, double theta = 0) {
    pose = {x, y, theta, millis()};
    if (sensorConfig.useEncoders) {
      lastLeftPos = leftEncoder.get_position() / 100.0 * WHEEL_CIRCUMFERENCE;
      lastRightPos = rightEncoder.get_position() / 100.0 * WHEEL_CIRCUMFERENCE;
    }
  }

  void update() {
    double currentTheta = pose.theta;

    if (sensorConfig.useIMU) {
      double imuHeading = inertial.get_heading();
      currentTheta = imuHeading * M_PI / 180.0;
    }

    if (sensorConfig.useEncoders) {
      double leftPos = leftEncoder.get_position() / 100.0 * WHEEL_CIRCUMFERENCE;
      double rightPos =
          rightEncoder.get_position() / 100.0 * WHEEL_CIRCUMFERENCE;

      double deltaLeft = leftPos - lastLeftPos;
      double deltaRight = rightPos - lastRightPos;

      double deltaCenter = (deltaLeft + deltaRight) / 2.0;
      double deltaTheta = (deltaRight - deltaLeft) / TRACK_WIDTH;

      double avgTheta = currentTheta + deltaTheta / 2.0;

      pose.x += deltaCenter * cos(avgTheta);
      pose.y += deltaCenter * sin(avgTheta);

      if (!sensorConfig.useIMU) {
        currentTheta += deltaTheta;
      }

      lastLeftPos = leftPos;
      lastRightPos = rightPos;
    }

    if (sensorConfig.useGPS && sensorConfig.gpsForCorrection) {
      gps_status_s_t status = gps.get_position_and_orientation();
      if (status.x != PROS_ERR_F && status.y != PROS_ERR_F) {
        double gpsX = status.x;
        double gpsY = status.y;
        pose.x = pose.x * 0.9 + gpsX * 0.1;
        pose.y = pose.y * 0.9 + gpsY * 0.1;
      }
    }

    pose.theta = currentTheta;
    pose.timestamp = millis();
  }

  Pose getPose() const { return pose; }

  Pose getGPSPose() const {
    Pose gpsPose = pose;
    if (sensorConfig.useGPS) {
      gps_status_s_t status = gps.get_position_and_orientation();
      if (status.x != PROS_ERR_F)
        gpsPose.x = status.x;
      if (status.y != PROS_ERR_F)
        gpsPose.y = status.y;
      if (status.yaw != PROS_ERR_F)
        gpsPose.theta = status.yaw * M_PI / 180.0;
    }
    return gpsPose;
  }
};

Odometry odom;

std::vector<Waypoint> recordedPath;
bool isRecording = false;

void savePathToSD(const char *filename) {
  FILE *f = fopen(filename, "wb");
  if (!f) {
    userInput.print(0, 0, "Save failed!");
    return;
  }

  uint32_t count = recordedPath.size();
  fwrite(&count, sizeof(uint32_t), 1, f);
  fwrite(recordedPath.data(), sizeof(Waypoint), count, f);

  fclose(f);
  userInput.print(0, 0, "Saved %d pts", count);
}

std::vector<Waypoint> loadPathFromSD(const char *filename) {
  std::vector<Waypoint> path;
  FILE *f = fopen(filename, "rb");
  if (!f)
    return path;

  uint32_t count;
  fread(&count, sizeof(uint32_t), 1, f);

  path.resize(count);
  fread(path.data(), sizeof(Waypoint), count, f);

  fclose(f);
  return path;
}

void startRecording() {
  recordedPath.clear();
  isRecording = true;
  odom.reset(0, 0, 0);
  userInput.print(0, 0, "Recording...");
}

void stopRecording() {
  isRecording = false;
  savePathToSD("/usd/path.dat");
}

void recordWaypoint(int16_t intake_cmd, int16_t middle_cmd, int16_t top_cmd) {
  if (!isRecording || recordedPath.size() >= MAX_WAYPOINTS)
    return;

  Pose current = odom.getPose();

  Waypoint wp;
  wp.x = current.x;
  wp.y = current.y;
  wp.theta = current.theta;
  wp.timestamp = current.timestamp;
  wp.intake_cmd = intake_cmd;
  wp.middle_cmd = middle_cmd;
  wp.top_cmd = top_cmd;

  recordedPath.push_back(wp);
}

struct LookaheadPoint {
  double x;
  double y;
  bool found;
};

LookaheadPoint findLookaheadPoint(const std::vector<Waypoint> &path,
                                  const Pose &robot, int &lastIndex) {
  LookaheadPoint result = {0, 0, false};

  for (int i = lastIndex; i < path.size() - 1; i++) {
    double x1 = path[i].x;
    double y1 = path[i].y;
    double x2 = path[i + 1].x;
    double y2 = path[i + 1].y;

    double dx = x2 - x1;
    double dy = y2 - y1;
    double fx = x1 - robot.x;
    double fy = y1 - robot.y;

    double a = dx * dx + dy * dy;
    double b = 2 * (fx * dx + fy * dy);
    double c = fx * fx + fy * fy - LOOKAHEAD_DISTANCE * LOOKAHEAD_DISTANCE;

    double discriminant = b * b - 4 * a * c;

    if (discriminant >= 0) {
      discriminant = sqrt(discriminant);
      double t1 = (-b - discriminant) / (2 * a);
      double t2 = (-b + discriminant) / (2 * a);

      double t = -1;
      if (t1 >= 0 && t1 <= 1)
        t = t1;
      if (t2 >= 0 && t2 <= 1 && t2 > t)
        t = t2;

      if (t >= 0) {
        result.x = x1 + t * dx;
        result.y = y1 + t * dy;
        result.found = true;
        lastIndex = i;
        return result;
      }
    }
  }

  if (path.size() > 0) {
    result.x = path.back().x;
    result.y = path.back().y;
    result.found = true;
  }

  return result;
}

struct MotorCommands {
  int left;
  int right;
};

MotorCommands calculatePurePursuit(const Pose &robot,
                                   const LookaheadPoint &target) {
  double dx = target.x - robot.x;
  double dy = target.y - robot.y;

  double angleToTarget = atan2(dy, dx);
  double angleError = angleToTarget - robot.theta;

  while (angleError > M_PI)
    angleError -= 2 * M_PI;
  while (angleError < -M_PI)
    angleError += 2 * M_PI;

  double distance = sqrt(dx * dx + dy * dy);
  double curvature = (2 * sin(angleError)) / LOOKAHEAD_DISTANCE;

  int baseSpeed = 80;

  double leftSpeed = baseSpeed * (1 + curvature * TRACK_WIDTH / 2.0);
  double rightSpeed = baseSpeed * (1 - curvature * TRACK_WIDTH / 2.0);

  // Clamp to [-127, 127]
  leftSpeed = std::max(-127.0, std::min(127.0, leftSpeed));
  rightSpeed = std::max(-127.0, std::min(127.0, rightSpeed));

  return {(int)leftSpeed, (int)rightSpeed};
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
  std::vector<Waypoint> path = loadPathFromSD("/usd/path.dat");
  if (path.empty()) {
    userInput.print(0, 0, "No path!");
    return;
  }

  odom.reset(path[0].x, path[0].y, path[0].theta);

  int lastIndex = 0;
  uint32_t startTime = millis();

  while (true) {
    odom.update();
    Pose robot = odom.getPose();

    LookaheadPoint target = findLookaheadPoint(path, robot, lastIndex);

    if (!target.found)
      break;

    MotorCommands cmd = calculatePurePursuit(robot, target);

    aleft.move(cmd.left);
    aright.move(cmd.right);

    uint32_t elapsed = millis() - startTime;
    for (const auto &wp : path) {
      if (abs((int)(elapsed - wp.timestamp)) < 100) {
        intake.move(wp.intake_cmd);
        middle.move(wp.middle_cmd);
        top.move(wp.top_cmd);
        break;
      }
    }

    double distToEnd =
        sqrt(pow(path.back().x - robot.x, 2) + pow(path.back().y - robot.y, 2));
    if (distToEnd < 3.0)
      break;

    delay(10);
  }

  aleft.brake();
  aright.brake();
  intake.brake();
  middle.brake();
  top.brake();
}

void opcontrol() {
  Task balls(ballTask);

  // Initialize sensors
  if (sensorConfig.useIMU)
    inertial.reset();
  if (sensorConfig.useEncoders) {
    leftEncoder.reset_position();
    rightEncoder.reset_position();
  }

  odom.reset();

  uint32_t lastRecordTime = 0;

  while (true) {
    odom.update();

    int16_t fwd = userInput.get_analog(ANALOG_LEFT_Y);
    int16_t trn = userInput.get_analog(ANALOG_RIGHT_X) * 0.6;

    aleft.move(fwd + trn);
    aright.move(fwd - trn);

    int16_t intake_cmd = userInput.get_digital(DIGITAL_R2)   ? -127
                         : userInput.get_digital(DIGITAL_R1) ? 127
                                                             : 0;

    int16_t middle_cmd =
        (userInput.get_digital(DIGITAL_L1) || userInput.get_digital(DIGITAL_L2))
            ? -127
        : userInput.get_digital(DIGITAL_R1) ? 127
                                            : 0;

    int16_t top_cmd = userInput.get_digital(DIGITAL_L1) ? 127
                      : (userInput.get_digital(DIGITAL_R1) ||
                         userInput.get_digital(DIGITAL_L2))
                          ? -127
                          : 0;

    if (userInput.get_digital_new_press(DIGITAL_X)) {
      startRecording();
    }

    if (userInput.get_digital_new_press(DIGITAL_B) && isRecording) {
      stopRecording();
    }

    if (isRecording && millis() - lastRecordTime >= RECORD_INTERVAL_MS) {
      recordWaypoint(intake_cmd, middle_cmd, top_cmd);
      lastRecordTime = millis();
    }

    if (userInput.get_digital_new_press(DIGITAL_A)) {
      match.toggle();
    }

    Pose current = odom.getPose();
    userInput.print(0, 0, "X:%.1f Y:%.1f", current.x, current.y);
    userInput.print(1, 0, "H:%.1f", current.theta * 180.0 / M_PI);

    delay(10);
  }
}