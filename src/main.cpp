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
#include <random>
#include <vector>

using namespace pros;

MotorGroup aright({-7, 5, 10});
MotorGroup aleft({19, -21, -6});
Motor intake(18);
Motor middle(3);
Motor top(4);

Controller userInput(E_CONTROLLER_MASTER);
adi::Pneumatics match('a', false);

IMU inertial1(11);
IMU inertial2(20);
GPS gps(12);
Rotation leftEncoder(13);
Rotation rightEncoder(14);

struct SensorConfig {
  bool useGPS = true;
  bool useEncoders = true;
  bool useIMU = true;
  bool useMCL = false;
  bool gpsForCorrection = true;
};

SensorConfig sensorConfig;

constexpr double WHEEL_DIAMETER = 2.75;
constexpr double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * M_PI;
constexpr double TRACK_WIDTH = 12.0;
constexpr double LOOKAHEAD_DISTANCE = 12.0;
constexpr int MAX_WAYPOINTS = 5000;
constexpr uint32_t RECORD_INTERVAL_MS = 50;

constexpr int NUM_PARTICLES = 500;
constexpr double FIELD_WIDTH = 144.0;
constexpr double FIELD_HEIGHT = 144.0;

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

struct Particle {
  double x;
  double y;
  double theta;
  double weight;
};

class MonteCarloLocalizer {
private:
  std::vector<Particle> particles;
  std::mt19937 rng;
  uint32_t lastGPSUpdateTime;
  bool gpsAvailable;

  double motionNoiseX = 0.5;
  double motionNoiseY = 0.5;
  double motionNoiseTheta = 0.05;
  double sensorNoiseGPS = 3.0;
  double sensorNoiseIMU = 0.08;

  double currentMotionNoise = 1.0;

public:
  MonteCarloLocalizer()
      : rng(std::random_device{}()), lastGPSUpdateTime(0), gpsAvailable(false) {
    particles.resize(NUM_PARTICLES);
  }

  void initialize(double x, double y, double theta, double spread = 6.0) {
    std::normal_distribution<> distX(x, spread);
    std::normal_distribution<> distY(y, spread);
    std::normal_distribution<> distTheta(theta, 0.3);

    for (auto &p : particles) {
      p.x = distX(rng);
      p.y = distY(rng);
      p.theta = distTheta(rng);
      p.weight = 1.0 / NUM_PARTICLES;
    }

    lastGPSUpdateTime = millis();
    gpsAvailable = false;
  }

  void predict(double deltaX, double deltaY, double deltaTheta) {
    uint32_t timeSinceGPS = millis() - lastGPSUpdateTime;
    if (timeSinceGPS > 2000) {
      currentMotionNoise = 1.0 + (timeSinceGPS / 1000.0) * 0.5;
    } else {
      currentMotionNoise = 1.0;
    }

    std::normal_distribution<> noiseX(0, motionNoiseX * currentMotionNoise);
    std::normal_distribution<> noiseY(0, motionNoiseY * currentMotionNoise);
    std::normal_distribution<> noiseTheta(0, motionNoiseTheta);

    for (auto &p : particles) {
      double cos_theta = cos(p.theta);
      double sin_theta = sin(p.theta);

      p.x += deltaX * cos_theta - deltaY * sin_theta + noiseX(rng);
      p.y += deltaX * sin_theta + deltaY * cos_theta + noiseY(rng);
      p.theta += deltaTheta + noiseTheta(rng);

      p.x = std::max(0.0, std::min(FIELD_WIDTH, p.x));
      p.y = std::max(0.0, std::min(FIELD_HEIGHT, p.y));

      while (p.theta > M_PI)
        p.theta -= 2 * M_PI;
      while (p.theta < -M_PI)
        p.theta += 2 * M_PI;
    }
  }

  void update(double gpsX, double gpsY, double imuTheta, bool hasGPS,
              bool hasIMU) {
    double totalWeight = 0.0;
    bool validGPS = (hasGPS && gpsX != PROS_ERR_F && gpsY != PROS_ERR_F);
    bool validIMU = (hasIMU && imuTheta != PROS_ERR_F);

    if (validGPS) {
      lastGPSUpdateTime = millis();
      gpsAvailable = true;
    }

    double meanX = 0, meanY = 0;
    if (validGPS) {
      for (const auto &p : particles) {
        meanX += p.x / NUM_PARTICLES;
        meanY += p.y / NUM_PARTICLES;
      }

      double distFromConsensus =
          sqrt(pow(gpsX - meanX, 2) + pow(gpsY - meanY, 2));

      if (distFromConsensus > 24.0 && gpsAvailable) {
        validGPS = false;
        userInput.print(2, 0, "GPS outlier!");
      }
    }

    for (auto &p : particles) {
      double weight = 1.0;

      if (validGPS) {
        double dx = p.x - gpsX;
        double dy = p.y - gpsY;
        double distError = sqrt(dx * dx + dy * dy);

        weight *=
            exp(-distError * distError / (2 * sensorNoiseGPS * sensorNoiseGPS));
      }

      if (validIMU) {
        double thetaError = p.theta - imuTheta;
        while (thetaError > M_PI)
          thetaError -= 2 * M_PI;
        while (thetaError < -M_PI)
          thetaError += 2 * M_PI;

        weight *= exp(-thetaError * thetaError /
                      (2 * sensorNoiseIMU * sensorNoiseIMU));
      }

      if (!validGPS && !validIMU) {
        weight = 1.0;
      }

      p.weight = weight;
      totalWeight += weight;
    }

    if (totalWeight > 0) {
      for (auto &p : particles) {
        p.weight /= totalWeight;
      }
    } else {
      for (auto &p : particles) {
        p.weight = 1.0 / NUM_PARTICLES;
      }
    }

    double nEff = 0.0;
    for (const auto &p : particles) {
      nEff += p.weight * p.weight;
    }
    nEff = 1.0 / nEff;
    double resampleThreshold =
        (validGPS || validIMU) ? NUM_PARTICLES / 2.0 : NUM_PARTICLES / 4.0;

    if (nEff < resampleThreshold) {
      resample();
    }
  }

  void resample() {
    std::vector<Particle> newParticles;
    newParticles.reserve(NUM_PARTICLES);

    std::uniform_real_distribution<> dist(0.0, 1.0 / NUM_PARTICLES);
    double r = dist(rng);
    double c = particles[0].weight;
    int i = 0;

    for (int m = 0; m < NUM_PARTICLES; m++) {
      double u = r + m * (1.0 / NUM_PARTICLES);
      while (u > c && i < NUM_PARTICLES - 1) {
        i++;
        c += particles[i].weight;
      }
      newParticles.push_back(particles[i]);
      newParticles.back().weight = 1.0 / NUM_PARTICLES;
    }

    particles = newParticles;
  }

  Pose getEstimate() const {
    double x = 0, y = 0;
    double sinTheta = 0, cosTheta = 0;

    for (const auto &p : particles) {
      x += p.x * p.weight;
      y += p.y * p.weight;
      sinTheta += sin(p.theta) * p.weight;
      cosTheta += cos(p.theta) * p.weight;
    }

    return {x, y, atan2(sinTheta, cosTheta), millis()};
  }

  double getUncertainty() const {
    Pose mean = getEstimate();
    double variance = 0;

    for (const auto &p : particles) {
      double dx = p.x - mean.x;
      double dy = p.y - mean.y;
      variance += (dx * dx + dy * dy) * p.weight;
    }

    return sqrt(variance);
  }

  bool isConverged() const { return getUncertainty() < 3.0; }

  bool hasGPSSignal() const {
    return gpsAvailable && (millis() - lastGPSUpdateTime < 1000);
  }

  void saveTrainingData(const char *filename) {
    FILE *f = fopen(filename, "w");
    if (!f)
      return;

    fprintf(f, "motionNoiseX: %.4f\n", motionNoiseX);
    fprintf(f, "motionNoiseY: %.4f\n", motionNoiseY);
    fprintf(f, "motionNoiseTheta: %.4f\n", motionNoiseTheta);
    fprintf(f, "sensorNoiseGPS: %.4f\n", sensorNoiseGPS);
    fprintf(f, "sensorNoiseIMU: %.4f\n", sensorNoiseIMU);

    fclose(f);
  }

  void loadTrainingData(const char *filename) {
    FILE *f = fopen(filename, "r");
    if (!f)
      return;

    fscanf(f, "motionNoiseX: %lf\n", &motionNoiseX);
    fscanf(f, "motionNoiseY: %lf\n", &motionNoiseY);
    fscanf(f, "motionNoiseTheta: %lf\n", &motionNoiseTheta);
    fscanf(f, "sensorNoiseGPS: %lf\n", &sensorNoiseGPS);
    fscanf(f, "sensorNoiseIMU: %lf\n", &sensorNoiseIMU);

    fclose(f);
  }
};

MonteCarloLocalizer mcl;

class Odometry {
private:
  Pose pose;
  double lastLeftPos;
  double lastRightPos;

  double getAveragedHeading() {
    if (!sensorConfig.useIMU)
      return pose.theta;

    double heading1 = inertial1.get_heading();
    double heading2 = inertial2.get_heading();

    if (heading1 == PROS_ERR_F && heading2 == PROS_ERR_F) {
      return pose.theta;
    } else if (heading1 == PROS_ERR_F) {
      return heading2 * M_PI / 180.0;
    } else if (heading2 == PROS_ERR_F) {
      return heading1 * M_PI / 180.0;
    }

    heading1 = heading1 * M_PI / 180.0;
    heading2 = heading2 * M_PI / 180.0;

    double diff = heading2 - heading1;
    while (diff > M_PI)
      diff -= 2 * M_PI;
    while (diff < -M_PI)
      diff += 2 * M_PI;

    return heading1 + diff / 2.0;
  }

public:
  Odometry() : pose{0, 0, 0, 0}, lastLeftPos(0), lastRightPos(0) {}

  void reset(double x = 0, double y = 0, double theta = 0) {
    pose = {x, y, theta, millis()};
    if (sensorConfig.useEncoders) {
      lastLeftPos = leftEncoder.get_position() / 100.0 * WHEEL_CIRCUMFERENCE;
      lastRightPos = rightEncoder.get_position() / 100.0 * WHEEL_CIRCUMFERENCE;
    }

    if (sensorConfig.useMCL) {
      mcl.initialize(x, y, theta);
    }
  }

  void update() {
    double deltaX = 0, deltaY = 0, deltaTheta = 0;
    double currentTheta = pose.theta;

    if (sensorConfig.useIMU) {
      currentTheta = getAveragedHeading();
    }

    if (sensorConfig.useEncoders) {
      double leftPos = leftEncoder.get_position() / 100.0 * WHEEL_CIRCUMFERENCE;
      double rightPos =
          rightEncoder.get_position() / 100.0 * WHEEL_CIRCUMFERENCE;

      double deltaLeft = leftPos - lastLeftPos;
      double deltaRight = rightPos - lastRightPos;

      double deltaCenter = (deltaLeft + deltaRight) / 2.0;
      deltaTheta = (deltaRight - deltaLeft) / TRACK_WIDTH;

      double avgTheta = currentTheta + deltaTheta / 2.0;

      deltaX = deltaCenter * cos(avgTheta);
      deltaY = deltaCenter * sin(avgTheta);

      pose.x += deltaX;
      pose.y += deltaY;

      if (!sensorConfig.useIMU) {
        currentTheta += deltaTheta;
      }

      lastLeftPos = leftPos;
      lastRightPos = rightPos;
    }

    if (sensorConfig.useMCL) {
      mcl.predict(deltaX, deltaY, deltaTheta);

      gps_status_s_t gpsStatus = gps.get_position_and_orientation();
      double gpsX = gpsStatus.x;
      double gpsY = gpsStatus.y;
      double imuTheta = currentTheta;

      mcl.update(gpsX, gpsY, imuTheta, sensorConfig.useGPS,
                 sensorConfig.useIMU);

      Pose mclEstimate = mcl.getEstimate();
      pose.x = mclEstimate.x;
      pose.y = mclEstimate.y;
      pose.theta = mclEstimate.theta;
    } else if (sensorConfig.useGPS && sensorConfig.gpsForCorrection) {
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

  if (sensorConfig.useMCL) {
    mcl.loadTrainingData("/usd/mcl_params.txt");
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
  userInput.clear();

  Task balls(ballTask);

  if (sensorConfig.useIMU) {
    inertial1.reset();
    inertial2.reset();
  }
  if (sensorConfig.useEncoders) {
    leftEncoder.reset_position();
    rightEncoder.reset_position();
  }
  gps.set_data_rate(5);

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

    if (userInput.get_digital_new_press(DIGITAL_UP)) {
      mcl.saveTrainingData("/usd/mcl_params.txt");
      userInput.print(0, 0, "MCL saved!");
      delay(1000);
    }

    Pose current = odom.getPose();
    userInput.print(0, 0, "X:%.1f Y:%.1f", current.x, current.y);
    userInput.print(1, 0, "Theta:%.1f", current.theta * 180.0 / M_PI);

    delay(10);
  }
}