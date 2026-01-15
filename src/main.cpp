#include "main.h"
#include "pros/adi.hpp"
#include "pros/gps.hpp"
#include "pros/imu.hpp"
#include "pros/misc.h"
#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <vector>

using namespace pros;

// ============================================================================
// HARDWARE CONFIGURATION
// ===========================================================================
pros::MotorGroup aright({-19, 13, 14});
MotorGroup aleft({9, -3, -4});
Motor intake(-10);
Motor middle(20);
Motor top(20);

Controller userInput(E_CONTROLLER_MASTER);
adi::Pneumatics match('a', false);

// Sensors (configure ports as needed)
IMU inertial1(7);           // Primary IMU
IMU inertial2(6);           // Secondary IMU
GPS gps(15);                // GPS port (in INCHES mode)
Rotation leftEncoder(1);    // Left tracking wheel
Rotation rightEncoder(-11); // Right tracking wheel

// ============================================================================
// CONFIGURATION FLAGS
// ============================================================================
struct SensorConfig {
  bool useGPS = false;
  bool useEncoders = true;
  bool useIMU = true;
  bool useMCL = false;           // Monte Carlo Localization
  bool gpsForCorrection = false; // true = correction, false = logging onlyz
};

SensorConfig sensorConfig;

// ============================================================================
// CONSTANTS
// ============================================================================
constexpr double WHEEL_DIAMETER = 2; // inches
constexpr double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * M_PI;
constexpr double TRACK_WIDTH = 12.72;        // inches between wheels
constexpr double LOOKAHEAD_DISTANCE = 8.0; // inches
constexpr int MAX_WAYPOINTS = 5000;
constexpr uint32_t RECORD_INTERVAL_MS = 100; // Record every 50ms

// MCL Constants
constexpr int NUM_PARTICLES = 100;
constexpr double FIELD_WIDTH = 144.0;  // 12 feet in inches
constexpr double FIELD_HEIGHT = 144.0; // 12 feet in inches

// ============================================================================
// DATA STRUCTURES
// ============================================================================
struct Pose {
  double x;           // inches
  double y;           // inches
  double theta;       // radians
  uint32_t timestamp; // milliseconds
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

// ============================================================================
// MONTE CARLO LOCALIZATION (GPS + IMU + Odometry Fusion)
// ============================================================================
// MCL without distance sensors works by:
// 1. Using GPS as absolute position measurements (when available)
// 2. Using IMU as absolute heading measurements
// 3. Using odometry for motion prediction between GPS updates
// 4. Filtering out bad GPS readings via particle consensus
// 5. Handling GPS dropouts gracefully by relying on odometry
// 6. AUTOMATICALLY adjusting sensor trust based on consistency

struct Particle {
  double x;
  double y;
  double theta;
  double weight;
};

uint32_t recordStartTime = 0;
double lastLeftCmd = 0;
double lastRightCmd = 0;
int leftPower = 0;
int rightPower = 0;
const double MAX_ACCEL = 6; // power units per loop

double slewLimit(double target, double last, double maxDelta) {
  if (target > last + maxDelta)
    return last + maxDelta;
  if (target < last - maxDelta)
    return last - maxDelta;
  return target;
}

double flipHeadingDeg(double h) {
  h = fmod(360.0 - h, 360.0);
  if (h < 0)
    h += 360.0;
  return h;
}

class MonteCarloLocalizer {
private:
  std::vector<Particle> particles;
  std::mt19937 rng;
  uint32_t lastGPSUpdateTime;
  bool gpsAvailable;

  // Base noise parameters (starting point)
  double baseMotionNoiseX = 0.5;
  double baseMotionNoiseY = 0.5;
  double baseMotionNoiseTheta = 0.05;
  double baseSensorNoiseGPS = 3.0;
  double baseSensorNoiseIMU = 0.08;

  // ADAPTIVE noise (automatically adjusted based on sensor performance)
  double adaptiveMotionNoiseX = 0.5;
  double adaptiveMotionNoiseY = 0.5;
  double adaptiveSensorNoiseGPS = 3.0;
  double adaptiveSensorNoiseIMU = 0.08;

  // Sensor consistency tracking
  std::vector<double> recentGPSErrors;
  std::vector<double> recentIMUErrors;
  const int ERROR_HISTORY_SIZE = 20;

  // Adaptive noise (increases when GPS unavailable)
  double currentMotionNoise = 1.0;

public:
  MonteCarloLocalizer()
      : rng(std::random_device{}()), lastGPSUpdateTime(0), gpsAvailable(false) {
    particles.resize(NUM_PARTICLES);
    recentGPSErrors.reserve(ERROR_HISTORY_SIZE);
    recentIMUErrors.reserve(ERROR_HISTORY_SIZE);
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

    // Reset adaptive parameters
    adaptiveMotionNoiseX = baseMotionNoiseX;
    adaptiveMotionNoiseY = baseMotionNoiseY;
    adaptiveSensorNoiseGPS = baseSensorNoiseGPS;
    adaptiveSensorNoiseIMU = baseSensorNoiseIMU;
    recentGPSErrors.clear();
    recentIMUErrors.clear();
  }

  // Automatically adjust sensor noise based on recent consistency
  void updateAdaptiveNoise() {
    // GPS noise adjustment
    if (recentGPSErrors.size() >= 5) {
      double avgError = 0;
      for (double err : recentGPSErrors) {
        avgError += err;
      }
      avgError /= recentGPSErrors.size();

      // If GPS consistently has low error, trust it more (lower noise)
      // If GPS has high error, trust it less (higher noise)
      adaptiveSensorNoiseGPS = baseSensorNoiseGPS * (0.5 + avgError / 6.0);

      // Clamp between reasonable bounds
      adaptiveSensorNoiseGPS =
          std::max(1.0, std::min(10.0, adaptiveSensorNoiseGPS));
    }

    // IMU noise adjustment
    if (recentIMUErrors.size() >= 5) {
      double avgError = 0;
      for (double err : recentIMUErrors) {
        avgError += err;
      }
      avgError /= recentIMUErrors.size();

      // Adjust IMU trust based on consistency
      adaptiveSensorNoiseIMU = baseSensorNoiseIMU * (0.5 + avgError / 0.2);
      adaptiveSensorNoiseIMU =
          std::max(0.03, std::min(0.3, adaptiveSensorNoiseIMU));
    }

    // Motion noise adjustment based on particle spread
    double spreadX = 0, spreadY = 0;
    Pose estimate = getEstimate();

    for (const auto &p : particles) {
      spreadX += pow(p.x - estimate.x, 2) * p.weight;
      spreadY += pow(p.y - estimate.y, 2) * p.weight;
    }

    spreadX = sqrt(spreadX);
    spreadY = sqrt(spreadY);

    // If particles spread a lot, odometry might be drifting - increase motion
    // noise
    if (spreadX > 5.0) {
      adaptiveMotionNoiseX = baseMotionNoiseX * 1.5;
    } else if (spreadX < 2.0) {
      adaptiveMotionNoiseX = baseMotionNoiseX * 0.8;
    }

    if (spreadY > 5.0) {
      adaptiveMotionNoiseY = baseMotionNoiseY * 1.5;
    } else if (spreadY < 2.0) {
      adaptiveMotionNoiseY = baseMotionNoiseY * 0.8;
    }
  }

  void predict(double deltaX, double deltaY, double deltaTheta) {
    // Increase uncertainty if GPS has been unavailable for a while
    uint32_t timeSinceGPS = millis() - lastGPSUpdateTime;
    if (timeSinceGPS > 2000) {
      // After 2 seconds without GPS, increase motion noise
      currentMotionNoise = 1.0 + (timeSinceGPS / 1000.0) * 0.5;
    } else {
      currentMotionNoise = 1.0;
    }

    // Use adaptive noise values
    std::normal_distribution<> noiseX(0, adaptiveMotionNoiseX *
                                             currentMotionNoise);
    std::normal_distribution<> noiseY(0, adaptiveMotionNoiseY *
                                             currentMotionNoise);
    std::normal_distribution<> noiseTheta(0, baseMotionNoiseTheta);

    for (auto &p : particles) {
      // Apply motion model with noise
      double cos_theta = cos(p.theta);
      double sin_theta = sin(p.theta);

      p.x += deltaX * cos_theta - deltaY * sin_theta + noiseX(rng);
      p.y += deltaX * sin_theta + deltaY * cos_theta + noiseY(rng);
      p.theta += deltaTheta + noiseTheta(rng);

      // Keep particles on field
      p.x = std::max(0.0, std::min(FIELD_WIDTH, p.x));
      p.y = std::max(0.0, std::min(FIELD_HEIGHT, p.y));

      // Normalize theta
      while (p.theta > M_PI)
        p.theta -= 2 * M_PI;
      while (p.theta < -M_PI)
        p.theta += 2 * M_PI;
    }
  }

  void update(double gpsX, double gpsY, double imuTheta, bool hasGPS,
              bool hasIMU) {
    // Update particle weights based on sensor measurements
    double totalWeight = 0.0;
    bool validGPS = (hasGPS && gpsX != PROS_ERR_F && gpsY != PROS_ERR_F);
    bool validIMU = (hasIMU && imuTheta != PROS_ERR_F);

    // Track GPS availability
    if (validGPS) {
      lastGPSUpdateTime = millis();
      gpsAvailable = true;
    }

    // Get particle consensus for outlier detection
    double meanX = 0, meanY = 0;
    for (const auto &p : particles) {
      meanX += p.x / NUM_PARTICLES;
      meanY += p.y / NUM_PARTICLES;
    }

    // Outlier detection for GPS (reject if too far from particle consensus)
    double distFromConsensus = 0;
    if (validGPS) {
      distFromConsensus = sqrt(pow(gpsX - meanX, 2) + pow(gpsY - meanY, 2));

      // If GPS reading is more than 24 inches from consensus, it's probably bad
      if (distFromConsensus > 24.0 && gpsAvailable) {
        validGPS = false; // Reject this GPS reading
      }
    }

    // Track sensor errors for adaptive learning
    if (validGPS) {
      // Store GPS error (how far it was from consensus)
      recentGPSErrors.push_back(distFromConsensus);
      if (recentGPSErrors.size() > ERROR_HISTORY_SIZE) {
        recentGPSErrors.erase(recentGPSErrors.begin());
      }
    }

    for (auto &p : particles) {
      double weight = 1.0;

      // GPS measurement model (absolute position) - uses ADAPTIVE noise
      if (validGPS) {
        double dx = p.x - gpsX;
        double dy = p.y - gpsY;
        double distError = sqrt(dx * dx + dy * dy);

        // Gaussian probability - uses adaptive sensor noise
        weight *= exp(-distError * distError /
                      (2 * adaptiveSensorNoiseGPS * adaptiveSensorNoiseGPS));
      }

      // IMU measurement model (absolute heading) - uses ADAPTIVE noise
      if (validIMU) {
        double thetaError = p.theta - imuTheta;
        while (thetaError > M_PI)
          thetaError -= 2 * M_PI;
        while (thetaError < -M_PI)
          thetaError += 2 * M_PI;

        // Track IMU consistency
        if (recentIMUErrors.size() < ERROR_HISTORY_SIZE) {
          recentIMUErrors.push_back(std::abs(thetaError));
          if (recentIMUErrors.size() > ERROR_HISTORY_SIZE) {
            recentIMUErrors.erase(recentIMUErrors.begin());
          }
        }

        // Gaussian probability - uses adaptive sensor noise
        weight *= exp(-thetaError * thetaError /
                      (2 * adaptiveSensorNoiseIMU * adaptiveSensorNoiseIMU));
      }

      // If no sensors available, all particles keep equal weight
      if (!validGPS && !validIMU) {
        weight = 1.0;
      }

      p.weight = weight;
      totalWeight += weight;
    }

    // Normalize weights
    if (totalWeight > 0) {
      for (auto &p : particles) {
        p.weight /= totalWeight;
      }
    } else {
      // If all weights are zero (shouldn't happen), reset to uniform
      for (auto &p : particles) {
        p.weight = 1.0 / NUM_PARTICLES;
      }
    }

    // Update adaptive noise parameters based on recent performance
    updateAdaptiveNoise();

    // Resample if effective sample size is low
    double nEff = 0.0;
    for (const auto &p : particles) {
      nEff += p.weight * p.weight;
    }
    nEff = 1.0 / nEff;

    // Resample more aggressively when we have good sensor data
    double resampleThreshold =
        (validGPS || validIMU) ? NUM_PARTICLES / 2.0 : NUM_PARTICLES / 4.0;

    if (nEff < resampleThreshold) {
      resample();
    }
  }

  void resample() {
    std::vector<Particle> newParticles;
    newParticles.reserve(NUM_PARTICLES);

    // Low variance resampling
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
    // Weighted average of particles
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

  // Get particle spread (uncertainty measure)
  double getUncertainty() const {
    Pose mean = getEstimate();
    double variance = 0;

    for (const auto &p : particles) {
      double dx = p.x - mean.x;
      double dy = p.y - mean.y;
      variance += (dx * dx + dy * dy) * p.weight;
    }

    return sqrt(variance); // Standard deviation in inches
  }

  bool isConverged() const {
    return getUncertainty() < 3.0; // Converged if uncertainty < 3 inches
  }

  bool hasGPSSignal() const {
    return gpsAvailable && (millis() - lastGPSUpdateTime < 1000);
  }

  // Get current adaptive noise values for debugging/display
  double getAdaptiveGPSNoise() const { return adaptiveSensorNoiseGPS; }
  double getAdaptiveIMUNoise() const { return adaptiveSensorNoiseIMU; }
  double getAdaptiveMotionNoiseX() const { return adaptiveMotionNoiseX; }
  double getAdaptiveMotionNoiseY() const { return adaptiveMotionNoiseY; }

  // Load/save noise parameters (optional - only if you want to persist learned
  // values)
  void saveTrainingData(const char *filename) {
    FILE *f = fopen(filename, "w");
    if (!f)
      return;

    fprintf(f, "motionNoiseX: %.4f\n", baseMotionNoiseX);
    fprintf(f, "motionNoiseY: %.4f\n", baseMotionNoiseY);
    fprintf(f, "motionNoiseTheta: %.4f\n", baseMotionNoiseTheta);
    fprintf(f, "sensorNoiseGPS: %.4f\n", baseSensorNoiseGPS);
    fprintf(f, "sensorNoiseIMU: %.4f\n", baseSensorNoiseIMU);

    fclose(f);
  }

  void loadTrainingData(const char *filename) {
    FILE *f = fopen(filename, "r");
    if (!f) {
      // No saved params, use current defaults
      return;
    }

    fscanf(f, "motionNoiseX: %lf\n", &baseMotionNoiseX);
    fscanf(f, "motionNoiseY: %lf\n", &baseMotionNoiseY);
    fscanf(f, "motionNoiseTheta: %lf\n", &baseMotionNoiseTheta);
    fscanf(f, "sensorNoiseGPS: %lf\n", &baseSensorNoiseGPS);
    fscanf(f, "sensorNoiseIMU: %lf\n", &baseSensorNoiseIMU);

    fclose(f);

    // Initialize adaptive values to base values
    adaptiveMotionNoiseX = baseMotionNoiseX;
    adaptiveMotionNoiseY = baseMotionNoiseY;
    adaptiveSensorNoiseGPS = baseSensorNoiseGPS;
    adaptiveSensorNoiseIMU = baseSensorNoiseIMU;
  }
};

MonteCarloLocalizer mcl;

// ============================================================================
// ODOMETRY CLASS
// ============================================================================
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

    // Check validity
    if (heading1 == PROS_ERR_F && heading2 == PROS_ERR_F) {
      return pose.theta;
    } else if (heading1 == PROS_ERR_F) {
      heading2 = flipHeadingDeg(heading2);
      return heading2 * M_PI / 180.0;
    } else if (heading2 == PROS_ERR_F) {
      return heading1 * M_PI / 180.0;
    }

    heading2 = flipHeadingDeg(heading2);

    // Convert to radians
    double h1 = heading1 * M_PI / 180.0;
    double h2 = heading2 * M_PI / 180.0;

    // ✅ Circular mean (wrap-safe)
    double x = cos(h1) + cos(h2);
    double y = sin(h1) + sin(h2);

    return atan2(y, x);
  }

public:
  Odometry() : pose{0, 0, 0, 0}, lastLeftPos(0), lastRightPos(0) {}

  void reset(double x = 0, double y = 0, double theta = 0) {
    pose = {x, y, theta, millis()};
    if (sensorConfig.useEncoders) {
      lastLeftPos = leftEncoder.get_position() / 36000.0 * WHEEL_CIRCUMFERENCE;
      lastRightPos =
          rightEncoder.get_position() / 36000.0 * WHEEL_CIRCUMFERENCE;
    }

    if (sensorConfig.useMCL) {
      mcl.initialize(x, y, theta);
    }
  }

  void update() {
    double deltaX = 0, deltaY = 0, deltaTheta = 0;
    double currentTheta = pose.theta;
    bool imuValid = !inertial1.is_calibrating() && !inertial2.is_calibrating();

    if (sensorConfig.useIMU && imuValid) {
      currentTheta = getAveragedHeading();
    }

    // Update position from encoders if enabled (in INCHES)
    if (sensorConfig.useEncoders) {
      double leftPos =
          leftEncoder.get_position() / 36000.0 * WHEEL_CIRCUMFERENCE;
      double rightPos =
          rightEncoder.get_position() / 36000.0 * WHEEL_CIRCUMFERENCE;

      double deltaLeft = leftPos - lastLeftPos;
      double deltaRight = rightPos - lastRightPos;

      double deltaCenter = (deltaLeft + deltaRight) / 2.0;

      // If IMU is enabled, ignore encoder heading
      double avgTheta;
      if (sensorConfig.useIMU && imuValid) {
        avgTheta = currentTheta;
      }

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

    // Monte Carlo Localization
    if (sensorConfig.useMCL) {
      // Predict step
      mcl.predict(deltaX, deltaY, deltaTheta);

      // Update with sensor measurements
      gps_status_s_t gpsStatus = gps.get_position_and_orientation();
      double gpsX =
          gpsStatus.x / 25.4; // Already in inches if GPS configured correctly
      double gpsY = gpsStatus.y / 25.4; // Already in inches
      double imuTheta = gpsStatus.yaw * M_PI / 180;

      mcl.update(gpsX, gpsY, imuTheta, sensorConfig.useGPS,
                 sensorConfig.useIMU);

      // Get MCL estimate
      Pose mclEstimate = mcl.getEstimate();
      pose.x = mclEstimate.x;
      pose.y = mclEstimate.y;
      pose.theta = mclEstimate.theta;
    }
    // GPS correction if enabled (and not using MCL)
    else if (sensorConfig.useGPS && sensorConfig.gpsForCorrection) {
      gps_status_s_t status = gps.get_position_and_orientation();
      if (status.x != PROS_ERR_F && status.y != PROS_ERR_F) {
        // GPS should already be in inches - verify configuration!
        double gpsX = status.x / 25.4;
        double gpsY = status.y / 25.4;
        double speed = fabs(deltaX) + fabs(deltaY);

        if (speed < 0.5) { // inches per update
          pose.x = pose.x * 0.9 + gpsX * 0.1;
          pose.y = pose.y * 0.9 + gpsY * 0.1;
        }
      }
    }

    pose.theta = currentTheta;
    pose.timestamp = millis();
  }

  Pose getPose() const { return pose; }

  // Get GPS data for logging (even if not used for correction)
  Pose getGPSPose() const {
    Pose gpsPose = pose;
    if (sensorConfig.useGPS) {
      gps_status_s_t status = gps.get_position_and_orientation();
      if (status.x != PROS_ERR_F)
        gpsPose.x = status.x / 25.4; // Already inches
      if (status.y != PROS_ERR_F)
        gpsPose.y = status.y / 25.4; // Already inches
      if (status.yaw != PROS_ERR_F)
        gpsPose.theta = status.yaw * M_PI / 180.0;
    }
    return gpsPose;
  }
};

Odometry odom;

// ============================================================================
// PATH RECORDING
// ============================================================================
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
  recordStartTime = millis();
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
  wp.x = current.x; // In inches
  wp.y = current.y; // In inches
  wp.theta = current.theta;
  wp.timestamp = millis() - recordStartTime;
  wp.intake_cmd = intake_cmd;
  wp.middle_cmd = middle_cmd;
  wp.top_cmd = top_cmd;

  recordedPath.push_back(wp);
}

// ============================================================================
// PURE PURSUIT ALGORITHM
// ============================================================================
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

  // If no intersection, use the last point
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

  // Normalize angle to [-pi, pi]
  while (angleError > M_PI)
    angleError -= 2 * M_PI;
  while (angleError < -M_PI)
    angleError += 2 * M_PI;

  // Calculate curvature
  double distance = sqrt(dx * dx + dy * dy);
  double curvature = (2 * sin(angleError)) / LOOKAHEAD_DISTANCE;
  curvature = std::clamp(curvature, -0.5, 0.5);

  // Base speed (can be adjusted)
  int baseSpeed = 80;

  // Calculate differential steering
  double leftSpeed = baseSpeed * (1 + curvature * TRACK_WIDTH / 2.0);
  double rightSpeed = baseSpeed * (1 - curvature * TRACK_WIDTH / 2.0);

  // Clamp to [-127, 127]
  leftSpeed = std::max(-127.0, std::min(127.0, leftSpeed));
  rightSpeed = std::max(-127.0, std::min(127.0, rightSpeed));

  return {(int)leftSpeed, (int)rightSpeed};
}

// ============================================================================
// MECHANISM CONTROL TASK
// ============================================================================
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

// ============================================================================
// AUTONOMOUS MODE
// ============================================================================
void autonomous() {
  gps.set_data_rate(5);
  std::vector<Waypoint> path = loadPathFromSD("/usd/path.dat");
  if (path.empty()) {
    userInput.print(0, 0, "No path!");
    return;
  }

  // Load MCL training data if using MCL
  if (sensorConfig.useMCL) {
    mcl.loadTrainingData("/usd/mcl_params.txt");
  }

  odom.reset(path[0].x, path[0].y, path[0].theta);

  int lastIndex = 0;
  uint32_t startTime = millis();

  while (true) {
    odom.update();
    Pose robot = odom.getPose();

    // Find lookahead point
    LookaheadPoint target = findLookaheadPoint(path, robot, lastIndex);

    if (!target.found)
      break;

    MotorCommands cmd = calculatePurePursuit(robot, target);

    leftPower = slewLimit(cmd.left, lastLeftCmd, MAX_ACCEL);
    rightPower = slewLimit(cmd.right, lastRightCmd, MAX_ACCEL);

    aleft.move(leftPower);
    aright.move(rightPower);

    lastLeftCmd = leftPower;
    lastRightCmd = rightPower;

    // Execute mechanism commands based on current waypoint timing
    uint32_t elapsed = millis() - startTime;
    for (const auto &wp : path) {
      if (abs((int)(elapsed - wp.timestamp)) < 100) {
        intake.move(wp.intake_cmd);
        middle.move(wp.middle_cmd);
        top.move(wp.top_cmd);
        break;
      }
    }

    // Check if we've reached the end
    double distToEnd =
        sqrt(pow(path.back().x - robot.x, 2) + pow(path.back().y - robot.y, 2));
    if (distToEnd < 3.0 && lastIndex >= path.size() - 2)
      break;

    delay(10);
  }

  aleft.brake();
  aright.brake();
  intake.brake();
  middle.brake();
  top.brake();
}

// ============================================================================
// OPERATOR CONTROL MODE
// ============================================================================
void opcontrol() {
  Task balls(ballTask);
  gps.set_data_rate(5);

  // Initialize sensors
  if (sensorConfig.useIMU) {
    inertial1.reset();
    inertial2.reset();
  }
  if (sensorConfig.useEncoders) {
    leftEncoder.reset_position();
    rightEncoder.reset_position();
  }

  // Load MCL parameters if using MCL
  if (sensorConfig.useMCL) {
    mcl.loadTrainingData("/usd/mcl_params.txt");
  }

  odom.reset();

  uint32_t lastRecordTime = 0;

  while (true) {
    // Update odometry
    odom.update();

    // Manual drive control
    int16_t fwd = userInput.get_analog(ANALOG_LEFT_Y);
    int16_t trn = userInput.get_analog(ANALOG_RIGHT_X) * 0.6;

    leftPower = slewLimit(fwd + trn, lastLeftCmd, MAX_ACCEL);
    rightPower = slewLimit(fwd - trn, lastRightCmd, MAX_ACCEL);

    aleft.move(leftPower);
    aright.move(rightPower);

    lastLeftCmd = leftPower;
    lastRightCmd = rightPower;

    // Get current mechanism commands
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

    // Recording controls
    if (userInput.get_digital_new_press(DIGITAL_X)) {
      startRecording();
    }

    if (userInput.get_digital_new_press(DIGITAL_B) && isRecording) {
      stopRecording();
    }

    // Record waypoint at intervals (in INCHES)
    if (isRecording && millis() - lastRecordTime >= RECORD_INTERVAL_MS) {
      recordWaypoint(intake_cmd, middle_cmd, top_cmd);
      lastRecordTime = millis();
    }

    // Pneumatics toggle
    if (userInput.get_digital_new_press(DIGITAL_A)) {
      match.toggle();
    }

    // Display current pose on controller (in INCHES)
    Pose current = odom.getPose();
    userInput.print(0, 0, "X:%.1f Y:%.1f", current.x, current.y);
    userInput.print(1, 0, "Theta:%.1f", current.theta * 180.0 / M_PI);

    delay(10);
  }
}