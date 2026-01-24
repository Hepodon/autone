#include "main.h"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/motor_group.hpp"

pros::MotorGroup aleft({9, -3, -4}, pros::MotorGearset::blue,
                       pros::v5::MotorUnits::degrees);
pros::MotorGroup aright({-19, 13, 14}, pros::MotorGearset::blue,
                        pros::v5::MotorUnits::degrees);

pros::Motor intake(-18);
pros::Motor middle(-10);
pros::Motor top(16);

pros::adi::Pneumatics match('a', false);

lemlib::Drivetrain DT(&aleft, &aright, 12.72, lemlib::Omniwheel::NEW_325, 800,
                      2);

pros::IMU inertial1(20);

pros::Rotation leftVertEnd(1);
pros::Rotation horiEnd(21);

lemlib::TrackingWheel vert(&leftVertEnd, lemlib::Omniwheel::NEW_2, -3.0 / 8.0,
                           1);
lemlib::TrackingWheel hor(&horiEnd, lemlib::Omniwheel::NEW_2, 1);

lemlib::OdomSensors sensors(nullptr, nullptr, nullptr, nullptr, &inertial1);

// lateral PID controller
lemlib::ControllerSettings
    lateral_controller(10,  // proportional gain (kP)
                       0,   // integral gain (kI)
                       3,   // derivative gain (kD)
                       3,   // anti windup
                       1,   // small error range, in inches
                       100, // small error range timeout, in milliseconds
                       3,   // large error range, in inches
                       500, // large error range timeout, in milliseconds
                       8    // maximum acceleration (slew)
    );

// angular PID controller
lemlib::ControllerSettings
    angular_controller(2,   // proportional gain (kP)
                       0,   // integral gain (kI)
                       10,  // derivative gain (kD)
                       3,   // anti windup
                       1,   // small error range, in degrees
                       100, // small error range timeout, in milliseconds
                       3,   // large error range, in degrees
                       500, // large error range timeout, in milliseconds
                       10   // maximum acceleration (slew)
    );

lemlib::Chassis chassis(DT, lateral_controller, angular_controller, sensors);

pros::Controller userInput(pros::E_CONTROLLER_MASTER);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  pros::lcd::initialize(); // initialize brain screen
  chassis.calibrate();     // calibrate sensors
  // print position to brain screen
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
  pros::Task screen_task([&]() {
    while (true) {
      // print robot location to the brain screen
      pros::lcd::print(0, "X: %f", chassis.getPose().x);         // x
      pros::lcd::print(1, "Y: %f", chassis.getPose().y);         // y
      pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
      // delay to save resources
      pros::delay(20);
    }
  });
  pros::Task ball_task([&]() {
    while (true) {
      intake.move(userInput.get_digital(DIGITAL_R2)   ? -127
                  : userInput.get_digital(DIGITAL_R1) ? 127
                                                      : 0);

      middle.move(userInput.get_digital(DIGITAL_L1) ||
                          userInput.get_digital(DIGITAL_L2)
                      ? -127
                  : userInput.get_digital(DIGITAL_R1) ? 127
                                                      : 0);

      top.move(userInput.get_digital(DIGITAL_L1)   ? 127
               : userInput.get_digital(DIGITAL_R1) ? -127
                                                   : 0);

      if (userInput.get_digital_new_press(DIGITAL_A)) {
        match.toggle();
      }

      pros::delay(10);
    }
  });
  // loop forever
  while (true) {
    // get left y and right x positions
    int leftY = userInput.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int rightX = userInput.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

    // move the robot
    chassis.arcade(leftY, rightX, true, 0.40);

    // delay to save resources
    pros::delay(15);
  }
}