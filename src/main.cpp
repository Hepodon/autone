#include "main.h"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motor_group.hpp"

pros::MotorGroup aleft({9, -3, -4}, pros::MotorGearset::blue,
                       pros::v5::MotorUnits::degrees);
pros::MotorGroup aright({-19, 13, 14}, pros::MotorGearset::blue,
                        pros::v5::MotorUnits::degrees);

pros::Motor intake(-18);
pros::Motor middle(-10);
pros::Motor top(16);

pros::adi::Pneumatics match('a', false);
pros::adi::Pneumatics arm('b', false);
pros::adi::Pneumatics middlePneu('c', false);

lemlib::Drivetrain DT(&aleft, &aright, 12.72, lemlib::Omniwheel::NEW_325, 480,
                      2);

pros::IMU inertial1(20);

lemlib::OdomSensors sensors(nullptr, nullptr, nullptr, nullptr, &inertial1);

int startTime;

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
  startTime = pros::millis();
  // print position to brain screen
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
void autonomous() {
  match.set_value(true);
  chassis.moveToPoint(0, 28, 1000);
  chassis.turnToHeading(-90, 1000);
  chassis.moveToPoint(-18, 28, 1000);
  middle.move(-127);
  top.move(-100);
  pros::delay(1000);
  chassis.moveToPoint(24, 28, 1000);
  top.move(127);
  match.set_value(false);
  pros::delay(1000);
  top.move(-100);
  chassis.moveToPoint(10, -30, 1500);
  chassis.turnToHeading(-135, 500);
  middlePneu.set_value(true);
}

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
  bool recordButtonPressed = false;
  bool saveButtonPressed = false;

  while (true) {
    int leftY = userInput.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int rightX = userInput.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

    chassis.arcade(leftY, rightX, true, 0.40);

    middle.move(userInput.get_digital(DIGITAL_L1) ||
                        userInput.get_digital(DIGITAL_L2)
                    ? -127
                : userInput.get_digital(DIGITAL_DOWN) ? 127
                                                      : 0);

    top.move(userInput.get_digital(DIGITAL_L1) ? 127
             : userInput.get_digital(DIGITAL_DOWN) ||
                     userInput.get_digital(DIGITAL_L2)
                 ? -127
                 : 0);

    // Pneumatics toggle
    if (userInput.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)) {
      match.toggle();
    }
    if (userInput.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
      arm.toggle();
    }
    if (userInput.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
      middlePneu.toggle();
    }

    pros::delay(10); // 10ms delay = 100 frames per second
  }
}