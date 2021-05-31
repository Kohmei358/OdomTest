#include "main.h"
#include "singleton.hpp"
#include "okapi/impl/util/timer.hpp"

//Initialize pointer to zero so that it can be initialized in first call to getInstance
Singleton *Singleton::instance = 0;
auto imuZ = IMU(19, IMUAxes::z);
Controller controller;

int numberOfLoops = 0;
float wrappedIMU = 0;

float sumError = 0;
float lastError = 0;
float deltaT = 0;

void IMUWrapperUpdate(){
	float currentAngle = 0;
	float lastAngle = 0;
	while(true){

		currentAngle = imuZ.get();
		if (currentAngle == 0){
			currentAngle = 0.0001;
		}
		if(currentAngle > 0 && lastAngle < 0 && abs(currentAngle) > 90.0){
			numberOfLoops--;
		}
		else if(currentAngle < 0 && lastAngle > 0 && abs(currentAngle) > 90.0){
			numberOfLoops++;
		}

		wrappedIMU = currentAngle + (360.0 * numberOfLoops);
		lastAngle = currentAngle;

		pros::delay(20);
	}
}

void displayTaskFnc(){
	Singleton *s = s->getInstance();
	shared_ptr<OdomChassisController> chassis = s->getChassis();

	controller.clear();

	while(true){

		char buff[100];
		char buff2[100];
		snprintf(buff, sizeof(buff), "(%3.1f,%3.1f)",chassis->getState().x.convert(inch),chassis->getState().y.convert(inch));
		// snprintf(buff2, sizeof(buff2), "T:%3.2f",chassis->getState().theta.convert(degree));
		snprintf(buff2, sizeof(buff2), "wIMU:%3.2f (%d)",wrappedIMU, numberOfLoops);

		std::string buffAsStdStr = buff;
		std::string buff2AsStdStr = buff2;
		controller.setText(1,3, buff2);

		// pros::lcd::print(0, "X: %f", chassis->getState().x.convert(inch));
		// pros::lcd::print(1, "Y: %f", chassis->getState().y.convert(inch));
		// pros::lcd::print(2, "T: %f", chassis->getState().theta.convert(degree));
		pros::lcd::print(0, "Dt: %f", deltaT);
		pros::lcd::print(1, "Sum: %f", sumError);

		pros::lcd::print(4, "L: %d", chassis->getModel()->getSensorVals()[0]);
		pros::lcd::print(5, "R: %d", chassis->getModel()->getSensorVals()[1]);
		pros::lcd::print(6, "B: %d", chassis->getModel()->getSensorVals()[2]);

		pros::delay(20);
	}
}

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	Singleton *s = s->getInstance();
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

		pros::Task IMUTask(IMUWrapperUpdate);
		pros::Task displayTask(displayTaskFnc);

		Controller controller;
		controller.clear();
		ControllerButton runAutoButton(ControllerDigital::X);
		ControllerButton AButton(ControllerDigital::A);
		ControllerButton BButton(ControllerDigital::B);
		ControllerButton CalButton(ControllerDigital::up);
		Singleton *s = s->getInstance();
		shared_ptr<OdomChassisController> chassis = s->getChassis();

		std::shared_ptr<AsyncMotionProfileController> profileController =
		AsyncMotionProfileControllerBuilder()
			.withLimits({
				2.0, // Maximum linear velocity of the Chassis in m/s
				1.3, // Maximum linear acceleration of the Chassis in m/s/s
				4.0 // Maximum linear jerk of the Chassis in m/s/s/s
			})
			.withOutput(chassis)
			.buildMotionProfileController();

		profileController->generatePath(
    {{0_ft, 0_ft, 0_deg}, {2_ft, 0_ft, 0_deg}}, "A");

		profileController->generatePath(
		{{0_ft, 0_ft, 0_deg}, {0_ft, 0_ft, 0_deg}}, "B");


		while (true) {
		// Arcade drive with the left stick
		chassis->getModel()->arcade(controller.getAnalog(ControllerAnalog::leftY),
															controller.getAnalog(ControllerAnalog::rightX));


		// Run the test autonomous routine if we press the button
		if (runAutoButton.changedToPressed()) {
				// Drive the robot in a square pattern using closed-loop control
				// chassis->turnAngle(90_deg);   // Turn in place 90 degrees
				// chassis->getModel()->resetSensors();
				// auto imuZ = IMU(19, IMUAxes::z);


				float kP = 0.006;
				// float kI = 0.002;
				float kI = 0;
				// float kD = 0.00065;
				float kD = 0;
				float targetAngle = 0;

				float error = targetAngle - imuZ.get();

				unsigned long lastTime = pros::millis();
				lastError = 0;
				sumError = 0;


				SettledUtil settledUtil( //5 deg, 5 deg /sec, hold for 250ms
				std::make_unique<Timer>(), 1, 1, 250_ms);

				while(!settledUtil.isSettled(error)){
					unsigned long now = pros::millis();
					deltaT = (now - lastTime)/1000.0; //ms to s

					error = targetAngle - imuZ.get();

					if(lastError < 0 != error < 0){
						sumError = 0;
					}

					sumError += (error * deltaT);

					float deriError = (error-lastError)/deltaT;

	 			  lastTime = now;
	 			  lastError = error;

					float turnPower = kP*error + kI*sumError + kD*deriError;

					chassis->getModel()->left(turnPower);
					chassis->getModel()->right(-turnPower);

					pros::delay(20);
				}

		}

		if (CalButton.changedToPressed()) {
			imuZ.calibrate();
			controller.clear();
			controller.setText(1, 0, "The IMU Calibrates...");
			pros::delay(3000);
			controller.clear();
		}
		if (AButton.changedToPressed()) {
					// chassis->moveDistance(24_in); // Drive forward 12 inches
					profileController->setTarget("A");
					profileController->waitUntilSettled();

					float kP = 0.007;
					float kI = 0;
					float kD = 0.0008;
					float targetAngle = 0;

					float error = targetAngle - imuZ.get();

					unsigned long lastTime;


					SettledUtil settledUtil( //5 deg, 5 deg /sec, hold for 250ms
					std::make_unique<Timer>(), 5, 5, 250_ms);

					while(!settledUtil.isSettled(error)){
						unsigned long now = pros::millis();
						float deltaT = (now - lastTime)/1000.0; //ms to s

						error = targetAngle - imuZ.get();
						sumError += (error * deltaT);

						float deriError = (error-lastError)/deltaT;

		 			  lastTime = now;
		 			  lastError = error;

						float turnPower = kP*error + kI*sumError + kD*deriError;
						chassis->getModel()->left(turnPower);
						chassis->getModel()->right(-turnPower);

						pros::delay(20);
					}
		}

		if (BButton.changedToPressed()) {
				// chassis->moveDistance(24_in); // Drive forward 12 inches
				// chassis->turnAngle(90_deg);   // Turn in place 90 degrees
				// profileController->setTarget("B");
				// profileController->waitUntilSettled();
		}
		// Wait and give up the time we don't need to other tasks.
		// Additionally, joystick values, motor telemetry, etc. all updates every 10 ms.
		pros::delay(10);
	}
}
