#include "main.h"
#include "singleton.hpp"
#include "okapi/impl/util/timer.hpp"

#define DIGITAL_SENSOR_PORTA 'A'
#define DIGITAL_SENSOR_PORTB 'B'
#define DIGITAL_SENSOR_PORTC 'C'

//Initialize pointer to zero so that it can be initialized in first call to getInstance
Singleton *Singleton::instance = 0;
auto imuZ = IMU(19, IMUAxes::z);
Controller controller;

// ADIUltrasonic sonar = ADIUltrasonic(7,8);

int numberOfLoops = 0;
bool settled = false;
float wrappedIMU = 0;
float startingOffsetDeg = 0;

float sumError = 0;
float lastError = 0;
float deltaT = 0;

MotorGroup Conveyor = MotorGroup({-1,9});
MotorGroup Intake = MotorGroup({18,-16});
MotorGroup Indexer = MotorGroup({4,-8});

pros::ADIDigitalOut intakeR ('G');
pros::ADIDigitalOut intakeL ('E');
// pros::ADIDigitalOut deploy (DIGITAL_SENSOR_PORTC);
pros::ADIDigitalOut deploy ('H');

// pros::ADIDigitalOut rightIntakePenu('C');
// pros::ADIDigitalOut leftIntakePenu('B');
// pros::ADIDigitalOut deploy('H');

void pidTurnToAngle(float targetDegree){
	Singleton *s = s->getInstance();
	shared_ptr<OdomChassisController> chassis = s->getChassis();
	float kI = 0.00055;//0.005
	float kP = 0.0048; //0.0056
	float kD = 0;
	float targetAngle = targetDegree;
	float error = targetAngle - wrappedIMU;
	unsigned long lastTime = pros::millis();
	lastError = 0;
	sumError = 0;
	SettledUtil settledUtil( //5 deg, 5 deg /sec, hold for 250ms
	std::make_unique<Timer>(), 5, 5, 50_ms);

	while(!settledUtil.isSettled(error)){
		unsigned long now = pros::millis();
		deltaT = (now - lastTime)/1000.0; //ms to s

		error = targetAngle - wrappedIMU;

		if((lastError < 0) != (error < 0)){
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

		wrappedIMU = currentAngle + (360.0 * numberOfLoops) + startingOffsetDeg;
		lastAngle = currentAngle;

		pros::delay(25);
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

		pros::lcd::print(0, "Dt: %f", deltaT);
		pros::lcd::print(1, "Sum: %f", sumError);

		// pros::lcd::print(4, "L: %d", chassis->getModel()->getSensorVals()[0]);
		// pros::lcd::print(5, "R: %d", chassis->getModel()->getSensorVals()[1]);
		// pros::lcd::print(6, "B: %d", chassis->getModel()->getSensorVals()[2]);

		pros::lcd::print(4, "X: %f", chassis->getState().x.convert(inch));
		pros::lcd::print(5, "Y: %f", chassis->getState().y.convert(inch));
		pros::lcd::print(6, "T: %f", chassis->getState().theta.convert(degree));
		// pros::lcd::print(7, "Sonar: %f", sonar.get());

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

	Singleton *s = s->getInstance(); //Make Chassis object (only happens on first call)

	pros::Task IMUTask(IMUWrapperUpdate); // Continous IMU readings use "wrappedIMU"
	pros::Task displayTask(displayTaskFnc);

	// deploy.set_value(true);
	// pros::delay(3000);
	// deploy.set_value(false);
	// pros::ADIDigitalOut intakeR ('A');
	// pros::ADIDigitalOut intakeL ('B');
	// pros::ADIDigitalOut deploy ('C');
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

	controller.clear();
	controller.setText(1, 0, "The IMU Calibrates...");
	pros::delay(100);
	imuZ.calibrate();
	numberOfLoops = 0;
	pros::delay(3000);
	controller.clear();

	deploy.set_value(true);
	intakeL.set_value(true); //Open
	intakeR.set_value(true);

	Intake.moveVoltage(-127*100); //Out to deploy

	pros::delay(500);

	Singleton *s = s->getInstance();
	shared_ptr<OdomChassisController> chassis = s->getChassis();
	shared_ptr<OdomChassisController> chassisRev = s->getChassisRev();
	shared_ptr<ChassisController> chassisOld = s->getChassisOld();

	std::shared_ptr<AsyncMotionProfileController> profileController =
	AsyncMotionProfileControllerBuilder()
		.withLimits({
			1.8, // Maximum linear velocity of the Chassis in m/s
			1.1, // Maximum linear acceleration of the Chassis in m/s/s
			3.0 // Maximum linear jerk of the Chassis in m/s/s/s
		})
		.withOutput(chassis)
		.buildMotionProfileController();

	std::shared_ptr<AsyncMotionProfileController> revProfileController =
	AsyncMotionProfileControllerBuilder()
		.withLimits({
			1.8, // Maximum linear velocity of the Chassis in m/s
			1.1, // Maximum linear acceleration of the Chassis in m/s/s
			3.0 // Maximum linear jerk of the Chassis in m/s/s/s
		})
		.withOutput(chassisRev)
		.buildMotionProfileController();

	std::shared_ptr<AsyncMotionProfileController> turnProfileController =
	AsyncMotionProfileControllerBuilder()
		.withLimits({
			1.0, // Maximum linear velocity of the Chassis in m/s
			0.6, // Maximum linear acceleration of the Chassis in m/s/s
			2.0 // Maximum linear jerk of the Chassis in m/s/s/s
		})
		.withOutput(chassis)
		.buildMotionProfileController();

		startingOffsetDeg = 0;


		//Intake Don't shoot
		Intake.moveVoltage(127*100);
		Conveyor.moveVoltage(127*100);
		Indexer.moveVoltage(-127*100);

		// Start here!;
		profileController->generatePath(
    {{0_ft, 0_ft, 0_deg}, {33.5_in, 0_ft, 0_deg}}, "FirstStraight");
		profileController->setTarget("FirstStraight");
		profileController->waitUntilSettled();

		Conveyor.moveVoltage(0);
		Indexer.moveVoltage(0);

		intakeL.set_value(false);
		intakeR.set_value(false); //Close

		pros::delay(1000);

		revProfileController->generatePath(
		{{0_ft, 0_ft, 0_deg}, {10_in, 0_ft, 0_deg}}, "REVToGoal1");
		revProfileController->setTarget("REVToGoal1");
		revProfileController->waitUntilSettled();

		pidTurnToAngle(45);

		chassis->getModel()->left(0);
		chassis->getModel()->right(0);

		profileController->generatePath( //Goal 1
		{{0_ft, 0_ft, 0_deg}, {10.5_in, 0_ft, 0_deg}}, "TowardsGoal1");
		profileController->setTarget("TowardsGoal1");
		profileController->waitUntilSettled();




		profileController->generatePath( //Goal 1
		{{0_ft, 0_ft, 0_deg}, {10.5_in, 0_ft, 0_deg}}, "TowardsGoal1");
		profileController->setTarget("TowardsGoal1");
		profileController->waitUntilSettled();

		profileController->generatePath( //Goal 1
		{{0_ft, 0_ft, 0_deg}, {4.5_in, 0_ft, 0_deg}}, "TowardsGoal1");
		profileController->setTarget("TowardsGoal1");
		profileController->waitUntilSettled();

		pros::delay(100);
		//
		// Intake.moveVoltage(127*100);
		// Conveyor.moveVoltage(127*100);
		// Indexer.moveVoltage(-127*100);

		pros::delay(1000);

		//Shoot
		Indexer.moveVoltage(127*100);

		pros::delay(500);

		// Intake.moveVoltage(0*100);
		Conveyor.moveVoltage(0*100);
		Indexer.moveVoltage(0*100);


		revProfileController->generatePath(
		{{0_ft, 0_ft, 0_deg}, {9_in, 0_ft, 0_deg}}, "REVToGoal1");
		revProfileController->setTarget("REVToGoal1");
		revProfileController->waitUntilSettled();

		Intake.moveVoltage(0*100);

		pidTurnToAngle(0);

		chassis->getModel()->left(0);
		chassis->getModel()->right(0);

		profileController->generatePath(
		{{0_ft, 0_ft, 0_deg}, {17.5_in, 0_ft, 0_deg}}, "TowardsGoal1");
		profileController->setTarget("TowardsGoal1");
		profileController->waitUntilSettled();

		pros::delay(30000); //Score Goal 1

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
		bool deployed = false;
		Controller controller;
		controller.clear();
		ControllerButton XButton(ControllerDigital::X);
		ControllerButton AButton(ControllerDigital::A);
		ControllerButton YButton(ControllerDigital::Y);
		ControllerButton BButton(ControllerDigital::B);

		ControllerButton L1Button(ControllerDigital::L1);
		ControllerButton L2Button(ControllerDigital::L2);
		ControllerButton R1Button(ControllerDigital::R1);
		ControllerButton R2Button(ControllerDigital::R2);

		ControllerButton UButton(ControllerDigital::up);
		ControllerButton LButton(ControllerDigital::left);
		ControllerButton RButton(ControllerDigital::right);
		ControllerButton DButton(ControllerDigital::down);

		Singleton *s = s->getInstance();
		shared_ptr<OdomChassisController> chassis = s->getChassis();




		// chassis->moveDistance(24_in); // Drive forward 12 inches

		// profileController->generatePath(
    // {{0_ft, 0_ft, 0_deg}, {2.5_ft, 0_ft, 0_deg}}, "A");
		//
		// turnProfileController->generatePath(
		// {{0_ft, 0_ft, 0_deg}, {2_ft, 2_ft, 0_deg}}, "B");


		while (true) {
			// Arcade drive with the left stick
			chassis->getModel()->arcade(controller.getAnalog(ControllerAnalog::rightY),
																controller.getAnalog(ControllerAnalog::rightX));


			int intakePower = 0;
			int conveyorPower = 0;
			int indexerPower = 0;

			if (L1Button.isPressed()) {
				intakePower+=127;
				conveyorPower+=127;
				indexerPower = -40;
			}
			else if (L2Button.isPressed()) {
				intakePower-=127;
				conveyorPower-=127;
				indexerPower-=127;
			}

			if (R1Button.isPressed()) {
				indexerPower = 127;
				conveyorPower = 127;
			}

			if (R2Button.isPressed()){
				intakeR.set_value(true);
				intakeL.set_value(true);
			}else{
				intakeR.set_value(false);
				intakeL.set_value(false);
			}
			Intake.moveVoltage(intakePower*100);
			Conveyor.moveVoltage(conveyorPower*100);
			Indexer.moveVoltage(indexerPower*100);

			if (XButton.changedToPressed()) {
				deployed = !deployed;
				deploy.set_value(deployed);
			}

			if(AButton.isPressed()){
				numberOfLoops = 0;
				autonomous();
			}

			pros::delay(20);
	}
}
