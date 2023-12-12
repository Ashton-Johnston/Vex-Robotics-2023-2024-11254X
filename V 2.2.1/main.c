#include "main.h"
// Motors
#define FRONT_RIGHT_MOTOR 13
#define FRONT_LEFT_MOTOR 14
#define BACK_LEFT_MOTOR 17
#define BACK_RIGHT_MOTOR 18

// Intake things
#define INTAKE_MOTOR 11
#define INTAKE_SPEED 127

// Pnematics things
#define PNEMATICS 1 // main intake drop function
#define WING1 5 // left wing 
#define WING2 6 // right wing

// Catapult things
#define CATAPULT_MOTOR 3 // originally port 3
#define CATAPULT_SPEED 120 // 127 MAX ||| -127 MIN
#define REDUCTION 1  // unused as od 12/09/23

// Sensors
#define IMU_PORT 20 // this is the sensor to see the heading of the robot
#define ROTATION_RIGHT_PORT 1
#define ROTATION_LEFT_PORT 7

// Autonomous things
#define AUTONOMOUS_SPEED 100 //unused as of 11/29/23


void drive_FN();
void intake_FN();
void pne_FN();
void cpt_FN();
void wing_FN();     //====================================================..............

void initialize() 
{
    //motor_set_reversed(FRONT_RIGHT_MOTOR, true);
    motor_set_reversed(BACK_LEFT_MOTOR, true);
    motor_set_reversed(FRONT_LEFT_MOTOR, true);
    motor_set_reversed(INTAKE_MOTOR, true);
    motor_set_brake_mode(FRONT_RIGHT_MOTOR, MOTOR_BRAKE_HOLD);
    motor_set_brake_mode(FRONT_LEFT_MOTOR, MOTOR_BRAKE_HOLD);
    motor_set_brake_mode(BACK_RIGHT_MOTOR, MOTOR_BRAKE_HOLD);
    motor_set_brake_mode(BACK_LEFT_MOTOR, MOTOR_BRAKE_HOLD);
    motor_set_brake_mode(INTAKE_MOTOR, MOTOR_BRAKE_COAST);
    adi_port_set_config(PNEMATICS, ADI_DIGITAL_OUT);
}

void disabled() {}

void competition_initialize() {}


// this is to move the robot forward with less code to make debugging quicker =-=-=-=-=-=-=-=-=-=-=-=-=-=-

void moveForward(double dist1, double vel0){
    int dist0 = dist1 * 2.43902;
    motor_move_absolute(FRONT_RIGHT_MOTOR, dist0, vel0);
    motor_move_absolute(BACK_RIGHT_MOTOR, dist0, vel0);
    motor_move_absolute(FRONT_LEFT_MOTOR, dist0, vel0);
    motor_move_absolute(BACK_LEFT_MOTOR, dist0, vel0);

    int pos = motor_get_position(FRONT_LEFT_MOTOR);

	rotation_reset(ROTATION_LEFT_PORT);
	rotation_reset(ROTATION_RIGHT_PORT);

    while(!(rotation_get_position(ROTATION_LEFT_PORT) < dist0 +5 && rotation_get_position(ROTATION_LEFT_PORT) > dist0 - 5)) {
        delay(1);
    }
}

void movebackward(double dist1, double vel0){
    int dist0 = dist1 * 2.43902;
    motor_move_absolute(FRONT_RIGHT_MOTOR, dist0, -1 * vel0);
    motor_move_absolute(BACK_RIGHT_MOTOR, dist0, -1 * vel0);
    motor_move_absolute(FRONT_LEFT_MOTOR, dist0, -1 * vel0);
    motor_move_absolute(BACK_LEFT_MOTOR, dist0, -1 * vel0);

    double pos = motor_get_position(FRONT_LEFT_MOTOR);
    
    rotation_reset(ROTATION_LEFT_PORT);
	rotation_reset(ROTATION_RIGHT_PORT);

    while(!(rotation_get_position(ROTATION_LEFT_PORT) < dist0 +5 && rotation_get_position(ROTATION_LEFT_PORT) > dist0 - 5)) {
        delay(1);
    }
}

void turnLeft(double rot0, double vel0){
    while(imu_get_rotation(IMU_PORT) > -1 * rot0){
        motor_move(FRONT_LEFT_MOTOR, -1 * vel0);
        motor_move(FRONT_RIGHT_MOTOR, vel0);
        motor_move(BACK_LEFT_MOTOR, -1 * vel0);
        motor_move(BACK_RIGHT_MOTOR, vel0);
    }

    double pos = motor_get_position(FRONT_LEFT_MOTOR);

	rotation_reset(ROTATION_LEFT_PORT);
	rotation_reset(ROTATION_RIGHT_PORT);
    
    while(!(rotation_get_position(ROTATION_LEFT_PORT) < rot0 + 5 && rotation_get_position(ROTATION_LEFT_PORT) > rot0 - 5)) {
        delay(1);
    }
}

void turnRight(double rot0, double vel0){
    while(imu_get_rotation(IMU_PORT) > rot0){
        motor_move(FRONT_LEFT_MOTOR, vel0);
        motor_move(FRONT_RIGHT_MOTOR, -1 * vel0);
        motor_move(BACK_LEFT_MOTOR, vel0);
        motor_move(BACK_RIGHT_MOTOR, -1 * vel0);
    }

	rotation_reset(ROTATION_LEFT_PORT);
	rotation_reset(ROTATION_RIGHT_PORT);
    
    while(!(rotation_get_position(ROTATION_LEFT_PORT) < rot0 +5 && rotation_get_position(ROTATION_LEFT_PORT) > rot0 - 5)) {
        delay(1);
    }
} 

void launchCat(double timeSeconds){
    motor_move(CATAPULT_MOTOR, 100);
    delay(timeSeconds * 1000);
    motor_brake(CATAPULT_MOTOR);
}
 
void autonomous(){ // =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
    adi_digital_write(PNEMATICS, true); // drop the intake down by firing the pnematics
    launchCat(1);

} //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-


void opcontrol(){
    task_t drive_task = task_create(drive_FN, NULL, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Drive");
    task_t intake_task = task_create(intake_FN, NULL, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Intake");
    task_t pnematic_task = task_create(pne_FN, NULL, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Pnematic");
    task_t catapult_task = task_create(cpt_FN, NULL, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Catapult");
    task_t wing_task = task_create(wing_FN,NULL, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Wings");
}

void wing_FN(){     //=====================================================..........................
    static bool in_on = false;
    for(;;) {
        if (controller_get_digital_new_press(CONTROLLER_MASTER, DIGITAL_X))
            in_on = !in_on;
        adi_digital_write(WING1, in_on);
        adi_digital_write(WING2, in_on);
        task_delay(20);
    }
}

void intake_FN() {
    static bool in_on = false;
    for(;;) {
        if (controller_get_digital_new_press(CONTROLLER_MASTER, DIGITAL_R2))
            in_on = !in_on;
        if (in_on) {
            motor_move(INTAKE_MOTOR, INTAKE_SPEED);
        } else if (controller_get_digital(CONTROLLER_MASTER, DIGITAL_R1))  {
            motor_move(INTAKE_MOTOR, -INTAKE_SPEED);
        } else {
            motor_brake(INTAKE_MOTOR);
        }
        task_delay(20);
    }
}

void drive_FN()
{
    for (;;) {
        int y = controller_get_analog(CONTROLLER_MASTER, ANALOG_LEFT_Y);
        int t = controller_get_analog(CONTROLLER_MASTER, -1 * ANALOG_RIGHT_X);
        if (y == 0 && t == 0) {
            motor_brake(FRONT_RIGHT_MOTOR);
            motor_brake(BACK_RIGHT_MOTOR);
            motor_brake(FRONT_LEFT_MOTOR);
            motor_brake(BACK_LEFT_MOTOR);

        } else {
            motor_move(FRONT_RIGHT_MOTOR, y + t);
            motor_move(FRONT_LEFT_MOTOR, y - t);
            motor_move(BACK_LEFT_MOTOR, y - t);
            motor_move(BACK_RIGHT_MOTOR, y + t);
        }
        task_delay(20);
 
    }
}

void cpt_FN()
{
    static bool c_on = false;
    for (;;) {
        //if (controller_get_digital_new_press(CONTROLLER_MASTER, DIGITAL_X))
        if (controller_get_digital_new_press(CONTROLLER_MASTER, DIGITAL_L1)) //-------------------
            c_on = !c_on;
        if (c_on) {
            motor_move(CATAPULT_MOTOR, CATAPULT_SPEED);
        } else {
            motor_brake(CATAPULT_MOTOR);
        }
        task_delay(20);
    }
}

void pne_FN()
{
    static bool in_on = false;
    for(;;) {
        if (controller_get_digital_new_press(CONTROLLER_MASTER, DIGITAL_DOWN))
            in_on = !in_on;
        adi_digital_write(PNEMATICS, in_on);
        task_delay(20);
    }
}

//Competition.autonomous(autonomous);
