#include "main.h"
#define FRONT_RIGHT_MOTOR 13
#define FRONT_LEFT_MOTOR 14
#define BACK_LEFT_MOTOR 17
#define BACK_RIGHT_MOTOR 18
#define INTAKE_MOTOR 11
#define INTAKE_SPEED 127
#define PNEMATICS 1 
#define IMU_PORT 20
#define CATAPULT_MOTOR 3
#define CATAPULT_SPEED 127
#define ROTATION_RIGHT_PORT 1
#define ROTATION_LEFT_PORT 7
#define AUTONOMOUS_SPEED 100 //unused as of 11/29

void drive_FN();
void intake_FN();
void pne_FN();
void cpt_FN();



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



//this is to move the robot forward with less code to prevent issues    =-=-=-=-=-=-=-=-=-=-=-=-=-=-

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
 
void autonomous(){//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

    // control variables
    int tolGreater = 5;
    int tolLesser = -5;
    int vel0 = 100;


// start of autononmous
    adi_digital_write(PNEMATICS, true);
    // move forward 900 mm
    int dist1 = 900;
    moveForward(900, vel0);
    // turn left
    turnLeft(90,100);
    int dist3 = 600;
    // move 600 mm fwd
    moveForward(600, vel0);
    turnLeft(90, 100);
    int dist6 = 900;
    // move 900 mm fwd
    moveForward(900, vel0);
    // intake the triball
    motor_move(INTAKE_MOTOR, 100);
    int dist7 = 20;
    // back up a smidge
    movebackward(20, vel0); 
    // right
    turnRight(90, 100);
    //move forward to the net thing
    moveForward(100, vel0);
    // unload the triball
    motor_move_absolute(INTAKE_MOTOR, -100, 100);
    
} //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-




//this pushes down the intake via pnematics


void opcontrol(){
    task_t drive_task = task_create(drive_FN, NULL, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Drive");
    task_t intake_task = task_create(intake_FN, NULL, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Intake");
    task_t pnematic_task = task_create(pne_FN, NULL, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Pnematic");
    task_t catapult_task = task_create(cpt_FN, NULL, TASK_PRIORITY_DEFAULT, TASK_STACK_DEPTH_DEFAULT, "Catapult");
}

void intake_FN() 
{
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
        int t = controller_get_analog(CONTROLLER_MASTER, ANALOG_RIGHT_X);
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
        if (controller_get_digital_new_press(CONTROLLER_MASTER, DIGITAL_L1))
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
