#include "../include/wheels_controller.h"
#include <time.h>

//----------------------------------------------------------------

t_wheels_controller::t_wheels_controller(void) {
	if (connect()) {
		printf("Connected...\n");
	}
	if (setup()) {
		printf("Setup complete...\n");
	}
}
//----------------------------------------------------------------

bool t_wheels_controller::connect() {
	//connect to serial ports
	if (!wheels_motors_controller.connect(2, 115200)) {
		std::cout << ("Error attaching to Jenny 5' upper arm motors!\n");
	}

	jenny5_event connect_to_wheels_motors_event(IS_ALIVE_EVENT);
	return wheels_motors_controller.wait_for_command_completion(connect_to_wheels_motors_event);
}
//----------------------------------------------------------------

bool t_wheels_controller::setup() {
	//create the motor controllers
	int wheels_motors_dir_pins[4] = { 2, 5, 8, 11 };
	int wheels_motors_step_pins[4] = { 3, 6, 9, 12 };
	int wheels_motors_enable_pins[4] = { 4, 7, 10, 13 };
	wheels_motors_controller.send_create_stepper_motors(4, wheels_motors_dir_pins, wheels_motors_step_pins, wheels_motors_enable_pins);

	jenny5_event create_wheels_motors_event(STEPPER_MOTORS_CONTROLLER_CREATED_EVENT);
	return wheels_motors_controller.wait_for_command_completion(create_wheels_motors_event);
}
//----------------------------------------------------------------

void t_wheels_controller::wait_for_action(int wait_for) {
	if (wait_for != NO_WAIT) {
		jenny5_event motor_done_event(STEPPER_MOTOR_MOVE_DONE_EVENT, wait_for);

		if (wait_for == WHEELS_MOVING) {
			wheels_motors_controller.wait_for_command_completion(motor_done_event, EVENT_INFO_TYPE | EVENT_INFO_PARAM1);
		}
		if (wait_for == WHEELS_ALIGNING) {
			wheels_motors_controller.wait_for_command_completion(motor_done_event, EVENT_INFO_TYPE | EVENT_INFO_PARAM1);
		}
	}
}
//----------------------------------------------------------------
double t_wheels_controller::get_distance_to_person() {
	return distance_to_person;
}
//----------------------------------------------------------------
double t_wheels_controller::get_maximum_distance_allowed() {
	return maximum_distance_allowed;
}
//----------------------------------------------------------------
double t_wheels_controller::get_minimum_distance_allowed() {
	return minimun_distance_allowed;
}
//----------------------------------------------------------------
void t_wheels_controller::set_distance_to_person(double new_distance) {
	distance_to_person = new_distance;
}
//----------------------------------------------------------------
void t_wheels_controller::set_maximum_distance_allowed(double new_distance) {
	maximum_distance_allowed = new_distance;
}
//----------------------------------------------------------------
void t_wheels_controller::set_minimum_distance_allowed(double new_distance) {
	minimun_distance_allowed = new_distance;
}
//----------------------------------------------------------------
double t_wheels_controller::get_modify_distance_by() {
	return modify_distance_by;
}
//----------------------------------------------------------------
void t_wheels_controller::set_modify_distance_by(double new_distance) {
	modify_distance_by = new_distance;
}
//----------------------------------------------------------------

void t_wheels_controller::move_backwards(double current_distance, int wait_for) {
	int num_steps = current_distance;//compute_steps_from_distance(current_distance);
	wheels_motors_controller.send_move_stepper_motor(MOTOR_FOOT_RIGHT, num_steps);
	wheels_motors_controller.set_stepper_motor_state(MOTOR_FOOT_RIGHT, COMMAND_SENT);
	wheels_motors_controller.send_move_stepper_motor(MOTOR_FOOT_LEFT, -num_steps);
	wheels_motors_controller.set_stepper_motor_state(MOTOR_FOOT_LEFT, COMMAND_SENT);
}
//----------------------------------------------------------------
void t_wheels_controller::move_forward(double current_distance, int wait_for) {
	int num_steps = current_distance;//compute_steps_from_distance(current_distance);
	wheels_motors_controller.send_move_stepper_motor(MOTOR_FOOT_LEFT, num_steps);
	wheels_motors_controller.set_stepper_motor_state(MOTOR_FOOT_LEFT, COMMAND_SENT);
	wheels_motors_controller.send_move_stepper_motor(MOTOR_FOOT_RIGHT, -num_steps);
	wheels_motors_controller.set_stepper_motor_state(MOTOR_FOOT_RIGHT, COMMAND_SENT);
}
//----------------------------------------------------------------
void t_wheels_controller::turn_left(int displacement, int wait_for) {
	int num_steps = displacement;
	wheels_motors_controller.send_move_stepper_motor(MOTOR_FOOT_RIGHT, num_steps);
	wheels_motors_controller.set_stepper_motor_state(MOTOR_FOOT_RIGHT, COMMAND_SENT);
}
//----------------------------------------------------------------
void t_wheels_controller::turn_right(int displacement, int wait_for) {
	int num_steps = displacement;
	wheels_motors_controller.send_move_stepper_motor(MOTOR_FOOT_LEFT, num_steps);
	wheels_motors_controller.set_stepper_motor_state(MOTOR_FOOT_LEFT, COMMAND_SENT);
}
//----------------------------------------------------------------

t_wheels_controller::~t_wheels_controller(void) {
	wheels_motors_controller.close_connection();
}
//----------------------------------------------------------------
