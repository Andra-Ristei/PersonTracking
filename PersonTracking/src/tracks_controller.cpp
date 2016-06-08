#include "../include/tracks_controller.h"
#include <time.h>

//----------------------------------------------------------------
t_tracks_controller::t_tracks_controller(void) {
	if (connect()) {
		printf("Connected to wheels...\n");
	}
	if (setup()) {
		printf("Wheels setup complete...\n");
	}
}
//----------------------------------------------------------------
bool t_tracks_controller::connect() {
	//connect to serial ports
	if (!wheels_motors_controller.connect(3, 115200)) {
		printf("Error attaching to Jenny 5' wheels!\n");
	}

	// now wait to see if I have been connected
	// wait for no more than 3 seconds. If it takes more it means that something is not right, so we have to abandon it
	clock_t start_time = clock();
	bool foot_responded = false;

	while (1) {
		if (!wheels_motors_controller.update_commands_from_serial())
			Sleep(5);
		if (!foot_responded)
			if (wheels_motors_controller.query_for_event(IS_ALIVE_EVENT, 0))  // have we received the event from Serial ?
				foot_responded = true;

		if (foot_responded)
			break;
		// measure the passed time 
		clock_t end_time = clock();

		double wait_time = (double) (end_time - start_time) / CLOCKS_PER_SEC;
		// if more than 3 seconds then game over
		if (wait_time > NUM_SECONDS_TO_WAIT_FOR_CONNECTION) {
			if (!foot_responded)
				printf("Foot does not respond! Game over!\n");
			return false;
		}
	}
	return true;
}
//----------------------------------------------------------------
bool t_tracks_controller::setup() {
	//create the motor controllers
	int wheels_motors_dir_pins[2] = { 2, 8 };
	int wheels_motors_step_pins[2] = { 3, 9 };
	int wheels_motors_enable_pins[2] = { 4, 10 };

	wheels_motors_controller.send_create_stepper_motors(2, wheels_motors_dir_pins, wheels_motors_step_pins, wheels_motors_enable_pins);

	clock_t start_time = clock();
	bool foot_motors_controler_created = false;

	while (1) {
		if (!wheels_motors_controller.update_commands_from_serial())
			Sleep(5);
		if (!foot_motors_controler_created)
			if (wheels_motors_controller.query_for_event(STEPPER_MOTORS_CONTROLLER_CREATED_EVENT, 0))  // have we received the event from Serial ?
				foot_motors_controler_created = true;

		if (foot_motors_controler_created)
			break;

		// measure the passed time 
		clock_t end_time = clock();

		double wait_time = (double) (end_time - start_time) / CLOCKS_PER_SEC;
		// if more than 3 seconds then game over
		if (wait_time > NUM_SECONDS_TO_WAIT_FOR_CONNECTION) {
			if (!foot_motors_controler_created)
				printf("Cannot create foot's motor controller! Game over!\n");
			return false;
		}
	}

	wheels_motors_controller.send_set_stepper_motor_speed_and_acceleration(MOTOR_tracks_LEFT, 1500, 500);
	wheels_motors_controller.send_set_stepper_motor_speed_and_acceleration(MOTOR_tracks_RIGHT, 1500, 500);

	return true;
}
//----------------------------------------------------------------
void t_tracks_controller::execute_commands_from_queue() {
	//extract movements for tracks
	// now extract the moves done from the queue
	if (wheels_motors_controller.get_stepper_motor_state(MOTOR_tracks_LEFT) == COMMAND_SENT) {// if a command has been sent
		if (wheels_motors_controller.query_for_event(STEPPER_MOTOR_MOVE_DONE_EVENT, MOTOR_tracks_LEFT)) { // have we received the event from Serial ?
			wheels_motors_controller.set_stepper_motor_state(MOTOR_tracks_LEFT, COMMAND_DONE);
			//printf("tracks: M%d# - done\n", MOTOR_tracks_LEFT);
		}
	}
	if (wheels_motors_controller.get_stepper_motor_state(MOTOR_tracks_RIGHT) == COMMAND_SENT) {// if a command has been sent
		if (wheels_motors_controller.query_for_event(STEPPER_MOTOR_MOVE_DONE_EVENT, MOTOR_tracks_RIGHT)) { // have we received the event from Serial ?
			wheels_motors_controller.set_stepper_motor_state(MOTOR_tracks_RIGHT, COMMAND_DONE);
			//printf("tracks: M%d# - done\n", MOTOR_tracks_RIGHT);
		}
	}
}
//----------------------------------------------------------------
double t_tracks_controller::get_distance_to_person() {
	return distance_to_person;
}
//----------------------------------------------------------------
double t_tracks_controller::get_maximum_distance_allowed() {
	return maximum_distance_allowed;
}
//----------------------------------------------------------------
double t_tracks_controller::get_minimum_distance_allowed() {
	return minimun_distance_allowed;
}
//----------------------------------------------------------------
void t_tracks_controller::set_distance_to_person(double new_distance) {
	distance_to_person = new_distance;
}
//----------------------------------------------------------------
void t_tracks_controller::set_maximum_distance_allowed(double new_distance) {
	maximum_distance_allowed = new_distance;
}
//----------------------------------------------------------------
void t_tracks_controller::set_minimum_distance_allowed(double new_distance) {
	minimun_distance_allowed = new_distance;
}
//----------------------------------------------------------------
double t_tracks_controller::get_modify_distance_by() {
	return modify_distance_by;
}
//----------------------------------------------------------------
void t_tracks_controller::set_modify_distance_by(double new_distance) {
	modify_distance_by = new_distance;
}
//----------------------------------------------------------------
int t_tracks_controller::compute_steps_from_distance(int current_distance, int limit) {
	int distance = abs(limit - current_distance);
	//x=d/pi*diam => nr rotatii roata mare, diam~=9
	//200 pasi/rotatie compl => x=x*200 = nr pasi
	//reductie roata mare de 5la1 => x*5.13
	//raport reductie rotita mare si reductie rotita mica => x*4
	double big_wheel_diameter = 10, steps_per_rotation = 200, big_wheel_reduction = 5.13, small_wheel_reduction = 4;
	double x = (double) distance / (M_PI*big_wheel_diameter);
	x = x*steps_per_rotation;
	x = x*big_wheel_reduction;
	x = x*small_wheel_reduction;

	return (int) x;
}
//----------------------------------------------------------------
void t_tracks_controller::move_backwards(int current_distance, int limit) {
	//compute the number of steps the motors have to move, based on the distance from the sensor
	int num_steps = compute_steps_from_distance(current_distance, limit);

	wheels_motors_controller.send_move_stepper_motor(MOTOR_tracks_LEFT, -num_steps);//-1000);
	wheels_motors_controller.set_stepper_motor_state(MOTOR_tracks_LEFT, COMMAND_SENT);
	printf("move left tracks backwards: M%d %d# - sent\n", MOTOR_tracks_LEFT, -num_steps);//-1000);

	wheels_motors_controller.send_move_stepper_motor(MOTOR_tracks_RIGHT, num_steps);//1000);
	wheels_motors_controller.set_stepper_motor_state(MOTOR_tracks_RIGHT, COMMAND_SENT);
	printf("move right tracks backwards: M%d %d# - sent\n", MOTOR_tracks_RIGHT, num_steps);//1000);
}
//----------------------------------------------------------------
void t_tracks_controller::move_forward(int current_distance, int limit) {
	//compute the number of steps the motors have to move, based on the distance
	int num_steps = compute_steps_from_distance(current_distance, limit);

	wheels_motors_controller.send_move_stepper_motor(MOTOR_tracks_LEFT, num_steps);//1000);
	wheels_motors_controller.set_stepper_motor_state(MOTOR_tracks_LEFT, COMMAND_SENT);
	printf("move left tracks forward: M%d %d# - sent\n", MOTOR_tracks_LEFT, num_steps);//1000);

	wheels_motors_controller.send_move_stepper_motor(MOTOR_tracks_RIGHT, -num_steps);//-1000);
	wheels_motors_controller.set_stepper_motor_state(MOTOR_tracks_RIGHT, COMMAND_SENT);
	printf("move right tracks forward: M%d %d# - sent\n", MOTOR_tracks_RIGHT, -num_steps);//-1000);
}
//----------------------------------------------------------------
void t_tracks_controller::turn_left(int displacement) {
	wheels_motors_controller.send_move_stepper_motor2(MOTOR_tracks_LEFT, displacement, MOTOR_tracks_RIGHT, displacement);
	wheels_motors_controller.set_stepper_motor_state(MOTOR_tracks_LEFT, COMMAND_SENT);
	wheels_motors_controller.set_stepper_motor_state(MOTOR_tracks_RIGHT, COMMAND_SENT);
	printf("turn tracks left: M%d %d M%d %d# - sent\n", MOTOR_tracks_LEFT, displacement, MOTOR_tracks_RIGHT, displacement);
}
//----------------------------------------------------------------
void t_tracks_controller::turn_right(int displacement) {
	wheels_motors_controller.send_move_stepper_motor2(MOTOR_tracks_RIGHT, displacement, MOTOR_tracks_LEFT, displacement);
	wheels_motors_controller.set_stepper_motor_state(MOTOR_tracks_RIGHT, COMMAND_SENT);
	wheels_motors_controller.set_stepper_motor_state(MOTOR_tracks_LEFT, COMMAND_SENT);
	printf("turn tracks right: M%d %d M%d %d# - sent\n", MOTOR_tracks_RIGHT, displacement, MOTOR_tracks_LEFT, displacement);
}
//----------------------------------------------------------------
t_tracks_controller::~t_tracks_controller(void) {
	wheels_motors_controller.close_connection();
}
//----------------------------------------------------------------
