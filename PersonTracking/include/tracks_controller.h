#ifndef wheels_controllerH
#define wheels_controllerH

#include "jenny5_command_module.h"
#include "jenny5_events.h"
#include <iostream>

#define _USE_MATH_DEFINES
#include <math.h>

//----------------------------------------------------------------
#define NO_WAIT -1
#define MOTOR_FOOT_LEFT 0
#define MOTOR_FOOT_RIGHT 1
#define NUM_SECONDS_TO_WAIT_FOR_CONNECTION 3

#define MOTOR_tracks_LEFT 0
#define MOTOR_tracks_RIGHT 1

#define TRACKS_MOTOR_REDUCTION 5
#define TRACKS_MOTOR_STEP_ANGLE 1.8
//----------------------------------------------------------------

class t_tracks_controller {
private:
	t_jenny5_command_module wheels_motors_controller;
	double distance_to_person = 50; //50cm
	double modify_distance_by = 1;  //1cm
	double minimun_distance_allowed = 30;  //30cm
	double maximum_distance_allowed = 200;  //200cm


public:
	t_tracks_controller(void);
	~t_tracks_controller(void);

	bool connect();
	bool setup();
	void execute_commands_from_queue();
	void turn_right(int displacement);
	void turn_left(int displacement);

	int compute_steps_from_distance(int current_distance, int limit);

	void move_forward(int current_distance, int limit);
	void move_backwards(int current_distance, int limit);

	double get_minimum_distance_allowed();
	double get_maximum_distance_allowed();
	double get_distance_to_person();
	double get_modify_distance_by();
	void set_minimum_distance_allowed(double new_distance);
	void set_maximum_distance_allowed(double new_distance);
	void set_distance_to_person(double new_distance);
	void set_modify_distance_by(double new_distance);

	void cancel_commands();
};

#endif