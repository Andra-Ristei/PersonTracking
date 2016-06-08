#ifndef HEAD_CONTROLLER_H_
#define HEAD_CONTROLLER_H_

#include "jenny5_command_module.h"
#include "jenny5_events.h"
#include <iostream>

//----------------------------------------------------------------
#define DIRECTION_LEFT 1
#define DIRECTION_RIGTH -1
#define DIRECTION_UP 1
#define DIRECTION_DOWN -1

#define MOTOR_HEAD_HORIZONTAL 0
#define MOTOR_HEAD_VERTICAL 1

#define NUM_SECONDS_TO_WAIT_FOR_CONNECTION 3
#define DOES_NOTHING_SLEEP 10
#define ULTRASONIC 0

#define HEAD_MOTOR_REDUCTION 27
#define HEAD_MOTOR_STEP_ANGLE 1.8
//----------------------------------------------------------------

class t_head_controller {
private:
	t_jenny5_command_module head_motors_controller;
public:
	t_head_controller(void);
	~t_head_controller(void);

	bool connect();
	bool setup();
	bool go_to_home_position();
	void read_from_serial_port();
	int read_sensor(int sensor);
	void ping_sensor(int sensor);
	void move_left(unsigned int num_steps);
	void move_right(unsigned int num_steps);
	void move_up(unsigned int num_steps);
	void move_down(unsigned int num_steps);
	void execute_commands_from_queue();
};

#endif //HEAD_CONTROLLER_H
