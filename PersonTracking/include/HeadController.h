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

#define MOVE_X 0
#define MOVE_Y 1

#define MOTOR_HEAD_HORIZONTAL 0
#define MOTOR_HEAD_VERTICAL 1

#define NUM_SECONDS_TO_WAIT_FOR_CONNECTION 3
//----------------------------------------------------------------

class t_head_controller {
private:
	t_jenny5_command_module head_motors_controller;
public:
	t_head_controller(void);
	~t_head_controller(void);

	bool connect();
	bool setup();
	bool go_to_home_position(t_jenny5_command_module &head_controller, char* error_string);

	void move_left(unsigned int num_steps);
	void move_right(unsigned int num_steps);
	void move_up(unsigned int num_steps);
	void move_down(unsigned int num_steps);

};

#endif //HEAD_CONTROLLER_H
